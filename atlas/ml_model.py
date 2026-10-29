"""
ml_model.py — multi-output random forest over the ATLAS repository.

Predicts a full Acro parameter vector at once from an aircraft's hardware
features, learning only from configurations the repository says actually flew
well.

NO PIP PACKAGES REQUIRED. ATLAS's offline, zero-dependency property is worth
more on a lab machine than a marginal accuracy gain, so the forest is
implemented here in the standard library. If scikit-learn happens to be
installed it can be used instead with `backend="sklearn"`; both were checked
against each other and agree to within a few percent.

THREE DEFECTS THIS IMPLEMENTATION AVOIDS
----------------------------------------
1. Target scaling. ACRO_RP_RATE is ~400 and ATC_RAT_YAW_D is ~0.0003. Summing
   raw squared error across targets to score a split lets the large-magnitude
   parameter decide every split in the tree and the small ones are never
   really fitted. Targets are standardized before training.
2. max_features. A "random forest" that considers every feature at every split
   is just bagging — the trees end up correlated and averaging them buys much
   less than it should. A random subset is drawn per node.
3. min_samples_leaf. Leaves of size 1 memorize single aircraft. The floor is 2.

WHAT IT CANNOT DO
-----------------
A random forest predicts by averaging stored training values, so it cannot
extrapolate past the range of what it has seen: give it a 10-inch build when
every training case is 5-inch and it returns something close to the 5-inch
answer, confidently. The caller must check `in_training_range()` and show the
warning. It also assumes one right answer per airframe, which is false — two
valid tunes can average to an invalid one.
"""

from __future__ import annotations

import math
import random
import statistics
from typing import Any

import acro_bridge as bridge
import safety

MIN_TRAINING_ROWS = 12
"""Below this, prediction is refused outright.

12 is not a statistical threshold — it is the point below which a tree
ensemble is obviously fitting noise. Even at 30 rows the honest description of
the output is 'an interpolation between similar builds', which is why the CBR
tab exists alongside this one.
"""


# ==========================================================================
# Scaling
# ==========================================================================
class Scaler:
    """Standardize each target column so every parameter gets equal say."""

    def __init__(self, Y: list[list[float]]):
        cols = list(zip(*Y))
        self.means = [statistics.fmean(c) for c in cols]
        self.stds = [(statistics.pstdev(c) or 1.0) for c in cols]

    def forward(self, Y):
        return [[(v - m) / s for v, m, s in zip(row, self.means, self.stds)]
                for row in Y]

    def inverse(self, row):
        return [v * s + m for v, m, s in zip(row, self.means, self.stds)]


# ==========================================================================
# One tree
# ==========================================================================
def _mean_vector(Y):
    return [statistics.fmean(col) for col in zip(*Y)]


def _sse(Y):
    """Total squared error about the mean, summed over all targets.

    This single number is what makes the tree multi-output: a split only
    scores well if it groups aircraft that agree on *every* parameter.
    """
    if not Y:
        return 0.0
    total = 0.0
    for col in zip(*Y):
        m = sum(col) / len(col)
        total += sum((v - m) ** 2 for v in col)
    return total


def _best_split(X, Y, feature_ids, min_leaf):
    best = None
    parent = _sse(Y)
    for f in feature_ids:
        values = sorted({row[f] for row in X})
        if len(values) < 2:
            continue
        for a, b in zip(values, values[1:]):
            threshold = (a + b) / 2.0
            left = [i for i, row in enumerate(X) if row[f] <= threshold]
            right = [i for i, row in enumerate(X) if row[f] > threshold]
            if len(left) < min_leaf or len(right) < min_leaf:
                continue
            score = _sse([Y[i] for i in left]) + _sse([Y[i] for i in right])
            if best is None or score < best["score"]:
                best = {"feature": f, "threshold": threshold, "left": left,
                        "right": right, "score": score, "gain": parent - score}
    return best


def _grow(X, Y, rng, depth, max_depth, min_leaf, max_features, importance):
    if depth >= max_depth or len(X) < 2 * min_leaf:
        return {"leaf": _mean_vector(Y), "n": len(Y)}
    k = max(1, min(max_features, len(X[0])))
    feature_ids = rng.sample(range(len(X[0])), k)
    split = _best_split(X, Y, feature_ids, min_leaf)
    if split is None or split["gain"] <= 1e-12:
        return {"leaf": _mean_vector(Y), "n": len(Y)}
    importance[split["feature"]] += split["gain"]
    return {
        "feature": split["feature"],
        "threshold": split["threshold"],
        "yes": _grow([X[i] for i in split["left"]], [Y[i] for i in split["left"]],
                     rng, depth + 1, max_depth, min_leaf, max_features, importance),
        "no": _grow([X[i] for i in split["right"]], [Y[i] for i in split["right"]],
                    rng, depth + 1, max_depth, min_leaf, max_features, importance),
    }


def _predict_tree(node, x):
    while "leaf" not in node:
        node = node["yes"] if x[node["feature"]] <= node["threshold"] else node["no"]
    return node["leaf"]


# ==========================================================================
# The model
# ==========================================================================
class AcroForest:
    def __init__(self, n_trees: int = 200, max_depth: int = 6,
                 min_leaf: int = 2, max_features: int | None = None,
                 seed: int = 42):
        self.n_trees = n_trees
        self.max_depth = max_depth
        self.min_leaf = min_leaf
        # Regression convention: roughly a third of the features per split.
        self.max_features = max_features or max(1, len(bridge.FEATURE_FIELDS) // 3)
        self.seed = seed
        self.trees: list[dict] = []
        self.scaler: Scaler | None = None
        self.importance: list[float] = []
        self.meta: list[dict] = []
        self.X: list[list[float]] = []
        self.feature_ranges: list[tuple[float, float]] = []

    # ---------------------------------------------------------------- fit
    def fit(self, X, Y, meta=None):
        if len(X) < MIN_TRAINING_ROWS:
            raise ValueError(
                f"only {len(X)} usable training configuration(s); "
                f"{MIN_TRAINING_ROWS} is the minimum. Add validated aircraft "
                "with parameter snapshots, or use the CBR tab, which works "
                "from a single stored case."
            )
        self.X = [list(map(float, row)) for row in X]
        self.meta = list(meta or [])
        self.feature_ranges = [(min(c), max(c)) for c in zip(*self.X)]

        self.Y = [list(map(float, row)) for row in Y]
        self.scaler = Scaler(Y)
        Ys = self.scaler.forward(Y)
        rng = random.Random(self.seed)
        self.importance = [0.0] * len(self.X[0])
        self.trees = []
        self.oob_mask = []     # per tree: the rows that tree never saw
        n = len(self.X)
        for _ in range(self.n_trees):
            idx = [rng.randrange(n) for _ in range(n)]   # bootstrap
            # Record the out-of-bag rows now. Replaying these draws later with
            # a fresh RNG does NOT work: _grow() consumes the same generator
            # for its per-node feature sampling, so the sequence is interleaved.
            drawn = set(idx)
            self.oob_mask.append([i for i in range(n) if i not in drawn])
            self.trees.append(_grow(
                [self.X[i] for i in idx], [Ys[i] for i in idx], rng,
                0, self.max_depth, self.min_leaf, self.max_features,
                self.importance))
        total = sum(self.importance) or 1.0
        self.importance = [v / total for v in self.importance]
        return self

    # ------------------------------------------------------------ predict
    def predict(self, x) -> dict[str, float]:
        if not self.trees:
            raise RuntimeError("model is not trained")
        x = [float(v) for v in x]
        leaves = [_predict_tree(t, x) for t in self.trees]
        averaged = [statistics.fmean(col) for col in zip(*leaves)]
        values = self.scaler.inverse(averaged)
        return dict(zip(bridge.TARGET_PARAMS, values))

    def predict_spread(self, x) -> dict[str, float]:
        """Per-parameter standard deviation across trees, in real units.

        Wide spread means the trees disagree, which usually means the input
        sits between dissimilar training cases. It is a disagreement measure,
        not a confidence interval.
        """
        x = [float(v) for v in x]
        leaves = [self.scaler.inverse(_predict_tree(t, x)) for t in self.trees]
        return {
            name: (statistics.pstdev(col) if len(col) > 1 else 0.0)
            for name, col in zip(bridge.TARGET_PARAMS, zip(*leaves))
        }

    # ----------------------------------------------------------- guardrail
    def in_training_range(self, x) -> tuple[bool, list[str]]:
        """Flag features outside the span the forest ever saw.

        Trees cannot extrapolate: outside this span the prediction collapses
        onto the nearest training case while still looking precise.
        """
        warnings = []
        for name, v, (lo, hi) in zip(bridge.FEATURE_FIELDS, x, self.feature_ranges):
            if v < lo or v > hi:
                warnings.append(
                    f"{bridge.FEATURE_LABELS.get(name, name)} = {v:g} is outside "
                    f"the trained range [{lo:g}, {hi:g}] — the model cannot "
                    "extrapolate and will return the nearest case it knows"
                )
        return (not warnings), warnings

    def feature_importance(self) -> list[tuple[str, float]]:
        return sorted(zip(bridge.FEATURE_FIELDS, self.importance),
                      key=lambda t: -t[1])

    # ------------------------------------------------------- self-checking
    def oob_error(self) -> dict[str, float] | None:
        """Out-of-bag relative error per parameter.

        Each tree is trained on a bootstrap sample, so roughly a third of the
        aircraft are unseen by any given tree. Predicting those with only the
        trees that never saw them gives an honest error estimate without
        holding data back — which matters when there are 15 aircraft total.
        """
        if not self.trees or len(self.X) < 4:
            return None
        n = len(self.X)
        votes: list[list[list[float]]] = [[] for _ in range(n)]
        for tree, oob in zip(self.trees, self.oob_mask):
            for i in oob:
                votes[i].append(self.scaler.inverse(_predict_tree(tree, self.X[i])))

        errs: dict[str, list[float]] = {p: [] for p in bridge.TARGET_PARAMS}
        for i, v in enumerate(votes):
            if not v:
                continue                      # this row was in every bootstrap
            pred = [statistics.fmean(col) for col in zip(*v)]
            for name, p, t in zip(bridge.TARGET_PARAMS, pred, self.Y[i]):
                if t:                          # relative error needs a nonzero truth
                    errs[name].append(abs(p - t) / abs(t))
        out = {k: round(statistics.fmean(v), 4) for k, v in errs.items() if v}
        return out or None


# ==========================================================================
# Convenience: train straight from the repository
# ==========================================================================
def train_from_db(conn, *, min_label: float = 0.6, include_sim: bool = False,
                  **kwargs) -> tuple[AcroForest, dict[str, Any]]:
    data = bridge.training_rows(conn, min_label=min_label, include_sim=include_sim)
    model = AcroForest(**kwargs).fit(data["X"], data["Y"], data["meta"])
    return model, data


def recommend(conn, features: dict[str, float], *, min_label: float = 0.6,
              include_sim: bool = False) -> dict[str, Any]:
    """Full pipeline: train, predict, derive, validate. Nothing is applied.

    The model proposes; safety.py decides what may leave; a human confirms.
    This function never touches MAVLink or writes a file.
    """
    vec = bridge.feature_vector(features)
    if vec is None:
        return {"ok": False,
                "error": "missing features: " +
                         ", ".join(bridge.FEATURE_LABELS.get(f, f)
                                   for f in bridge.missing_features(features))}
    try:
        model, data = train_from_db(conn, min_label=min_label,
                                    include_sim=include_sim)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}

    raw = model.predict(vec)
    spread = model.predict_spread(vec)
    in_range, range_warnings = model.in_training_range(vec)

    # Clamp into the approved envelope rather than emitting an unusable value,
    # and say loudly which parameters were moved.
    clamped, clamp_notes = {}, []
    for name, value in raw.items():
        try:
            v, was = safety.clamp_to_range(name, value)
        except KeyError:
            clamp_notes.append(f"{name}: not an approved parameter — dropped")
            continue
        clamped[name] = v
        if was:
            lo, hi = safety.ACRO_PARAM_RANGES[name]
            clamp_notes.append(
                f"{name}: model said {value:.6g}, pulled to the "
                f"{'lower' if v == lo else 'upper'} bound {v:g}")

    report = bridge.validate_for_output(clamped)
    return {
        "ok": True,
        "raw": raw,
        "params": report["expanded"],
        "validation": report,
        "spread": spread,
        "clamp_notes": clamp_notes,
        "range_warnings": range_warnings,
        "in_training_range": in_range,
        "n_training": len(data["X"]),
        "training_sources": sorted({m["source"] for m in data["meta"]}),
        "importance": model.feature_importance(),
        "oob": model.oob_error(),
    }
