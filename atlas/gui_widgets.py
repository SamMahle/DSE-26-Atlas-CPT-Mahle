"""
gui_widgets.py — NiceGUI building blocks shared by every ATLAS screen.

Theme, confirmation and message dialogs, a file and folder picker that
browses this computer, coloured read-only text panes, tables, and an
editable combo box. Nothing here knows about aircraft or parameters.

Why a custom file picker: ATLAS runs as a local web server and the window is a
browser page. A browser's own file dialog uploads a file's *content* but never
reveals its path, and ATLAS needs real paths — it copies evidence into
repository_files/, writes exports next to each other, and points AMC at
folders. The server is this same computer, so the picker lists its disks
directly.
"""

from __future__ import annotations

import fnmatch
import html
import os
import string
import sys
from pathlib import Path
from typing import Any, Callable, Iterable

from nicegui import ui

# The ATLAS palette, carried over from the Tkinter interface.
C = {"bg": "#0b0f14", "panel": "#121821", "panel2": "#18212c", "line": "#263241", "text": "#edf2f7",
     "muted": "#8fa0b3", "red": "#d33a4e", "red2": "#a51f35", "green": "#43c17a", "amber": "#e6ad45",
     "bad": "#ff8795"}

# Text-pane tag -> CSS class. The tag names are the ones the Tkinter panes used.
TAG_CLASS = {"warn": "t-warn", "sim": "t-warn", "bad": "t-bad", "block": "t-bad",
             "good": "t-good", "ok": "t-good", "hw": "t-good", "muted": "t-muted",
             "amc": "t-warn", "safety_atlas": "t-bad", "atlas": "", "par": "t-good",
             "inf": "t-muted", "tpl": "t-warn", "rec": "", "": ""}

CSS = f"""
:root {{ --atlas-bg:{C['bg']}; --atlas-panel:{C['panel']}; --atlas-panel2:{C['panel2']};
         --atlas-line:{C['line']}; --atlas-text:{C['text']}; --atlas-muted:{C['muted']}; }}
body, .q-page, .nicegui-content {{ background:{C['bg']} !important; color:{C['text']};
         font-family:'Segoe UI', system-ui, -apple-system, sans-serif; }}
.nicegui-content {{ padding:0 !important; }}
.atlas-eyebrow {{ font-size:11px; font-weight:600; letter-spacing:.08em; color:{C['red']}; }}
.atlas-title {{ font-size:22px; font-weight:600; color:{C['text']}; line-height:1.2; }}
.atlas-sub {{ font-size:13px; color:{C['muted']}; line-height:1.45; }}
.atlas-card {{ background:{C['panel']} !important; border:1px solid {C['line']}; border-radius:6px;
         box-shadow:none !important; padding:16px; width:100%; }}
.atlas-card-title {{ font-size:11px; font-weight:600; letter-spacing:.06em; color:{C['muted']};
         text-transform:uppercase; margin-bottom:6px; }}
.atlas-metric {{ font-size:28px; font-weight:600; color:{C['text']}; line-height:1.1; }}
.atlas-metric-label {{ font-size:10px; font-weight:600; letter-spacing:.06em; color:{C['muted']}; }}
.atlas-pane {{ background:{C['panel']}; border:1px solid {C['line']}; border-radius:6px; }}
.atlas-pane pre, .atlas-mono {{ font-family:Consolas, 'Cascadia Mono', Menlo, monospace; font-size:12.5px;
         white-space:pre-wrap; word-break:break-word; margin:0; padding:12px 14px; color:{C['text']}; }}
.t-warn {{ color:{C['amber']}; }} .t-bad {{ color:{C['bad']}; }} .t-good {{ color:{C['green']}; }}
.t-muted {{ color:{C['muted']}; }}
.atlas-verdict {{ background:{C['panel']}; border:1px solid {C['line']}; border-left:4px solid {C['muted']};
         border-radius:4px; padding:9px 14px; font-weight:600; font-size:13px; width:100%; }}
.atlas-status {{ font-size:12px; color:{C['muted']}; }}
.atlas-table {{ background:{C['panel']} !important; border:1px solid {C['line']}; border-radius:6px; }}
.atlas-table thead tr th {{ background:{C['panel2']}; color:{C['muted']}; font-weight:600; font-size:12px;
         position:sticky; top:0; z-index:1; }}
.atlas-table tbody td {{ font-size:13px; }}
.atlas-table tbody tr.selected td, .atlas-table tbody tr.selected {{ background:rgba(211,58,78,.28) !important; }}
.atlas-table tbody tr {{ cursor:pointer; }}
.atlas-dialog {{ background:{C['panel']} !important; border:1px solid {C['line']}; min-width:440px; max-width:860px; }}
.atlas-dialog-title {{ font-size:18px; font-weight:600; }}
.atlas-dialog-body {{ white-space:pre-wrap; font-size:13px; color:{C['text']}; max-height:60vh; overflow:auto; }}
.atlas-tabs .q-tab {{ text-transform:none; font-weight:600; letter-spacing:.01em; }}
.atlas-tabs .q-tab--active {{ color:{C['text']}; }}
.atlas-tabs .q-tab--inactive {{ color:{C['muted']}; }}
.atlas-picker-item {{ border-radius:4px; }}
.atlas-picker-item.selected {{ background:rgba(211,58,78,.28); }}
.q-field--outlined .q-field__control {{ background:{C['panel2']}; }}
.atlas-footer {{ background:{C['bg']} !important; border-top:1px solid {C['line']}; }}
.q-btn {{ flex-shrink:0; }}
/* grid and flex children may shrink below their content; wide tables scroll inside themselves */
.nicegui-grid > *, .nicegui-column > *, .atlas-card {{ min-width:0; }}
.atlas-table .q-table__middle {{ overflow-x:auto; }}
"""


def install_theme() -> None:
    """Apply the ATLAS look to the current page."""
    ui.add_css(CSS)
    ui.colors(primary=C["red"], secondary=C["panel2"], accent=C["red2"], positive=C["green"],
              negative=C["red"], warning=C["amber"], info=C["muted"], dark=C["bg"], dark_page=C["bg"])
    ui.dark_mode(True)


# --------------------------------------------------------------------------- layout helpers
def eyebrow(text: str) -> ui.label:
    return ui.label(text).classes("atlas-eyebrow")


def title(text: str) -> ui.label:
    return ui.label(text).classes("atlas-title")


def sub(text: str = "") -> ui.label:
    return ui.label(text).classes("atlas-sub")


def card(heading: str | None = None) -> ui.card:
    """A bordered panel; use as a context manager. The heading sits inside it."""
    c = ui.card().classes("atlas-card gap-1")
    if heading:
        with c:
            ui.label(heading).classes("atlas-card-title")
    return c


def primary(text: str, on_click: Callable | None = None, icon: str | None = None) -> ui.button:
    return ui.button(text, on_click=on_click, icon=icon).props("unelevated no-caps no-wrap color=primary")


def secondary(text: str, on_click: Callable | None = None, icon: str | None = None) -> ui.button:
    return ui.button(text, on_click=on_click, icon=icon).props("unelevated no-caps no-wrap color=secondary text-color=white")


def danger(text: str, on_click: Callable | None = None, icon: str | None = None) -> ui.button:
    return ui.button(text, on_click=on_click, icon=icon).props("outline no-caps no-wrap color=negative")


def text_input(label: str, value: str = "", *, required: bool = False, width: str | None = None) -> ui.input:
    w = ui.input(label + ("  *" if required else ""), value=value).props("outlined dense stack-label")
    w.classes(width or "w-full")
    return w


def fixed_select(label: str, options: list, value: Any = None, *, required: bool = False,
                 on_change: Callable | None = None, width: str | None = None) -> ui.select:
    """A drop-down that only accepts its listed options (Tk's readonly Combobox)."""
    w = ui.select(options, label=label + ("  *" if required else ""), value=value, on_change=on_change)
    w.props("outlined dense stack-label options-dense").classes(width or "w-full")
    return w


class Combo:
    """Free text with a list of suggestions (Tk's editable Combobox).

    A plain input with a drop-down button beside it, rather than a Quasar
    select in input mode: that discards typed text the operator never confirms
    with Enter, and a manufacturer typed and tabbed past must not vanish.
    """

    def __init__(self, label: str, options: Iterable[str] = (), value: str = "", *,
                 required: bool = False, on_change: Callable[[str], Any] | None = None,
                 width: str | None = None):
        self._on_change = on_change
        self.input = text_input(label, value, required=required, width=width)
        with self.input.add_slot("append"):
            self._button = ui.button(icon="arrow_drop_down").props("flat dense round size=sm color=grey-5")
            with self._button:
                self._menu = ui.menu().props("auto-close")
        if on_change:
            self.input.on("blur", lambda: on_change(self.value))
        self.set_options(options)

    @property
    def value(self) -> str:
        return (self.input.value or "").strip()

    @value.setter
    def value(self, v: str) -> None:
        self.input.value = v or ""

    def set_options(self, options: Iterable[str]) -> None:
        self.options = [str(o) for o in options]
        self._menu.clear()
        with self._menu:
            if not self.options:
                ui.item("No saved choices").props("disable dense")
            for opt in self.options:
                ui.item(opt, on_click=lambda o=opt: self._pick(o)).props("dense")

    def _pick(self, option: str) -> None:
        self.value = option
        if self._on_change:
            self._on_change(option)


# --------------------------------------------------------------------------- text panes
class TextPane:
    """A read-only, scrollable pane of coloured lines (Tk's Text with tags)."""

    def __init__(self, height: str = "18rem", *, mono: bool = True):
        self._parts: list[tuple[str, str]] = []
        self._area = ui.scroll_area().classes("atlas-pane w-full").style(f"height:{height}")
        with self._area:
            self._html = ui.html("", sanitize=False, tag="pre" if mono else "div")
            if not mono:
                self._html.classes("atlas-sub").style("white-space:pre-wrap;padding:10px 12px")

    def clear(self) -> None:
        self._parts = []
        self._render()

    def set_text(self, text: str, tag: str = "") -> None:
        self._parts = [(text, tag)]
        self._render()

    def write(self, text: str, tag: str = "") -> None:
        self._parts.append((text, tag))
        self._render()

    def set_lines(self, lines: Iterable[tuple[str, str]]) -> None:
        self._parts = [(t + "\n", tag) for t, tag in lines]
        self._render()

    @property
    def is_deleted(self) -> bool:
        """True once the pane's dialog has been closed; background work then stops writing."""
        return self._html.is_deleted

    def _render(self) -> None:
        if self._html.is_deleted:
            return
        self._html.content = "".join(
            f'<span class="{TAG_CLASS.get(tag, "")}">{html.escape(text)}</span>' if TAG_CLASS.get(tag)
            else html.escape(text) for text, tag in self._parts)


# --------------------------------------------------------------------------- tables
class DataTable:
    """A table with click-to-select rows and per-row colour.

    ``columns`` is a list of (key, label) pairs. Each row is a dict holding a
    value for every key, the row key, and optionally ``_tag`` (a TextPane tag
    name) to colour the row. ``selection`` is 'single' (click selects) or
    'multiple' (click toggles).
    """

    def __init__(self, columns: list[tuple[str, str]], *, row_key: str = "id", selection: str = "single",
                 height: str = "24rem", on_select: Callable[[list[dict]], Any] | None = None,
                 on_double_click: Callable[[dict], Any] | None = None, wrap: bool = False):
        self.row_key = row_key
        self.multiple = selection == "multiple"
        self._on_select = on_select
        self._selected: list = []          # row keys, in click order
        cols = [{"name": k, "label": label, "field": k, "align": "left", "sortable": True}
                for k, label in columns]
        self.table = ui.table(rows=[], columns=cols, row_key=row_key, pagination=0)
        self.table.classes("atlas-table w-full").style(f"max-height:{height}")
        self.table.props("flat dense hide-bottom" + (" wrap-cells" if wrap else ""))
        # Selection is tracked here and drawn with a row class, so it works the
        # same for single and multiple selection without a checkbox column.
        self.table.add_slot("body", r"""
            <q-tr :props="props" :class="props.row._cls"
                  @click="() => $parent.$emit('row_click', props.row)"
                  @dblclick="() => $parent.$emit('row_dblclick', props.row)">
              <q-td v-for="col in props.cols" :key="col.name" :props="props">{{ col.value }}</q-td>
            </q-tr>""")
        self.table.on("row_click", lambda e: self._clicked(e.args))
        if on_double_click:
            self.table.on("row_dblclick", lambda e: self._double(e.args, on_double_click))

    def set_rows(self, rows: list[dict], keep_selection: bool = True) -> None:
        for r in rows:
            r["_base"] = TAG_CLASS.get(r.pop("_tag", ""), "")
        keys = {r[self.row_key] for r in rows}
        self._selected = [k for k in self._selected if k in keys] if keep_selection else []
        self.table.rows = rows
        self._apply()

    def _apply(self) -> None:
        chosen = set(self._selected)
        for r in self.table.rows:
            r["_cls"] = (r.get("_base", "") + (" selected" if r[self.row_key] in chosen else "")).strip()
        self.table.update()

    @property
    def rows(self) -> list[dict]:
        return self.table.rows

    @property
    def selected(self) -> list[dict]:
        by_key = {r[self.row_key]: r for r in self.table.rows}
        return [by_key[k] for k in self._selected if k in by_key]

    @property
    def selected_key(self):
        return self._selected[0] if self._selected else None

    def select_key(self, key, notify: bool = True) -> None:
        self._selected = [key] if any(r[self.row_key] == key for r in self.table.rows) else []
        self._apply()
        if notify and self._on_select:
            self._on_select(self.selected)

    def _clicked(self, row: dict) -> None:
        key = row[self.row_key]
        if self.multiple:
            self._selected = [k for k in self._selected if k != key] if key in self._selected \
                else self._selected + [key]
        else:
            self._selected = [key]
        self._apply()
        if self._on_select:
            self._on_select(self.selected)

    def _double(self, row: dict, handler: Callable[[dict], Any]):
        key = row[self.row_key]
        self._selected = [key]
        self._apply()
        return handler(row)


# --------------------------------------------------------------------------- dialogs
def _dialog_shell(heading: str, body: str, kind: str = "info"):
    colour = {"info": C["text"], "warn": C["amber"], "error": C["bad"], "ok": C["green"]}.get(kind, C["text"])
    dialog = ui.dialog()
    with dialog, ui.card().classes("atlas-dialog gap-3"):
        ui.label(heading).classes("atlas-dialog-title").style(f"color:{colour}")
        ui.label(body).classes("atlas-dialog-body")
        buttons = ui.row().classes("w-full justify-end gap-2")
    return dialog, buttons


def alert(heading: str, body: str, kind: str = "info") -> None:
    """A message the operator dismisses (Tk's showinfo / showwarning / showerror)."""
    dialog, buttons = _dialog_shell(heading, body, kind)
    with buttons:
        primary("OK", dialog.close)
    dialog.on("hide", dialog.delete)
    dialog.open()


async def confirm(heading: str, body: str, *, yes: str = "Proceed", no: str = "Cancel",
                  kind: str = "warn") -> bool:
    """Yes/no question; True only if the operator chooses ``yes`` (Tk's askyesno)."""
    dialog, buttons = _dialog_shell(heading, body, kind)
    dialog.props("persistent")
    with buttons:
        secondary(no, lambda: dialog.submit(False))
        primary(yes, lambda: dialog.submit(True))
    result = await dialog
    dialog.delete()
    return result is True


async def prompt(heading: str, label: str, value: str = "") -> str | None:
    """One line of text, or None if cancelled (Tk's askstring)."""
    dialog = ui.dialog().props("persistent")
    with dialog, ui.card().classes("atlas-dialog gap-3"):
        ui.label(heading).classes("atlas-dialog-title")
        field = text_input(label, value)
        field.on("keydown.enter", lambda: dialog.submit(field.value))
        with ui.row().classes("w-full justify-end gap-2"):
            secondary("Cancel", lambda: dialog.submit(None))
            primary("OK", lambda: dialog.submit(field.value))
    result = await dialog
    dialog.delete()
    return None if result is None else str(result)


# --------------------------------------------------------------------------- file picker
_last_dir: dict[str, Path] = {}


def _roots() -> list[Path]:
    if sys.platform.startswith("win"):
        return [Path(f"{d}:\\") for d in string.ascii_uppercase if os.path.exists(f"{d}:\\")]
    return [Path("/")]


async def _pick(mode: str, heading: str, start: str | Path | None, patterns: Iterable[str] = (),
                filename: str = "", memory: str = "default") -> Path | None:
    """Browse this computer. mode: 'file' (open), 'dir' (choose folder) or 'save'."""
    patterns = tuple(p.lower() for p in patterns)
    here = Path(start).expanduser() if start else _last_dir.get(memory, Path.home())
    if here.is_file():
        here = here.parent
    while not here.exists() and here != here.parent:
        here = here.parent
    state = {"dir": here.resolve(), "chosen": None, "show_all": not patterns}

    dialog = ui.dialog().props("persistent")
    with dialog, ui.card().classes("atlas-dialog gap-2").style("width:760px;max-width:95vw"):
        ui.label(heading).classes("atlas-dialog-title")
        with ui.row().classes("w-full items-center no-wrap gap-2"):
            up_btn = ui.button(icon="arrow_upward").props("flat dense round").tooltip("Up one folder")
            home_btn = ui.button(icon="home").props("flat dense round").tooltip("Home folder")
            path_box = text_input("Folder").classes("grow")
            roots = _roots()
            if len(roots) > 1:
                drive = fixed_select("Drive", [str(r) for r in roots], width="w-24")
        listing = ui.scroll_area().classes("atlas-pane w-full").style("height:46vh")
        name_box = text_input("File name", filename) if mode in ("file", "save") else None
        with ui.row().classes("w-full items-center justify-between"):
            all_box = ui.checkbox("Show all files", value=state["show_all"]) if patterns else None
            note = ui.label("").classes("atlas-status")
            with ui.row().classes("gap-2"):
                secondary("Cancel", lambda: dialog.submit(None))
                ok_text = {"file": "Open", "dir": "Select This Folder", "save": "Save"}[mode]
                ok_btn = primary(ok_text)

    def matches(p: Path) -> bool:
        return state["show_all"] or any(fnmatch.fnmatch(p.name.lower(), pat) for pat in patterns)

    def go(folder: Path) -> None:
        try:
            entries = sorted(folder.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower()))
        except OSError as exc:
            note.text = f"Cannot open {folder}: {exc.strerror or exc}"
            return
        state["dir"] = folder
        path_box.value = str(folder)
        note.text = ""
        listing.clear()
        with listing:
            shown = 0
            for p in entries:
                if p.name.startswith(".") or p.name.startswith("$"):
                    continue
                try:
                    is_dir = p.is_dir()
                except OSError:
                    continue
                if not is_dir and (mode == "dir" or not matches(p)):
                    continue
                shown += 1
                with ui.item(on_click=lambda p=p, d=is_dir: clicked(p, d)).props("dense clickable") \
                        .classes("atlas-picker-item") as item:
                    with ui.item_section().props("avatar"):
                        ui.icon("folder" if is_dir else "description").classes(
                            "text-amber-4" if is_dir else "text-grey-5")
                    with ui.item_section():
                        ui.label(p.name)
                if not is_dir:
                    item.on("dblclick", lambda p=p: finish(p))
            if not shown:
                ui.label("(empty)").classes("atlas-status q-pa-md")

    def clicked(p: Path, is_dir: bool) -> None:
        if is_dir:
            go(p)
        elif name_box is not None:
            name_box.value = p.name

    async def finish(path: Path | None = None) -> None:
        if path is None:
            if mode == "dir":
                path = state["dir"]
            else:
                name = (name_box.value or "").strip()
                if not name:
                    note.text = "Enter or choose a file name."
                    return
                path = Path(name) if Path(name).is_absolute() else state["dir"] / name
        if mode == "file" and not path.is_file():
            note.text = f"{path.name} does not exist."
            return
        if mode == "save" and path.exists():
            if not await confirm("Replace file?", f"{path}\n\nalready exists. Replace it?", yes="Replace"):
                return
        dialog.submit(path)

    up_btn.on_click(lambda: go(state["dir"].parent))
    home_btn.on_click(lambda: go(Path.home()))
    path_box.on("keydown.enter", lambda: go(Path(path_box.value).expanduser()))
    ok_btn.on_click(lambda: finish())
    if name_box is not None:
        name_box.on("keydown.enter", lambda: finish())
    if all_box is not None:
        def toggle_all(e):
            state["show_all"] = e.value
            go(state["dir"])
        all_box.on_value_change(toggle_all)
    if len(_roots()) > 1:
        drive.on_value_change(lambda e: e.value and go(Path(e.value)))

    go(state["dir"])
    result = await dialog
    dialog.delete()
    if result is not None:
        result = Path(result)
        _last_dir[memory] = result if mode == "dir" else result.parent
    return result


async def pick_file(heading: str, patterns: Iterable[str] = (), start=None, memory: str = "default") -> Path | None:
    return await _pick("file", heading, start, patterns, memory=memory)


async def pick_dir(heading: str, start=None, memory: str = "dirs") -> Path | None:
    return await _pick("dir", heading, start, memory=memory)


async def pick_save(heading: str, filename: str, patterns: Iterable[str] = (), start=None,
                    memory: str = "save") -> Path | None:
    return await _pick("save", heading, start, patterns, filename=filename, memory=memory)
