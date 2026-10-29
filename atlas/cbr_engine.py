"""Offline, explainable case-based retrieval. Scores are NOT safety probabilities."""
import re
WEIGHTS={'frame':22,'frame_size':8,'fc':20,'esc':10,'motor':14,'kv':12,'stator':8,'quantity':6}
FLIGHT_DEPENDENT_PREFIXES=('ATC_RAT_','ATC_ANG_','PSC_','INS_HNTCH_','INS_HNTC2_','AUTOTUNE_','RATE_','Q_A_RAT_','Q_A_ANG_')
CRITICAL_PREFIXES=('SERVO','MOT_','FRAME_','BRD_','RC','ARMING_','FS_','BATT_','GPS_','COMPASS_','EK3_','EK2_')
def norm(x): return re.sub(r'\s+',' ',str(x or '').strip().lower())
def number(x):
 """First number in a value, so '5 inch' and '5' compare as equal.

 Plain float() returns None for '5 inch', '220mm' and '5"', which made
 frame_size score 0% on an exact match and silently degraded every retrieval
 where one side carried a unit and the other did not."""
 if x is None:return None
 try:return float(str(x).strip())
 except (TypeError,ValueError):pass
 m=re.search(r'-?\d+(?:\.\d+)?',str(x))
 return float(m.group(0)) if m else None
def extract(spec,kind):
 m=re.search(r'(?:'+kind+r')\s*([\d.]+)',spec or '',re.I)
 return m.group(1) if m else ''
def features(a,comps):
 d={'frame':a['frame_type'] or '', 'frame_size':a['frame_size'] or ''}
 for x in comps:
  cat=x['category']; key={'Flight Controller':'fc','ESC':'esc','Motor':'motor'}.get(cat)
  if key:
   d[key]=' '.join(filter(None,[x['manufacturer'],x['model']]))
  if cat=='Motor':
   d['kv']=extract(x['specs'],'KV') or (re.search(r'([\d.]+)\s*KV',x['specs'] or '',re.I).group(1) if re.search(r'([\d.]+)\s*KV',x['specs'] or '',re.I) else '')
   d['stator']=extract(x['specs'],'Stator');d['quantity']=extract(x['specs'],'Qty')
 return d
def compare(target,case,weights=None):
 weights=weights or WEIGHTS; details=[]; total=0; earned=0
 for key,w in weights.items():
  a,b=norm(target.get(key)),norm(case.get(key))
  if not a:continue
  total+=w
  if not b:sim=0; reason='not recorded'
  elif key in ('kv','frame_size','quantity') and number(a) is not None and number(b) is not None:
   av,bv=number(a),number(b);sim=max(0,1-abs(av-bv)/max(abs(av),abs(bv),1));reason='numeric proximity'
  elif a==b:sim=1;reason='exact match'
  elif key in ('fc','esc','motor') and (a in b or b in a):sim=.75;reason='partial model match'
  else:sim=0;reason='different'
  earned+=w*sim;details.append({'field':key,'target':a,'case':b,'score':round(sim*100),'reason':reason})
 return round(100*earned/total,1) if total else 0,details

def retrieve(conn,target,weights=None):
 rows=[]
 for a in conn.execute('SELECT * FROM aircraft'):
  comps=conn.execute('SELECT * FROM components WHERE aircraft_id=?',(a['id'],)).fetchall()
  score,details=compare(target,features(a,comps),weights)
  files=conn.execute("SELECT COUNT(*) FROM parameter_files WHERE aircraft_id=? AND source='hardware'",(a['id'],)).fetchone()[0]
  rows.append({'id':a['id'],'name':a['name'],'status':a['validation_status'],'score':score,'details':details,'files':files})
 return sorted(rows,key=lambda x:(x['score'],x['files']>0),reverse=True)

def candidate(conn,aircraft_id):
 """Conservative reuse: export source snapshot with warnings, never merge unknown firmware."""
 # source='hardware': a simulated run files the tune it flew, and without this
 # filter the newest snapshot for an aircraft can be a SITL tune, which would be
 # exported as a reusable candidate alongside that aircraft's hardware validation.
 pf=conn.execute("SELECT * FROM parameter_files WHERE aircraft_id=? AND source='hardware' ORDER BY id DESC LIMIT 1",(aircraft_id,)).fetchone()
 if not pf:
  sim=conn.execute("SELECT COUNT(*) FROM parameter_files WHERE aircraft_id=? AND source<>'hardware'",(aircraft_id,)).fetchone()[0]
  raise ValueError('This case has no hardware parameter snapshot to reuse.'+(f' It has {sim} simulated snapshot(s), which are not exportable as candidates.' if sim else ''))
 rows=conn.execute('SELECT name,value FROM parameters WHERE parameter_file_id=? ORDER BY id',(pf['id'],)).fetchall()
 if not rows:raise ValueError('The selected parameter snapshot has no parsed parameters.')
 seen={}; warnings=[]
 for r in rows:
  name=str(r['name']).strip();value=str(r['value']).strip()
  if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,31}',name) or not re.fullmatch(r'[-+\d.eE]+',value):
   warnings.append('Skipped malformed or nonnumeric parameter: '+name);continue
  seen[name]=value
  if name.startswith(FLIGHT_DEPENDENT_PREFIXES):warnings.append('Flight-dependent: '+name)
  elif name.startswith(CRITICAL_PREFIXES):warnings.append('Verify hardware/safety mapping: '+name)
 if not seen:raise ValueError('No exportable numeric parameters found.')
 return pf,seen,warnings
