#!/usr/bin/env bash
set -e
python3 - <<'PY'
import os,signal,time
from pathlib import Path
def snapshot():
    result={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            fields=(p/'stat').read_text().split(') ',1)[1].split()
            args=(p/'cmdline').read_bytes().split(b'\0')
            result[int(p.name)]=(int(fields[1]),fields[19],args,fields[0])
        except (OSError,ValueError,IndexError):continue
    return result
processes=snapshot()
roots=[pid for pid,(_,_,args,_) in processes.items() if
    (b'launch' in args and b'bg1_leader_coverage' in args and b'demo.launch.py' in args)]
targets=set(roots)
while True:
    children={pid for pid,(parent,_,_,_) in processes.items() if parent in targets}
    if children<=targets:break
    targets|=children
def alive():
    current=snapshot()
    return [pid for pid in targets if pid in current and current[pid][1]==processes[pid][1] and current[pid][3]!='Z']
for pid in roots:
    try:os.kill(pid,signal.SIGINT)
    except ProcessLookupError:pass
for sig,grace in [(None,5),(signal.SIGINT,3),(signal.SIGTERM,3)]:
    if sig is not None:
        for pid in alive():
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
    until=time.monotonic()+grace
    while alive() and time.monotonic()<until:time.sleep(.1)
remaining=alive()
print('Leader coverage session stopped.' if not remaining else f'Processes still running: {remaining}')
raise SystemExit(bool(remaining))
PY
