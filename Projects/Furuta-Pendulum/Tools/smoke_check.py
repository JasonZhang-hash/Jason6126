"""Check snapshot syntax and selected headless paths without GUI or hardware."""
from pathlib import Path
import ast, json, subprocess, sys
root=Path(__file__).resolve().parents[1]
files=list((root/'Simulation').glob('*.py'))
for p in files: ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p))
print(f'Syntax passed: {len(files)} Python sources')
for name in ['furuta_measured_core.py','furuta_energy_swingup_gui.py']:
    cmd=[sys.executable,str(root/'Simulation'/name),'--headless']
    if name.endswith('_gui.py'): cmd += ['--seconds','2']
    r=subprocess.run(cmd,cwd=root/'Simulation',capture_output=True,text=True,timeout=40)
    print(name, 'exit=',r.returncode, r.stdout.strip(), r.stderr.strip())
    if r.returncode: raise SystemExit(r.returncode)
print('Smoke checks passed; this is not proof of successful swing-up or hardware stability.')
