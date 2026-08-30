"""Fast VIBE-only scan across all .log files to locate ones with REAL motor
vibration (VIBE magnitude well above the ~0.02 stationary noise floor).
Only splits VIBE lines -> fast."""
import os
import numpy as np

LOGS_DIR = r'd:\ardupilot_TG700\logs'

def scan(path):
    vib_col = None
    fmt_cols = None
    mags = []
    with open(path, 'r', errors='replace') as f:
        for line in f:
            if line.startswith('FMT') and ', VIBE,' in line:
                parts = [p.strip() for p in line.split(',')]
                fmt_cols = parts[5:]
                continue
            if line.startswith('VIBE,'):
                if fmt_cols is None:
                    continue
                parts = [p.strip() for p in line.split(',')]
                d = dict(zip(fmt_cols, parts[1:1+len(fmt_cols)]))
                try:
                    vx=float(d['VibeX']); vy=float(d['VibeY']); vz=float(d['VibeZ'])
                    mags.append((vx*vx+vy*vy+vz*vz)**0.5)
                except Exception:
                    pass
    if not mags:
        return None
    a = np.asarray(mags)
    return dict(n=len(a), mean=a.mean(), p95=np.percentile(a,95), mx=a.max())

if __name__ == '__main__':
    results = []
    for f in sorted(os.listdir(LOGS_DIR)):
        if not f.lower().endswith('.log'):
            continue
        p = os.path.join(LOGS_DIR, f)
        if os.path.getsize(p) < 1_000_000:
            continue
        try:
            r = scan(p)
        except Exception as e:
            r = None
        if r:
            results.append((f, r))
            print(f"maxVIBE={r['mx']:7.2f} p95={r['p95']:6.2f} mean={r['mean']:6.2f} n={r['n']:6d}  {f.encode('ascii','replace').decode()}")
    print("\n=== SORTED BY maxVIBE (candidates with real vibration on top) ===")
    for f, r in sorted(results, key=lambda x: -x[1]['mx']):
        print(f"  maxVIBE={r['mx']:7.2f}  p95={r['p95']:6.2f}  {f.encode('ascii','replace').decode()}")
