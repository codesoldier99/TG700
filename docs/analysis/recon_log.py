"""
Reconnaissance pass over an ArduPilot text-format DataFlash (.log) file.
Discovers message schemas from FMT lines, counts message types, reports
IMU instances + logging rates, and ESC/RPM availability + ranges.
No external deps beyond stdlib.
"""
import sys
from collections import defaultdict, Counter

def recon(path, max_lines=None):
    fmt = {}                      # name -> list of column names
    counts = Counter()
    imu_times = defaultdict(list) # instance -> [TimeUS]
    esc_rpm = defaultdict(list)   # instance -> [rpm]
    esc_cols = None
    imu_cols = None
    n = 0
    with open(path, 'r', errors='replace') as f:
        for line in f:
            n += 1
            if max_lines and n > max_lines:
                break
            if not line:
                continue
            # split on comma, strip
            parts = [p.strip() for p in line.split(',')]
            if not parts:
                continue
            mtype = parts[0]
            if mtype == 'FMT':
                # FMT, Type, Length, Name, Format, Col1,Col2,...
                if len(parts) >= 5:
                    name = parts[3]
                    cols = parts[5:]
                    fmt[name] = cols
                continue
            counts[mtype] += 1
            if mtype == 'IMU':
                cols = fmt.get('IMU')
                if cols and len(parts)-1 >= len(cols):
                    d = dict(zip(cols, parts[1:1+len(cols)]))
                    try:
                        inst = int(float(d.get('I', -1)))
                        imu_times[inst].append(int(float(d['TimeUS'])))
                    except Exception:
                        pass
                    imu_cols = cols
            elif mtype == 'ESC':
                cols = fmt.get('ESC')
                if cols and len(parts)-1 >= len(cols):
                    esc_cols = cols
                    d = dict(zip(cols, parts[1:1+len(cols)]))
                    inst = d.get('Instance', d.get('I', d.get('Instc', '0')))
                    rpm = d.get('RPM', None)
                    try:
                        esc_rpm[int(float(inst))].append(float(rpm))
                    except Exception:
                        pass

    print("=== FILE ===", path)
    print("lines scanned:", n)
    print("\n=== MESSAGE COUNTS (top 25) ===")
    for k, v in counts.most_common(25):
        print(f"  {k:8s} {v}")

    print("\n=== IMU FMT COLUMNS ===")
    print(" ", imu_cols)
    print("\n=== IMU INSTANCES & LOGGING RATE ===")
    for inst in sorted(imu_times):
        t = imu_times[inst]
        if len(t) > 10:
            dt = [(t[i+1]-t[i]) for i in range(len(t)-1) if 0 < t[i+1]-t[i] < 1e7]
            dt_med = sorted(dt)[len(dt)//2] if dt else 0
            rate = 1e6/dt_med if dt_med else 0
            dur = (t[-1]-t[0])/1e6
            print(f"  IMU[{inst}]: n={len(t)}, median dt={dt_med}us, rate~{rate:.1f}Hz, dur~{dur:.1f}s")

    print("\n=== ESC FMT COLUMNS ===")
    print(" ", esc_cols)
    print("\n=== ESC RPM RANGE per instance ===")
    for inst in sorted(esc_rpm):
        r = esc_rpm[inst]
        if r:
            print(f"  ESC[{inst}]: n={len(r)}, min={min(r):.0f}, max={max(r):.0f}, "
                  f"mean={sum(r)/len(r):.0f}")

def find_log(substr, ext='.log', logs_dir=r'd:\ardupilot_TG700\logs'):
    import os
    matches = [f for f in os.listdir(logs_dir)
               if substr in f and f.lower().endswith(ext)]
    if not matches:
        raise FileNotFoundError(f"no {ext} file containing '{substr}' in {logs_dir}")
    # prefer the largest match
    matches.sort(key=lambda f: os.path.getsize(os.path.join(logs_dir, f)), reverse=True)
    return os.path.join(logs_dir, matches[0])

if __name__ == '__main__':
    substr = sys.argv[1]
    ml = int(sys.argv[2]) if len(sys.argv) > 2 else None
    path = find_log(substr)
    recon(path, ml)
