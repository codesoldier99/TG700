"""
Gating analysis (v2, corrected) for the vibration-centric proposal.

Fixes vs v1:
  - Common ABSOLUTE-time uniform grid for all IMUs (v1 aligned each IMU to its
    own t0 -> destroyed inter-IMU coherence -> false F2 conclusion).
  - Activity picked by true vibration proxy (VIBE) / max motor PWM, not mean.
  - Prints activity timeline; contrasts a QUIET vs LOUD window to separate
    real airframe MOTION (grows little with throttle) from VIBRATION (grows).

Questions:
  F1: energy in 1-15 Hz control band, and does it grow with throttle/RPM?
  F2: inter-IMU coherence (properly time-aligned).
Deps: numpy, scipy, matplotlib.
"""
import os, sys
import numpy as np
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

LOGS_DIR = r'd:\ardupilot_TG700\logs'
OUT_DIR  = r'd:\ardupilot_TG700\docs\analysis\out'
os.makedirs(OUT_DIR, exist_ok=True)

CTRL_BAND = (1.0, 15.0)
BANDS = [(0.1,1.0),(1.0,15.0),(15.0,50.0),(50.0,150.0)]
FS = 300.0


def find_log(substr, ext='.log'):
    matches = [f for f in os.listdir(LOGS_DIR) if substr in f and f.lower().endswith(ext)]
    if not matches:
        raise FileNotFoundError(f"no {ext} containing '{substr}'")
    matches.sort(key=lambda f: os.path.getsize(os.path.join(LOGS_DIR, f)), reverse=True)
    return os.path.join(LOGS_DIR, matches[0])


def parse(path):
    fmt = {}
    imu = defaultdict(lambda: {'t': [], 'gx': [], 'gy': [], 'gz': [], 'ax': [], 'ay': [], 'az': []})
    rcou = {'t': [], 'max': []}
    vibe = {'t': [], 'mag': []}
    with open(path, 'r', errors='replace') as f:
        for line in f:
            parts = [p.strip() for p in line.split(',')]
            if not parts:
                continue
            m = parts[0]
            if m == 'FMT':
                if len(parts) >= 5:
                    fmt[parts[3]] = parts[5:]
                continue
            if m == 'IMU':
                cols = fmt.get('IMU')
                if not cols or len(parts)-1 < len(cols):
                    continue
                d = dict(zip(cols, parts[1:1+len(cols)]))
                try:
                    inst = int(float(d['I'])); r = imu[inst]
                    r['t'].append(int(float(d['TimeUS'])))
                    r['gx'].append(float(d['GyrX'])); r['gy'].append(float(d['GyrY'])); r['gz'].append(float(d['GyrZ']))
                    r['ax'].append(float(d['AccX'])); r['ay'].append(float(d['AccY'])); r['az'].append(float(d['AccZ']))
                except Exception:
                    pass
            elif m == 'RCOU':
                cols = fmt.get('RCOU')
                if not cols or len(parts)-1 < len(cols):
                    continue
                d = dict(zip(cols, parts[1:1+len(cols)]))
                try:
                    vals = [float(d[c]) for c in cols if c != 'TimeUS']
                    mv = [v for v in vals if 900 <= v <= 2100]
                    if mv:
                        rcou['t'].append(int(float(d['TimeUS']))); rcou['max'].append(max(mv))
                except Exception:
                    pass
            elif m == 'VIBE':
                cols = fmt.get('VIBE')
                if not cols or len(parts)-1 < len(cols):
                    continue
                d = dict(zip(cols, parts[1:1+len(cols)]))
                try:
                    vx = float(d.get('VibeX', d.get('VibeX ', 0)))
                    vy = float(d['VibeY']); vz = float(d['VibeZ'])
                    vibe['t'].append(int(float(d['TimeUS']))); vibe['mag'].append((vx*vx+vy*vy+vz*vz)**0.5)
                except Exception:
                    pass
    return imu, rcou, vibe


def build_common_grid(imu, insts, fs=FS):
    """Resample all IMU channels onto ONE common absolute-time grid."""
    t0 = max(np.asarray(imu[i]['t'], float)[0] for i in insts) * 1e-6
    t1 = min(np.asarray(imu[i]['t'], float)[-1] for i in insts) * 1e-6
    n = int((t1 - t0) * fs)
    tg = t0 + np.arange(n) / fs           # absolute seconds
    out = {}
    for i in insts:
        t = np.asarray(imu[i]['t'], float) * 1e-6
        for key in ('gx','gy','gz','ax','ay','az'):
            out[(i,key)] = np.interp(tg, t, np.asarray(imu[i][key], float))
    return tg, out


def timeline(tg, grid, insts, rcou, vibe, nbin=24):
    """Print coarse activity timeline to locate motor spin-up."""
    print("\n=== ACTIVITY TIMELINE (bin: t, maxPWM, VIBE, IMU0 gyro-RMS) ===")
    t_rel = tg - tg[0]
    dur = t_rel[-1]
    rt = (np.asarray(rcou['t'], float)*1e-6) if rcou['t'] else np.array([])
    rmax = np.asarray(rcou['max'], float) if rcou['t'] else np.array([])
    vt = (np.asarray(vibe['t'], float)*1e-6) if vibe['t'] else np.array([])
    vmag = np.asarray(vibe['mag'], float) if vibe['t'] else np.array([])
    g0 = np.sqrt(grid[(insts[0],'gx')]**2 + grid[(insts[0],'gy')]**2 + grid[(insts[0],'gz')]**2)
    edges = np.linspace(0, dur, nbin+1)
    rows = []
    for k in range(nbin):
        a, b = edges[k], edges[k+1]
        pwm = rmax[(rt-rt[0]>=a)&(rt-rt[0]<b)].mean() if rt.size else float('nan')
        vb = vmag[(vt-vt[0]>=a)&(vt-vt[0]<b)].mean() if vt.size else float('nan')
        gr = g0[(t_rel>=a)&(t_rel<b)].std()
        rows.append((a, pwm, vb, gr))
        print(f"  t={a:6.1f}s  PWMmax~{pwm:7.1f}  VIBE~{vb:6.2f}  gyroRMS~{gr:6.3f}")
    return rows


def band_energy(f, pxx, lo, hi):
    m = (f >= lo) & (f < hi)
    return np.trapz(pxx[m], f[m])


def welch_win(x, fs=FS):
    x = x - x.mean()
    return signal.welch(x, fs=fs, nperseg=int(fs*4), noverlap=int(fs*2))


def analyze(substr):
    path = find_log(substr)
    print("=== ANALYZING ===", os.path.basename(path).encode('ascii','replace').decode())
    imu, rcou, vibe = parse(path)
    insts = sorted(imu.keys())
    print("IMU instances:", insts, "| RCOU pts:", len(rcou['t']), "| VIBE pts:", len(vibe['t']))
    tg, grid = build_common_grid(imu, insts)
    dur = tg[-1]-tg[0]
    print(f"common grid: {len(tg)} samples @ {FS}Hz, dur ~{dur:.0f}s")

    rows = timeline(tg, grid, insts, rcou, vibe)

    # pick LOUD window = max VIBE bin; QUIET window = min VIBE bin (both >=40s from data)
    valid = [(r[0], r[2]) for r in rows if not np.isnan(r[2])]
    if valid:
        loud_s = max(valid, key=lambda x: x[1])[0]
        quiet_s = min(valid, key=lambda x: x[1])[0]
    else:
        # fall back to gyroRMS
        loud_s = max(rows, key=lambda x: x[3])[0]
        quiet_s = min(rows, key=lambda x: x[3])[0]
    win = 40.0
    t_rel = tg - tg[0]

    def band_report(tag, s):
        print(f"\n=== F1 [{tag}] window [{s:.0f},{s+win:.0f}]s : band-energy fraction ===")
        mask = (t_rel>=s)&(t_rel<s+win)
        res = {}
        for key,label in [('gx','GyrX'),('gy','GyrY'),('gz','GyrZ'),('az','AccZ')]:
            x = grid[(insts[0],key)][mask]
            if len(x) < FS*8:
                continue
            f,pxx = welch_win(x)
            tot = band_energy(f,pxx,0.1,150)
            fr = [band_energy(f,pxx,lo,hi)/tot if tot>0 else 0 for lo,hi in BANDS]
            res[key]=(f,pxx,tot)
            ss = "  ".join([f"{lo:.0f}-{hi:.0f}:{fr[j]*100:5.1f}%" for j,(lo,hi) in enumerate(BANDS)])
            print(f"  IMU0 {label} | totE={tot:.3e} | {ss}")
        return res, mask

    res_loud, mask_loud = band_report("LOUD", loud_s)
    res_quiet, mask_quiet = band_report("QUIET", quiet_s)

    # KEY: does absolute in-band energy grow from quiet->loud? (vibration) or not? (motion/noise)
    print("\n=== VIBRATION vs MOTION test: absolute 1-15Hz gyro energy quiet->loud ===")
    for key in ('gx','gy','gz'):
        if key in res_loud and key in res_quiet:
            fl,pl,_ = res_loud[key]; fq,pq,_ = res_quiet[key]
            el = band_energy(fl,pl,*CTRL_BAND); eq = band_energy(fq,pq,*CTRL_BAND)
            ratio = el/eq if eq>0 else float('inf')
            print(f"  {key}: quiet={eq:.3e}  loud={el:.3e}  ratio={ratio:.1f}x")

    # F2 coherence with PROPER common-time alignment, LOUD window
    print("\n=== F2: INTER-IMU COHERENCE (common-time aligned, LOUD window) ===")
    for key,label in [('gx','GyrX'),('gy','GyrY'),('az','AccZ')]:
        a = grid[(insts[0],key)][mask_loud]; b = grid[(insts[1],key)][mask_loud]
        a=a-a.mean(); b=b-b.mean()
        fco,cxy = signal.coherence(a,b,fs=FS,nperseg=int(FS*4))
        m1=(fco>=1)&(fco<15); m2=(fco>=15)&(fco<50)
        print(f"  IMU0-IMU1 {label}: coh 1-15Hz={cxy[m1].mean():.3f}  15-50Hz={cxy[m2].mean():.3f}")

    # plots: PSD overlay loud window
    plt.figure(figsize=(11,6))
    for inst in insts:
        x = grid[(inst,'gx')][mask_loud]
        f,pxx = welch_win(x)
        plt.semilogy(f,pxx,label=f'IMU[{inst}] GyrX')
    plt.axvspan(*CTRL_BAND,color='orange',alpha=0.2,label='control band')
    plt.xlim(0,150); plt.xlabel('Hz'); plt.ylabel('PSD'); plt.title(f'GyrX PSD LOUD [{loud_s:.0f}s] - 3 IMUs')
    plt.legend(); plt.grid(True,which='both',alpha=0.3)
    p=os.path.join(OUT_DIR,'psd_gyroX_loud.png'); plt.savefig(p,dpi=110,bbox_inches='tight'); plt.close()
    print("\nsaved:",p)

    # quiet vs loud overlay (GyrX)
    plt.figure(figsize=(11,6))
    xL=grid[(insts[0],'gx')][mask_loud]; fL,pL=welch_win(xL)
    xQ=grid[(insts[0],'gx')][mask_quiet]; fQ,pQ=welch_win(xQ)
    plt.semilogy(fL,pL,label=f'LOUD [{loud_s:.0f}s]')
    plt.semilogy(fQ,pQ,label=f'QUIET [{quiet_s:.0f}s]')
    plt.axvspan(*CTRL_BAND,color='orange',alpha=0.2,label='control band')
    plt.xlim(0,150); plt.xlabel('Hz'); plt.ylabel('PSD'); plt.title('GyrX PSD: quiet vs loud (IMU0)')
    plt.legend(); plt.grid(True,which='both',alpha=0.3)
    p2=os.path.join(OUT_DIR,'psd_gyroX_quiet_vs_loud.png'); plt.savefig(p2,dpi=110,bbox_inches='tight'); plt.close()
    print("saved:",p2)


if __name__ == '__main__':
    analyze(sys.argv[1] if len(sys.argv) > 1 else '20260621')
