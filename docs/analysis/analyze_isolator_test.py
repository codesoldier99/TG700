# -*- coding: utf-8 -*-
"""
Isolator test analysis: ZLG CAN Bridge + 600V no-load, 16 motors.
ESC#15 uses 15_2_noload config; all others standard.
"""
import os, sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOGS_DIR = r'd:\ardupilot_TG700\logs'
OUT_DIR  = r'd:\ardupilot_TG700\docs\analysis\out'
os.makedirs(OUT_DIR, exist_ok=True)

def find_log(substr, ext='.log'):
    matches = [f for f in os.listdir(LOGS_DIR) if substr in f and f.lower().endswith(ext)]
    if not matches:
        raise FileNotFoundError(f"no {ext} containing '{substr}'")
    matches.sort(key=lambda f: os.path.getsize(os.path.join(LOGS_DIR, f)), reverse=True)
    return os.path.join(LOGS_DIR, matches[0])

def parse(path):
    fmt = {}
    esc  = defaultdict(lambda: {'t':[],'rpm':[],'volt':[],'curr':[],'temp':[],'err':[],'mottemp':[]})
    cans = {'t':[], 'bo':[], 'tto':[], 'rer':[]}
    vibe = {'t':[], 'vx':[], 'vy':[], 'vz':[]}
    imu0 = {'t':[], 'gx':[], 'gy':[], 'gz':[]}
    bat  = {'t':[], 'volt':[], 'curr':[]}
    with open(path, 'r', errors='replace') as f:
        for line in f:
            parts = [p.strip() for p in line.split(',')]
            if not parts: continue
            m = parts[0]
            if m == 'FMT':
                if len(parts) >= 5:
                    fmt[parts[3]] = parts[5:]
                continue
            if m == 'ESC':
                c = fmt.get('ESC')
                if not c or len(parts)-1 < len(c): continue
                d = dict(zip(c, parts[1:1+len(c)]))
                try:
                    i = int(float(d['Instance'])); r = esc[i]
                    r['t'].append(int(float(d['TimeUS'])))
                    r['rpm'].append(float(d['RPM']))
                    r['volt'].append(float(d['Volt']))
                    r['curr'].append(float(d['Curr']))
                    r['temp'].append(float(d['Temp']))
                    r['mottemp'].append(float(d.get('MotTemp', 0)))
                    r['err'].append(int(float(d['Err'])))
                except: pass
            elif m == 'CANS':
                c = fmt.get('CANS')
                if not c or len(parts)-1 < len(c): continue
                d = dict(zip(c, parts[1:1+len(c)]))
                try:
                    cans['t'].append(int(float(d['TimeUS'])))
                    cans['bo'].append(int(float(d.get('Bo', d.get('BusOff', 0)))))
                    cans['tto'].append(int(float(d.get('Tto', d.get('TxTimeout', 0)))))
                    cans['rer'].append(int(float(d.get('Rer', d.get('RxErr', 0)))))
                except: pass
            elif m == 'VIBE':
                c = fmt.get('VIBE')
                if not c or len(parts)-1 < len(c): continue
                d = dict(zip(c, parts[1:1+len(c)]))
                try:
                    vibe['t'].append(int(float(d['TimeUS'])))
                    vibe['vx'].append(float(d['VibeX']))
                    vibe['vy'].append(float(d['VibeY']))
                    vibe['vz'].append(float(d['VibeZ']))
                except: pass
            elif m == 'IMU':
                c = fmt.get('IMU')
                if not c or len(parts)-1 < len(c): continue
                d = dict(zip(c, parts[1:1+len(c)]))
                try:
                    if int(float(d['I'])) == 0:
                        imu0['t'].append(int(float(d['TimeUS'])))
                        imu0['gx'].append(float(d['GyrX']))
                        imu0['gy'].append(float(d['GyrY']))
                        imu0['gz'].append(float(d['GyrZ']))
                except: pass
            elif m == 'BAT':
                c = fmt.get('BAT')
                if not c or len(parts)-1 < len(c): continue
                d = dict(zip(c, parts[1:1+len(c)]))
                try:
                    bat['t'].append(int(float(d['TimeUS'])))
                    bat['volt'].append(float(d.get('Volt', d.get('VoltR', 0))))
                    bat['curr'].append(float(d.get('Curr', 0)))
                except: pass
    return esc, cans, vibe, imu0, bat

def rel(t_us_list):
    if not t_us_list: return np.array([])
    t = np.asarray(t_us_list, float) * 1e-6
    return t - t[0]

def analyze(substr):
    path = find_log(substr)
    print("FILE:", os.path.basename(path).encode('ascii','replace').decode())
    esc, cans, vibe, imu0, bat = parse(path)

    # --- 1. CANS BusOff ---
    print("\n[1] CAN Bus Health (CANS)")
    if cans['t']:
        bo   = np.asarray(cans['bo'])
        tto  = np.asarray(cans['tto'])
        rer  = np.asarray(cans['rer'])
        tc   = rel(cans['t'])
        dur  = tc[-1] - tc[0]
        bo_events  = int(np.diff(bo,  prepend=bo[0]).clip(0).sum())
        tto_events = int(np.diff(tto, prepend=tto[0]).clip(0).sum())
        print(f"  Duration: {dur:.1f}s")
        print(f"  BusOff total increment: {bo_events}  ({bo_events/dur:.3f}/s)")
        print(f"  TxTimeout total increment: {tto_events}  ({tto_events/dur:.3f}/s)")
        print(f"  RxErr final: {rer[-1]}")
        if bo_events == 0:
            print("  *** BusOff = 0 -- CAN bus completely clean! ***")
        elif bo_events / dur < 1:
            print("  >> BusOff < 1/s -- significant improvement vs pre-isolator")
        else:
            print("  !! BusOff still high -- needs further investigation")
    else:
        print("  No CANS records found")

    # --- 2. Bus voltage from ESC ---
    print("\n[2] Bus Voltage (from ESC telemetry)")
    all_v = []
    for i in sorted(esc.keys()):
        v = np.asarray(esc[i]['volt'])
        all_v.extend(v[v > 100].tolist())
    if all_v:
        av = np.asarray(all_v)
        print(f"  ESC Volt: min={av.min():.1f}V  max={av.max():.1f}V  mean={av.mean():.1f}V")

    # --- 3. Per-ESC summary ---
    print("\n[3] ESC Per-Instance Summary")
    print(f"  {'ESC':>4} {'n':>5} {'RPM_mean':>9} {'RPM_max':>8} {'RPM_min+':>9} "
          f"{'Volt_mean':>9} {'Curr_mean':>9} {'Temp_max':>8} {'Err_cnt':>7} {'Err_codes':>14}")
    for i in sorted(esc.keys()):
        r = esc[i]
        if not r['t']: continue
        rpm  = np.asarray(r['rpm'])
        volt = np.asarray(r['volt'])
        curr = np.asarray(r['curr'])
        temp = np.asarray(r['temp'])
        err  = np.asarray(r['err'])
        rpm_pos = rpm[rpm > 0]
        err_nz  = err[err != 0]
        err_codes = sorted(set(err_nz.tolist())) if len(err_nz) else []
        volt_m = volt[volt>100].mean() if (volt>100).any() else 0
        flag = "  <-- ESC#15 noload-cfg" if i == 15 else ""
        print(f"  {i:>4} {len(rpm):>5} {rpm.mean():>9.0f} {rpm.max():>8.0f} "
              f"{rpm_pos.min() if len(rpm_pos) else 0:>9.0f} "
              f"{volt_m:>9.1f} {curr.mean():>9.3f} {temp.max():>8.1f} "
              f"{len(err_nz):>7} {str(err_codes):>14}{flag}")

    # --- 4. ESC#15 detail ---
    print("\n[4] ESC#15 Detail (blinking LED)")
    r15 = esc.get(15)
    if r15 and r15['t']:
        t15  = rel(r15['t'])
        rpm  = np.asarray(r15['rpm'])
        err  = np.asarray(r15['err'])
        curr = np.asarray(r15['curr'])
        volt = np.asarray(r15['volt'])
        print(f"  Samples: {len(t15)}, span: {t15[-1]:.1f}s")
        rpm_pos = rpm[rpm>0]
        print(f"  RPM: mean={rpm.mean():.0f}  max={rpm.max():.0f}  "
              f"min(>0)={rpm_pos.min() if len(rpm_pos) else 0:.0f}")
        err_idx = np.where(err != 0)[0]
        print(f"  Err non-zero count: {len(err_idx)}")
        if len(err_idx):
            unique_err = sorted(set(err[err_idx].tolist()))
            print(f"  Err codes: {unique_err}")
            for k, idx in enumerate(err_idx[:20]):
                print(f"    t={t15[idx]:7.2f}s  RPM={rpm[idx]:6.0f}  "
                      f"Err={err[idx]}  Curr={curr[idx]:.3f}A  Volt={volt[idx]:.1f}V")
            if len(err_idx) > 20:
                print(f"    ... total {len(err_idx)} occurrences")
        else:
            print("  Err = 0 throughout (no error codes)")
        # dropout detection
        drops = np.where((rpm[:-1] > 100) & (rpm[1:] == 0))[0]
        print(f"  RPM dropout events (RPM>100 -> 0): {len(drops)}")
        for idx in drops[:10]:
            print(f"    t={t15[idx]:.2f}s  RPM:{rpm[idx]:.0f} -> 0")
    else:
        print("  No ESC#15 data")

    # --- 5. Config diff ---
    print("\n[5] ESC#15 (15_2_noload) vs standard (14.json) key differences")
    diffs = [
        ("noload_detect_enable",        "0 (off)",  "1 (ON)",  "15_2 enables no-load detection -> may trigger Err=8 at no-load"),
        ("carrier_freq_khz",            "12",        "16",      "Lower switch freq -> less EMI but possible audio noise"),
        ("idling_speed_rpm",            "500",       "300",     "15_2 has higher idle; 14 idles at 300rpm"),
        ("stall_count",                 "5",         "2",       "14 triggers stall protection faster (2 counts)"),
        ("stall_recover_enable",        "1 (ON)",    "0 (off)", "14 disables auto stall recovery"),
        ("throttle_lost_flag_time_ms",  "100ms",     "60ms",    "14 declares throttle-lost faster (60ms)"),
        ("over_voltage_protect_enable", "1 (ON)",    "0 (off)", "14 disables OVP (experiment mode)"),
        ("high_temp_protect_enable",    "1 (ON)",    "0 (off)", "14 disables high-temp protection (experiment)"),
        ("status_period_ms",            "100ms",     "200ms",   "14 sends telemetry at half rate"),
    ]
    print(f"  {'Parameter':<40} {'15_2_noload':<15} {'14.json':<12}  Note")
    for name, v15, v14, note in diffs:
        print(f"  {name:<40} {v15:<15} {v14:<12}  {note}")

    # --- 6. VIBE ---
    print("\n[6] Vibration (no propeller)")
    if vibe['t']:
        vx = np.asarray(vibe['vx']); vy = np.asarray(vibe['vy']); vz = np.asarray(vibe['vz'])
        mag = np.sqrt(vx**2+vy**2+vz**2)
        print(f"  VibeX: mean={vx.mean():.3f}  max={vx.max():.3f}")
        print(f"  VibeY: mean={vy.mean():.3f}  max={vy.max():.3f}")
        print(f"  VibeZ: mean={vz.mean():.3f}  max={vz.max():.3f}")
        print(f"  VibeMag: mean={mag.mean():.3f}  max={mag.max():.3f}")
        print("  (ref: in-flight with propellers ~3-23; stationary ~0.02)")

    # --- Plots ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 11))

    ax = axes[0]
    colors = plt.cm.tab20(np.linspace(0, 1, 16))
    for k, i in enumerate(sorted(esc.keys())):
        r = esc[i]
        if not r['t']: continue
        t = rel(r['t']); rpm = np.asarray(r['rpm'])
        if i == 15:
            ax.plot(t, rpm, 'k--', lw=2.5, label='ESC15 (noload-cfg)', zorder=5)
        else:
            ax.plot(t, rpm, color=colors[k % 16], lw=0.9, alpha=0.7, label=f'ESC{i}')
    ax.set_ylabel('RPM'); ax.set_title('All ESC RPM (ESC#15 bold dashed)')
    ax.legend(ncol=4, fontsize=7); ax.grid(True, alpha=0.3)

    ax = axes[1]
    if r15 and r15['t']:
        r = r15; t15 = rel(r['t'])
        rpm = np.asarray(r['rpm']); err = np.asarray(r['err']); curr = np.asarray(r['curr'])
        ax.plot(t15, rpm, 'b-', lw=1.5, label='ESC15 RPM')
        ax2 = ax.twinx()
        ax2.plot(t15, curr, 'g-', lw=0.8, alpha=0.5, label='Curr(A)')
        err_idx = np.where(err != 0)[0]
        if len(err_idx):
            ax.scatter(t15[err_idx], rpm[err_idx], c='red', s=40, zorder=5,
                       label=f'Err!=0 (n={len(err_idx)})')
        ax.set_ylabel('RPM', color='b'); ax2.set_ylabel('Current(A)', color='g')
        ax.set_title('ESC#15 RPM + Current + Err events')
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1+lines2, labels1+labels2, fontsize=8)
        ax.grid(True, alpha=0.3)

    ax = axes[2]
    if cans['t']:
        tc = rel(cans['t'])
        bo = np.asarray(cans['bo']); tto = np.asarray(cans['tto'])
        ax.plot(tc, bo,  'r-',  lw=1.5, label='BusOff cumulative')
        ax.plot(tc, tto, 'orange', lw=1.5, label='TxTimeout cumulative')
        ax.set_ylabel('Cumulative count'); ax.set_xlabel('Time (s)')
        ax.set_title('CAN Bus Health - Post Isolator Install')
        ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    p = os.path.join(OUT_DIR, f'isolator_test.png')
    plt.savefig(p, dpi=120, bbox_inches='tight'); plt.close()
    print(f"\nPlot saved: {p}")

if __name__ == '__main__':
    analyze(sys.argv[1] if len(sys.argv) > 1 else '200702')
