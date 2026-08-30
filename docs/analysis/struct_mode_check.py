"""Decisive test: is the ~6-7 Hz gyro peak a STRUCTURAL MODE (RPM-invariant)
or a motor sub-harmonic (tracks RPM)?
For several flight windows, report motor fundamental (from ESC RPM) and the
dominant low-band peak frequency. If the low peak stays ~constant while the
fundamental moves -> structural/aeroelastic mode.
"""
import os
import numpy as np
from collections import defaultdict
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOGS_DIR = r'd:\ardupilot_TG700\logs'
OUT_DIR  = r'd:\ardupilot_TG700\docs\analysis\out'
FS = 300.0

def find_log(substr, ext='.log'):
    m=[f for f in os.listdir(LOGS_DIR) if substr in f and f.lower().endswith(ext)]
    m.sort(key=lambda f: os.path.getsize(os.path.join(LOGS_DIR,f)), reverse=True)
    return os.path.join(LOGS_DIR,m[0])

def parse(path):
    fmt={}; imu=defaultdict(lambda:{'t':[],'gx':[],'gy':[],'gz':[]}); esc=defaultdict(lambda:{'t':[],'rpm':[]})
    with open(path,'r',errors='replace') as f:
        for line in f:
            if line.startswith('FMT'):
                p=[x.strip() for x in line.split(',')]
                if len(p)>=5: fmt[p[3]]=p[5:]
                continue
            if line.startswith('IMU,'):
                c=fmt.get('IMU');
                if not c: continue
                p=[x.strip() for x in line.split(',')]
                if len(p)-1<len(c): continue
                d=dict(zip(c,p[1:1+len(c)]))
                try:
                    i=int(float(d['I']))
                    if i!=0: continue
                    r=imu[i]; r['t'].append(int(float(d['TimeUS'])))
                    r['gx'].append(float(d['GyrX'])); r['gy'].append(float(d['GyrY'])); r['gz'].append(float(d['GyrZ']))
                except: pass
            elif line.startswith('ESC,'):
                c=fmt.get('ESC')
                if not c: continue
                p=[x.strip() for x in line.split(',')]
                if len(p)-1<len(c): continue
                d=dict(zip(c,p[1:1+len(c)]))
                try:
                    i=int(float(d['Instance'])); esc[i]['t'].append(int(float(d['TimeUS']))); esc[i]['rpm'].append(float(d['RPM']))
                except: pass
    return imu,esc

def peak_in(f,pxx,lo,hi):
    m=(f>=lo)&(f<hi)
    if not m.any(): return None,None
    idx=np.argmax(pxx[m]); fs_=f[m]; ps_=pxx[m]
    return fs_[idx], ps_[idx]

def main():
    path=find_log('20260501')
    print("FILE",os.path.basename(path).encode('ascii','replace').decode())
    imu,esc=parse(path)
    r=imu[0]; t=np.asarray(r['t'],float)*1e-6
    n=int((t[-1]-t[0])*FS); tg=t[0]+np.arange(n)/FS
    gx=np.interp(tg,t,np.asarray(r['gx'],float))
    gy=np.interp(tg,t,np.asarray(r['gy'],float))
    gz=np.interp(tg,t,np.asarray(r['gz'],float))
    t0=t[0]
    windows=[150,201,301,1307,1508,2211,2312,2360]
    wlen=30.0
    plot_data=[]
    print(f"\n{'win(s)':>7} {'Nmot':>5} {'fund(Hz)':>9} {'lowPeak(Hz)':>12} {'midPeak(Hz)':>12}  {'fund/low':>8}")
    for wstart in windows:
        ws=t0+wstart; we=ws+wlen
        mask=(tg>=ws)&(tg<we)
        if mask.sum()<FS*8: continue
        # motor fundamental
        funds=[]
        for i in sorted(esc.keys()):
            et=np.asarray(esc[i]['t'],float)*1e-6; er=np.asarray(esc[i]['rpm'],float)
            mm=(et>=ws)&(et<we)
            if mm.sum()>2 and er[mm].mean()>50: funds.append(er[mm].mean()/60.0)
        if not funds:
            continue
        fund=np.mean(funds)
        g=(gx[mask]+gy[mask]+gz[mask])/3.0
        # use each axis power sum instead
        pall=None
        for arr in (gx,gy,gz):
            x=arr[mask]-arr[mask].mean()
            f,p=signal.welch(x,fs=FS,nperseg=int(FS*4),noverlap=int(FS*2))
            pall = p if pall is None else pall+p
        flow,_=peak_in(f,pall,4,9)
        fmid,_=peak_in(f,pall,10,18)
        ratio = fund/flow if flow else 0
        print(f"{wstart:7.0f} {len(funds):5d} {fund:9.1f} {flow:12.1f} {fmid:12.1f}  {ratio:8.2f}")
        if wstart in (301,1508,2312):
            plot_data.append((wstart, fund, f, pall))

    if plot_data:
        plt.figure(figsize=(11,6))
        colors=['tab:blue','tab:green','tab:red']
        for k,(wstart,fund,f,pall) in enumerate(plot_data):
            plt.semilogy(f,pall/pall.max(),color=colors[k],label=f'{wstart:.0f}s (fund={fund:.1f}Hz)')
            plt.axvline(fund,color=colors[k],ls='--',alpha=0.7,lw=1.2)
        plt.axvspan(6.0,7.6,color='orange',alpha=0.25,label='RPM-invariant peak ~6.5Hz')
        plt.xlim(0,35); plt.xlabel('Hz'); plt.ylabel('normalized PSD')
        plt.title('Structural-mode test: dashed=motor fundamental (moves), orange=fixed ~6.5Hz peak')
        plt.legend(fontsize=9); plt.grid(True,which='both',alpha=0.3)
        p=os.path.join(OUT_DIR,'structural_mode_test.png')
        plt.savefig(p,dpi=120,bbox_inches='tight'); plt.close()
        print("saved",p)

if __name__=='__main__':
    main()
