"""Confirm whether the in-control-band spectral peaks are MOTOR ORDERS
(vibration) rather than pure maneuvering motion, by overlaying per-motor
fundamental frequencies (from ESC RPM telemetry) on the gyro PSD.
Also contrasts a hover-ish window (high VIBE, lower commanded rate)."""
import os, sys
import numpy as np
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

LOGS_DIR = r'd:\ardupilot_TG700\logs'
OUT_DIR  = r'd:\ardupilot_TG700\docs\analysis\out'
FS = 300.0

def find_log(substr, ext='.log'):
    m=[f for f in os.listdir(LOGS_DIR) if substr in f and f.lower().endswith(ext)]
    m.sort(key=lambda f: os.path.getsize(os.path.join(LOGS_DIR,f)), reverse=True)
    return os.path.join(LOGS_DIR,m[0])

def parse(path):
    fmt={}
    imu=defaultdict(lambda:{'t':[],'gx':[],'gy':[],'gz':[]})
    esc=defaultdict(lambda:{'t':[],'rpm':[]})
    vibe={'t':[],'mag':[]}
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
                    i=int(float(d['I'])); r=imu[i]
                    r['t'].append(int(float(d['TimeUS'])))
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
            elif line.startswith('VIBE,'):
                c=fmt.get('VIBE')
                if not c: continue
                p=[x.strip() for x in line.split(',')]
                if len(p)-1<len(c): continue
                d=dict(zip(c,p[1:1+len(c)]))
                try:
                    vibe['t'].append(int(float(d['TimeUS'])))
                    vibe['mag'].append((float(d['VibeX'])**2+float(d['VibeY'])**2+float(d['VibeZ'])**2)**0.5)
                except: pass
    return imu,esc,vibe

def analyze(substr, wstart, wlen=40.0):
    path=find_log(substr)
    print("FILE",os.path.basename(path).encode('ascii','replace').decode())
    imu,esc,vibe=parse(path)
    insts=sorted(imu.keys())
    t0=np.asarray(imu[insts[0]]['t'],float)[0]*1e-6
    # window absolute
    ws=t0+wstart; we=ws+wlen
    # gyro PSD IMU0 (magnitude of 3 axes averaged)
    r=imu[insts[0]]
    t=np.asarray(r['t'],float)*1e-6
    n=int((t[-1]-t[0])*FS); tg=t[0]+np.arange(n)/FS
    mask=(tg>=ws)&(tg<we)
    def psd(key):
        x=np.interp(tg,t,np.asarray(r[key],float))[mask]; x=x-x.mean()
        return signal.welch(x,fs=FS,nperseg=int(FS*4),noverlap=int(FS*2))
    f,pgx=psd('gx'); _,pgy=psd('gy'); _,pgz=psd('gz')
    # motor fundamentals in window
    funds=[]
    for i in sorted(esc.keys()):
        et=np.asarray(esc[i]['t'],float)*1e-6; er=np.asarray(esc[i]['rpm'],float)
        m=(et>=ws)&(et<we)
        if m.sum()>3:
            rpm=er[m].mean()
            if rpm>50: funds.append(rpm/60.0)
    funds=np.array(funds)
    print(f"window [{wstart:.0f},{wstart+wlen:.0f}]s ; {len(funds)} active motors ; "
          f"fundamental range {funds.min():.1f}-{funds.max():.1f} Hz ; mean {funds.mean():.1f} Hz")
    print(f"2nd harmonic range {2*funds.min():.1f}-{2*funds.max():.1f} Hz")

    plt.figure(figsize=(11,6))
    plt.semilogy(f,pgx,label='GyrX'); plt.semilogy(f,pgy,label='GyrY',alpha=0.7); plt.semilogy(f,pgz,label='GyrZ',alpha=0.7)
    plt.axvspan(1,15,color='orange',alpha=0.15,label='control band 1-15Hz')
    for k,ff in enumerate(funds):
        plt.axvline(ff,color='red',ls='--',alpha=0.5,lw=0.8,label='motor fundamental' if k==0 else None)
        plt.axvline(2*ff,color='purple',ls=':',alpha=0.4,lw=0.8,label='2nd harmonic' if k==0 else None)
    plt.xlim(0,80); plt.xlabel('Hz'); plt.ylabel('PSD')
    plt.title(f'IMU0 gyro PSD + motor orders, window {wstart:.0f}s')
    plt.legend(fontsize=8); plt.grid(True,which='both',alpha=0.3)
    p=os.path.join(OUT_DIR,f'order_overlay_{int(wstart)}s.png')
    plt.savefig(p,dpi=120,bbox_inches='tight'); plt.close()
    print("saved",p)

if __name__=='__main__':
    substr=sys.argv[1] if len(sys.argv)>1 else '20260501'
    wstart=float(sys.argv[2]) if len(sys.argv)>2 else 2312.0
    analyze(substr,wstart)
