#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the tilt SVGs to PNG with headless Edge so they can be eyeballed.
Pure ASCII source on purpose (survives a cp936 write path)."""
import os
import subprocess
import sys
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.normpath(os.path.join(HERE, "..", "images"))
TMP = os.path.normpath(os.path.join(HERE, "_preview"))
os.makedirs(TMP, exist_ok=True)

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"

JOBS = [
    ("TG700_\u503e\u89d2\u5b89\u88c5_\u65b0\u65b9\u6848\u4fef\u89c6\u56fe.svg", 900, 720),
    ("TG700_\u503e\u89d2\u5b89\u88c5_\u529b\u77e9\u81c2\u5bf9\u6bd4.svg", 900, 780),
    ("TG700_\u503e\u89d2\u5b89\u88c5_\u5171\u8f74\u7ec4\u5256\u9762.svg", 900, 470),
]

for idx, (name, w, h) in enumerate(JOBS, 1):
    src = os.path.join(IMG, name)
    if not os.path.exists(src):
        sys.exit("missing " + src)
    out = os.path.join(TMP, "preview%d.png" % idx)
    if os.path.exists(out):
        os.remove(out)
    uri = "file:///" + urllib.parse.quote(src.replace("\\", "/"))
    cmd = [EDGE, "--headless", "--disable-gpu", "--no-first-run",
           "--force-device-scale-factor=1",
           "--window-size=%d,%d" % (w, h),
           "--screenshot=" + out, uri]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   timeout=120)
    ok = os.path.exists(out)
    print("%s -> %s (%s)" % (name, out,
                             ("%d bytes" % os.path.getsize(out)) if ok else "FAILED"))
