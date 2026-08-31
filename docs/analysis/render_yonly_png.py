#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the +-Y-only scheme SVGs to PNG with headless Edge."""
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
    ("TG700_yonly_top.svg", 1020, 640),
    ("TG700_yonly_section.svg", 900, 430),
    ("TG700_yonly_arm.svg", 940, 470),
]

for idx, (name, w, h) in enumerate(JOBS, 1):
    src = os.path.join(IMG, name)
    if not os.path.exists(src):
        sys.exit("missing " + src)
    out = os.path.join(TMP, "yonly%d.png" % idx)
    if os.path.exists(out):
        os.remove(out)
    uri = "file:///" + urllib.parse.quote(src.replace("\\", "/"))
    subprocess.run([EDGE, "--headless", "--disable-gpu", "--no-first-run",
                    "--force-device-scale-factor=1",
                    "--window-size=%d,%d" % (w, h),
                    "--screenshot=" + out, uri],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
    ok = os.path.exists(out)
    print("%s -> %s (%s)" % (name, out,
                             ("%d bytes" % os.path.getsize(out)) if ok else "FAILED"))
