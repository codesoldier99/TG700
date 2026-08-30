#!/bin/bash
# ---------------------------------------------------------------------------
# TG700 / ZeroOne M9 firmware build, run from WSL:
#     wsl -d Ubuntu-22.04 -- bash -c "tr -d '\r' < /mnt/d/ardupilot_TG700/build_tg700.sh > /tmp/b.sh; bash /tmp/b.sh"
#
# The toolchain lives under $HOME (no sudo was available on this machine):
#   $HOME/opt/gcc-arm-none-eabi-10-2020-q4-major   ARM cross compiler 10.2.1
#   $HOME/opt/local/usr/bin/make                   GNU make 4.3, unpacked from a .deb
#   ~/.local/lib/python3.10/site-packages          empy 3.3.4 (pip --user)
#
# The repo is checked out with core.autocrlf=true, so scripts that are executed
# through a shebang and files consumed by make must be LF or they fail with
# "/usr/bin/env: 'python3\r'" or "missing separator".  strip_crlf() below keeps
# those files LF; it is idempotent and only rewrites files that actually
# contain CRLF.
# ---------------------------------------------------------------------------
set -o pipefail

REPO=/mnt/d/ardupilot_TG700
BOARD=ZeroOneM9
JOBS=${JOBS:-8}
LOG="$REPO/build_zeroonem9.log"

export PATH="$HOME/opt/local/usr/bin:$HOME/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$HOME/.local/bin:$PATH"
cd "$REPO" || exit 1

strip_crlf() {
    python3 - "$@" <<'PYEOF'
import os, sys
n = 0
for root in sys.argv[1:]:
    walk = [(os.path.dirname(root), None, [os.path.basename(root)])] if os.path.isfile(root) \
           else os.walk(root)
    for dirpath, _, files in walk:
        for fn in files:
            p = os.path.join(dirpath, fn)
            ext = os.path.splitext(fn)[1]
            if ext not in ('.py', '.mk', '.S', '.s', '.sh', '') and fn != 'Makefile':
                continue
            try:
                b = open(p, 'rb').read()
            except OSError:
                continue
            if b'\r\n' not in b:
                continue
            # only touch shebang scripts and things make/as will read
            if not (b.startswith(b'#!') or ext in ('.mk', '.S', '.s') or fn.startswith('Makefile')):
                continue
            open(p, 'wb').write(b.replace(b'\r\n', b'\n'))
            n += 1
print("  normalised %d file(s) to LF" % n)
PYEOF
}

echo "=== 1/3  line endings ==="
strip_crlf Tools libraries/AP_HAL_ChibiOS/hwdef modules/CrashDebug

echo "=== 2/3  configure ($BOARD) ==="
python3 ./waf configure --board "$BOARD" 2>&1 | tail -5 || exit 1

echo "=== 3/3  build plane (-j$JOBS), full log -> $LOG ==="
date
python3 ./waf plane -j"$JOBS" > "$LOG" 2>&1
RC=$?
date
tail -12 "$LOG"

if [ $RC -ne 0 ]; then
    echo "BUILD FAILED (rc=$RC) - see $LOG"
    exit $RC
fi

echo
echo "=== artifacts ==="
ls -la build/$BOARD/bin/
md5sum build/$BOARD/bin/arduplane.apj build/$BOARD/bin/arduplane.bin
echo
echo "flash build/$BOARD/bin/arduplane.apj with Mission Planner"
