#!/bin/bash
# Confirm both TG700 frame-type strings made it into the built firmware,
# and that the two files touched for v7.0 compiled without warnings.
cd /mnt/d/ardupilot_TG700 || exit 1
ELF=build/ZeroOneM9/bin/arduplane
LOG=build_zeroonem9.log

echo "=== frame strings in $ELF ==="
for s in TG700 COAX16_AX4 COAX16_COR; do
    printf '  %-12s ' "$s"
    if strings "$ELF" | grep -qx "$s"; then echo "PRESENT"; else echo "*** MISSING ***"; fi
done

echo
echo "=== stale strings that must be gone (old scheme names) ==="
for s in COAX16_CB COAX16 TG700_V5; do
    printf '  %-12s ' "$s"
    if strings "$ELF" | grep -qx "$s"; then echo "still present"; else echo "absent (ok)"; fi
done

echo
echo "=== compiler diagnostics for the edited files ==="
grep -nE 'AP_MotorsMatrix(_TG700)?\.(cpp|h)' "$LOG" \
    | grep -iE 'warning|error' | head -20
echo "  (no lines above = clean)"

echo
echo "=== build identity ==="
grep -E '^githash' "$LOG" | tail -1
ls -la build/ZeroOneM9/bin/arduplane.apj
md5sum build/ZeroOneM9/bin/arduplane.apj
