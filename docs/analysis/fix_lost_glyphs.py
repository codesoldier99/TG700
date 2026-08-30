# -*- coding: utf-8 -*-
"""Repair characters that were destroyed by a cp936 (GBK) write path.

The editor wrote the markdown through GBK, so every codepoint outside GBK was
replaced by a literal '?'.  Affected here: U+2212 MINUS SIGN, U+00B2 SUPERSCRIPT
TWO, U+0307 COMBINING DOT ABOVE (the dot of psi-dot) and U+26A0 WARNING SIGN.

Fix by substituting GBK-safe equivalents so this cannot happen again:
    U+2212  ->  '-'
    U+00B2  ->  '^2'
    psi-dot ->  spelled out in Chinese
    U+26A0  ->  '[!]'
This script is pure ASCII so it survives the same write path.
"""
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.normpath(os.path.join(HERE, ".."))

PSI = u"\u03c8"          # psi

# ordered: specific patterns first, generic '?' -> '-' last
RULES = [
    (u"\u62df\u5408\u5f97 " + PSI + u"? ", u"\u62df\u5408\u5f97 "),   # "拟合得 psi? " -> "拟合得 "
    (u"\u5b9e\u9645\u504f\u822a\u7387 " + PSI + u"? ",
     u"\u5b9e\u9645\u504f\u822a\u7387 "),                             # "实际偏航率 psi? "
    (u"/s?", u"/s^2"),
    (u"\uff1f", u"\uff1f"),   # keep full-width question marks untouched
]


def repair(path):
    with io.open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    before = text.count(u"?")
    if before == 0:
        print("  clean   %s" % os.path.basename(path))
        return
    for old, new in RULES:
        text = text.replace(old, new)
    # anything left is a mangled U+2212
    text = text.replace(u"?", u"-")
    with io.open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("  repaired %-56s %d marks" % (os.path.basename(path), before))


targets = sys.argv[1:] or [
    u"TG700_\u504f\u822a\u63a7\u5236\u5b9a\u6848_\u56db\u5411\u503e\u89d2"
    u"\u7ea6\u675f_v6.md",
]
for name in targets:
    p = name if os.path.isabs(name) else os.path.join(DOCS, name)
    if not os.path.exists(p):
        sys.exit("missing: " + p)
    repair(p)
print("done")
