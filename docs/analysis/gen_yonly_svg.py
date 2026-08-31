#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagrams for the +-Y-only tilt scheme (COAX16_AY, v8.0).
Pure ASCII source: CJK written as backslash-u escapes so the file survives a
legacy-code-page write path.

Outputs (UTF-8 SVG) under docs/images/:
  TG700_yonly_top.svg       top view, tilt direction per motor
  TG700_yonly_section.svg   coaxial pod section, front row vs rear row
  TG700_yonly_arm.svg       why the moment arm drops to |X| for every motor
"""
import math
import os

OUT = "docs/images"
X_ARM, Y_OUT, Y_IN = 1.665, 3.4, 1.6

# pod -> (name, x, y, ring)
PODS = [
    ("P1", +X_ARM, +Y_OUT, "out"), ("P2", +X_ARM, +Y_IN, "in"),
    ("P3", +X_ARM, -Y_IN, "in"),   ("P4", +X_ARM, -Y_OUT, "out"),
    ("P5", -X_ARM, +Y_OUT, "out"), ("P6", -X_ARM, +Y_IN, "in"),
    ("P7", -X_ARM, -Y_IN, "in"),   ("P8", -X_ARM, -Y_OUT, "out"),
]

CJK = "'Microsoft YaHei','Noto Sans CJK SC','WenQuanYi Zen Hei',sans-serif"
C_UP, C_LO = "#1f6fb4", "#c1452b"          # upper / lower motor colours
C_ARROW, C_INK, C_MUTE = "#12862f", "#1c1c1c", "#6b6b6b"


def hdr(w, h, title, sub):
    g = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d"'
         ' viewBox="0 0 %d %d" font-family="%s">' % (w, h, w, h, CJK),
         '<defs>',
         '<marker id="au" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7"'
         ' markerHeight="7" orient="auto-start-reverse">'
         '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_UP,
         '<marker id="al" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7"'
         ' markerHeight="7" orient="auto-start-reverse">'
         '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_LO,
         '<marker id="ak" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6"'
         ' markerHeight="6" orient="auto-start-reverse">'
         '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_MUTE,
         '<marker id="ag" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7"'
         ' markerHeight="7" orient="auto-start-reverse">'
         '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_ARROW,
         '</defs>',
         '<rect width="100%" height="100%" fill="#ffffff"/>',
         '<text x="%d" y="32" text-anchor="middle" font-size="20"'
         ' font-weight="bold" fill="%s">%s</text>' % (w // 2, C_INK, title)]
    if sub:
        g.append('<text x="%d" y="55" text-anchor="middle" font-size="14"'
                 ' fill="%s">%s</text>' % (w // 2, C_MUTE, sub))
    return g


def txt(x, y, s, size=13, fill=C_INK, anchor="middle", weight="normal"):
    return ('<text x="%.1f" y="%.1f" text-anchor="%s" font-size="%d" fill="%s"'
            ' font-weight="%s">%s</text>' % (x, y, anchor, size, fill, weight, s))


def arrow(x1, y1, x2, y2, color, w=3.0, mk="ag"):
    return ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s"'
            ' stroke-width="%.1f" marker-end="url(#%s)"/>'
            % (x1, y1, x2, y2, color, w, mk))


# ===========================================================================
# figure 1 - top view: every motor tilts along +-Y
# ===========================================================================
def fig_top():
    W, H = 1020, 640
    s, R = 84.0, 34.0
    cx, cy = W / 2, 330.0
    g = hdr(W, H,
            "\u56fe1  TG700 \u00b1Y \u5355\u5411\u503e\u89d2\u65b9\u6848 (COAX16_AY) "
            "\u2014 \u4fef\u89c6\u56fe, \u673a\u5934\u671d\u4e0a",
            "\u524d\u6392 4 \u7ec4\uff1a\u4e0a\u5c42\u671d\u5de6 / \u4e0b\u5c42\u671d\u53f3"
            "\u3002\u3000\u540e\u6392 4 \u7ec4\uff1a\u4e0a\u5c42\u671d\u53f3 / \u4e0b\u5c42\u671d\u5de6"
            "\u3002\u3000\u5185\u5916\u73af\u88c5\u6cd5\u76f8\u540c")

    for xb in (+X_ARM, -X_ARM):
        yy = cy - xb * s
        g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e6e6e6"'
                 ' stroke-width="11"/>'
                 % (cx - Y_OUT * s - 30, yy, cx + Y_OUT * s + 30, yy))
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#d4d4d4"'
             ' stroke-width="2" stroke-dasharray="7 5"/>'
             % (cx, cy - X_ARM * s - 40, cx, cy + X_ARM * s + 40))

    g.append(arrow(cx, cy - 24, cx, cy - X_ARM * s - 36, C_MUTE, 1.8, "ak"))
    g.append(txt(cx + 8, cy - X_ARM * s - 40, "+X \u673a\u5934", 12, C_MUTE, "start"))
    g.append(arrow(cx + 24, cy, cx + 112, cy, C_MUTE, 1.8, "ak"))
    g.append(txt(cx + 118, cy + 4, "+Y \u53f3", 12, C_MUTE, "start"))
    g.append('<circle cx="%.1f" cy="%.1f" r="4.5" fill="%s"/>' % (cx, cy, C_MUTE))
    g.append(txt(cx - 10, cy + 20, "CG", 12, C_MUTE, "end"))

    for name, xb, yb, ring in PODS:
        px, py = cx + yb * s, cy - xb * s
        front = xb > 0
        # upper: front->left, rear->right ; lower: opposite
        up_dx = -1.0 if front else +1.0
        lo_dx = -up_dx
        g.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#f2f2f2"'
                 ' stroke="#b8b8b8" stroke-width="2"/>' % (px, py, R))
        # two stacked arrows: upper above the centre line, lower below
        L = 46.0
        g.append(arrow(px + up_dx * 6, py - 11, px + up_dx * (6 + L), py - 11,
                       C_UP, 3.2, "au"))
        g.append(arrow(px + lo_dx * 6, py + 13, px + lo_dx * (6 + L), py + 13,
                       C_LO, 3.2, "al"))
        lbl = "%s %s" % (name, "\u5916" if ring == "out" else "\u5185")
        g.append(txt(px, py - (R + 26) if front else py + (R + 20), lbl,
                     15, C_INK, "middle", "bold"))
        g.append(txt(px, py - (R + 8) if front else py + (R + 38),
                     "\u4e0a CW / \u4e0b CCW", 11, C_MUTE))

    ly = H - 64
    g.append('<rect x="24" y="%.1f" width="%d" height="58" rx="8" fill="#fafafa"'
             ' stroke="#dddddd"/>' % (ly - 22, W - 48))
    g.append(txt(40, ly - 3, "\u56fe\u4f8b:", 13, C_INK, "start", "bold"))
    g.append(arrow(86, ly - 8, 132, ly - 8, C_UP, 3.2, "au"))
    g.append(txt(140, ly - 3, "\u4e0a\u5c42 (CW) \u503e\u89d2\u65b9\u5411",
                 13, C_UP, "start"))
    g.append(arrow(300, ly - 8, 346, ly - 8, C_LO, 3.2, "al"))
    g.append(txt(354, ly - 3, "\u4e0b\u5c42 (CCW) \u503e\u89d2\u65b9\u5411",
                 13, C_LO, "start"))
    g.append(txt(560, ly - 3,
                 "\u6868\u4e0e ESC \u5168\u90e8\u4e0d\u52a8, \u53ea\u91cd\u6392 16 "
                 "\u4e2a\u503e\u89d2\u5ea7", 13, C_INK, "start"))
    g.append(txt(40, ly + 21,
                 "\u53e3\u8bc0\uff1a\u540c\u4e00\u6392\u7684 4 \u7ec4\u88c5\u6cd5\u5b8c\u5168"
                 "\u4e00\u6837\uff1b\u524d\u540e\u6392\u76f8\u53cd\uff1b\u540c\u7ec4\u4e0a\u4e0b"
                 "\u76f8\u53cd\u3002\u4e0d\u533a\u5206\u5185\u5916\u73af\u3002",
                 13, C_INK, "start"))
    g.append("</svg>")
    return "\n".join(g)


# ===========================================================================
# figure 2 - pod section: front row vs rear row
# ===========================================================================
def fig_section():
    W, H = 900, 430
    g = hdr(W, H,
            "\u56fe2  \u5171\u8f74\u7ec4\u5256\u9762 \u2014 \u4ece\u673a\u5c3e\u5411"
            "\u673a\u5934\u770b (\u5de6\u53f3\u5c31\u662f\u673a\u4f53\u7684\u5de6\u53f3)",
            "\u503e\u89d2\u5168\u90e8\u5728\u5de6\u53f3\u65b9\u5411\uff0c\u6ca1\u6709\u4efb"
            "\u4f55\u524d\u540e\u5206\u91cf")
    for k, (title, up_left) in enumerate((
            ("\u524d\u6392 P1-P4\uff1a\u4e0a\u5c42\u671d\u5de6\uff0c\u4e0b\u5c42\u671d\u53f3", True),
            ("\u540e\u6392 P5-P8\uff1a\u4e0a\u5c42\u671d\u53f3\uff0c\u4e0b\u5c42\u671d\u5de6", False))):
        ox = 40 + k * (W / 2 - 10)
        cw_ = W / 2 - 70
        g.append('<rect x="%.1f" y="82" width="%.1f" height="286" rx="10"'
                 ' fill="#fbfbfb" stroke="#dcdcdc"/>' % (ox, cw_))
        g.append(txt(ox + cw_ / 2, 108, title, 14, C_INK, "middle", "bold"))
        mx = ox + cw_ / 2
        g.append('<line x1="%.1f" y1="140" x2="%.1f" y2="330" stroke="#9a9a9a"'
                 ' stroke-width="7"/>' % (mx, mx))
        g.append(txt(mx, 350, "\u7535\u673a\u5ea7 / \u81c2", 12, C_MUTE))
        for j, (lab, yy, rot, col, mk) in enumerate((
                ("\u4e0a\u5c42", 174, "CW", C_UP, "au"),
                ("\u4e0b\u5c42", 292, "CCW", C_LO, "al"))):
            left = up_left if j == 0 else (not up_left)
            sgn = -1.0 if left else 1.0
            ang = math.radians(16.0) * (-sgn)      # exaggerated for legibility
            half = 72.0
            dx, dy = half * math.cos(ang), -half * math.sin(ang)
            g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s"'
                     ' stroke-width="6" stroke-linecap="round"/>'
                     % (mx - dx, yy - dy, mx + dx, yy + dy, col))
            g.append('<circle cx="%.1f" cy="%.1f" r="7" fill="%s"/>' % (mx, yy, col))
            g.append(txt(mx - half - 14, yy + 5, lab, 13, C_INK, "end", "bold"))
            g.append(txt(mx + half + 14, yy + 5, rot, 13, col, "start", "bold"))
            # horizontal force component
            g.append(arrow(mx, yy - 26, mx + sgn * 60, yy - 26, col, 2.8, mk))
            g.append(txt(mx + sgn * 74, yy - 30,
                         "\u5de6" if left else "\u53f3", 13, col,
                         "end" if left else "start", "bold"))
        g.append(txt(ox + cw_ / 2, 392,
                     "\u4e24\u53f0\u6c34\u5e73\u529b\u53cd\u5411 \u2192 "
                     "\u63a8\u529b\u5dee\u52a8\u65f6\u529b\u77e9\u76f8\u52a0\uff0c"
                     "\u7ec4\u5185\u63a8\u529b\u548c\u4e0d\u53d8", 12, C_ARROW))
    g.append("</svg>")
    return "\n".join(g)


# ===========================================================================
# figure 3 - moment arm comparison
# ===========================================================================
def fig_arm():
    W, H = 940, 470
    s = 74.0
    cx, cy = 300.0, 250.0
    g = hdr(W, H,
            "\u56fe3  \u4e3a\u4ec0\u4e48\u5916\u4fa7\u7ec4\u4e5f\u53ea\u5269 1.665 m "
            "\u529b\u77e9\u81c2",
            "\u6c34\u5e73\u529b\u6cbf \u00b1Y \u65f6\uff0cMz = x\u00b7Fy\uff0c\u529b\u77e9\u81c2"
            "\u53ea\u770b |X|\uff0c\u4e0e\u5728\u5185\u4fa7\u8fd8\u662f\u5916\u4fa7\u65e0\u5173")

    # P1 (front right outer) as the worked example
    cx, cy = 190.0, 330.0
    sc = 54.0
    px, py = cx + Y_OUT * sc, cy - X_ARM * sc
    # CG cross-hairs
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#dcdcdc"'
             ' stroke-width="1.8" stroke-dasharray="6 4"/>'
             % (cx, py - 46, cx, cy + 40))
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#dcdcdc"'
             ' stroke-width="1.8" stroke-dasharray="6 4"/>'
             % (cx - 34, cy, px + 130, cy))
    g.append('<circle cx="%.1f" cy="%.1f" r="5" fill="%s"/>' % (cx, cy, C_MUTE))
    g.append(txt(cx - 10, cy + 20, "CG", 12, C_MUTE, "end"))
    # arm / wing bar through the pod row
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e6e6e6"'
             ' stroke-width="9"/>' % (cx - 24, py, px + 34, py))
    g.append('<circle cx="%.1f" cy="%.1f" r="24" fill="#f2f2f2" stroke="#b8b8b8"'
             ' stroke-width="2"/>' % (px, py))
    g.append(txt(px, py - 36, "P1 \u524d\u53f3\u5916", 14, C_INK, "middle", "bold"))

    # |Y| - the arm that is no longer reachable
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#c4c4c4"'
             ' stroke-width="2.2" stroke-dasharray="5 4"/>' % (cx, py, px, py))
    g.append(txt((cx + px) / 2, py - 14,
                 "|Y| = 3.400 m  (\u6cbf \u00b1X \u503e\u89d2\u624d\u7528\u5f97\u4e0a)",
                 13, "#9a9a9a"))

    # |X| - the arm that actually applies
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s"'
             ' stroke-width="2.6" stroke-dasharray="5 4"/>'
             % (px, py, px, cy, C_ARROW))
    g.append(txt(px - 12, (py + cy) / 2 - 4, "|X| = 1.665 m",
                 14, C_ARROW, "end", "bold"))
    g.append(txt(px - 12, (py + cy) / 2 + 16, "\u6709\u6548\u529b\u77e9\u81c2",
                 12, C_ARROW, "end"))

    # the +-Y force and its line of action
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#bfe0c6"'
             ' stroke-width="1.6" stroke-dasharray="4 4"/>'
             % (cx - 24, py, px + 132, py))
    g.append(arrow(px + 24, py, px + 104, py, C_ARROW, 3.4))
    g.append(txt(px + 112, py + 5, "Fy", 14, C_ARROW, "start", "bold"))
    g.append(txt(cx - 24, cy + 62,
                 "Mz = x\u00b7Fy \u2014 \u529b\u7684\u4f5c\u7528\u7ebf\u5230 CG "
                 "\u7684\u5782\u76f4\u8ddd\u79bb\u5c31\u662f |X|\uff0c", 13, C_INK, "start"))
    g.append(txt(cx - 24, cy + 84,
                 "\u6240\u4ee5\u5916\u4fa7\u7ec4\u7684 3.4 m \u5b8c\u5168\u7528\u4e0d\u4e0a"
                 "\uff0c16 \u53f0\u7535\u673a\u529b\u77e9\u81c2\u5168\u90e8\u76f8\u540c\u3002",
                 13, C_INK, "start"))

    # numbers panel
    bx = 590
    g.append('<rect x="%d" y="96" width="%d" height="300" rx="10" fill="#fbfbfb"'
             ' stroke="#dcdcdc"/>' % (bx, W - bx - 30))
    rows = [
        ("\u65b9\u6848", "\u504f\u822a\u6743\u9650", ""),
        ("\u7406\u60f3\u5207\u5411 5\u00b0", "1.8523", "123%"),
        ("\u00b1X/\u00b1Y \u56db\u5411 5\u00b0", "1.5085", "100%"),
        ("**\u00b1Y \u5355\u5411 5\u00b0**", "1.2057", "80%"),
        ("\u00b1Y \u5355\u5411 6.31\u00b0", "1.5090", "100%"),
        ("\u00b1Y \u5355\u5411 8\u00b0", "1.8986", "126%"),
        ("\u00b1Y \u5355\u5411 10\u00b0", "2.3578", "156%"),
    ]
    for i, (a, b, c) in enumerate(rows):
        yy = 128 + i * 38
        bold = "bold" if i == 0 or "**" in a else "normal"
        a = a.replace("**", "")
        col = C_ARROW if "**" in rows[i][0] else C_INK
        if i == 0:
            g.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#c8c8c8"'
                     ' stroke-width="1.4"/>' % (bx + 10, yy + 12, W - 40, yy + 12))
        g.append(txt(bx + 20, yy, a, 13, col, "start", bold))
        g.append(txt(bx + 190, yy, b, 13, col, "end", bold))
        g.append(txt(W - 44, yy, c, 13, col, "end", bold))
    g.append(txt(bx + 20, 128 + 7 * 38 + 6,
                 "6.31\u00b0 \u5c31\u80fd\u8ffd\u5e73\u56db\u5411\u65b9\u6848",
                 12, C_ARROW, "start", "bold"))
    g.append(txt(bx + 20, 128 + 7 * 38 + 26,
                 "\u5347\u529b\u4ee3\u4ef7\u4ec5 0.61% (700kg \u6298 4.2kg)",
                 12, C_MUTE, "start"))
    g.append("</svg>")
    return "\n".join(g)


os.makedirs(OUT, exist_ok=True)
for fn, body in (("TG700_yonly_top.svg", fig_top()),
                 ("TG700_yonly_section.svg", fig_section()),
                 ("TG700_yonly_arm.svg", fig_arm())):
    p = os.path.join(OUT, fn)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        f.write(body)
    print("wrote %s  (%d bytes)" % (p, len(body.encode("utf-8"))))
