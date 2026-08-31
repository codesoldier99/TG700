#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the scheme-B (v5.0, co-rotating checkerboard) installation diagrams.

Pure ASCII source: every CJK glyph is written as a backslash-u escape so the file
survives being rewritten by editors that default to a legacy code page.

Outputs (UTF-8 SVG) under docs/images/:
  TG700_v5_install_top.svg        top view: rotation + tilt per pod
  TG700_v5_install_section.svg    section: both rotors tilt the same way
  TG700_v5_install_checklist.svg  which 8 motors must be reversed
"""
import math
import os

OUT = "docs/images"

# ---- body geometry (m) ----
X_ARM, Y_OUT, Y_IN = 1.665, 3.4, 1.6

# pod -> (name, x, y, ring, rotation, tilt)
PODS = [
    ("P1", +X_ARM, +Y_OUT, "out", "CW",  "+X"),
    ("P2", +X_ARM, +Y_IN,  "in",  "CCW", "+Y"),
    ("P3", +X_ARM, -Y_IN,  "in",  "CW",  "-Y"),
    ("P4", +X_ARM, -Y_OUT, "out", "CCW", "+X"),
    ("P5", -X_ARM, +Y_OUT, "out", "CCW", "-X"),
    ("P6", -X_ARM, +Y_IN,  "in",  "CW",  "+Y"),
    ("P7", -X_ARM, -Y_IN,  "in",  "CCW", "-Y"),
    ("P8", -X_ARM, -Y_OUT, "out", "CW",  "-X"),
]

# Going from scheme A (upper CW / lower CCW) to scheme B: a pod that ends up CW
# keeps its upper motor and flips the lower one; a CCW pod flips the upper one.
REVERSE = {p[0]: ("\u4e0b" if p[4] == "CW" else "\u4e0a") for p in PODS}

TILT_CN = {"+X": "\u673a\u5934", "-X": "\u673a\u5c3e",
           "+Y": "\u53f3", "-Y": "\u5de6"}
TILT_GLYPH = {"+X": "\u2191", "-X": "\u2193", "+Y": "\u2192", "-Y": "\u2190"}

CJK = "'Microsoft YaHei','Noto Sans CJK SC','WenQuanYi Zen Hei',sans-serif"
C_CW, C_CCW = "#1f6fb4", "#c1452b"
C_ARROW, C_INK, C_MUTE = "#12862f", "#1c1c1c", "#6b6b6b"


def hdr(w, h, title, sub):
    g = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
        'viewBox="0 0 %d %d" font-family="%s">' % (w, h, w, h, CJK),
        '<defs>',
        '<marker id="tip" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7"'
        ' markerHeight="7" orient="auto-start-reverse">'
        '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_ARROW,
        '<marker id="tipk" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6"'
        ' markerHeight="6" orient="auto-start-reverse">'
        '<path d="M0,0 L10,5 L0,10 z" fill="%s"/></marker>' % C_INK,
        '</defs>',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="%d" y="32" text-anchor="middle" font-size="20" font-weight="bold"'
        ' fill="%s">%s</text>' % (w // 2, C_INK, title),
    ]
    if sub:
        g.append('<text x="%d" y="55" text-anchor="middle" font-size="14"'
                 ' fill="%s">%s</text>' % (w // 2, C_MUTE, sub))
    return g


def txt(x, y, s, size=13, fill=C_INK, anchor="middle", weight="normal"):
    return ('<text x="%.1f" y="%.1f" text-anchor="%s" font-size="%d" fill="%s"'
            ' font-weight="%s">%s</text>' % (x, y, anchor, size, fill, weight, s))


def arrow(x1, y1, x2, y2, color=C_ARROW, w=3.0, mk="tip"):
    return ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s"'
            ' stroke-width="%.1f" marker-end="url(#%s)"/>'
            % (x1, y1, x2, y2, color, w, mk))


def spin(cx, cy, r, cw, color):
    """Circular arrow showing rotation sense as seen from above."""
    a0, a1 = (-60, 200) if cw else (240, -20)
    x0, y0 = cx + r * math.cos(math.radians(a0)), cy - r * math.sin(math.radians(a0))
    x1, y1 = cx + r * math.cos(math.radians(a1)), cy - r * math.sin(math.radians(a1))
    return ('<path d="M%.1f,%.1f A%.1f,%.1f 0 1 %d %.1f,%.1f" fill="none"'
            ' stroke="%s" stroke-width="2.3" marker-end="url(#tipk)"'
            ' opacity="0.9"/>'
            % (x0, y0, r, r, 0 if cw else 1, x1, y1, color))


# ===========================================================================
# figure 1 - top view
# ===========================================================================
def fig_top():
    W, H = 1020, 660
    s, R = 84.0, 36.0
    cx, cy = W / 2, 336.0
    g = hdr(W, H,
            "\u56fe1  TG700 \u65b9\u6848B (v5.0 \u540c\u8f6c\u68cb\u76d8) "
            "\u5b89\u88c5\u4fef\u89c6\u56fe \u2014 \u4ece\u4e0a\u65b9\u770b, \u673a\u5934\u671d\u4e0a",
            "\u6bcf\u7ec4\u4e0a\u4e0b\u4e24\u53f0\u8f6c\u5411\u76f8\u540c\u3001\u503e\u89d2\u540c\u5411"
            "\uff1b\u503e\u89d2\u5168\u90e8\u80cc\u79bb\u91cd\u5fc3\u5411\u5916")

    # wing reference bars + centreline
    for xb in (+X_ARM, -X_ARM):
        yy = cy - xb * s
        g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e6e6e6"'
                 ' stroke-width="11"/>'
                 % (cx - Y_OUT * s - 24, yy, cx + Y_OUT * s + 24, yy))
    g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#d4d4d4"'
             ' stroke-width="2" stroke-dasharray="7 5"/>'
             % (cx, cy - X_ARM * s - 34, cx, cy + X_ARM * s + 34))

    # body axes
    g.append(arrow(cx, cy - 26, cx, cy - X_ARM * s - 30, C_MUTE, 1.8, "tipk"))
    g.append(txt(cx + 8, cy - X_ARM * s - 34, "+X \u673a\u5934", 12, C_MUTE, "start"))
    g.append(arrow(cx + 26, cy, cx + 118, cy, C_MUTE, 1.8, "tipk"))
    g.append(txt(cx + 124, cy + 4, "+Y \u53f3", 12, C_MUTE, "start"))
    g.append('<circle cx="%.1f" cy="%.1f" r="4.5" fill="%s"/>' % (cx, cy, C_MUTE))
    g.append(txt(cx - 10, cy + 20, "CG", 12, C_MUTE, "end"))

    for name, xb, yb, ring, rot, tilt in PODS:
        px, py = cx + yb * s, cy - xb * s
        cw = (rot == "CW")
        col = C_CW if cw else C_CCW
        g.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" fill-opacity="0.10"'
                 ' stroke="%s" stroke-width="2.4"/>' % (px, py, R, col, col))
        g.append(spin(px, py, R * 0.66, cw, col))

        dx = {"+Y": 1.0, "-Y": -1.0}.get(tilt, 0.0)
        dy = {"+X": -1.0, "-X": 1.0}.get(tilt, 0.0)
        # outer pods get a longer arrow (vertical, plenty of room);
        # inner pods a shorter one so it stops clear of the outer pod
        L = 62.0 if ring == "out" else 40.0
        g.append(arrow(px + dx * (R + 2), py + dy * (R + 2),
                       px + dx * (R + 2 + L), py + dy * (R + 2 + L)))

        l1 = "%s %s" % (name, "\u5916" if ring == "out" else "\u5185")
        l2 = "\u4e0a\u4e0b\u5747 " + rot
        l3 = "\u503e\u89d2 %s%s" % (TILT_GLYPH[tilt], TILT_CN[tilt])
        if ring == "out":
            # label block outboard of the pod (arrow is vertical)
            side = 1 if yb > 0 else -1
            ax = px + side * (R + 12)
            anch = "start" if side > 0 else "end"
            g.append(txt(ax, py - 12, l1, 15, C_INK, anch, "bold"))
            g.append(txt(ax, py + 7, l2, 12, col, anch))
            g.append(txt(ax, py + 25, l3, 12, C_ARROW, anch))
        else:
            # label block above (front row) / below (rear row); arrow is horizontal
            up = xb > 0
            by = py - (R + 40) if up else py + (R + 22)
            g.append(txt(px, by, l1, 15, C_INK, "middle", "bold"))
            g.append(txt(px, by + 19, l2, 12, col))
            g.append(txt(px, by + 37, l3, 12, C_ARROW))

    # legend
    ly = H - 62
    g.append('<rect x="24" y="%.1f" width="%d" height="56" rx="8" fill="#fafafa"'
             ' stroke="#dddddd"/>' % (ly - 22, W - 48))
    g.append(txt(40, ly - 2, "\u56fe\u4f8b:", 13, C_INK, "start", "bold"))
    g.append('<circle cx="96" cy="%.1f" r="10" fill="%s" fill-opacity="0.12"'
             ' stroke="%s" stroke-width="2"/>' % (ly - 7, C_CW, C_CW))
    g.append(txt(112, ly - 2, "CW \u7ec4  P1,P3,P6,P8", 13, C_CW, "start"))
    g.append('<circle cx="290" cy="%.1f" r="10" fill="%s" fill-opacity="0.12"'
             ' stroke="%s" stroke-width="2"/>' % (ly - 7, C_CCW, C_CCW))
    g.append(txt(306, ly - 2, "CCW \u7ec4  P2,P4,P5,P7", 13, C_CCW, "start"))
    g.append(arrow(494, ly - 7, 538, ly - 7))
    g.append(txt(548, ly - 2, "\u503e\u89d2\u65b9\u5411 (\u4e0a\u4e0b\u4e24\u53f0\u540c\u5411)",
                 13, C_ARROW, "start"))
    g.append(txt(40, ly + 22,
                 "\u53e3\u8bc0\uff1a\u5916\u4fa7 4 \u7ec4\u6cbf X \u8f74\u80cc\u79bb\u91cd\u5fc3"
                 "\uff08\u524d\u6392\u2192\u673a\u5934\uff0c\u540e\u6392\u2192\u673a\u5c3e\uff09\uff1b"
                 "\u5185\u4fa7 4 \u7ec4\u6cbf Y \u8f74\u80cc\u79bb\u4e2d\u7ebf"
                 "\uff08\u53f3\u4fa7\u2192\u53f3\uff0c\u5de6\u4fa7\u2192\u5de6\uff09\u3002",
                 13, C_INK, "start"))
    g.append("</svg>")
    return "\n".join(g)


# ===========================================================================
# figure 2 - coaxial pod section, A vs B
# ===========================================================================
def fig_section():
    W, H = 900, 440
    g = hdr(W, H,
            "\u56fe2  \u5171\u8f74\u7ec4\u5256\u9762 \u2014 "
            "\u65b9\u6848A(\u5bf9\u8f6c\u3001\u503e\u89d2\u53cd\u5411) \u4e0e "
            "\u65b9\u6848B(\u540c\u8f6c\u3001\u503e\u89d2\u540c\u5411)",
            "\u65b9\u6848B \u53d6\u6d88\u4e86\u201c\u4e0a\u4e0b\u955c\u50cf\u201d\u8fd9\u4e00\u6b65"
            "\uff0c\u4e5f\u5c31\u53d6\u6d88\u4e86\u73b0\u573a\u6700\u5bb9\u6613\u88c5\u9519\u7684\u5730\u65b9")

    for k, (title, mirrored) in enumerate((
            ("\u65b9\u6848A \u5bf9\u8f6c\uff1a\u4e0a CW / \u4e0b CCW\uff0c"
             "\u503e\u89d2\u5fc5\u987b\u53cd\u5411", True),
            ("\u65b9\u6848B \u540c\u8f6c\uff1a\u4e0a\u4e0b\u540c\u5411\uff0c"
             "\u503e\u89d2\u540c\u5411", False))):
        ox = 40 + k * (W / 2 - 10)
        cw_ = W / 2 - 70
        g.append('<rect x="%.1f" y="82" width="%.1f" height="290" rx="10"'
                 ' fill="#fbfbfb" stroke="#dcdcdc"/>' % (ox, cw_))
        g.append(txt(ox + cw_ / 2, 108, title, 14, C_INK, "middle", "bold"))

        mx = ox + cw_ / 2
        g.append('<line x1="%.1f" y1="142" x2="%.1f" y2="336" stroke="#9a9a9a"'
                 ' stroke-width="7"/>' % (mx, mx))
        g.append(txt(mx, 356, "\u7535\u673a\u5ea7 / \u81c2", 12, C_MUTE))

        for j, (lab, yy, rot) in enumerate((
                ("\u4e0a\u5c42", 176, "CW"),
                ("\u4e0b\u5c42", 296, ("CCW" if mirrored else "CW")))):
            sgn = 1.0 if (j == 0 or not mirrored) else -1.0
            ang = math.radians(16.0) * sgn            # exaggerated for legibility
            half = 72.0
            dx, dy = half * math.cos(ang), -half * math.sin(ang)
            col = C_CW if rot == "CW" else C_CCW
            g.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s"'
                     ' stroke-width="6" stroke-linecap="round"/>'
                     % (mx - dx, yy - dy, mx + dx, yy + dy, col))
            g.append('<circle cx="%.1f" cy="%.1f" r="7" fill="%s"/>' % (mx, yy, col))
            g.append(txt(mx - half - 14, yy + 5, lab, 13, C_INK, "end", "bold"))
            g.append(txt(mx + half + 14, yy + 5, rot, 13, col, "start", "bold"))
            g.append(arrow(mx, yy, mx + sgn * 54, yy - 38, C_ARROW, 2.6))

        note = ("\u4e0a\u4e0b\u503e\u89d2\u65b9\u5411\u5fc5\u987b\u76f8\u53cd\uff0c"
                "\u88c5\u53cd\u5219\u504f\u822a\u53cd\u5411\u6216\u5f52\u96f6" if mirrored else
                "\u4e0a\u4e0b\u503e\u89d2\u5b8c\u5168\u4e00\u81f4\uff0c\u65e0\u955c\u50cf\u53ef\u9519")
        g.append(txt(ox + cw_ / 2, 400, note, 13,
                     C_CCW if mirrored else C_ARROW, "middle", "bold"))
    g.append("</svg>")
    return "\n".join(g)


# ===========================================================================
# figure 3 - reversal checklist
# ===========================================================================
def fig_checklist():
    W, H = 900, 590
    g = hdr(W, H,
            "\u56fe3  \u7531\u73b0\u5f79\u72b6\u6001(\u65b9\u6848A) \u6539\u4e3a\u65b9\u6848B "
            "\u9700\u53cd\u8f6c\u7684 8 \u53f0\u7535\u673a",
            "\u53cd\u8f6c = \u6539 ESC \u8f6c\u5411 + \u6362\u53cd\u624b\u6868"
            "\uff08\u4e24\u8005\u5fc5\u987b\u540c\u65f6\u505a\uff09")

    x0, y0, rowh, colw = 60, 88, 40, 780
    g.append('<rect x="%d" y="%d" width="%d" height="%d" rx="8" fill="#fbfbfb"'
             ' stroke="#dcdcdc"/>' % (x0, y0, colw, rowh * 9 + 8))
    hdrs = ("\u81c2\u4f4d", "\u73af\u4f4d", "\u65b9\u6848B \u8f6c\u5411",
            "\u4e0a\u5c42", "\u4e0b\u5c42", "\u9700\u53cd\u8f6c", "\u503e\u89d2\u65b9\u5411")
    xs = (100, 190, 300, 420, 510, 610, 730)
    for h, xx in zip(hdrs, xs):
        g.append(txt(xx, y0 + 26, h, 13, C_INK, "middle", "bold"))
    g.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#c8c8c8"'
             ' stroke-width="1.6"/>' % (x0, y0 + 36, x0 + colw, y0 + 36))

    for i, (name, xb, yb, ring, rot, tilt) in enumerate(PODS):
        yy = y0 + 36 + rowh * (i + 1) - 12
        if i % 2 == 0:
            g.append('<rect x="%d" y="%.1f" width="%d" height="%d" fill="#f4f4f4"/>'
                     % (x0 + 1, yy - 26, colw - 2, rowh))
        col = C_CW if rot == "CW" else C_CCW
        up_r = (rot != "CW")                     # CCW pods flip their upper motor
        cells = (name,
                 "\u5916" if ring == "out" else "\u5185",
                 "\u4e0a\u4e0b\u5747 " + rot,
                 "CW\u2192CCW" if up_r else "CW",
                 "CCW" if up_r else "CCW\u2192CW",
                 name + REVERSE[name],
                 "%s (%s)" % (TILT_CN[tilt], tilt))
        for j, (c, xx) in enumerate(zip(cells, xs)):
            fill, wt = C_INK, "normal"
            if j == 2:
                fill = col
            elif j == 5:
                fill, wt = C_CCW, "bold"
            elif j == 0:
                wt = "bold"
            elif j in (3, 4):
                fill = C_CCW if "\u2192" in c else C_MUTE
            g.append(txt(xx, yy, c, 13, fill, "middle", wt))

    fy = y0 + rowh * 9 + 46
    g.append(txt(x0, fy, "\u53cd\u8f6c\u6e05\u5355\uff1a" + "\u3001".join(
        "%s%s" % (p[0], REVERSE[p[0]]) for p in PODS), 14, C_CCW, "start", "bold"))
    g.append(txt(x0, fy + 28,
                 "\u5b8c\u6210\u540e\u5fc5\u987b\u7528\u7535\u673a\u6d4b\u8bd5\u9010\u53f0\u786e\u8ba4"
                 "\u8f6c\u5411\u4e0e\u6868\u5411\u4e00\u81f4\uff0c\u5426\u5219\u8be5\u7ec4\u7684"
                 "\u5347\u529b\u4e0e\u504f\u822a\u4f1a\u540c\u65f6\u51fa\u9519\u3002",
                 13, C_INK, "start"))
    g.append(txt(x0, fy + 52,
                 "\u53c2\u6570\uff1aQ_FRAME_TYPE = 20 \u5207\u5230\u65b9\u6848B\uff1b"
                 "= 0 \u6216 1 \u56de\u5230\u65b9\u6848A\u3002\u6539\u53c2\u6570\u540e"
                 "\u9700\u91cd\u542f\u98de\u63a7\u624d\u751f\u6548\u3002",
                 13, C_INK, "start"))
    g.append(txt(x0, fy + 76,
                 "\u6ce8\u610f\uff1a\u5171\u8f74\u540c\u8f6c\u4f1a\u5931\u53bb\u4e0b\u6868"
                 "\u56de\u6536\u4e0a\u6868\u65cb\u6d41\u7684\u6536\u76ca\uff0c"
                 "\u60ac\u505c\u6548\u7387\u7565\u964d\u3002",
                 13, C_MUTE, "start"))
    g.append("</svg>")
    return "\n".join(g)


os.makedirs(OUT, exist_ok=True)
for fn, body in (("TG700_v5_install_top.svg", fig_top()),
                 ("TG700_v5_install_section.svg", fig_section()),
                 ("TG700_v5_install_checklist.svg", fig_checklist())):
    p = os.path.join(OUT, fn)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        f.write(body)
    print("wrote %s  (%d bytes)" % (p, len(body.encode("utf-8"))))
