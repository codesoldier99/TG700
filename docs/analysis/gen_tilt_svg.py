#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 TG700 新倾角方案(±X/±Y 四向约束)的示意图 SVG。

坐标映射: 机体 +X(机头) -> 屏幕上方; 机体 +Y(右) -> 屏幕右方。
"""
import math
import os

OUT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "images"))

RED, BLUE, INK, GREY, FAINT = "#e03131", "#1971c2", "#212529", "#868e96", "#ced4da"

FONT = ('font-family="Microsoft YaHei,Noto Sans CJK SC,SimHei,'
        'DejaVu Sans,sans-serif"')

DEFS = """  <defs>
    <marker id="aR" viewBox="0 0 10 10" refX="9" refY="5"
            markerWidth="5.5" markerHeight="5.5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{red}"/>
    </marker>
    <marker id="aB" viewBox="0 0 10 10" refX="9" refY="5"
            markerWidth="5.5" markerHeight="5.5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{blue}"/>
    </marker>
    <marker id="aK" viewBox="0 0 10 10" refX="9" refY="5"
            markerWidth="5" markerHeight="5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{ink}"/>
    </marker>
    <marker id="aG" viewBox="0 0 10 10" refX="9" refY="5"
            markerWidth="5" markerHeight="5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{grey}"/>
    </marker>
    <marker id="tR" viewBox="0 0 10 10" refX="5" refY="5"
            markerWidth="4.5" markerHeight="4.5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{red}"/>
    </marker>
    <marker id="tG" viewBox="0 0 10 10" refX="5" refY="5"
            markerWidth="4.5" markerHeight="4.5" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{grey}"/>
    </marker>
  </defs>
""".format(red=RED, blue=BLUE, ink=INK, grey=GREY)


def head(w, h, title, sub):
    s = ['<?xml version="1.0" encoding="UTF-8"?>',
         '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
         'viewBox="0 0 %d %d" %s>' % (w, h, w, h, FONT),
         DEFS,
         '  <rect width="%d" height="%d" fill="#ffffff"/>' % (w, h),
         '  <text x="%d" y="34" font-size="21" font-weight="bold" '
         'text-anchor="middle" fill="%s">%s</text>' % (w // 2, INK, title)]
    if sub:
        s.append('  <text x="%d" y="58" font-size="13" text-anchor="middle" '
                 'fill="%s">%s</text>' % (w // 2, GREY, sub))
    return s


def txt(x, y, size, s, col=INK, anchor="start", bold=False):
    return ('  <text x="%.1f" y="%.1f" font-size="%s" text-anchor="%s" '
            'fill="%s"%s>%s</text>'
            % (x, y, size, anchor, col, ' font-weight="bold"' if bold else '', s))


def line(x1, y1, x2, y2, col, w=2.0, dash=None, marker=None):
    d = ' stroke-dasharray="%s"' % dash if dash else ''
    m = ' marker-end="url(#%s)"' % marker if marker else ''
    return ('  <line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
            'stroke-width="%.1f"%s%s/>' % (x1, y1, x2, y2, col, w, d, m))


def dim_h(x1, x2, y, label, col, size=12, bold=False, above=True):
    """水平尺寸线, 两端带短竖线"""
    mk = "t" + ("R" if col == RED else "G")
    return [line(x1, y, x2, y, col, 1.5, marker=mk),
            line(x2, y, x1, y, col, 1.5, marker=mk),
            line(x1, y - 6, x1, y + 6, col, 1.3),
            line(x2, y - 6, x2, y + 6, col, 1.3),
            txt((x1 + x2) / 2, y - 11 if above else y + 20, size, label, col,
                "middle", bold)]


def dim_v(x, y1, y2, label, col, size=12, bold=False):
    """竖直尺寸线, 标签放在左侧"""
    mk = "t" + ("R" if col == RED else "G")
    return [line(x, y1, x, y2, col, 1.5, marker=mk),
            line(x, y2, x, y1, col, 1.5, marker=mk),
            line(x - 6, y1, x + 6, y1, col, 1.3),
            line(x - 6, y2, x + 6, y2, col, 1.3),
            txt(x - 12, (y1 + y2) / 2 + 4, size, label, col, "end", bold)]


# ===========================================================================
# 图 1 · 俯视图总览
# ===========================================================================
CX, CY, SC = 450.0, 300.0, 55.0

PODS = [
    ("P1", "前右外", ( 1.665,  3.4), "X", "机头", "机尾"),
    ("P2", "前右内", ( 1.665,  1.6), "Y", "左",   "右"),
    ("P3", "前左内", ( 1.665, -1.6), "Y", "左",   "右"),
    ("P4", "前左外", ( 1.665, -3.4), "X", "机尾", "机头"),
    ("P5", "后右外", (-1.665,  3.4), "X", "机头", "机尾"),
    ("P6", "后右内", (-1.665,  1.6), "Y", "右",   "左"),
    ("P7", "后左内", (-1.665, -1.6), "Y", "右",   "左"),
    ("P8", "后左外", (-1.665, -3.4), "X", "机尾", "机头"),
]

SCREEN_DIR = {"机头": (0.0, -1.0), "机尾": (0.0, 1.0),
              "右": (1.0, 0.0), "左": (-1.0, 0.0)}


def top_view():
    L, OFF = 58.0, 9.5
    s = head(900, 720,
             "天工700 电机座倾角安装方向　±X/±Y 四向约束方案",
             "俯视图 · 机头朝上 · 倾角均为 5° · 红实线箭头 = 上层 CW 桨的倾斜朝向，"
             "蓝虚线箭头 = 下层 CCW 桨")

    s.append('  <rect x="%.1f" y="%.1f" width="60" height="184" rx="14" '
             'fill="#f8f9fa" stroke="#adb5bd" stroke-width="1.6"/>'
             % (CX - 30, CY - 92))
    for _, _, p, _, _, _ in PODS:
        s.append(line(CX, CY, CX + SC * p[1], CY - SC * p[0], "#adb5bd", 1.6))

    s.append(line(CX, CY - 96, CX, CY - 144, INK, 2.2, marker="aK"))
    s.append(txt(CX, CY - 152, 14, "机头 +X", INK, "middle", True))
    s.append(line(CX + 36, CY, CX + 92, CY, INK, 2.2, marker="aK"))
    s.append(txt(CX + 100, CY + 5, 14, "+Y 右", INK, "start", True))
    s.append('  <circle cx="%.1f" cy="%.1f" r="4" fill="%s"/>' % (CX, CY, INK))
    s.append(txt(CX, CY + 24, 11.5, "重心", GREY, "middle"))

    for tag, name, p, axis, dir_up, dir_lo in PODS:
        x, y = CX + SC * p[1], CY - SC * p[0]
        du, dl = SCREEN_DIR[dir_up], SCREEN_DIR[dir_lo]
        pu = (-du[1] * OFF, du[0] * OFF)
        pl = (dl[1] * OFF, -dl[0] * OFF)
        s.append(line(x + pu[0], y + pu[1], x + pu[0] + du[0] * L,
                      y + pu[1] + du[1] * L, RED, 3.6, marker="aR"))
        s.append(line(x + pl[0], y + pl[1], x + pl[0] + dl[0] * L,
                      y + pl[1] + dl[1] * L, BLUE, 3.6, dash="8 4.5", marker="aB"))
        s.append('  <circle cx="%.1f" cy="%.1f" r="17" fill="#fff" stroke="%s" '
                 'stroke-width="2.6"/>' % (x, y, INK))
        s.append(txt(x, y + 5, 13, tag, INK, "middle", True))
        if axis == "X":
            side = 1 if p[1] > 0 else -1
            s.append(txt(x + side * 22, y + 5, 11.5, name, GREY,
                         "start" if side > 0 else "end"))
        else:
            s.append(txt(x, y - 30 if p[0] > 0 else y + 38, 11.5, name,
                         GREY, "middle"))

    s += dim_h(CX, CX + 1.6 * SC, 512, "内侧 |Y| = 1.6 m", GREY)
    s += dim_h(CX, CX + 3.4 * SC, 548, "外侧 |Y| = 3.4 m", GREY, above=False)
    s.append(txt(CX - 2.0 * SC, 532, 12, "|X| = 1.665 m　（8 组吊舱全部相同）",
                 GREY, "middle"))

    s.append('  <rect x="30" y="584" width="840" height="96" rx="9" '
             'fill="#f8f9fa" stroke="#dee2e6" stroke-width="1.4"/>')
    cols = [
        (52, RED, "外侧四组 P1 P4 P5 P8 —— 沿机头 / 机尾方向倾斜（力矩臂 3.4 m）",
         ["右侧 P1、P5：上层朝机头，下层朝机尾",
          "左侧 P4、P8：上层朝机尾，下层朝机头"]),
        (470, BLUE, "内侧四组 P2 P3 P6 P7 —— 沿左 / 右方向倾斜（力矩臂 1.665 m）",
         ["前排 P2、P3：上层朝左，下层朝右",
          "后排 P6、P7：上层朝右，下层朝左"]),
    ]
    for xc, col, hdr, items in cols:
        s.append(txt(xc, 610, 12.6, hdr, col, "start", True))
        for i, it in enumerate(items):
            s.append(txt(xc + 16, 636 + i * 22, 12.4, it, INK))

    s.append(txt(450, 706, 12, "四个朝向全部以机体为基准（机头 / 机尾 / 左 / 右），"
                               "不依赖观察者站在哪一侧 —— 这是本方案相对旧「切向」"
                               "规则的关键改进", GREY, "middle"))
    s.append('</svg>')
    return "\n".join(s)


# ===========================================================================
# 图 2 · 力矩臂对比（为什么外侧选 ±X、内侧选 ±Y）
# ===========================================================================
def arm_compare():
    s = head(900, 780,
             "为什么外侧沿 ±X、内侧沿 ±Y：偏航力矩臂对比",
             "水平力产生的偏航力矩 = 力 × 力矩臂，而力矩臂 = 重心到「力的作用线」的垂直距离")

    SCL = 52.0
    panels = [
        (290.0, "外侧吊舱 P1（X = 1.665 m，Y = 3.4 m）", 1.665, 3.4, "X",
         ["外侧吊舱离机身中线很远（|Y| = 3.4 m）：",
          "沿机头 / 机尾方向推，作用线离重心 3.4 m；",
          "沿左 / 右方向推，作用线只离重心 1.665 m。",
          "",
          "→ 外侧四组必须沿 ±X（机头 / 机尾）倾斜",
          "　 力矩臂 3.400 m，是另一种装法的 2.04 倍"]),
        (640.0, "内侧吊舱 P2（X = 1.665 m，Y = 1.6 m）", 1.665, 1.6, "Y",
         ["内侧吊舱离中线较近（|Y| = 1.6 m），已经",
          "略小于前后距离 1.665 m，两种装法几乎打平，",
          "但沿左 / 右方向略占优。",
          "",
          "→ 内侧四组沿 ±Y（左 / 右）倾斜",
          "　 力矩臂 1.665 m，略优于 1.600 m"]),
    ]

    for gy, title, rx, ry, best, notes in panels:
        gx = 200.0
        mx, my = gx + ry * SCL, gy - rx * SCL
        s.append(txt(gx - 30, gy - 178, 14.5, title, INK, "start", True))
        s.append(line(gx, gy, gx + 250, gy, "#adb5bd", 1.4))
        s.append(line(gx, gy, gx, gy - 150, "#adb5bd", 1.4))
        s.append(txt(gx + 256, gy + 4, 11.5, "+Y", GREY))
        s.append(txt(gx - 20, gy - 154, 11.5, "+X", GREY))
        s.append('  <circle cx="%.1f" cy="%.1f" r="4.5" fill="%s"/>' % (gx, gy, INK))
        s.append(txt(gx - 8, gy + 20, 11.5, "重心", INK, "end"))

        winX = (best == "X")
        cX = RED if winX else GREY
        cY = GREY if winX else RED

        # 作用线: 沿 ±X 的力 -> 竖直作用线; 沿 ±Y 的力 -> 水平作用线
        s.append(line(mx, gy + 46, mx, my - 76, cX, 1.4, dash="6 4"))
        s.append(line(gx - 42, my, gx + 258, my, cY, 1.4, dash="6 4"))

        s.append(line(mx, my, mx, my - 62, cX, 3.6 if winX else 2.4,
                      marker="aR" if winX else "aG"))
        s.append(txt(mx, my - 72, 12, "沿 ±X 的水平力", cX, "middle", winX))
        s.append(line(mx, my, mx + 62, my, cY, 3.6 if not winX else 2.4,
                      marker="aR" if not winX else "aG"))
        s.append(txt(mx + 32, my - 14, 12, "沿 ±Y 的水平力", cY, "start", not winX))

        s += dim_h(gx, mx, gy + 46,
                   "力矩臂 = |Y| = %.3f m" % ry, cX, 12.4, winX, above=False)
        s += dim_v(gx - 42, gy, my,
                   "力矩臂 = |X| = %.3f m" % rx, cY, 12.4, not winX)

        s.append('  <circle cx="%.1f" cy="%.1f" r="16" fill="#fff" stroke="%s" '
                 'stroke-width="2.6"/>' % (mx, my, INK))
        s.append(txt(mx, my + 5, 12, "P1" if winX else "P2", INK, "middle", True))

        s.append('  <rect x="530" y="%.1f" width="340" height="164" rx="9" '
                 'fill="#f8f9fa" stroke="#dee2e6" stroke-width="1.4"/>'
                 % (gy - 172))
        for i, t in enumerate(notes):
            if not t:
                continue
            c = RED if t.startswith("→") or t.startswith("　") else INK
            s.append(txt(548, gy - 146 + i * 24, 12.4, t, c, "start",
                         t.startswith("→")))

    s.append(line(30, 420, 870, 420, "#dee2e6", 1.4, dash="6 4"))
    s.append(txt(450, 766, 12.2,
                 "理想纯切向装法的力矩臂是 |r|（外侧 3.786 m / 内侧 2.309 m）；"
                 "四向约束下只能取到 3.400 / 1.665，合计保留设计偏航权限的 83.5%",
                 GREY, "middle"))
    s.append('</svg>')
    return "\n".join(s)


# ===========================================================================
# 图 3 · 单个共轴组剖面
# ===========================================================================
def section_view():
    s = head(900, 500,
             "单个共轴组剖面：上下两台电机必须反向倾斜 5°",
             "以 P1（前右外，沿 ±X 倾斜）为例 · 从飞机右侧看进去 · 机头在左 · "
             "图中倾角已放大约 3 倍以便观察")

    ax = 300.0
    ARM = 88.0
    EX = 3.0
    TH = math.radians(5.0) * EX
    LEN = 92.0

    s.append(line(ax, 150, ax, 402, "#adb5bd", 7.0))
    s.append('  <rect x="%.1f" y="402" width="120" height="18" rx="6" '
             'fill="#f1f3f5" stroke="#adb5bd" stroke-width="1.6"/>' % (ax - 60))
    s.append(txt(ax, 438, 11.5, "吊舱立柱 / 机臂", GREY, "middle"))

    for yc, col, mk, dash, label, lean, hint in (
            (190.0, RED, "aR", None, "上层　CW 顺时针桨", -1.0, "水平分力 → 机头"),
            (330.0, BLUE, "aB", "8 4.5", "下层　CCW 逆时针桨", +1.0, "水平分力 → 机尾")):
        s.append(line(ax, yc, ax, yc - LEN, FAINT, 1.4, dash="4 3"))
        tx = ax + lean * math.sin(TH) * LEN
        ty = yc - math.cos(TH) * LEN
        s.append(line(ax, yc, tx, ty, col, 3.8, dash=dash, marker=mk))
        s.append(txt(tx + lean * 12, ty - 8, 12, "推力", col,
                     "end" if lean < 0 else "start", True))
        s.append('  <path d="M %.1f %.1f A 46 46 0 0 %d %.1f %.1f" fill="none" '
                 'stroke="%s" stroke-width="1.3"/>'
                 % (ax, yc - 46, 0 if lean < 0 else 1,
                    ax + lean * math.sin(TH) * 46, yc - math.cos(TH) * 46, col))
        s.append(txt(ax + lean * 34, yc - 52, 11.5, "5°", col,
                     "end" if lean < 0 else "start"))
        s.append('  <ellipse cx="%.1f" cy="%.1f" rx="%.1f" ry="7" fill="none" '
                 'stroke="%s" stroke-width="3" transform="rotate(%.2f %.1f %.1f)"/>'
                 % (ax, yc, ARM, col, lean * 5.0 * EX, ax, yc))
        s.append('  <circle cx="%.1f" cy="%.1f" r="11" fill="%s"/>' % (ax, yc, col))
        s.append(txt(ax - ARM - 18, yc + 5, 12.5, label, col, "end", True))
        # 水平分力画在各自桨盘的外侧, 避免上下混淆
        hy = yc - 28 if lean < 0 else yc + 28
        hx = ax + lean * 76
        s.append(line(ax, hy, hx, hy, col, 2.8, marker=mk))
        s.append(txt(hx + lean * 10, hy - 9 if lean < 0 else hy + 17, 11.5, hint,
                     col, "end" if lean < 0 else "start"))

    s.append(line(140, 380, 74, 380, INK, 2.2, marker="aK"))
    s.append(txt(107, 402, 13, "机头", INK, "middle", True))

    notes = [
        ("上下两台的水平分力方向相反，构成一个纯力偶", INK, True),
        ("→ 绕重心产生偏航力矩，力矩臂 = |Y| = 3.4 m", INK, False),
        ("→ 上下推力相等时水平净力为零，不干扰升力与姿态", INK, False),
        ("", INK, False),
        ("若上下装成同向：力偶抵消，偏航权限只剩桨反扭矩（约 2%）", RED, False),
        ("若整组装反：偏航符号翻转，PID 变正反馈，一离地就自旋", RED, False),
        ("", INK, False),
        ("本方案取消原 2° 向心分量（四向约束下无法与切向分量叠加）", GREY, False),
    ]
    s.append('  <rect x="530" y="150" width="348" height="212" rx="9" '
             'fill="#f8f9fa" stroke="#dee2e6" stroke-width="1.4"/>')
    for i, (t, c, b) in enumerate(notes):
        if not t:
            continue
        s.append(txt(548, 176 + i * 23, 12.3, t, c, "start", b))
    s.append('</svg>')
    return "\n".join(s)


for fname, gen in (("TG700_倾角安装_新方案俯视图.svg", top_view),
                   ("TG700_倾角安装_力矩臂对比.svg", arm_compare),
                   ("TG700_倾角安装_共轴组剖面.svg", section_view)):
    path = os.path.join(OUT_DIR, fname)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(gen())
    print("wrote %s (%d bytes)" % (path, os.path.getsize(path)))
