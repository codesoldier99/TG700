# -*- coding: utf-8 -*-
"""
Convert the TG700 markdown report into a self-contained HTML file, then to PDF
via headless Edge.  Source is deliberately pure ASCII so it survives being
written through a cp936 (GBK) code path unharmed.

- SVG images are inlined as real <svg> elements (vector, stays sharp in the PDF)
- PNG images are inlined as base64 data URIs
=> the resulting HTML has zero external references, which makes Edge's
   --print-to-pdf deterministic.
"""

import base64
import glob
import html
import os
import re
import subprocess
import sys
import time

import markdown

DOC_DIR = os.path.dirname(os.path.abspath(__file__))

CSS = r"""
@page { size: A4; margin: 17mm 15mm 16mm 15mm; }

* { -webkit-print-color-adjust: exact; print-color-adjust: exact; }

html { font-size: 10.5pt; }

body {
  font-family: "Microsoft YaHei", "DengXian", "SimSun", sans-serif;
  line-height: 1.54;
  color: #1a1d20;
  margin: 0;
}

/* ---------- headings ---------- */
h1, h2, h3, h4 { line-height: 1.35; page-break-after: avoid; break-after: avoid; }

h1 {
  font-size: 17pt;
  margin: 0 0 14px 0;
  padding: 8px 0 8px 0;
  border-bottom: 2.5px solid #1971c2;
  color: #0b3d68;
}
/* every chapter heading starts a fresh page (the title h1 is exempt) */
h1.chapter { page-break-before: always; break-before: page; }

h2 {
  font-size: 13.5pt;
  margin: 20px 0 9px 0;
  padding-left: 9px;
  border-left: 5px solid #1971c2;
  color: #10456f;
}

h3 {
  font-size: 11.8pt;
  margin: 15px 0 7px 0;
  color: #1c5c8c;
}

/* ---------- text ---------- */
p { margin: 6px 0; }
strong { color: #0b3d68; }
hr { border: none; border-top: 1px solid #dee2e6; margin: 16px 0; }

a { color: #1971c2; text-decoration: none; }

/* ---------- inline + block code ---------- */
code {
  font-family: Consolas, "Courier New", monospace;
  font-size: 0.90em;
  background: #f1f3f5;
  border: 1px solid #e3e6e8;
  border-radius: 3px;
  padding: 1px 4px;
  color: #b02a37;
}

pre {
  background: #f8f9fa;
  border: 1px solid #dee2e6;
  border-left: 3.5px solid #868e96;
  border-radius: 4px;
  padding: 9px 12px;
  margin: 10px 0;
  overflow: visible;
  white-space: pre-wrap;
  word-wrap: break-word;
  page-break-inside: avoid;
  break-inside: avoid;
}
pre code {
  background: none;
  border: none;
  padding: 0;
  color: #212529;
  font-size: 8.9pt;
  line-height: 1.45;
}

/* ---------- tables ---------- */
table {
  border-collapse: collapse;
  width: 100%;
  margin: 11px 0;
  font-size: 9.3pt;
  page-break-inside: avoid;
  break-inside: avoid;
}
th, td {
  border: 1px solid #ccd1d6;
  padding: 5px 7px;
  vertical-align: top;
}
th {
  background: #e7f0f7;
  color: #0b3d68;
  font-weight: bold;
  text-align: left;
}
tr:nth-child(even) td { background: #fafbfc; }

/* ---------- blockquote ---------- */
blockquote {
  margin: 11px 0;
  padding: 8px 13px;
  background: #fff9e6;
  border-left: 4px solid #f0a30a;
  color: #4a3c1a;
  page-break-inside: avoid;
  break-inside: avoid;
}
blockquote p { margin: 4px 0; }

/* ---------- lists ---------- */
ul, ol { margin: 7px 0; padding-left: 24px; }
li { margin: 3px 0; }

/* ---------- figures ---------- */
figure {
  margin: 14px 0;
  text-align: center;
  page-break-inside: avoid;
  break-inside: avoid;
}
figure svg { max-width: 100%; height: auto; }
figure img { max-width: 100%; height: auto; }
figcaption {
  margin-top: 5px;
  font-size: 8.8pt;
  color: #6c757d;
}

/* ---------- title block ---------- */
.doc-head { text-align: center; margin-bottom: 6px; }
.doc-head .sub { color: #6c757d; font-size: 10pt; margin-top: 4px; }
"""


def inline_svg(svg_path):
    with open(svg_path, "r", encoding="utf-8") as f:
        svg = f.read()
    # strip xml prolog / doctype / comments if present
    svg = re.sub(r"<\?xml.*?\?>", "", svg, flags=re.S)
    svg = re.sub(r"<!DOCTYPE.*?>", "", svg, flags=re.S)
    svg = svg.strip()
    # make ids unique so multiple inlined SVGs cannot clash on marker ids
    tag = "s%d_" % (abs(hash(svg_path)) % 100000)
    ids = set(re.findall(r'\bid="([^"]+)"', svg))
    for i in sorted(ids, key=len, reverse=True):
        svg = svg.replace('id="%s"' % i, 'id="%s%s"' % (tag, i))
        svg = svg.replace("url(#%s)" % i, "url(#%s%s)" % (tag, i))
    # let CSS control the size
    svg = re.sub(r'(<svg\b[^>]*?)\swidth="[\d.]+"', r"\1", svg, count=1)
    svg = re.sub(r'(<svg\b[^>]*?)\sheight="[\d.]+"', r"\1", svg, count=1)
    return svg


def data_uri(path):
    ext = os.path.splitext(path)[1].lower().lstrip(".")
    mime = {"png": "image/png", "jpg": "image/jpeg", "jpeg": "image/jpeg", "gif": "image/gif"}.get(ext, "application/octet-stream")
    with open(path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return "data:%s;base64,%s" % (mime, b64)


def embed_images(html_text, base_dir):
    """Replace <img src=...> with inline SVG or base64 data URIs."""
    pattern = re.compile(r'<img\b[^>]*?src="([^"]+)"[^>]*?/?>')

    def repl(m):
        whole = m.group(0)
        src = html.unescape(m.group(1))
        if src.startswith(("http://", "https://", "data:")):
            return whole
        alt_m = re.search(r'alt="([^"]*)"', whole)
        alt = html.unescape(alt_m.group(1)) if alt_m else ""
        path = os.path.normpath(os.path.join(base_dir, src))
        if not os.path.exists(path):
            sys.stderr.write("  WARN missing image: %s\n" % src)
            return whole
        cap = ('<figcaption>%s</figcaption>' % html.escape(alt)) if alt else ""
        if path.lower().endswith(".svg"):
            body = inline_svg(path)
            print("  inlined SVG   %s" % os.path.basename(path))
        else:
            body = '<img src="%s" alt="%s"/>' % (data_uri(path), html.escape(alt))
            print("  inlined image %s" % os.path.basename(path))
        return "<figure>%s%s</figure>" % (body, cap)

    return pattern.sub(repl, html_text)


def main():
    if len(sys.argv) > 1:
        md_path = sys.argv[1]
        if not os.path.isabs(md_path):
            md_path = os.path.join(DOC_DIR, md_path)
        if not os.path.exists(md_path):
            sys.exit("no such markdown file: %s" % md_path)
    else:
        md_files = sorted(glob.glob(os.path.join(DOC_DIR, "TG700_*.md")))
        if not md_files:
            sys.exit("no TG700_*.md found in %s" % DOC_DIR)
        if len(md_files) > 1:
            sys.exit("several TG700_*.md found, pass one explicitly:\n  " +
                     "\n  ".join(os.path.basename(p) for p in md_files))
        md_path = md_files[0]
    print("source : %s" % os.path.basename(md_path))

    with open(md_path, "r", encoding="utf-8") as f:
        text = f.read()

    body = markdown.markdown(
        text,
        extensions=["tables", "fenced_code", "sane_lists", "attr_list", "nl2br"],
        output_format="html5",
    )

    # mark chapter-level h1 (all but the very first h1) for page breaks
    h1s = list(re.finditer(r"<h1>", body))
    for m in reversed(h1s[1:]):
        body = body[: m.start()] + '<h1 class="chapter">' + body[m.end():]

    body = embed_images(body, DOC_DIR)

    doc = (
        "<!DOCTYPE html>\n<html lang=\"zh-CN\">\n<head>\n"
        "<meta charset=\"utf-8\"/>\n"
        "<title>TG700</title>\n"
        "<style>\n" + CSS + "\n</style>\n</head>\n<body>\n"
        + body +
        "\n</body>\n</html>\n"
    )

    html_path = os.path.splitext(md_path)[0] + ".html"
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(doc)
    print("html   : %s (%d bytes)" % (os.path.basename(html_path), os.path.getsize(html_path)))

    pdf_path = os.path.splitext(md_path)[0] + ".pdf"
    if os.path.exists(pdf_path):
        os.remove(pdf_path)

    edge = None
    for cand in (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                 r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"):
        if os.path.exists(cand):
            edge = cand
            break
    if edge is None:
        sys.exit("Edge not found")

    uri = "file:///" + html_path.replace("\\", "/").replace(" ", "%20")
    # Edge 152: --no-pdf-header-footer is the flag that actually suppresses the
    # date / URL / page-number band (--print-to-pdf-no-header is ignored).
    cmd = [edge, "--headless", "--disable-gpu", "--no-first-run",
           "--no-pdf-header-footer",
           "--print-to-pdf=" + pdf_path, uri]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)

    for _ in range(30):
        if os.path.exists(pdf_path) and os.path.getsize(pdf_path) > 0:
            break
        time.sleep(1)

    if not os.path.exists(pdf_path):
        sys.exit("PDF generation failed")

    stamp_page_numbers(pdf_path)
    print("pdf    : %s (%d bytes)" % (os.path.basename(pdf_path), os.path.getsize(pdf_path)))


def stamp_page_numbers(pdf_path):
    """Add a discreet centred page number to every page."""
    try:
        import fitz
    except ImportError:
        print("  (pymupdf absent, skipping page numbers)")
        return
    doc = fitz.open(pdf_path)
    total = doc.page_count
    for i, page in enumerate(doc, start=1):
        label = "%d / %d" % (i, total)
        w = fitz.get_text_length(label, fontname="helv", fontsize=8.5)
        page.insert_text(
            fitz.Point((page.rect.width - w) / 2.0, page.rect.height - 22),
            label, fontname="helv", fontsize=8.5, color=(0.45, 0.48, 0.51),
        )
    doc.saveIncr()
    doc.close()
    print("  stamped page numbers on %d pages" % total)


if __name__ == "__main__":
    main()
