# -*- coding: utf-8 -*-
"""把 paper/piper_repro_report.md 渲染成 DOCX（主交付物）与 PDF（分发版）。

用法（仓库根目录）：
    python paper/build_piper_report.py

设计：Markdown 是唯一内容源；DOCX 用 python-docx 走 docx_kit 的样式，
PDF 用 PyMuPDF Story 走 HTML/CSS。两边读同一份块结构，避免文案漂移。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

import docx_kit as K

SRC = "paper/piper_repro_report.md"
DOCX_OUT = "PiPER_FACTR2_复现说明报告.docx"
PDF_OUT = "PiPER_FACTR2_复现说明报告.pdf"

TITLE = "PiPER 标准版 FACTR 2 复现实施报告"
SUBTITLE = "从信号采集、力矩尺度核验，到 NEXT 训练、接触验证与 FIRST 策略评估"
HEADER = "PiPER 标准版 · FACTR 2 复现实施报告"
META = [
    ("论　　文", "FACTR 2: Learning External Force Sensing for Commodity\nRobot Arms Improves Policy Learning"),
    ("作　　者", "S. Oh*, J. J. Liu*, T. Tao*, P. Han, K. Shaw, S. Funabashi,\nR. Salakhutdinov, D. Pathak（*共同第一作者）"),
    ("机　　构", "Carnegie Mellon University · Waseda University"),
    ("来　　源", "arXiv:2606.12406v2 [cs.RO]　项目页 jasonjzliu.com/factr2"),
    ("适用平台", "AgileX PiPER 标准版；固件、末端与控制链需开工前核验"),
    ("版本日期", "修订版 2.0　2026 年 10 月 19 日"),
    ("报告定位", "复现实施方案与验收规范；不代表实机实验已经完成"),
]

# ============================================================ Markdown 解析
INLINE = re.compile(r"(\*\*.+?\*\*|`[^`]+`|\*[^*]+\*)")


def _iter_runs(text, bold=False, italic=False):
    """递归处理嵌套行内标记（例如 **粗体里的 `代码`**）。"""
    for seg in INLINE.split(text):
        if not seg:
            continue
        if seg.startswith("**") and seg.endswith("**") and len(seg) > 4:
            yield from _iter_runs(seg[2:-2], True, italic)
        elif seg.startswith("`") and seg.endswith("`") and len(seg) > 2:
            yield (seg[1:-1], bold, True, italic)
        elif seg.startswith("*") and seg.endswith("*") and len(seg) > 2:
            yield from _iter_runs(seg[1:-1], bold, True)
        else:
            yield (seg, bold, False, italic)


def parse_inline(text):
    """→ [(text, bold, code, italic)]"""
    return list(_iter_runs(text))


def strip_inline(text):
    return "".join(t for t, *_ in parse_inline(text))


def parse_md(path):
    lines = open(path, encoding="utf-8").read().splitlines()
    blocks, i, n = [], 0, len(lines)
    while i < n:
        ln = lines[i]
        s = ln.strip()
        if not s:
            i += 1
            continue
        if s.startswith("```"):
            lang = s[3:].strip()
            i += 1
            buf = []
            while i < n and not lines[i].strip().startswith("```"):
                buf.append(lines[i])
                i += 1
            i += 1
            blocks.append(("code", lang, "\n".join(buf)))
            continue
        if s.lower() in ("<!--pagebreak-->", "<!-- pagebreak -->"):
            blocks.append(("pagebreak",))
            i += 1
            continue
        if s == "---":
            blocks.append(("hr",))
            i += 1
            continue
        m = re.match(r"^(#{1,4})\s+(.*)$", s)
        if m:
            blocks.append((f"h{len(m.group(1))}", m.group(2).strip()))
            i += 1
            continue
        # 图片：![caption](path)
        m = re.match(r"^!\[(.+?)\]\((.+?)\)$", s)
        if m:
            blocks.append(("fig", m.group(2), m.group(1)))
            i += 1
            continue
        if s.startswith(">"):
            buf = []
            while i < n and lines[i].strip().startswith(">"):
                buf.append(lines[i].strip()[1:].strip())
                i += 1
            blocks.append(("quote", " ".join(x for x in buf if x)))
            continue
        if s.startswith("|"):
            rows = []
            while i < n and lines[i].strip().startswith("|"):
                raw = lines[i].strip()
                if not re.match(r"^\|[\s:\-\|]+\|$", raw):
                    cells = [c.strip() for c in raw.strip("|").split("|")]
                    rows.append(cells)
                i += 1
            if rows:
                blocks.append(("table", rows[0], rows[1:]))
            continue
        if re.match(r"^[-*]\s+", s):
            items = []
            while i < n and re.match(r"^[-*]\s+", lines[i].strip()):
                items.append(re.sub(r"^[-*]\s+", "", lines[i].strip()))
                i += 1
            blocks.append(("ul", items))
            continue
        if re.match(r"^\d+\.\s+", s):
            items = []
            start = int(re.match(r"^(\d+)\.", s).group(1))
            while i < n and re.match(r"^\d+\.\s+", lines[i].strip()):
                items.append(re.sub(r"^\d+\.\s+", "", lines[i].strip()))
                i += 1
            blocks.append(("ol", items, start))
            continue
        buf = [s]
        i += 1
        while i < n:
            nxt = lines[i].strip()
            if (not nxt or nxt.startswith(("#", "|", ">", "```", "![", "---"))
                    or re.match(r"^[-*]\s+", nxt) or re.match(r"^\d+\.\s+", nxt)):
                break
            buf.append(nxt)
            i += 1
        blocks.append(("p", " ".join(buf)))
    return blocks


def disp_len(s):
    return sum(2 if ord(c) > 0x2E80 else 1 for c in strip_inline(s))


def col_widths(headers, rows, total=16.0):
    n = len(headers)
    lens = []
    for c in range(n):
        vals = [disp_len(headers[c])] + [disp_len(r[c]) for r in rows if c < len(r)]
        lens.append(max(4, min(max(vals), 46)))
    raw = [x ** 0.85 for x in lens]          # 压一下长列，避免一列吃掉全宽
    ws = [total * x / sum(raw) for x in raw]
    ws = [max(1.5, w) for w in ws]
    k = total / sum(ws)
    return [round(w * k, 2) for w in ws]


# ============================================================ DOCX
def docx_inline(p, text, size=10.5, color=K.TXT, cn=K.BODY_CN, en=K.BODY_EN):
    for txt, bold, code, italic in parse_inline(text):
        r = p.add_run(txt)
        if code:
            K.set_run_font(r, size=size - 0.8, bold=False, color="1F4E79",
                           cn="Consolas", en="Consolas")
        else:
            K.set_run_font(r, size=size, bold=bold, color=color, cn=cn, en=en, italic=italic)


def add_rich_table(doc, headers, rows, widths, size=9.0):
    """支持单元格内 **bold** / `code` 的表格，样式对齐 docx_kit.add_table。"""
    from docx.enum.table import WD_TABLE_ALIGNMENT
    t = doc.add_table(rows=1, cols=len(headers))
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    K.table_borders(t)
    K.cell_margins(t)
    hdr = t.rows[0]
    trPr = hdr._tr.get_or_add_trPr()
    trPr.append(OxmlElement("w:tblHeader"))
    for i, h in enumerate(headers):
        c = hdr.cells[i]
        c.width = Cm(widths[i])
        K.cell_bg(c, K.NAVY)
        K.cell_text(c, strip_inline(h), size=size, bold=True, color="FFFFFF",
                    cn=K.HEAD_CN, en=K.HEAD_EN)
    for ri, row in enumerate(rows):
        tr = t.add_row()
        trPr = tr._tr.get_or_add_trPr()
        trPr.append(OxmlElement("w:cantSplit"))
        for ci, val in enumerate(row):
            cell = tr.cells[ci]
            cell.width = Cm(widths[ci])
            cell.text = ""
            p = cell.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.LEFT if ci == 0 else WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_before = Pt(1)
            p.paragraph_format.space_after = Pt(1)
            p.paragraph_format.line_spacing = 1.18
            docx_inline(p, val, size=size, color=K.TXT)
            if ri % 2 == 1:
                K.cell_bg(cell, "F7FAFD")
    return t


def rich_callout(doc, text):
    """左侧强调条的提示框，正文支持行内标记。"""
    from docx.enum.table import WD_TABLE_ALIGNMENT
    title, body = None, text
    m = re.match(r"^\*\*(.+?)\*\*[：:]\s*(.*)$", text, re.S)
    if m:
        title, body = m.group(1), m.group(2)
    t = doc.add_table(rows=1, cols=1)
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    K.no_borders(t)
    cell = t.rows[0].cells[0]
    cell.width = Cm(16.0)
    K.cell_bg(cell, K.BOXBG)
    K.cell_margins(t, top=140, bottom=140, left=240, right=200)
    K.cell_left_bar(cell, K.BLUE, 24)
    cell.text = ""
    p0 = cell.paragraphs[0]
    p0.paragraph_format.space_after = Pt(3)
    p0.paragraph_format.line_spacing = 1.2
    if title:
        r0 = p0.add_run(strip_inline(title))
        K.set_run_font(r0, size=10.2, bold=True, color=K.NAVY, cn=K.HEAD_CN, en=K.HEAD_EN)
    p1 = cell.add_paragraph()
    p1.paragraph_format.space_after = Pt(0)
    p1.paragraph_format.line_spacing = 1.5
    p1.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    docx_inline(p1, body, size=9.8)
    K._p(doc, space_before=0, space_after=7)


def code_block(doc, text):
    t = doc.add_table(rows=1, cols=1)
    from docx.enum.table import WD_TABLE_ALIGNMENT
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    K.no_borders(t)
    cell = t.rows[0].cells[0]
    cell.width = Cm(16.0)
    K.cell_bg(cell, "F6F8FA")
    K.cell_left_bar(cell, "9DB6D0", 18)
    K.cell_margins(t, top=100, bottom=100, left=200, right=140)
    cell.text = ""
    first = True
    for line in text.split("\n"):
        p = cell.paragraphs[0] if first else cell.add_paragraph()
        first = False
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.line_spacing = 1.18
        r = p.add_run(line if line else " ")
        K.set_run_font(r, size=8.3, color="22303F", cn="Consolas", en="Consolas")
    K._p(doc, space_before=0, space_after=8)


def quote_block(doc, text):
    rich_callout(doc, text)


def cover(doc):
    K._p(doc, space_before=6, space_after=0)
    p = K._p(doc, space_before=0, space_after=0, line=1.0)
    K.add_p_bottom_border(p, K.NAVY, 18, 6)

    p = K._p(doc, space_before=14, space_after=4, line=1.2)
    r = p.add_run(COVER_KICKER)
    K.set_run_font(r, size=10, bold=True, color=K.BLUE, cn=K.HEAD_CN, en=K.HEAD_EN)

    p = K._p(doc, space_before=4, space_after=6, line=1.12)
    r = p.add_run(COVER_TITLE_DOCX)
    K.set_run_font(r, size=27, bold=True, color=K.NAVY, cn=K.HEAD_CN, en=K.HEAD_EN)

    p = K._p(doc, space_before=2, space_after=2, line=1.35)
    r = p.add_run(SUBTITLE)
    K.set_run_font(r, size=10.5, color=K.GREY, cn=K.HEAD_CN, en=K.HEAD_EN)

    p = K._p(doc, space_before=10, space_after=0, line=1.35)
    r = p.add_run("NEXT 神经外部力矩估计　·　FIRST 力信息驱动重采样　·　"
                  "自由空间 L1 目标 ≤ 0.05 N·m")
    K.set_run_font(r, size=10, bold=True, color=K.BLUE, cn=K.HEAD_CN, en=K.HEAD_EN)

    K._p(doc, space_before=6, space_after=0)
    p = K._p(doc, space_before=0, space_after=0, line=1.0)
    K.add_p_bottom_border(p, K.LINE, 6, 4)

    if COVER_FIG:
        K.figure(doc, COVER_FIG, COVER_CAP, width_cm=13.6)

    K._p(doc, space_before=6, space_after=2)
    t = doc.add_table(rows=0, cols=2)
    t.autofit = False
    K.no_borders(t)
    for key, val in META:
        cells = t.add_row().cells
        cells[0].width = Cm(2.6)
        cells[1].width = Cm(13.2)
        p0 = cells[0].paragraphs[0]
        p0.paragraph_format.space_after = Pt(3)
        r = p0.add_run(key)
        K.set_run_font(r, size=8.6, bold=True, color=K.BLUE, cn=K.HEAD_CN, en=K.HEAD_EN)
        p1 = cells[1].paragraphs[0]
        p1.paragraph_format.space_after = Pt(3)
        p1.paragraph_format.line_spacing = 1.25
        r = p1.add_run(val)
        K.set_run_font(r, size=8.6, color=K.GREY, cn=K.BODY_CN, en=K.BODY_EN)

    K._p(doc, space_before=8, space_after=0)
    p = K._p(doc, space_before=0, space_after=0, line=1.35,
             align=WD_ALIGN_PARAGRAPH.LEFT)
    r = p.add_run(COVER_LEAD)
    K.set_run_font(r, size=9.5, bold=True, color=K.NAVY, cn=K.BODY_CN, en=K.BODY_EN)

    doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)


def build_docx(blocks):
    doc = Document()
    style = doc.styles["Normal"]
    style.font.size = Pt(10.5)
    style.font.name = K.BODY_EN
    style.font.color.rgb = RGBColor.from_string(K.TXT)
    style.element.rPr.rFonts.set(qn("w:eastAsia"), K.BODY_CN)

    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    sec.top_margin, sec.bottom_margin = Cm(2.2), Cm(2.0)
    sec.left_margin, sec.right_margin = Cm(2.5), Cm(2.5)
    sec.header_distance, sec.footer_distance = Cm(1.1), Cm(1.0)

    hp = sec.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    r = hp.add_run(HEADER)
    K.set_run_font(r, size=8, color="8C9AAB", cn=K.HEAD_CN, en=K.HEAD_EN)

    fp = sec.footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for txt, fld in (("— ", False), (None, True), (" —", False)):
        r = fp.add_run(txt or "")
        K.set_run_font(r, size=8.5, color=K.GREY)
        if fld:
            K.add_field(r, "PAGE")

    cover(doc)

    for b in blocks:
        kind = b[0]
        if kind == "h1":                       # 文档内 # 标题视作章节（本报告无）
            K.heading(doc, b[1], 1)
        elif kind == "h2":
            K.heading(doc, b[1], 1)
        elif kind == "h3":
            K.heading(doc, b[1], 2)
        elif kind == "h4":
            K.heading(doc, b[1], 3)
        elif kind == "p":
            p = K._p(doc, space_before=0, space_after=7, line=1.62,
                     align=WD_ALIGN_PARAGRAPH.JUSTIFY, first_indent=21)
            docx_inline(p, b[1])
        elif kind == "ul":
            for it in b[1]:
                p = K._p(doc, space_before=0, space_after=4, line=1.55)
                p.paragraph_format.left_indent = Pt(20)
                p.paragraph_format.first_line_indent = Pt(-10)
                r0 = p.add_run("▪ ")
                K.set_run_font(r0, size=9, color=K.BLUE)
                docx_inline(p, it)
        elif kind == "ol":
            start = b[2] if len(b) > 2 else 1
            for n, it in enumerate(b[1], start):
                p = K._p(doc, space_before=0, space_after=4, line=1.55)
                p.paragraph_format.left_indent = Pt(22)
                p.paragraph_format.first_line_indent = Pt(-12)
                r0 = p.add_run(f"{n}. ")
                K.set_run_font(r0, size=10, bold=True, color=K.BLUE)
                docx_inline(p, it)
        elif kind == "table":
            headers, rows = b[1], b[2]
            ncol = len(headers)
            rows = [r + [""] * (ncol - len(r)) if len(r) < ncol else r[:ncol] for r in rows]
            ws = col_widths(headers, rows)
            size = 8.6 if ncol >= 5 else 9.0
            add_rich_table(doc, headers, rows, widths=ws, size=size)
            K._p(doc, space_before=0, space_after=9)
        elif kind == "code":
            code_block(doc, b[2])
        elif kind == "quote":
            quote_block(doc, b[1])
        elif kind == "fig":
            K.figure(doc, b[1], b[2], width_cm=14.6)
        elif kind == "hr":
            K.divider(doc)
        elif kind == "pagebreak":
            doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

    K.fix_math_fonts(doc)
    doc.save(DOCX_OUT)
    print("docx:", DOCX_OUT, round(os.path.getsize(DOCX_OUT) / 1024 / 1024, 2), "MB")


# ============================================================ PDF
CSS = """
@font-face { font-family: cn; src: url(msyh.ttc); }
@font-face { font-family: cnb; src: url(msyhbd.ttc); }
@font-face { font-family: mono; src: url(consola.ttf); }
* { font-family: cn; }
body { font-size: 9.6pt; line-height: 1.66; color: #1A1A1A; }
h1 { font-family: cnb; font-size: 15pt; color: #16365C; margin: 15pt 0 7pt 0;
     border-bottom: 1.2pt solid #2E75B6; padding-bottom: 3pt; }
h2 { font-family: cnb; font-size: 11.6pt; color: #16365C; margin: 12pt 0 5pt 0; }
h3 { font-family: cnb; font-size: 10.4pt; color: #2E75B6; margin: 9pt 0 4pt 0; }
p { margin: 0 0 6pt 0; text-align: justify; text-indent: 19pt; }
p.url { text-align: left; text-indent: 0; word-break: break-all; }
li { margin: 0 0 3pt 0; }
ul, ol { margin: 3pt 0 7pt 0; padding-left: 16pt; }
table { border-collapse: collapse; width: 100%; margin: 5pt 0 9pt 0; font-size: 8.2pt;
        table-layout: fixed; break-inside: avoid; }
td, th { word-wrap: break-word; }
th { color: #16365C; font-family: cnb; padding: 3.5pt 3pt;
     border: 0.5pt solid #C9D6E4; border-bottom: 1.1pt solid #16365C; text-align: center; }
td { padding: 3pt; border: 0.5pt solid #C9D6E4; text-align: center; }
td:first-child { text-align: left; }
pre { font-family: mono; font-size: 7.6pt; line-height: 1.42; background: #F6F8FA;
      border-left: 2.5pt solid #9DB6D0; padding: 6pt 8pt; margin: 5pt 0 9pt 0;
      white-space: pre-wrap; }
code { font-family: mono; font-size: 8.4pt; color: #1F4E79; }
blockquote { background: #F4F8FC; border-left: 3pt solid #2E75B6; padding: 7pt 10pt;
             margin: 6pt 0 9pt 0; }
blockquote p { text-indent: 0; margin: 0; font-size: 9pt; }
img { margin: 4pt 0 2pt 0; }
.cap { font-size: 8pt; color: #595959; text-align: center; margin: 0 0 10pt 0; }
.ct { font-family: cnb; font-size: 10pt; color: #2E75B6; margin: 0 0 6pt 0; }
.h1c { font-family: cnb; font-size: 25pt; color: #16365C; margin: 0 0 8pt 0;
       line-height: 1.28; }
.sub { font-size: 10.5pt; color: #595959; margin: 0 0 14pt 0; line-height: 1.5; }
.rule { border-bottom: 2pt solid #16365C; margin: 0 0 14pt 0; }
.rule2 { border-bottom: 0.6pt solid #C9D6E4; margin: 10pt 0; }
.mt { font-size: 8.4pt; line-height: 1.55; }
.mt td { border: none; padding: 2pt 0; text-align: left; }
.mt td.k { color: #2E75B6; font-family: cnb; width: 21%; }
.mt td.v { color: #595959; }
.mline { font-size: 8.6pt; line-height: 1.5; color: #595959; text-indent: 0; margin: 0 0 3pt 0; }
.mline .k { font-family: cnb; color: #2E75B6; }
"""


def pdf_cover():
    if COVER_FIG:
        fig = ('<p style="text-align:center;text-indent:0;margin-top:8pt">'
               f'<img src="{COVER_FIG}" style="width:96%"></p>'
               f'<p class="cap">{COVER_CAP}</p>')
    else:
        fig = ""
    rows = "".join(
        f'<p class="mline"><span class="k">{k}</span>　{html_inline(v).replace(chr(10), "<br/>")}</p>'
        for k, v in META)
    return f"""
    <div class="rule"></div>
    <p class="ct">{COVER_KICKER}</p>
    <p class="h1c">{COVER_TITLE_HTML}</p>
    <p class="sub">{SUBTITLE}</p>
    <p class="sub" style="color:#2E75B6;font-family:cnb;font-size:9.6pt">
      {COVER_TAGS}</p>
    <div class="rule2"></div>
    {rows}
    <div class="rule2"></div>
    <p style="font-family:cnb;font-size:9.4pt;color:#16365C;text-indent:0">
      {COVER_LEAD}</p>
    {fig}
    """


def to_html(blocks):
    out = []
    for b in blocks:
        kind = b[0]
        if kind in ("h1", "h2", "h3", "h4"):
            tag = {"h1": "h1", "h2": "h1", "h3": "h2", "h4": "h3"}[kind]
            out.append(f"<{tag}>{html_inline(b[1])}</{tag}>")
        elif kind == "p":
            longtok = re.search(r"[A-Za-z0-9_./:-]{25,}", b[1])
            cls = ' class="url"' if ("http" in b[1] or longtok) else ""
            out.append(f"<p{cls}>{html_inline(b[1])}</p>")
        elif kind == "ul":
            out.append("<ul>" + "".join(f"<li>{html_inline(x)}</li>" for x in b[1]) + "</ul>")
        elif kind == "ol":
            start = b[2] if len(b) > 2 else 1
            attr = f' start="{start}"' if start != 1 else ""
            out.append(f"<ol{attr}>" + "".join(f"<li>{html_inline(x)}</li>" for x in b[1]) + "</ol>")
        elif kind == "table":
            headers, rows = b[1], b[2]
            ncol = len(headers)
            rows = [r + [""] * (ncol - len(r)) if len(r) < ncol else r[:ncol] for r in rows]
            th = "".join(f"<th>{html_inline(h)}</th>" for h in headers)
            trs = "".join("<tr>" + "".join(f"<td>{html_inline(c)}</td>" for c in r) + "</tr>"
                          for r in rows)
            ws = col_widths(headers, rows)
            total = sum(ws)
            cols = "".join(f"<col style='width:{w / total * 100:.1f}%'/>" for w in ws)
            out.append(f"<table><colgroup>{cols}</colgroup>"
                       f"<thead><tr>{th}</tr></thead><tbody>{trs}</tbody></table>")
        elif kind == "code":
            txt = b[2].replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            out.append(f"<pre>{txt}</pre>")
        elif kind == "quote":
            out.append(f"<blockquote><p>{html_inline(b[1])}</p></blockquote>")
        elif kind == "fig":
            out.append(f'<p style="text-align:center;text-indent:0">'
                       f'<img src="{b[1]}" style="width:88%"></p>'
                       f'<p class="cap">{html_inline(b[2])}</p>')
        elif kind == "hr":
            out.append('<hr/>')
    return "\n".join(out)


def html_inline(text):
    parts = []
    for txt, bold, code, italic in parse_inline(text):
        t = txt.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        if code:
            parts.append(f"<code>{t}</code>")
        elif bold:
            parts.append(f"<b>{t}</b>")
        elif italic:
            parts.append(f"<i>{t}</i>")
        else:
            parts.append(t)
    return "".join(parts)


def build_pdf(blocks):
    import pymupdf
    arch = pymupdf.Archive("C:/Windows/Fonts")
    arch.add(".")                                  # 让 <img src="paper/..."> 可解析
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), "_piper_report_raw.pdf")
    writer = pymupdf.DocumentWriter(tmp)
    mb = pymupdf.paper_rect("a4")
    where = mb + (64, 62, -64, -58)

    def run(html, fresh_page=False):
        story = pymupdf.Story(html=html, user_css=CSS, archive=arch)
        more = 1
        first = True
        while more:
            if not (fresh_page and first):
                pass
            dev = writer.begin_page(mb)
            more, _ = story.place(where)
            story.draw(dev)
            writer.end_page()
            first = False

    run("<html><body>" + pdf_cover() + "</body></html>")          # 封面独占一页
    seg = []
    for b in blocks:
        if b[0] == "pagebreak":
            if seg:
                run("<html><body>" + to_html(seg) + "</body></html>")
            seg = []
        else:
            seg.append(b)
    if seg:
        run("<html><body>" + to_html(seg) + "</body></html>")
    writer.close()
    d = pymupdf.open(tmp)
    try:
        d.subset_fonts()
    except Exception as e:
        print("subset skipped:", e)
    # 页码
    for i in range(1, d.page_count):
        pg = d[i]
        txt = f"— {i} —"
        pg.insert_text((pg.rect.width / 2 - len(txt) * 2.0, pg.rect.height - 30),
                       txt, fontname="helv", fontsize=8.5,
                       color=(0.35, 0.35, 0.35))
    d.save(PDF_OUT, garbage=4, deflate=True)
    print("pages:", d.page_count)
    d.close()
    try:
        os.remove(tmp)
    except OSError:
        pass
    print("pdf:", PDF_OUT, round(os.path.getsize(PDF_OUT) / 1024 / 1024, 2), "MB")


# 封面图（无则留空）
COVER_KICKER = "复 现 说 明 报 告"
COVER_TITLE_DOCX = "在 PiPER 标准版上\n复现 FACTR 2"
COVER_TITLE_HTML = "在 PiPER 标准版上<br/>复现 FACTR 2"
COVER_FIG = "paper/figures/doc/fig01_overview.jpg"
COVER_CAP = ("论文图 1：NEXT 用 10 分钟自由运动数据在无外力传感器的低成本机械臂上估计外部关节力矩；"
             "FIRST 再用该信号把示教切成自由运动 / 接触前 / 接触三段并重采样。")
COVER_TAGS = "NEXT 神经外部力矩估计　·　FIRST 力信息驱动重采样　·　自由空间 L1 目标 ≤ 0.05 N·m"
COVER_LEAD = ("本报告回答三个问题：这台机器能复现到哪一层？每一步怎么验收？"
              "论文的数字为什么在你这里不会一模一样？")


def use_profile(name):
    """切换生成目标：report（复现报告）、piper（官方参数）或 guide（复现指南）。"""
    global SRC, DOCX_OUT, PDF_OUT, TITLE, SUBTITLE, HEADER, META, COVER_FIG, COVER_CAP
    global COVER_KICKER, COVER_TITLE_DOCX, COVER_TITLE_HTML, COVER_TAGS, COVER_LEAD
    if name == "guide":
        SRC = "paper/repro_guide.md"
        DOCX_OUT = "PiPER_FACTR2_复现指南.docx"
        PDF_OUT = "PiPER_FACTR2_复现指南.pdf"
        TITLE = "PiPER · FACTR 2 复现操作指南"
        SUBTITLE = "按 L0–L4 逐级推进：每一步做什么、拿什么验收、卡住查哪里"
        HEADER = "PiPER · FACTR 2 复现操作指南"
        COVER_FIG, COVER_CAP = None, ""
        COVER_KICKER = "复 现 操 作 指 南"
        COVER_TITLE_DOCX = "在 PiPER 上\n把力觉跑通"
        COVER_TITLE_HTML = "在 PiPER 上<br/>把力觉跑通"
        COVER_TAGS = "L0 信号链　·　力矩尺度核验　·　NEXT 训练　·　接触验证　·　FIRST 策略"
        COVER_LEAD = ("这本册子只讲执行：每一阶段做什么、拿什么验收、卡住查哪里；"
                      "论证与证据边界见《复现实施报告》。")
        META[:] = [
            ("配套文档", "《PiPER 标准版 FACTR 2 复现实施报告》修订版 2.0"),
            ("适用平台", "AgileX PiPER 标准版：6 DoF，1.5 kg 负载，\n无关节力矩传感器，CAN 1 Mbps"),
            ("文档定位", "执行手册：阶段任务、命令、验收门与故障处理；\n论证与证据边界见实施报告"),
            ("使用前提", "已通读实施报告第 4 至 9 章；本册只讲怎么做"),
            ("版本日期", "2026 年 10 月 19 日"),
        ]
        return
    if name == "piper":
        SRC = "piper_docs/PiPER_标准版_官方参数与使用说明.md"
        DOCX_OUT = "piper_docs/PiPER_标准版_官方参数与使用说明.docx"
        PDF_OUT = "piper_docs/PiPER_标准版_官方参数与使用说明.pdf"
        SUBTITLE = "AgileX PiPER 标准版：完整技术参数、接口定义、上电流程、SDK 与 CAN 协议速查"
        HEADER = "PiPER 标准版 · 官方参数与使用说明"
        COVER_FIG, COVER_CAP = None, ""
        COVER_KICKER = "官 方 资 料 · 中 文 整 理"
        COVER_TITLE_DOCX = "PiPER 标准版\n参数与使用说明"
        COVER_TITLE_HTML = "PiPER 标准版<br/>参数与使用说明"
        COVER_TAGS = "完整规格　·　接口定义　·　上电流程　·　SDK 与 CAN 协议速查"
        COVER_LEAD = ("官方资料的中文重排：查参数不必翻英文手册，"
                      "标注「工程建议」处为复现经验补充，非厂家原文。")
        META[:] = [
            ("机　　型", "AgileX Robotics PiPER 标准版（6 自由度轻量机械臂）"),
            ("整理依据", "官方产品页、快速上手手册 V1.0、用户手册中文 V1.1.0、\nCAN 协议 API 手册 v1.5、官方 SDK 与 ROS 仓库"),
            ("抓取时间", "2026-10-18"),
            ("配套文件", "同目录下 3 份官方原始文件（PDF / XLSX）已存档，\n离线可查，无需联网"),
            ("文档性质", "官方资料的中文摘录与重排；标注「工程建议」处为复现经验补充"),
        ]


if __name__ == "__main__":
    if len(sys.argv) > 1:
        use_profile(sys.argv[1])
    blocks = parse_md(SRC)
    # 去掉文首 H1 与 meta blockquote（封面已单独绘制）
    while blocks and blocks[0][0] in ("h1", "quote", "hr", "p"):
        blocks.pop(0)
    build_docx(blocks)
    build_pdf(blocks)
