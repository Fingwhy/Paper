# -*- coding: utf-8 -*-
"""生成 FACTR 2 论文解析报告 (docx)。"""
import os
from docx import Document
from docx.shared import Pt, Cm, RGBColor, Emu
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.section import WD_SECTION
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
from PIL import Image

FIG = "paper/figures/doc"
OUT = "FACTR2_论文解析报告.docx"

NAVY = "16365C"
BLUE = "2E75B6"
GREY = "595959"
LIGHT = "EDF2F8"
LINE = "C9D6E4"
BOXBG = "F4F8FC"
TXT = "1A1A1A"

BODY_CN = "宋体"
BODY_EN = "Times New Roman"
HEAD_CN = "微软雅黑"
HEAD_EN = "Arial"


# ---------------------------------------------------------------- 基础工具
def set_run_font(run, size=10.5, bold=False, color=TXT, cn=BODY_CN, en=BODY_EN,
                 italic=False):
    run.font.size = Pt(size)
    run.bold = bold
    run.italic = italic
    run.font.color.rgb = RGBColor.from_string(color)
    run.font.name = en
    rPr = run._element.get_or_add_rPr()
    rFonts = rPr.find(qn("w:rFonts"))
    if rFonts is None:
        rFonts = OxmlElement("w:rFonts")
        rPr.append(rFonts)
    rFonts.set(qn("w:ascii"), en)
    rFonts.set(qn("w:hAnsi"), en)
    rFonts.set(qn("w:eastAsia"), cn)
    rFonts.set(qn("w:cs"), en)


def _p(doc, space_before=0, space_after=0, align=None, line=1.5, first_indent=None):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.space_before = Pt(space_before)
    pf.space_after = Pt(space_after)
    pf.line_spacing = line
    if align is not None:
        p.alignment = align
    if first_indent is not None:
        pf.first_line_indent = Pt(first_indent)
    return p


def set_shading(el, hexcolor):
    tcPr = el.get_or_add_tcPr() if hasattr(el, "get_or_add_tcPr") else el
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hexcolor)
    tcPr.append(shd)


TCPR_AFTER_SHD = ("w:noWrap", "w:tcMar", "w:textDirection", "w:tcFitText", "w:vAlign",
                  "w:hideMark", "w:headers", "w:cellIns", "w:cellDel", "w:cellMerge",
                  "w:tcPrChange")
TCPR_AFTER_BORDERS = ("w:shd",) + TCPR_AFTER_SHD
TBLPR_AFTER_BORDERS = ("w:shd", "w:tblLayout", "w:tblCellMar", "w:tblLook",
                       "w:tblCaption", "w:tblDescription", "w:tblPrChange")
TBLPR_AFTER_CELLMAR = ("w:tblLook", "w:tblCaption", "w:tblDescription", "w:tblPrChange")
PPR_AFTER_PBDR = ("w:shd", "w:tabs", "w:suppressAutoHyphens", "w:kinsoku", "w:wordWrap",
                  "w:overflowPunct", "w:topLinePunct", "w:autoSpaceDE", "w:autoSpaceDN",
                  "w:bidi", "w:adjustRightInd", "w:snapToGrid", "w:spacing", "w:ind",
                  "w:contextualSpacing", "w:mirrorIndents", "w:suppressOverlap", "w:jc",
                  "w:textDirection", "w:textAlignment", "w:textboxTightWrap",
                  "w:outlineLvl", "w:divId", "w:cnfStyle", "w:rPr", "w:sectPr",
                  "w:pPrChange")


def cell_bg(cell, hexcolor, after=TCPR_AFTER_SHD):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hexcolor)
    tcPr.insert_element_before(shd, *after)


def cell_left_bar(cell, color=BLUE, sz=24):
    tcPr = cell._tc.get_or_add_tcPr()
    borders = OxmlElement("w:tcBorders")
    left = OxmlElement("w:left")
    left.set(qn("w:val"), "single")
    left.set(qn("w:sz"), str(sz))
    left.set(qn("w:space"), "0")
    left.set(qn("w:color"), color)
    borders.append(left)
    tcPr.insert_element_before(borders, *TCPR_AFTER_BORDERS)


def add_p_bottom_border(p, color=BLUE, sz=10, space=4):
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), str(sz))
    bottom.set(qn("w:space"), str(space))
    bottom.set(qn("w:color"), color)
    pbdr.append(bottom)
    pPr.insert_element_before(pbdr, *PPR_AFTER_PBDR)


def cell_margins(table, top=60, bottom=60, left=100, right=100):
    tblPr = table._tbl.tblPr
    mar = OxmlElement("w:tblCellMar")
    for k, v in (("top", top), ("left", left), ("bottom", bottom), ("right", right)):
        e = OxmlElement(f"w:{k}")
        e.set(qn("w:w"), str(v))
        e.set(qn("w:type"), "dxa")
        mar.append(e)
    tblPr.insert_element_before(mar, *TBLPR_AFTER_CELLMAR)


def table_borders(table, color=LINE, sz=6, inside=True):
    tblPr = table._tbl.tblPr
    borders = OxmlElement("w:tblBorders")
    names = ["top", "left", "bottom", "right"]
    if inside:
        names += ["insideH", "insideV"]
    for name in names:
        e = OxmlElement(f"w:{name}")
        e.set(qn("w:val"), "single")
        e.set(qn("w:sz"), str(sz))
        e.set(qn("w:space"), "0")
        e.set(qn("w:color"), color)
        borders.append(e)
    tblPr.insert_element_before(borders, *TBLPR_AFTER_BORDERS)


def no_borders(table):
    tblPr = table._tbl.tblPr
    borders = OxmlElement("w:tblBorders")
    for name in ["top", "left", "bottom", "right", "insideH", "insideV"]:
        e = OxmlElement(f"w:{name}")
        e.set(qn("w:val"), "none")
        e.set(qn("w:sz"), "0")
        e.set(qn("w:space"), "0")
        borders.append(e)
    tblPr.insert_element_before(borders, *TBLPR_AFTER_BORDERS)


def cell_text(cell, text, size=9.5, bold=False, color=TXT, align=WD_ALIGN_PARAGRAPH.CENTER,
              cn=BODY_CN, en=BODY_EN, line=1.15):
    cell.text = ""
    p = cell.paragraphs[0]
    p.alignment = align
    p.paragraph_format.space_before = Pt(1)
    p.paragraph_format.space_after = Pt(1)
    p.paragraph_format.line_spacing = line
    r = p.add_run(text)
    set_run_font(r, size=size, bold=bold, color=color, cn=cn, en=en)


def add_table(doc, headers, rows, widths, header_fill=NAVY, size=9.2,
              first_col_left=True, zebra=True, highlight_last=False):
    t = doc.add_table(rows=1, cols=len(headers))
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    table_borders(t)
    cell_margins(t)
    hdr = t.rows[0]
    trPr = hdr._tr.get_or_add_trPr()
    th = OxmlElement("w:tblHeader")
    trPr.append(th)
    for i, h in enumerate(headers):
        c = hdr.cells[i]
        c.width = Cm(widths[i])
        cell_bg(c, header_fill)
        cell_text(c, h, size=size, bold=True, color="FFFFFF", cn=HEAD_CN, en=HEAD_EN)
    for ri, row in enumerate(rows):
        tr = t.add_row()
        cells = tr.cells
        trPr = tr._tr.get_or_add_trPr()
        trPr.append(OxmlElement("w:cantSplit"))
        for ci, val in enumerate(row):
            cells[ci].width = Cm(widths[ci])
            al = WD_ALIGN_PARAGRAPH.LEFT if (ci == 0 and first_col_left) else WD_ALIGN_PARAGRAPH.CENTER
            is_last = highlight_last and ri == len(rows) - 1
            bold = is_last
            cell_text(cells[ci], val, size=size, bold=bold,
                      color=NAVY if is_last else TXT, align=al)
            if is_last:
                cell_bg(cells[ci], "E4EEF9")
            elif zebra and ri % 2 == 1:
                cell_bg(cells[ci], "F7FAFD")
        # 整表尽量不跨页：所有行与下一行（及表后的图注）保持同页
        for c in cells:
            for p in c.paragraphs:
                p.paragraph_format.keep_with_next = True
    return t


def vmerge(doc_table, col, a, b, text, size=9.0, bold=False, color=TXT):
    """纵向合并 a..b 行的第 col 列，并写入居中文字。"""
    from docx.enum.table import WD_ALIGN_VERTICAL
    merged = doc_table.cell(a, col).merge(doc_table.cell(b, col))
    for p in merged.paragraphs[1:]:
        p._p.getparent().remove(p._p)
    first = merged.paragraphs[0]
    for r in list(first.runs):
        r._r.getparent().remove(r._r)
    first.alignment = WD_ALIGN_PARAGRAPH.CENTER
    first.paragraph_format.space_before = Pt(1)
    first.paragraph_format.space_after = Pt(1)
    r = first.add_run(text)
    set_run_font(r, size=size, bold=bold, color=color)
    merged.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
    return merged


def heading(doc, text, level=1, number=None):
    if level == 1:
        p = _p(doc, space_before=20, space_after=9, line=1.2)
        r = p.add_run(text)
        set_run_font(r, size=15, bold=True, color=NAVY, cn=HEAD_CN, en=HEAD_EN)
        # 下方细线
        add_p_bottom_border(p, BLUE, 10, 4)
    elif level == 2:
        p = _p(doc, space_before=14, space_after=6, line=1.2)
        r = p.add_run(text)
        set_run_font(r, size=12, bold=True, color=NAVY, cn=HEAD_CN, en=HEAD_EN)
    else:
        p = _p(doc, space_before=10, space_after=4, line=1.2)
        r = p.add_run(text)
        set_run_font(r, size=10.8, bold=True, color=BLUE, cn=HEAD_CN, en=HEAD_EN)
    p.style = doc.styles["Normal"]
    p.paragraph_format.keep_with_next = True
    p.paragraph_format.keep_together = True
    return p


def body(doc, text, indent=True, space_after=6, size=10.5, color=TXT):
    p = _p(doc, space_before=0, space_after=space_after, line=1.55,
           align=WD_ALIGN_PARAGRAPH.JUSTIFY,
           first_indent=(size * 2) if indent else None)
    r = p.add_run(text)
    set_run_font(r, size=size, color=color)
    return p


def rich(doc, chunks, indent=True, space_after=6, size=10.5):
    """chunks: list of (text, bold)"""
    p = _p(doc, space_before=0, space_after=space_after, line=1.55,
           align=WD_ALIGN_PARAGRAPH.JUSTIFY,
           first_indent=(size * 2) if indent else None)
    for txt, bold in chunks:
        r = p.add_run(txt)
        set_run_font(r, size=size, bold=bold, color=TXT)
    return p


def bullet(doc, text, bold_prefix=None, size=10.5):
    p = _p(doc, space_before=0, space_after=4, line=1.5)
    p.paragraph_format.left_indent = Pt(18)
    p.paragraph_format.first_line_indent = Pt(-10)
    r0 = p.add_run("▪ ")
    set_run_font(r0, size=size - 1, color=BLUE)
    if bold_prefix:
        rb = p.add_run(bold_prefix)
        set_run_font(rb, size=size, bold=True, color=NAVY)
    r = p.add_run(text)
    set_run_font(r, size=size, color=TXT)
    return p


def formula(doc, chunks, number=None):
    """chunks: list of (text, kind) kind in {'', 'i', 'sub'}"""
    p = _p(doc, space_before=6, space_after=8, line=1.35, align=WD_ALIGN_PARAGRAPH.CENTER)
    for ch in chunks:
        txt, kind = (ch, "") if isinstance(ch, str) else ch
        r = p.add_run(txt)
        if kind == "sub":
            r.font.size = Pt(8)
            r.font.subscript = True
            r.font.color.rgb = RGBColor.from_string(TXT)
            r.font.name = BODY_EN
        elif kind == "i":
            set_run_font(r, size=10.5, italic=True, color=TXT)
        else:
            set_run_font(r, size=10.5, color=TXT)
    if number:
        rn = p.add_run("      (" + str(number) + ")")
        set_run_font(rn, size=9.5, color=GREY)
    return p


def caption(doc, text, size=8.8):
    p = _p(doc, space_before=4, space_after=12, line=1.25, align=WD_ALIGN_PARAGRAPH.CENTER)
    r = p.add_run(text)
    set_run_font(r, size=size, color=GREY, cn=HEAD_CN, en=HEAD_EN)
    return p


def figure(doc, path, cap, width_cm=15.4):
    p = _p(doc, space_before=8, space_after=2, line=1.0, align=WD_ALIGN_PARAGRAPH.CENTER)
    p.paragraph_format.keep_with_next = True
    p.add_run().add_picture(path, width=Cm(width_cm))
    caption(doc, cap)


def callout(doc, title, text, fill=BOXBG, bar=BLUE):
    t = doc.add_table(rows=1, cols=1)
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    no_borders(t)
    cell = t.rows[0].cells[0]
    cell.width = Cm(16.0)
    cell_bg(cell, fill)
    cell_margins(t, top=140, bottom=140, left=260, right=200)
    # 左侧强调条
    cell_left_bar(cell, bar, 24)

    p0 = cell.paragraphs[0]
    p0.paragraph_format.space_after = Pt(3)
    p0.paragraph_format.line_spacing = 1.2
    if title:
        r0 = p0.add_run(title)
        set_run_font(r0, size=10.5, bold=True, color=NAVY, cn=HEAD_CN, en=HEAD_EN)
    p1 = cell.add_paragraph()
    p1.paragraph_format.space_after = Pt(0)
    p1.paragraph_format.line_spacing = 1.5
    p1.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    r1 = p1.add_run(text)
    set_run_font(r1, size=10, color=TXT)
    _p(doc, space_before=0, space_after=6)


def divider(doc):
    p = _p(doc, space_before=2, space_after=2, line=1.0)
    add_p_bottom_border(p, LINE, 4, 1)


def add_field(run, instr):
    fldChar1 = OxmlElement("w:fldChar")
    fldChar1.set(qn("w:fldCharType"), "begin")
    instrText = OxmlElement("w:instrText")
    instrText.set(qn("xml:space"), "preserve")
    instrText.text = instr
    fldChar2 = OxmlElement("w:fldChar")
    fldChar2.set(qn("w:fldCharType"), "end")
    run._r.append(fldChar1)
    run._r.append(instrText)
    run._r.append(fldChar2)


def img_w(path, width_cm):
    im = Image.open(path)
    return width_cm * im.height / im.width


# ------------------------------------------------ 数学符号字体修正
import copy

CJK_RANGES = ((0x4E00, 0x9FFF), (0x3400, 0x4DBF), (0x3000, 0x303F),
              (0xFF00, 0xFFEF), (0x2E80, 0x2EFF))
GREEK = set("θτΔΣασβγλμΩδφπ≈±‖·√²³⁻")


def _is_cjk(ch):
    o = ord(ch)
    return any(a <= o <= b for a, b in CJK_RANGES)


def _segments(text):
    segs, buf, cur = [], "", None
    for ch in text:
        k = _is_cjk(ch)
        if cur is None or k == cur:
            buf += ch
            cur = k
        else:
            segs.append((buf, cur))
            buf, cur = ch, k
    if buf:
        segs.append((buf, cur))
    return segs


def fix_math_fonts(doc):
    """让希腊字母等数学符号走西文字体，避免落到中文字体里变成怪字形。"""
    body = doc.element.body
    for r in list(body.iter(qn("w:r"))):
        rPr = r.find(qn("w:rPr"))
        if rPr is None:
            continue
        rFonts = rPr.find(qn("w:rFonts"))
        if rFonts is None:
            continue
        cn = rFonts.get(qn("w:eastAsia"))
        en = rFonts.get(qn("w:ascii")) or rFonts.get(qn("w:hAnsi"))
        if not cn or not en or cn == en:
            continue
        ts = r.findall(qn("w:t"))
        if len(ts) != 1:
            continue
        text = ts[0].text or ""
        if not any(ch in GREEK for ch in text):
            continue
        segs = _segments(text)
        if len(segs) <= 1:
            rFonts.set(qn("w:eastAsia"), en)
            continue
        parent = r.getparent()
        idx = parent.index(r)
        new_runs = []
        for txt, is_cjk in segs:
            nr = copy.deepcopy(r)
            t = nr.find(qn("w:t"))
            t.text = txt
            t.set(qn("xml:space"), "preserve")
            rf = nr.find(qn("w:rPr")).find(qn("w:rFonts"))
            if not is_cjk:
                rf.set(qn("w:eastAsia"), en)
            new_runs.append(nr)
        parent.remove(r)
        for i, nr in enumerate(new_runs):
            parent.insert(idx + i, nr)


# ---------------------------------------------------------------- 文档
doc = Document()
style = doc.styles["Normal"]
style.font.size = Pt(10.5)
style.font.name = BODY_EN
style.font.color.rgb = RGBColor.from_string(TXT)
style.element.rPr.rFonts.set(qn("w:eastAsia"), BODY_CN)

sec = doc.sections[0]
sec.page_width = Cm(21.0)
sec.page_height = Cm(29.7)
sec.top_margin = Cm(2.3)
sec.bottom_margin = Cm(2.1)
sec.left_margin = Cm(2.5)
sec.right_margin = Cm(2.5)
sec.header_distance = Cm(1.2)
sec.footer_distance = Cm(1.1)

# 页眉
hp = sec.header.paragraphs[0]
hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
hr = hp.add_run("FACTR 2 · 学习外部力感知，让普通机械臂学会接触")
set_run_font(hr, size=8, color="8C9AAB", cn=HEAD_CN, en=HEAD_EN)

# 页脚
fp = sec.footer.paragraphs[0]
fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
fr = fp.add_run("— ")
set_run_font(fr, size=8.5, color=GREY)
fr2 = fp.add_run()
set_run_font(fr2, size=8.5, color=GREY)
add_field(fr2, "PAGE")
fr3 = fp.add_run(" —")
set_run_font(fr3, size=8.5, color=GREY)

# ============================================================ 封面
_p(doc, space_before=2, space_after=0)
p = _p(doc, space_before=0, space_after=0, line=1.0)
add_p_bottom_border(p, NAVY, 18, 6)

p = _p(doc, space_before=12, space_after=4, line=1.2)
r = p.add_run("论 文 解 析 报 告")
set_run_font(r, size=10, bold=True, color=BLUE, cn=HEAD_CN, en=HEAD_EN)

p = _p(doc, space_before=4, space_after=4, line=1.12)
r = p.add_run("FACTR 2：让普通机械臂\n「学会」感知外部力")
set_run_font(r, size=27, bold=True, color=NAVY, cn=HEAD_CN, en=HEAD_EN)

p = _p(doc, space_before=2, space_after=2, line=1.3)
r = p.add_run("Learning External Force Sensing for Commodity Robot Arms\nImproves Policy Learning")
set_run_font(r, size=11.5, color=GREY, cn=HEAD_CN, en="Georgia", italic=True)

p = _p(doc, space_before=12, space_after=0, line=1.35)
r = p.add_run("NEXT：神经外部力矩估计   ·   FIRST：力信息驱动的重采样训练")
set_run_font(r, size=10.5, bold=True, color=BLUE, cn=HEAD_CN, en=HEAD_EN)

_p(doc, space_before=4, space_after=0)
p = _p(doc, space_before=0, space_after=0, line=1.0)
add_p_bottom_border(p, LINE, 6, 4)

figure(doc, os.path.join(FIG, "fig01_overview.jpg"),
       "论文图 1：方法总览。(a) NEXT 仅用 10 分钟自由运动数据，即可在无外力传感器的低成本机械臂"
       "（Piper、YAM、Nero）上估计外部关节力矩，支撑力反馈遥操作；(b) FIRST 用学到的力矩把示教切成"
       "自由空间 / 接触前 / 接触三段，并在训练时对接触相关片段重采样。", width_cm=14.2)

_p(doc, space_before=4, space_after=0)
meta = [
    ("作者", "Steven Oh*、Jason Jingzhou Liu*、Tony Tao*、Philip Han、Kenneth Shaw、\n"
             "Satoshi Funabashi、Ruslan Salakhutdinov、Deepak Pathak（*共同第一作者）"),
    ("机构", "卡内基梅隆大学（CMU）  ·  早稻田大学（Waseda University）"),
    ("来源", "arXiv:2606.12406v2  [cs.RO]；cs.AI、cs.LG、eess.SY   ·   2026-06-10 投稿，2026-08-12 修订"),
    ("项目页", "https://jasonjzliu.com/factr2"),
]
t = doc.add_table(rows=0, cols=2)
t.autofit = False
no_borders(t)
cell_margins(t, top=40, bottom=40, left=0, right=80)
for k, v in meta:
    row = t.add_row().cells
    row[0].width = Cm(2.2)
    row[1].width = Cm(13.4)
    cell_text(row[0], k, size=9, bold=True, color=GREY, align=WD_ALIGN_PARAGRAPH.LEFT,
              cn=HEAD_CN, en=HEAD_EN)
    cell_text(row[1], v, size=9, color=GREY, align=WD_ALIGN_PARAGRAPH.LEFT)

doc.add_page_break()

# ============================================================ 目录
heading(doc, "目  录", 1)
toc = [
    ("一", "论文速览：一句话结论与三条贡献", ""),
    ("二", "研究背景：为什么「力」这么贵", ""),
    ("三", "方法总览：FACTR 2 = NEXT + FIRST", ""),
    ("3.1", "NEXT：用 10 分钟数据学出「自由空间力矩」", ""),
    ("3.2", "FIRST：按接触相位重塑训练分布", ""),
    ("四", "实验与结果", ""),
    ("4.1", "外部力矩估计精度：接近甚至优于专用传感器", ""),
    ("4.2", "力反馈遥操作：20 人用户研究", ""),
    ("4.3", "模仿学习实验设置", ""),
    ("4.4", "主结果：五个长时序任务全面提升", ""),
    ("4.5", "消融：接触前比接触中更值得重采样", ""),
    ("4.6", "为什么有效：相位级验证损失", ""),
    ("4.7", "上采样倍数与逐任务配比", ""),
    ("4.8", "额外的三组对照实验", ""),
    ("五", "关键结论与可迁移的洞见", ""),
    ("六", "局限与作者自述的边界", ""),
    ("七", "复现要点：给动手实现者的清单", ""),
    ("八", "交付文件清单与延伸阅读", ""),
]
for num, title, _ in toc:
    p = _p(doc, space_before=3, space_after=3, line=1.35)
    p.paragraph_format.left_indent = Pt(0 if "." not in num else 26)
    r = p.add_run(num + "　")
    set_run_font(r, size=10.5 if "." not in num else 10,
                 bold="." not in num, color=NAVY if "." not in num else BLUE,
                 cn=HEAD_CN, en=HEAD_EN)
    r2 = p.add_run(title)
    set_run_font(r2, size=10.5 if "." not in num else 10,
                 bold="." not in num, color=TXT,
                 cn=HEAD_CN if "." not in num else BODY_CN,
                 en=HEAD_EN if "." not in num else BODY_EN)

_p(doc, space_before=10, space_after=0)
callout(doc, "阅读提示",
        "本报告基于 arXiv 官方 PDF（22 页，含附录）逐节整理。报告中出现的数值均取自原文表格或正文；"
        "凡属「由图读数」的近似值均已明确标注。第七节结合复现视角补充了原文未展开的工程细节与风险点，"
        "属于笔者的分析，请与原论文区分。")

doc.add_page_break()

# ============================================================ 一
heading(doc, "一、论文速览：一句话结论与三条贡献", 1)

callout(doc, "一句话总结",
        "FACTR 2 用「学出来的外部关节力矩」替代昂贵的力/力矩传感器：NEXT 只需 10 分钟自由运动数据、"
        "1 分钟训练，就能让没有力矩传感器的机械臂获得接近专用传感器的外力估计；再用该信号把示教轨迹"
        "切成自由运动 / 接触前 / 接触三段并按相位重采样（FIRST），在 5 个长时序接触任务上把任务进度"
        "相对此前最强的力感知策略再提升约 17 个百分点。")

body(doc, "接触富操作（插装、拧紧、柔性物体操作）高度依赖力觉，但力/力矩传感器长期以来只存在于高端"
          "平台。这篇论文的野心不是造更好的传感器，而是把「力感知」变成一段可以在任何机械臂上运行的"
          "软件：先估计，再使用。")

heading(doc, "三条核心贡献", 2)
bullet(doc, "提出 NEXT（Neural External Torque Estimation），一种无需专用力传感器、也无需机器人"
            "专属系统辨识的外部关节力矩估计方法。它只在 10 分钟自由运动数据上训练一个循环网络，"
            "估计质量超过此前方法。",
       bold_prefix="数据驱动的无力传感器外部力矩估计。")
bullet(doc, "把估计出的力信号接入双边遥操作，使 Piper、YAM 这类没有力矩传感器的低成本机械臂也能做"
            "力反馈遥操作，反馈质量与带传感器的系统相当。",
       bold_prefix="低成本机械臂上的力反馈遥操作。")
bullet(doc, "提出 FIRST（Force-Informed Re-Sampling Training）：用力矩估计识别示教中的接触前与接触"
            "阶段，在行为克隆训练时提高这些阶段样本的采样概率，从而提升策略在接触富任务上的表现。",
       bold_prefix="力信息驱动的重采样训练。")

_p(doc, space_before=4)
add_table(doc,
          ["维度", "内容"],
          [["问题", "低成本机械臂缺少力感知，无法采集力反馈、也无法学习力感知策略"],
           ["关键思路", "不去测量力，而是「学掉」自由空间动力学，残差即外力"],
           ["样本效率", "10 分钟自由运动数据 + 1 分钟训练（单张 RTX 3090）"],
           ["部署成本", "1.76 ms / 次前向（单核 CPU，≈568 Hz），实际以 100 Hz 运行"],
           ["硬件跨度", "AgileX Piper（约 2,500 美元）→ Franka（约 30,000 美元）"],
           ["主结果", "5 个接触富任务上任务进度相对最强基线 +17 pt 左右"]],
          widths=[3.0, 12.6], size=9.2, first_col_left=False)
caption(doc, "表 1　论文核心信息速览")

# ============================================================ 二
heading(doc, "二、研究背景：为什么「力」这么贵", 1)

body(doc, "人类做插装、拧紧、擦拭这类动作时几乎不需要思考，靠的是手指上的力觉反馈；而让机器人仅凭视觉"
          "推断这些细微的接触状态极其困难。因此力觉常被视为接触富操作的必要能力。问题在于，专用的"
          "六维力/力矩传感器（ATI Gamma、Bota 等）通常只出现在 Franka、Flexiv、KUKA 这类高价平台上。")

body(doc, "论文给出了一个很直观的成本对比：AgileX Piper 整机约 2,500 美元，而 Franka 约 30,000 美元；"
          "给末端加装力传感器的改装成本有时甚至超过机械臂本身。技术路线上也不轻松——传统压阻式传感器"
          "需要在精密加工的弹性体上贴多组应变片，笨重且昂贵；较新的电容式（CoinFT）与磁性触觉皮肤"
          "（ReSkin）虽然便宜，但其软材料并非为末端常见的大力值设计。")

heading(doc, "既有的「无力传感器」路线及其瓶颈", 2)
body(doc, "既然装不起传感器，就有人尝试从电机电流反推外力。这条路线的基本关系是 τm = K·Im（电机力矩"
          "≈ 力矩常数 × 电流），再用动力学模型算出「自由空间本应需要的力矩」，两者相减即视为外力。"
          "论文把这条路线的问题讲得很清楚：")
bullet(doc, "需要准确的质量矩阵 M、科氏/离心项 C、重力项 g 以及摩擦等其余项；而只用电机侧量测时，"
            "惯性、重力、摩擦参数的辨识本身就很难。",
       bold_prefix="解析建模 + 系统辨识：")
bullet(doc, "残差里不只包含接触力矩，还混入了非线性摩擦、静摩擦、背隙、迟滞、温漂、传感噪声、"
            "力矩纹波、死区与饱和等未建模效应，且其中很多是「历史相关」的，仅凭瞬时关节状态无法解释。",
       bold_prefix="残差并不干净：")
bullet(doc, "基于自编码器的异常检测只能判断「有没有碰撞」，难以给出连续的外力估计；监督式方法则"
            "需要额外采集并标注接触数据。",
       bold_prefix="学习式方法的局限：")

body(doc, "这正是 NEXT 的切入点：与其费力辨识物理参数，不如直接用一段带时序的网络把「自由空间力矩」"
          "整体学掉——包括那些写不进刚体动力学方程的硬件特性。")

# ============================================================ 三
heading(doc, "三、方法总览：FACTR 2 = NEXT + FIRST", 1)

body(doc, "整个系统分两步，各自解决一个独立的问题：第一步 NEXT 回答「没有力传感器，力信号从哪来」；"
          "第二步 FIRST 回答「有了力信号，怎么让它真正提升策略」。FACTR 2 相对前作 FACTR 的关键跃迁，"
          "是把适用范围从「自带力矩传感器的机械臂」扩展到「任何现成机械臂」。")

heading(doc, "3.1　NEXT：用 10 分钟数据学出「自由空间力矩」", 2)

heading(doc, "核心恒等式与它的困难", 3)
body(doc, "外部关节力矩定义为：由与环境物理交互在关节空间产生的力矩，扣除机器人自由运动本身所需的"
          "力矩。在有专用力矩传感的机械臂上，这个量可以直接读到；没有传感器时，只能估计：")
formula(doc, [("τ", ""), ("ext", "sub"), (" = τ", ""), ("m", "sub"), (" − τ", ""), ("f", "sub"),
              (" ,    τ", ""), ("f", "sub"), (" = M(q)q̈ + C(q,q̇)q̇ + g(q) + τ", ""), ("other", "sub")],
        number=1)
body(doc, "式中 τf 是「若无外部接触，完成当前观测运动所需的力矩」。模型法的问题在上一节已经说明："
          "τf 的任何误差都会原样进入 τext。NEXT 的做法是把 τf 整个交给神经网络：")
formula(doc, [("τ", ""), ("f", "sub"), (" = f", ""), ("θ", "sub"), ("(x) ,    τ", ""), ("ext", "sub"),
              (" = τ", ""), ("m", "sub"), (" − f", ""), ("θ", "sub"), ("(x)")], number=2)
body(doc, "其中 fθ 是参数为 θ 的网络，x 是一段本体感知观测的历史。注意这里没有任何「系统辨识」步骤，"
          "也不需要负载标定或力传感器——这是它能被搬到廉价平台上的根本原因。")

heading(doc, "输入、监督与网络结构", 3)
body(doc, "训练集 Dfree 由「无外部接触」的机械臂轨迹构成。因为整段轨迹都没有接触，测到的电机力矩就等于"
          "自由空间力矩，于是可以直接拿 τm 当监督标签。输入构造为一段滑动窗口历史：")
formula(doc, [("x", ""), ("i", "sub"), (" = [ q", ""), ("i−H:i", "sub"), (" ,  q̇", ""),
              ("i−H:i", "sub"), (" ,  Δq", ""), ("d,i−H:i", "sub"), (" ]")], number=3)
body(doc, "q 为关节位置，q̇ 为关节速度，Δqd 为指令目标位置与当前位置之差（即跟踪误差）。跟踪误差这一项"
          "很关键：它隐含了控制器为了维持轨迹付出了多少额外力矩，等价于给网络提供了一个「执行器努力程度」"
          "的线索。训练用简单的 L2 回归：")
formula(doc, [("L(θ) = (1/N) Σ", ""), ("i=1..N", "sub"), (" ‖ f", ""), ("θ", "sub"),
              ("(x", ""), ("i", "sub"), (") − τ", ""), ("m,i", "sub"), (" ‖²", ""), ("2", "sub")],
        number=4)
add_table(doc,
          ["超参数", "取值", "超参数", "取值"],
          [["循环主干", "2 层 LSTM，隐层 128", "损失函数", "均方误差（MSE）"],
           ["预测头", "2 层 MLP，隐层 256", "优化器", "AdamW"],
           ["Dropout", "0.1", "基础学习率", "1 × 10⁻³"],
           ["历史长度 H", "50 步", "权重衰减", "1 × 10⁻⁶"],
           ["输入模态", "q, q̇, Δqd", "梯度裁剪", "1.0"],
           ["数据频率", "100 Hz", "早停", "验证损失，patience 20"],
           ["硬件", "RTX 3090 (24 GB)", "训练耗时", "约 1 分钟（含早停）"]],
          widths=[3.1, 5.0, 3.0, 4.6], size=8.8, first_col_left=True)
caption(doc, "表 2　NEXT 自由空间逆动力学网络的架构与训练超参数（原论文表 3）")

body(doc, "一个容易忽略的工程细节：网络并不在整条轨迹上维持隐藏状态，而是采用「无状态滑窗」形式——"
          "每个序列都重置隐藏状态。这样既能拿到近期运动上下文，又不会在长轨迹上累积隐藏状态漂移。"
          "部署时只要维护一个最近 N 帧观测的滚动缓冲区即可。")

figure(doc, os.path.join(FIG, "fig02_deployment.jpg"),
       "论文图 2：NEXT 的部署流程。先把各关节实测电流乘力矩常数 K 得到实测关节力矩，再用自由空间数据"
       "训练的 LSTM 预测自由空间力矩，两者相减即得外部关节力矩。", width_cm=13.4)

heading(doc, "数据采集：10 分钟的自由运动", 3)
body(doc, "论文给出的采集流程非常朴素，可以照着做：① 让每个关节在其行程范围内单独运动；② 让末端做"
          "涉及多关节联动的类笛卡尔运动；③ 慢速与快速动作各重复若干次，以覆盖速度相关动力学。这些轨迹"
          "可以由运动规划自动生成，也可以录制一次后在新的机器人上回放。图 3 展示了 Piper 上 10 分钟数据"
          "对关节位置误差、位置、速度与自由空间力矩的覆盖情况。")

figure(doc, os.path.join(FIG, "fig08_dataset_coverage.jpg"),
       "论文图 8：Piper 上用于训练 NEXT 的 10 分钟自由运动数据分布，覆盖六个关节的 Δq、q、q̇ 与 τf。",
       width_cm=13.0)

heading(doc, "3.2　FIRST：按接触相位重塑训练分布", 2)

body(doc, "作者观察到一件很具体的事：接触富任务的失败并不均匀分布，而是集中在两处——接触发生前那一小段"
          "需要精确对位的时刻，以及接触过程中需要受控交互与细粒度修正的时刻。自由空间里的大幅运动反而"
          "很少出错。于是 FIRST 的思路是：不改策略结构、不改损失函数，只改「每个 batch 里抽到哪些样本」。")

heading(doc, "把轨迹切成三个阶段", 3)
body(doc, "用 NEXT 给出的外部力矩构造一个标量接触分数 ft = ‖τext,t‖₁。为了避开阈值附近的抖动，采用"
          "双阈值滞回：ft ≥ T_high 时判为接触，ft ≤ T_low 时解除，介于两者之间则保持上一时刻状态。"
          "接触起点就是状态从非接触翻转为接触的时刻：")
formula(doc, [("T", ""), ("onset", "sub"), (" = { t | c", ""), ("t−1", "sub"), (" = 0 ,  c", ""),
              ("t", "sub"), (" = 1 }")], number=6)
body(doc, "设控制频率为 F，则相位标签为：")
formula(doc, [("s", ""), ("t", "sub"), (" =  Contact (C)  若 c", ""), ("t", "sub"), (" = 1 ;    "
              "Pre-Contact (PC)  若存在 t", ""), ("onset", "sub"), (" ∈ T", ""), ("onset", "sub"),
              (" 使 t", ""), ("onset", "sub"), (" − F ≤ t < t", ""), ("onset", "sub"),
              (" ;    Free Space (F)  其余")], number=7)
body(doc, "也就是说，接触前阶段被定义为「接触起点前 1 秒」的窗口（当控制频率为 F 时即 F 个控制周期）。"
          "图 4 是这个自动分段的效果：自由运动段力矩贴近零，物理交互时明显抬升，分段与机器人的真实交互"
          "状态一致。整个过程不需要任何人工标注。")

figure(doc, os.path.join(FIG, "fig10_segmentation.jpg"),
       "论文图 10：用学到的外部力矩自动切分示教轨迹。归一化的 τext 把轨迹分为自由运动、接触前与接触"
       "三个阶段，图像叠加显示预测标签与机器人实际交互状态吻合。", width_cm=11.6)

heading(doc, "按相位加权采样", 3)
body(doc, "给三个阶段分别赋予采样权重 wF、wPC、wC，并要求 wPC、wC > wF。训练时每个数据点被抽到的概率为：")
formula(doc, [("p", ""), ("t", "sub"), (" = w(s", ""), ("t", "sub"), (") / Σ", ""), ("j=1..N", "sub"),
              (" w(s", ""), ("j", "sub"), (")")], number=9)
body(doc, "这就是全部机制——只改变 batch 的构成，回归目标本身一字未动。策略一侧的设定是：观测 "
          "ot =（M 路相机图像, 关节位置 qt, 估计外部力矩 τext,t），输出长度为 k 的动作块（论文取 30），"
          "用 flow matching 的速度场目标训练：")
formula(doc, [("L", ""), ("FM", "sub"), ("(θ) = E [ ‖ v", ""), ("θ", "sub"), ("(x", ""), ("α", "sub"),
              (", α, o", ""), ("t", "sub"), (") − (x", ""), ("1", "sub"), (" − x", ""), ("0", "sub"),
              (") ‖² ]")], number=5)
body(doc, "其中 x1 为专家动作块，x0 ~ N(0, I) 为噪声，xα = (1−α)x0 + αx1，α ~ U(0,1)。")

# ============================================================ 四
heading(doc, "四、实验与结果", 1)

heading(doc, "4.1　外部力矩估计精度：接近甚至优于专用传感器", 2)

body(doc, "评价分两种场景。接触场景在 Franka 上做——因为它自带关节力矩传感器与出厂标定的外部力矩"
          "估计，可以作为高质量参照真值；作者采集 5 分钟未见过的数据，由人施加外力。自由空间场景下真值"
          "恒为零，因此用零力矩作真值，同时把 Franka 传感器自身的读数也当作一个基线，用来衡量「专用"
          "传感器能有多准」。对比对象是 FILIC（基于 MuJoCo 逆动力学的残差法）与扰动观测器（DO，基于"
          "广义动量残差）。")

_t3 = add_table(doc,
                ["运动类型", "外部力矩方法", "L1 误差 (Nm) ↓"],
                [["接触", "外部传感器（真值）", "0.000 ± 0.000"],
                 ["", "FILIC", "4.395 ± 1.531"],
                 ["", "扰动观测器（DO）", "1.471 ± 0.761"],
                 ["", "NEXT（本文）", "0.547 ± 0.348"],
                 ["自由空间", "自由空间（真值）", "0.000 ± 0.000"],
                 ["", "FILIC", "2.460 ± 1.358"],
                 ["", "扰动观测器（DO）", "2.429 ± 1.298"],
                 ["", "外部传感器（Franka 自带）", "0.449 ± 0.208"],
                 ["", "NEXT（本文）", "0.414 ± 0.278"]],
                widths=[3.4, 7.4, 4.9], size=9.0)
vmerge(_t3, 0, 1, 4, "接触")
vmerge(_t3, 0, 5, 9, "自由空间")
caption(doc, "表 3　Franka 上的平均关节力矩 L1 误差（原论文表 1）。自由空间以零力矩为真值，接触段以 "
             "Franka 内置外部力矩估计为真值。")

rich(doc, [("接触段 ", False), ("0.547 Nm", True),
           (" 的平均误差，比 FILIC 低 ", False), ("87.6%", True), ("，比扰动观测器低 ", False),
           ("62.8%", True), ("。", False)])
rich(doc, [("自由空间更有意思：NEXT 的 ", False), ("0.414 Nm", True),
           (" 甚至优于 Franka 自带传感器（0.449 Nm）。原因在于 NEXT 是从这台硬件自己采的数据里直接"
            "学硬件特性，估计更平滑、更贴近零；而专用传感器仍有噪声。这一点在图 5 左半部分看得很清楚："
            "FILIC 与 DO 明显漂移，NEXT 则稳在零附近。", False)])

figure(doc, os.path.join(FIG, "fig05_torque_curves.jpg"),
       "论文图 5：左——自由空间外力应恒为零，NEXT 的估计比专用传感器更平滑且贴近零，FILIC 与扰动观测器"
       "明显漂移；右——接触段 NEXT 紧贴传感器信号，两个基线在接触切换处偏差很大。", width_cm=12.6)

body(doc, "低成本平台上，由于 Piper 没有外部力矩传感，无法重复接触段评估，作者只做自由空间评估（真值"
          "为 0），结果见下表。0.018 Nm 的误差意味着在廉价机械臂上这个信号已经「足够干净」。")

add_table(doc,
          ["外部力矩方法", "误差 (Nm) ↓"],
          [["自由运动（真值）", "0.000 ± 0.000"],
           ["FILIC", "0.435 ± 0.225"],
           ["扰动观测器（DO）", "0.248 ± 0.132"],
           ["NEXT（本文）", "0.018 ± 0.012"]],
          widths=[8.0, 6.4], size=9.2, first_col_left=True)
caption(doc, "表 4　AgileX Piper 自由空间下的平均关节力矩 L1 误差（原论文表 5）。NEXT 误差比 FILIC 低约 "
             "95.9%。")

heading(doc, "4.2　力反馈遥操作：20 人用户研究", 2)

body(doc, "力矩估计准不准，最终要看「人用起来是否更省力」。作者在 Franka 上做擦拭（wiping）任务，"
          "招募 20 名被试，对比 5 种反馈条件：无反馈、基于扰动观测器的反馈、主从位置-位置反馈（PP）、"
          "FACTR Teleop（用 Franka 自带力矩传感器）、以及 FACTR Teleop + NEXT 估计。两种 FACTR 变体"
          "共用同一条反馈控制律：")
formula(doc, [("τ", ""), ("feedback", "sub"), (" = K", ""), ("f,p", "sub"), (" · τ", ""),
              ("ext", "sub")], number=8)
body(doc, "即把从臂感受到的外部关节力矩按增益 Kf,p 缩放后施加到主臂上；差别只在于 τext 的来源是专用"
          "传感器还是 NEXT。评价指标是被试的 1–5 分易用性评分，以及完成任务实际施加的关节力矩——"
          "施加力矩越低，说明对力的感知越好、多余发力越少。")

figure(doc, os.path.join(FIG, "fig04_user_study_franka.jpg"),
       "论文图 4：Franka 上的用户研究。基于 NEXT 的力反馈优于基线，且与使用专用力传感器的 "
       "FACTR Teleop 相当。", width_cm=9.6)

body(doc, "结论是：NEXT 反馈的易用性评分高于各基线，完成任务所需的关节力矩也更低，与使用专用力矩"
          "传感器的 FACTR Teleop 相当。也就是说，「估计出来的力」在人的感知层面已经够用了。作者随后在"
          "Piper 上重复了同一研究（去掉 FACTR Teleop 一项，因为 Piper 没有专用力矩传感），NEXT 同样"
          "优于各基线。")

figure(doc, os.path.join(FIG, "fig09_user_study_piper.jpg"),
       "论文图 9：Piper 上各外部力矩反馈方法的用户研究结果。", width_cm=9.0)

heading(doc, "4.3　模仿学习实验设置", 2)

body(doc, "策略学习部分在 5 个真实世界的长时序、接触富任务上评估，覆盖从双臂灵巧手到双臂夹爪的不同"
          "形态：")
bullet(doc, "双臂灵巧手把两个小乐高块装配到大乐高块上。", bold_prefix="LEGO 装配（LEAP Hand V2）：")
bullet(doc, "双臂夹爪把两条皮带绕过两组滑轮。", bold_prefix="NIST 皮带装配：")
bullet(doc, "把两组滑轮组插到 NIST 板的销钉上。", bold_prefix="NIST 插装：")
bullet(doc, "先插入螺丝刀，再完成盒体装配。", bold_prefix="工具归位（Tool Clean Up）：")
bullet(doc, "抓取瓶盖、对位、拧紧直到到位。", bold_prefix="拧瓶盖（Cap Screwing）：")

figure(doc, os.path.join(FIG, "fig03_tasks.jpg"),
       "论文图 3：FIRST 评估用的五个长时序接触富任务。每个任务由多个阶段组成，多数需要精确对位或"
       "细粒度的力敏感调整。", width_cm=15.6)

body(doc, "每个任务在双臂 Piper 上采集 250 条示教，使用 4 路 RGB 相机（2 路腕部 + 2 路顶视）。每个策略"
          "跑 20 次 rollout，报告任务进度（各阶段完成率），而非二值的成功率——这对多阶段任务更能反映"
          "「做到哪一步」。策略实现上，所有方法共享同一套架构与行为克隆目标，以便公平对比：图像用 "
          "DINOv3-Base 编码，本体感知与力输入经 MLP 投成 token，动作块长度 30，flow matching 训练 20 个"
          "epoch（架构改编自多模态 Diffusion Transformer），另在 ACT 上重复全部实验。")

add_table(doc,
          ["基线", "说明"],
          [["Base Policy", "仅图像 + 关节位置，无任何力信息"],
           ["Base + Torque", "在 Base 上把 NEXT 估计的外部力矩作为额外输入"],
           ["FACTR", "使用外部力矩，并在训练时按噪声调度对输入图像做模糊（视觉课程）"],
           ["TA-VLA", "用辅助目标重建输入的力/力矩信号，以塑造表征"],
           ["FIRST（本文）", "用力矩作输入，并在训练时对接触前 / 接触样本上采样"]],
          widths=[4.2, 11.4], size=9.0)
caption(doc, "表 5　模仿学习对比基线的定义（原论文 6.2 节）")

heading(doc, "4.4　主结果：五个长时序任务全面提升", 2)

figure(doc, os.path.join(FIG, "fig06_task_progress.jpg"),
       "论文图 6：flow matching 策略在五个接触富任务上的任务进度。每组从左至右依次为 Base、"
       "Base+Torque、FACTR、TA-VLA、FIRST。", width_cm=15.6)

body(doc, "FIRST 在全部五个任务上都取得最高任务进度，在多阶段的长时序任务上优势尤其明显——例如 NIST "
          "皮带装配需要在自由运动、接触前对位与接触富操作之间可靠切换。定性观察上，FIRST 在关键接触"
          "事件之前会产生更稳健的「恢复行为」，比如插入、缠绕、拧紧之前的细粒度重新对位。")

body(doc, "论文没有把主结果的逐任务数值列成表格，只给出柱状图（上图为原文图 6 的重新裁切）。为便于"
          "查阅，下表按图读数整理，保留到整数百分比；其中 FIRST 一列的数值与原文表 2 中「该任务最优"
          "重采样相位」的精确值一致，可视为准确值，其余列由图估读、仅供参考。")

add_table(doc,
          ["任务", "Base", "Base+Torque", "FACTR", "TA-VLA", "FIRST"],
          [["LEGO 装配", "≈39%", "≈65%", "≈57%", "≈63%", "91.3%"],
           ["NIST 皮带装配", "≈15%", "≈43%", "≈36%", "≈41%", "76.7%"],
           ["NIST 插装", "≈79%", "≈87%", "≈86%", "≈88%", "93.3%"],
           ["工具归位", "≈46%", "≈53%", "≈62%", "≈69%", "80.5%"],
           ["拧瓶盖", "≈43%", "≈56%", "≈75%", "≈78%", "82.5%"],
           ["平均", "≈44%", "≈61%", "≈63%", "≈68%", "84.9%"]],
          widths=[3.8, 2.4, 2.9, 2.2, 2.2, 2.4], size=9.0, highlight_last=True)
caption(doc, "表 6　任务进度对比（由图 6 读数整理，FIRST 列取自原文表 2 精确值）。相对最强的力感知基线 "
             "TA-VLA，FIRST 平均高约 16.8 个百分点，与原文「超过 17%」的说法一致。")

body(doc, "提升最剧烈的是 NIST 皮带装配（≈41% → 76.7%）与 LEGO 装配（≈63% → 91.3%）——恰好都是"
          "「对齐—接触」反复切换、且阶段数最多的任务。而在 NIST 插装这类基础策略本身已接近 90% 的任务上，"
          "提升空间本就有限（≈88% → 93.3%）。这一分布本身就在支持论文的动机：收益来自接触附近的那些"
          "关键帧，而不是全程平均用力。")

heading(doc, "4.5　消融：接触前比接触中更值得重采样", 2)

body(doc, "最反直觉、也最有价值的结论在这一节。作者固定策略与训练流程，只改变「上采样哪个阶段」：")

add_table(doc,
          ["任务", "仅上采样接触 (C)", "仅上采样接触前 (PC)", "接触前 + 接触 (PC+C)"],
          [["LEGO 装配", "0.697", "0.913", "0.875"],
           ["NIST 皮带装配", "0.494", "0.767", "0.649"],
           ["NIST 插装", "0.800", "0.933", "0.933"],
           ["工具归位", "0.650", "0.805", "0.775"],
           ["拧瓶盖", "0.707", "0.673", "0.825"],
           ["平均", "0.670", "0.818", "0.811"]],
          widths=[4.2, 3.9, 3.9, 3.6], size=9.2, highlight_last=True)
caption(doc, "表 7　上采样相位消融（原论文表 2，指标为任务进度）。")

bullet(doc, "最佳平均成绩，说明「接触发生前的那一小段」才是决定成败的地方——它决定了机器人能否以"
            "正确的位置和姿态进入接触。",
       bold_prefix="只上采样接触前（PC）拿到 0.818")
bullet(doc, "唯一的例外，因为它的成功依赖于维持接触、感知「已拧紧」，持续接触样本不可替代。"
            "这也是唯一 PC+C 优于纯 PC 的任务。",
       bold_prefix="拧瓶盖相反，PC+C 最好（0.825）")
bullet(doc, "只上采样接触样本甚至不如不上采样（0.670 vs 基线）。换句话说，光盯着「接触中」的数据反而"
            "可能损害性能——很可能因为过度聚焦接触瞬间会让策略丢掉接近段与整体轨迹上下文。",
       bold_prefix="只上采样接触（C）几乎无效")

body(doc, "在 ACT 上重复同样的实验（原文图 11），趋势一致：上采样接触前与接触阶段都能提升任务进度。"
          "这说明 FIRST 的收益不依赖于某一种策略架构。")

figure(doc, os.path.join(FIG, "fig11_act_progress.jpg"),
       "论文图 11：ACT 策略上五个任务的任务进度，趋势与 flow matching 一致。", width_cm=15.4)

heading(doc, "4.6　为什么有效：相位级验证损失", 2)

body(doc, "为了解释上采样为何有效，作者在 NIST 皮带装配任务上按相位拆分验证损失。结论清晰：默认采样下，"
          "接触前与接触段的验证损失明显高于自由运动段——这两段本来就「更难学」。上采样之后，这两段的"
          "损失被拉低、向自由运动段的损失靠拢。")

figure(doc, os.path.join(FIG, "fig07_validation_loss.jpg"),
       "论文图 7：默认采样下接触前与接触段的验证损失更高；FIRST 通过上采样降低这两段的损失。",
       width_cm=12.6)

body(doc, "这张图值得单独记住：它把「数据分布不均导致难样本欠拟合」这件抽象的事，量化成了三条可以"
          "直接观测的曲线。这比只报告最终成功率更有说服力。")

heading(doc, "4.7　上采样倍数与逐任务配比", 2)

body(doc, "上采样不是越多越好。定义倍数 m 为被上采样阶段相对自由运动的权重（接触前上采样为 "
          "wF:wPC:wC = 1:m:1；接触前+接触上采样为 1:m:m），在 NIST 皮带、NIST 插装、工具归位（ACT）以及"
          "工具归位（flow matching）上做消融：中等强度最好，普遍在 m = 5 附近达到峰值，再往上就饱和甚至"
          "退化。过度上采样会让训练分布过度偏向接触相关片段，牺牲对整条轨迹的覆盖。")

figure(doc, os.path.join(FIG, "fig12_upsample_magnitude.jpg"),
       "论文图 12：适度的上采样提升任务进度，m ≈ 5 附近最佳；更大的倍数会饱和或退化。", width_cm=14.6)

add_table(doc,
          ["任务", "最优重采样配比 wF : wPC : wC", "含义"],
          [["LEGO 装配", "1 : 5 : 1", "接触前样本权重提升 5 倍"],
           ["NIST 皮带装配", "1 : 5 : 1", "同上"],
           ["NIST 插装", "1 : 5 : 1", "同上"],
           ["工具归位", "1 : 5 : 1", "同上"],
           ["拧瓶盖", "1 : 3 : 3", "接触前与接触同时提权，持续接触是成败关键"]],
          widths=[4.2, 5.0, 6.4], size=9.0)
caption(doc, "表 8　FIRST 在各任务上采用的重采样配比（原论文表 8）。配比按任务进度实测选择。")

heading(doc, "4.8　额外的三组对照实验", 2)

heading(doc, "（1）不把力矩喂给策略，只用它做离线分段", 3)
body(doc, "一个很实用的问题：如果我不想改策略输入（比如已有系统不方便加输入维度），NEXT 还有用吗？"
          "答案是肯定的。此时 NEXT 只在训练前离线把示教切好相位，策略只吃标准视觉 + 本体感知。结果显示，"
          "仅靠相位重采样就能超过标准行为克隆：")
add_table(doc,
          ["策略", "上采样策略", "皮带（无 τ）", "工具（无 τ）", "皮带（有 τ）", "工具（有 τ）"],
          [["ACT", "无", "0.220", "0.600", "0.347", "0.650"],
           ["ACT", "PC", "0.360", "0.600", "0.687", "0.650"],
           ["ACT", "PC + C", "0.283", "0.763", "0.770", "0.788"],
           ["Flow Matching", "无", "0.150", "0.463", "0.431", "0.525"],
           ["Flow Matching", "PC", "0.213", "0.575", "0.767", "0.805"],
           ["Flow Matching", "PC + C", "0.241", "0.525", "0.649", "0.775"]],
          widths=[3.0, 2.4, 3.0, 3.0, 3.0, 3.0], size=8.4)
caption(doc, "表 9　相位重采样在有 / 无力矩输入下的效果（原论文表 9，指标为任务成功率）。"
             "「皮带」指 NIST 皮带装配，「工具」指工具归位。")
body(doc, "两个趋势：横向看，喂入力矩的策略一律更高——力矩提供了额外的状态信息；纵向看，仅重采样也"
          "有明确增益。最好的组合是两者同时使用：NEXT 提供可输入的力矩，FIRST 用同一信号切相位做重采样。")

heading(doc, "（2）原始电流 vs. 学到的外部力矩", 3)
body(doc, "低成本机械臂通常能直接读到电机电流，那为什么不直接把电流当力信号用（TA-VLA 就是这么做的）？"
          "作者的论证是：电流里同时包含自由空间运动所需力矩与接触力矩，还混着硬件与控制器效应，策略"
          "必须自己去「解耦」；而 NEXT 已经把自由空间部分减掉了，给出的是干净的接触量。")
add_table(doc,
          ["方法", "NIST 插装 ↑", "拧瓶盖 ↑"],
          [["ACT（基线）", "0.220", "0.440"],
           ["ACT + 原始关节电流", "0.431", "0.440"],
           ["ACT + 学习的外部力矩", "0.567", "0.525"]],
          widths=[6.6, 3.6, 3.6], size=9.2)
caption(doc, "表 10　以原始电流 vs. 学习外部力矩作为输入的策略表现（原论文表 10）。")
body(doc, "两组对比里，学到的外部力矩都最好。定性观察很说明问题：用原始电流的策略经常在瓶盖已经拧紧后"
          "继续旋转，导致「拧过头」失败；用外部力矩的策略更能识别到位的接触状态并及时停止。注意力分析"
          "（图 10）也支持这一点——在关键接触事件附近，外部力矩 token 的注意力明显抬升，而电流 token "
          "的注意力更弱、与事件的对齐也更差（作者也声明注意力只是诊断，不构成因果证明）。")

figure(doc, os.path.join(FIG, "fig13_attention.jpg"),
       "论文图 13：拧瓶盖过程中的注意力值。学习到的外部力矩在关键接触事件附近产生更清晰的注意力，"
       "而原始关节电流没有。", width_cm=12.6)

heading(doc, "（3）改损失 vs. 改采样分布", 3)
body(doc, "同样想「强调关键阶段」，也可以把损失按相位加权。作者做了一组直接对照：")
add_table(doc,
          ["阶段强调机制", "强调的相位", "任务进度"],
          [["无", "—", "0.453"],
           ["加权回归", "PC", "0.345"],
           ["加权回归", "PC + C", "0.478"],
           ["FIRST（本文）", "PC", "0.573"],
           ["FIRST（本文）", "PC + C", "0.501"]],
          widths=[6.0, 4.0, 4.6], size=9.2)
caption(doc, "表 11　LEGO 装配（ACT）上不同「阶段强调」机制的对比（原论文表 11）。")
body(doc, "加权回归基本没用（PC 单独加权甚至比不加权更差），而改采样分布稳定有效。原因推断是：直接放大"
          "稀疏接触样本的损失贡献会让优化不稳定，策略可能过拟合高权重片段、忽略轨迹的整体上下文；"
          "而按批次上采样保留了原始回归目标，只是让策略更频繁地「见到」关键的接触前与接触过渡。")

# ============================================================ 五
heading(doc, "五、关键结论与可迁移的洞见", 1)

rich(doc, [("1）残差法的瓶颈不在公式，而在「未建模效应」。", True),
           (" 用网络替代 M、C、g 的解析计算，等价于把摩擦迟滞、背隙、温漂、控制器跟踪行为一起学掉。"
            "这是 NEXT 只用 10 分钟数据就能胜过系统辨识的根本原因。", False)])
rich(doc, [("2）时序上下文是必需品，不是锦上添花。", True),
           (" 消融里 MLP 明显弱于循环网络，且历史从 10 步增加到 50 步持续带来收益；q、q̇ 之外再加上"
            "跟踪误差 Δqd 又进一步降低误差。三者缺一，误差都会上升。", False)])
rich(doc, [("3）「估计」与「使用」应当解耦。", True),
           (" NEXT 用 50 步历史做力估计，而策略只吃当前帧的一维力矩信号。若把长历史直接喂给策略，"
            "等于让它同时承担动力学推断、接触定位与视觉运动决策，反而容易过拟合轨迹；把历史留在专用"
            "模块里、只交出压缩后的接触量，是更好的分工。", False)])
rich(doc, [("4）数据分布比网络结构更值得动刀。", True),
           (" FIRST 没有改一个字的损失函数，只在采样分布上做了文章，就在五个任务上稳定提升 17 个百分点；"
            "对照实验里改损失（加权回归）几乎无效。对做行为克隆的人来说，这是个性价比极高的杠杆。", False)])
rich(doc, [("5）最值得复用的经验法则：接触前样本比接触中样本更值钱。", True),
           (" 这与直觉相反，但五个任务里有四个支持它。只有「持续接触才算完成」的任务（拧紧类）"
            "才需要同时上采样接触段。", False)])

# ============================================================ 六
heading(doc, "六、局限与作者自述的边界", 1)

bullet(doc, "外部力矩的绝对尺度依赖力矩常数 K（通常由厂商提供或由电流—力矩曲线导出）。若 K 不准，"
            "绝对量级需要对外部力传感器标定。作者指出这在实践中影响不大：遥操作的反馈力本来就会按"
            "舒适度缩放，策略输入也会做归一化。",
       bold_prefix="绝对尺度问题。")
bullet(doc, "NEXT 是机型专属的，跨机械臂因硬件差异可能需要重新训练。",
       bold_prefix="机型绑定。")
bullet(doc, "结合论文细节可补充三点：一是接触检测依赖双阈值 T_high / T_low，需按机型标定；二是评估的"
            "是关节空间力矩而非末端六维力，做末端阻抗控制时需要额外的雅可比映射；三是任务进度指标"
            "本身依赖人工划分阶段（拧瓶盖还要按「过拧次数」扣分），不同任务间不完全可比。",
       bold_prefix="报告之外的补充观察（笔者注）。")

# ============================================================ 七
heading(doc, "七、复现要点：给动手实现者的清单", 1)

body(doc, "本节是笔者基于原文与附录整理的工程化建议，帮助把论文落到实际平台上。")

heading(doc, "7.1　信号链与前置条件", 2)
bullet(doc, "必须能读到每个关节的电机电流 Im，并拿到力矩常数 K；若固件已直接给出力矩更省事。"
            "这是整条链路的地基，拿不到就没有 NEXT。", bold_prefix="硬件前提：")
bullet(doc, "采集 100 Hz 量级的关节位置 q、速度 q̇，以及指令位置 qd（用于算跟踪误差 Δqd = qd − q）。"
            "很多 SDK 不直接暴露 q̇，需要滤波后差分，注意别把噪声放大成「假外力」。",
       bold_prefix="状态量：")
bullet(doc, "先在自由空间（完全无接触、无负载）跑一段检验：把 τm 与网络预测的 τf 相减，看残差是否"
            "稳定在零附近。图 5 左侧就是标准的验收视图。", bold_prefix="第一步验收：")

heading(doc, "7.2　数据采集与训练", 2)
body(doc, "10 分钟数据的覆盖策略可以直接照抄：单关节全行程 → 多关节联动类笛卡尔运动 → 快慢速重复。"
          "录制一次后在新机器上回放，是快速复制到多台同型号机械臂上的实用技巧。训练配置很小：2 层 "
          "LSTM（隐层 128）+ 2 层 MLP（隐层 256），H = 50，AdamW，1e-3 学习率，单张 3090 约 1 分钟，"
          "开早停。推理在单核 CPU 上 1.76 ms/次，因此遥操作回路按 100 Hz 跑毫无压力。")

heading(doc, "7.3　FIRST 的落地细节", 2)
bullet(doc, "阈值 T_high / T_low 是两个需要按机型调的参数，滞回宽度决定分段是否抖动。建议先用遥操作"
            "用力「碰一下」记录曲线，观察 ‖τext‖₁ 的抬升幅度再定阈值。", bold_prefix="阈值标定：")
bullet(doc, "先把数据比例调成 1:5:1 跑通，再针对「持续接触型」任务尝试 1:3:3。m 不要盲目加大，"
            "m = 5 附近是原文的甜点，再大就饱和甚至退化。", bold_prefix="配比起点：")
bullet(doc, "只改采样器，不动损失。这是最容易实现、也最不容易出问题的方式——把数据集包装成按相位"
            "加权的 WeightedRandomSampler 即可。", bold_prefix="实现方式：")
bullet(doc, "如果平台不便给策略增加输入维度，可以只把 NEXT 当作离线切分工具，仅用相位标签重采样，"
            "按表 9 的经验仍能拿到可观增益。", bold_prefix="降级方案：")

heading(doc, "7.4　容易踩的坑", 2)
bullet(doc, "K 值不准会让所有力矩整体缩放，此时策略（归一化输入）可能还能用，但基于绝对阈值的接触"
            "检测会失准——两处要分开标定。", bold_prefix="力矩常数：")
bullet(doc, "自由空间数据里混入轻微接触（比如拖线、擦到桌面）会直接污染监督标签，让网络把外力"
            "「学成」自由空间的一部分，此后这部分力就再也测不到了。", bold_prefix="数据纯净度：")
bullet(doc, "原文在 Piper 上的自由空间误差是 0.018 Nm；若你的复现误差在这个量级附近，通常说明链路正常；"
            "若到 0.2 Nm 以上，先查 q̇ 的差分噪声与 K 的取值。", bold_prefix="误差量级参考：")
bullet(doc, "Piper 没有力矩真值，接触段无法直接评估。可行的替代是做一个「称重 / 悬挂已知负载」的"
            "静态标定，或用带力传感的末端做交叉验证。", bold_prefix="缺少真值：")

# ============================================================ 八
heading(doc, "八、交付文件清单与延伸阅读", 1)

add_table(doc,
          ["文件", "说明"],
          [["paper/2606.12406v2.pdf", "arXiv 官方原文（22 页，含附录与参考文献）"],
           ["paper/paper_fulltext.md", "按页切分的全文文本，保留原始分页与图表标题"],
           ["paper/paper_cn_summary.md", "本报告的中文 Markdown 版（便于二次编辑或入库）"],
           ["paper/figures/", "从原文提取 / 无损裁切的 13 张论文插图（PNG）"],
           ["paper/figures/doc/", "压缩后的插图（JPEG，供文档嵌入）"],
           ["paper/pages/", "原文 22 页的整页渲染图（PNG，便于快速浏览版式）"],
           ["paper/captions.txt", "全部图表标题的纯文本汇总"],
           ["FACTR2_论文解析报告.docx", "本报告（可编辑）"],
           ["FACTR2_论文解析报告.pdf", "本报告的 PDF 版本（便于分发）"]],
          widths=[5.6, 10.0], size=8.8)
caption(doc, "表 12　本次解析在工作空间内生成的交付文件")

heading(doc, "关键参考文献", 2)
body(doc, "[6] J. J. Liu, Y. Li, K. Shaw, T. Tao, R. Salakhutdinov, D. Pathak. FACTR: Force-Attending "
          "Curriculum Training for Contact-Rich Policy Learning. RSS 2025. —— 前作，NEXT 的直接基础。",
     indent=False, size=9.5, space_after=4)
body(doc, "[17] Z. Zhang et al. Elucidating the Design Space of Torque-aware Vision-Language-Action "
          "Models (TA-VLA). CoRL 2025. —— 主结果中最强的力感知基线。",
     indent=False, size=9.5, space_after=4)
body(doc, "[22] H. Ge et al. FILIC: Dual-loop Force-guided Imitation Learning with Impedance Torque "
          "Control. 2025. —— 力矩估计与策略两处都用到的基线。",
     indent=False, size=9.5, space_after=4)
body(doc, "[43] S. Mamedov, S. Mikhel. Practical Aspects of Model-based Collision Detection. Frontiers "
          "in Robotics and AI, 2020. —— 扰动观测器（DO）基线。",
     indent=False, size=9.5, space_after=4)
body(doc, "[46] T. Z. Zhao, V. Kumar, S. Levine, C. Finn. Learning Fine-Grained Bimanual Manipulation "
          "with Low-Cost Hardware (ACT). RSS 2023. —— ACT 策略基线。",
     indent=False, size=9.5, space_after=4)
body(doc, "[50] O. Siméoni et al. DINOv3. 2025. —— 策略的图像编码器。",
     indent=False, size=9.5, space_after=10)

divider(doc)
p = _p(doc, space_before=8, space_after=0, line=1.4)
r = p.add_run("本报告由论文 PDF 原文整理生成，所有结果与数值均引自原论文；第七节为复现视角的补充分析。"
              "原文项目页：https://jasonjzliu.com/factr2")
set_run_font(r, size=8.5, color=GREY)

fix_math_fonts(doc)
doc.save(OUT)
print("saved:", OUT, round(os.path.getsize(OUT) / 1024 / 1024, 2), "MB")
