import pymupdf, os, re, json

PDF = "paper/2606.12406v2.pdf"
OUT_DIR = "paper"
doc = pymupdf.open(PDF)

# 1) full text per page
lines = []
lines.append("# FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning\n")
lines.append("> 全文文本由 arXiv 官方 PDF (arXiv:2606.12406v2) 自动提取，保留原始分页与图表标题。\n")
for i, page in enumerate(doc):
    lines.append(f"\n\n<!-- ===================== PDF Page {i+1} ===================== -->\n")
    lines.append(page.get_text("text"))

txt = "\n".join(lines)
with open(os.path.join(OUT_DIR, "paper_fulltext.md"), "w", encoding="utf-8") as f:
    f.write(txt)
print("fulltext chars:", len(txt))

# 2) figure / table captions
caps = []
for i, page in enumerate(doc):
    t = page.get_text("text")
    for m in re.finditer(r"(Figure\s+\d+[:.].{0,400}|Table\s+\d+[:.].{0,400})", t, re.S):
        s = " ".join(m.group(1).split())
        caps.append(f"[p{i+1}] {s}")
with open(os.path.join(OUT_DIR, "captions.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(caps))
print("captions:", len(caps))

# 3) render pages to png for inspection (first N pages)
os.makedirs(os.path.join(OUT_DIR, "pages"), exist_ok=True)
for i, page in enumerate(doc):
    pix = page.get_pixmap(dpi=110)
    pix.save(os.path.join(OUT_DIR, "pages", f"p{i+1:02d}.png"))
print("pages rendered")

# 4) extract embedded raster images bigger than threshold
os.makedirs(os.path.join(OUT_DIR, "images"), exist_ok=True)
n = 0
seen = set()
for i, page in enumerate(doc):
    for img in page.get_images(full=True):
        xref = img[0]
        if xref in seen:
            continue
        seen.add(xref)
        try:
            info = doc.extract_image(xref)
        except Exception:
            continue
        w, h = info.get("width", 0), info.get("height", 0)
        if w < 200 or h < 200:
            continue
        n += 1
        ext = info.get("ext", "png")
        fn = f"images/fig_{n:02d}_p{i+1:02d}_{w}x{h}.{ext}"
        with open(os.path.join(OUT_DIR, fn), "wb") as f:
            f.write(info["image"])
print("images extracted:", n)
