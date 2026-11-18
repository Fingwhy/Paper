import os
from PIL import Image

SRC = "paper/figures"
DST = "paper/figures/doc"
os.makedirs(DST, exist_ok=True)

for fn in sorted(os.listdir(SRC)):
    if not fn.lower().endswith(".png"):
        continue
    im = Image.open(os.path.join(SRC, fn)).convert("RGB")
    maxw = 1700
    if im.width > maxw:
        h = int(im.height * maxw / im.width)
        im = im.resize((maxw, h), Image.LANCZOS)
    out = os.path.join(DST, os.path.splitext(fn)[0] + ".jpg")
    im.save(out, quality=90, optimize=True)
    print(fn, "->", im.size, round(os.path.getsize(out) / 1024), "KB")
