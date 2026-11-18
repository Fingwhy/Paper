import pymupdf, os
from PIL import Image, ImageChops

doc = pymupdf.open("paper/2606.12406v2.pdf")
os.makedirs("paper/figures", exist_ok=True)

def trim(path, pad=8):
    im = Image.open(path).convert("RGB")
    bg = Image.new("RGB", im.size, (255, 255, 255))
    diff = ImageChops.difference(im, bg)
    bbox = diff.getbbox()
    if bbox:
        x0, y0, x1, y1 = bbox
        x0, y0 = max(0, x0 - pad), max(0, y0 - pad)
        x1, y1 = min(im.width, x1 + pad), min(im.height, y1 + pad)
        im = im.crop((x0, y0, x1, y1))
    im.save(path)

jobs = [
    (7, (60, 35, 530, 175), "fig06_task_progress.png"),
    (7, (275, 178, 530, 250), "fig07_validation_loss.png"),
    (18, (60, 115, 530, 220), "fig11_act_progress.png"),
]
for pno, rect, name in jobs:
    page = doc[pno]
    out = "paper/figures/" + name
    page.get_pixmap(dpi=500, clip=pymupdf.Rect(*rect)).save(out)
    trim(out)
    im = Image.open(out)
    print(name, im.size)
