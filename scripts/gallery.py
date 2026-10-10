"""README gallery: compress hero renders to JPEG and build photo-vs-render comparison panels.

Run: .venv/bin/python scripts/gallery.py [--renders docs/images] [--out docs/images]
Reference photos: Smithsonian NASM (CC0) and NASA Apollo (public domain), read from docs/references/ (2000 px
copies, created from the full-resolution downloads in refs/images/ when missing) with a credits README.
"""
import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
REFS = os.path.join(ROOT, "refs", "images")                # full-resolution downloads (git-ignored)
DOC_REFS = os.path.join(ROOT, "docs", "references")       # committed 2000 px copies of the photos used
SOURCES = {  # photo prefix -> (credit, source page, licence)
    "NASM-": ("Smithsonian National Air and Space Museum, Lunar Roving Vehicle Qualification Test Unit",
              "https://airandspace.si.edu/collection-objects/lunar-roving-vehicle-qualification-test-unit/nasm_A19760746000",
              "CC0 (Smithsonian Open Access)"),
    "as15-": ("NASA, Apollo 15 Hasselblad frame", "https://apollojournals.org/alsj/a15/images15.html", "Public domain"),
    "as17-": ("NASA, Apollo 17 Hasselblad frame", "https://apollojournals.org/alsj/a17/images17.html", "Public domain"),
}


def doc_ref(name):
    """Path of the committed copy of a reference photo, creating it from the full-resolution download if needed."""
    out = os.path.join(DOC_REFS, name)
    if not os.path.exists(out):
        for d in ("nasm_qtu", "."):
            p = os.path.join(REFS, d, name)
            if os.path.exists(p):
                os.makedirs(DOC_REFS, exist_ok=True)
                im = Image.open(p)
                im.draft("RGB", (4000, 4000))
                im = im.convert("RGB")
                im.thumbnail((2000, 2000), Image.LANCZOS)
                im.save(out, quality=88, optimize=True)
                break
    return out if os.path.exists(out) else None


def write_credits(names):
    lines = ["# Reference photographs", "", "Copies (2000 px) of the photographs used for the comparison panels in the README.", "",
             "| Photo | Credit | Source | Licence |", "|---|---|---|---|"]
    for n in sorted(names):
        credit, url, lic = next(v for k, v in SOURCES.items() if n.startswith(k))
        lines.append(f"| [{n}]({n}) | {credit} | [{url.split('/')[2]}]({url}) | {lic} |")
    with open(os.path.join(DOC_REFS, "README.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
Image.MAX_IMAGE_PIXELS = None


def load_ref(name, size):
    for p in [doc_ref(name)]:
        if p:
            im = Image.open(p)
            im.draft("RGB", (size[0] * 2, size[1] * 2))
            im = im.convert("RGB")
            # centre-crop to the render's aspect
            w, h = im.size
            ar = size[0] / size[1]
            if w / h > ar:
                nw = int(h * ar)
                im = im.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
            else:
                nh = int(w / ar)
                im = im.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))
            return im.resize(size, Image.LANCZOS)
    return None


def label(im, text):
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", max(14, im.size[1] // 28))
    except OSError:
        font = ImageFont.load_default()
    d.rectangle((0, 0, im.size[0], im.size[1] // 18), fill=(0, 0, 0))
    d.text((10, 2), text, fill=(235, 235, 235), font=font)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--renders", default="docs/images")
    ap.add_argument("--out", default="docs/images")
    a = ap.parse_args()
    import hero
    os.makedirs(a.out, exist_ok=True)
    for name, (_light, _cam, _tgt, _lens, ref, *_variants) in hero.SHOTS.items():
        src = os.path.join(a.renders, f"{name}.png")
        if not os.path.exists(src):
            continue
        r = Image.open(src).convert("RGB")
        r.save(os.path.join(a.out, f"{name}.jpg"), quality=90, optimize=True)
        if ref:
            half = (r.size[0] // 2, r.size[1] // 2)
            p = load_ref(ref, half)
            if p is not None:
                c = Image.new("RGB", (half[0] * 2, half[1]))
                c.paste(label(p, f"Reference: {ref}"), (0, 0))
                c.paste(label(r.resize(half, Image.LANCZOS), "Model (Cycles)"), (half[0], 0))
                c.save(os.path.join(a.out, f"compare_{name}.jpg"), quality=88, optimize=True)
        print("gallery", name)
    used = [v[4] for v in hero.SHOTS.values() if v[4] and doc_ref(v[4])]
    if used:
        write_credits(used)


if __name__ == "__main__":
    main()
