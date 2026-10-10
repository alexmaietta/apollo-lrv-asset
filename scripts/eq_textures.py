"""Generate equipment textures: the LCRU control panel faceplate legend.

Run: .venv/bin/python scripts/eq_textures.py      (writes textures/gen/eq_lcru_panel.png)

Layout and legends are read off the NASM LCRU (NASM-NASM2020-00053, straight-on crop) and the
LCRU handbook Figure 4-1 (HSI-481184 PDF p.41). The plate is 12.0 x 4.6 in, grey-blue with white
legends. Panel coordinates are inches: u from the plate's left edge as seen from the front
(vehicle right, -Y) to the right, v down from the top edge. equipment.py maps the panel-parallel
faces with uv = (u / PANEL_W, 1 - v / PANEL_H) and places the controls at the same (u, v).
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

PANEL_W, PANEL_H = 12.0, 4.6
DPI = 220
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "textures", "gen")

PLATE = (60, 65, 76)       # grey-blue anodised faceplate (00053 under museum light)
WHITE = (232, 232, 226)
DIAL = (226, 222, 205)     # meter face, ivory
INK = (40, 40, 42)

# control positions (u, v) in inches, shared with equipment.py
MONITOR_SWITCH = (2.75, 1.75)
METER = (1.0, 3.45)        # centre of the square bezel
METER_R = 0.56
MODE_KNOB = (6.35, 2.95)
POWER_SWITCH = (9.55, 1.45)
BREAKER = (11.0, 1.25)
CONNECTOR = (10.7, 3.25)
SCREWS = ((4.35, 0.75), (7.9, 0.75), (7.3, 4.25), (0.35, 0.4), (11.65, 4.25), (11.65, 0.4))

_FONTS = {}


def font(cap_in):
    key = round(cap_in, 3)
    if key not in _FONTS:
        for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
                  "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf",
                  "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
            if os.path.exists(p):
                _FONTS[key] = ImageFont.truetype(p, max(6, int(cap_in * DPI / 0.72)))
                break
        else:
            _FONTS[key] = ImageFont.load_default()
    return _FONTS[key]


def px(u, v):
    return (u * DPI, v * DPI)


class Canvas:
    def __init__(self):
        self.im = Image.new("RGB", (int(PANEL_W * DPI), int(PANEL_H * DPI)), PLATE)
        self.d = ImageDraw.Draw(self.im)

    def text(self, u, v, s, cap=0.16, anchor="mm", fill=WHITE, rot=0.0):
        if rot:
            f = font(cap)
            bb = self.d.textbbox((0, 0), s, font=f)
            w, h = bb[2] - bb[0] + 8, bb[3] - bb[1] + 8
            tile = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            ImageDraw.Draw(tile).text((w / 2, h / 2), s, font=f, fill=fill, anchor="mm")
            tile = tile.rotate(rot, resample=Image.BICUBIC, expand=True)
            x, y = px(u, v)
            self.im.paste(tile, (int(x - tile.width / 2), int(y - tile.height / 2)), tile)
        else:
            self.d.text(px(u, v), s, font=font(cap), fill=fill, anchor=anchor)

    def line(self, pts, w=0.025, fill=WHITE):
        self.d.line([px(*p) for p in pts], fill=fill, width=max(1, int(w * DPI)), joint="curve")

    def rect(self, u0, v0, u1, v1, w=0.025, fill=None, outline=WHITE):
        self.d.rectangle([px(u0, v0), px(u1, v1)], fill=fill, outline=outline, width=max(1, int(w * DPI)))

    def circle(self, u, v, r, fill=None, outline=None, w=0.02):
        x, y = px(u, v)
        rr = r * DPI
        self.d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=fill, outline=outline, width=max(1, int(w * DPI)))


def header(c, u0, u1, v, label, cap=0.2):
    """Section header: '<-- LABEL -->' bracket line with turned-down ends (as on the LCRU)."""
    f = font(cap)
    bb = c.d.textbbox((0, 0), label, font=f)
    half = (bb[2] - bb[0]) / DPI / 2 + 0.08
    um = (u0 + u1) / 2
    c.text(um, v, label, cap)
    c.line([(u0, v + 0.18), (u0 + 0.06, v), (um - half, v)])
    c.line([(um + half, v), (u1 - 0.06, v), (u1, v + 0.18)])


def make_panel():
    c = Canvas()
    # MONITOR
    header(c, 0.15, 3.65, 0.42, "MONITOR")
    c.text(2.55, 0.95, "S-BD AGC", 0.15)
    c.text(1.35, 1.95, "RAD TEMP", 0.15)
    c.text(2.85, 2.45, "POWER", 0.15)
    # temperature / voltage calibration table to the right of the meter
    u0, v0, cw, rh = 1.95, 2.75, 0.45, 0.25
    cols = [u0, u0 + 0.35, u0 + 0.35 + cw, u0 + 0.35 + 2 * cw]
    rows = [v0 + rh * i for i in range(8)]
    for u in cols:
        c.line([(u, v0), (u, rows[-1])], 0.02)
    for v in rows:
        c.line([(u0, v), (cols[-1], v)], 0.02)
    c.text((cols[1] + cols[2]) / 2, v0 + rh / 2, "°F", 0.12)
    c.text((cols[2] + cols[3]) / 2, v0 + rh / 2, "V", 0.12)
    for i in range(5):
        c.text((cols[0] + cols[1]) / 2, v0 + rh * (i + 1.5), str(i + 1), 0.13)
    c.text((cols[0] + cols[1]) / 2, v0 + rh * 6.5, "LIM", 0.09)
    c.text((cols[1] + cols[2]) / 2, v0 + rh * 6.5, "1/4", 0.1)
    c.text((cols[2] + cols[3]) / 2, v0 + rh * 6.5, "1/4", 0.1)
    c.rect(cols[3], rows[5], cols[3] + cw, rows[7], 0.02)
    c.text(cols[3] + cw / 2, v0 + rh * 5.5, "AGC", 0.1)
    c.text(cols[3] + cw / 2, v0 + rh * 6.5, "1/5", 0.1)
    # meter face (seen through the bezel): ivory, tick arc 0..5, pointer
    mu, mv = METER
    c.circle(mu, mv, METER_R, fill=DIAL)
    for i in range(26):
        a = math.radians(225 - 270 * i / 25)
        r0 = METER_R * (0.78 if i % 5 else 0.68)
        c.line([(mu + r0 * math.cos(a), mv - r0 * math.sin(a)),
                (mu + METER_R * 0.9 * math.cos(a), mv - METER_R * 0.9 * math.sin(a))], 0.015, INK)
    for i in range(6):
        a = math.radians(225 - 54 * i)
        c.text(mu + METER_R * 0.55 * math.cos(a), mv - METER_R * 0.55 * math.sin(a), str(i), 0.09, fill=INK)
    a = math.radians(105)
    c.line([(mu, mv + 0.1), (mu + METER_R * 0.8 * math.cos(a), mv - METER_R * 0.8 * math.sin(a))], 0.05, INK)
    # S-BD MODE
    header(c, 4.0, 8.75, 0.45, "S-BD MODE")
    ku, kv = MODE_KNOB
    c.line([(4.05, 2.8), (3.95, 2.65), (3.95, 1.55), (4.05, 1.4), (4.5, 1.4)], 0.025)
    c.text(4.95, 1.4, "RCVR 1", 0.15)
    c.line([(5.45, 1.4), (5.6, 1.4), (5.65, 1.55)], 0.025)
    c.line([(5.45, 1.1), (5.5, 0.95), (6.15, 0.95)], 0.025)
    c.text(6.65, 0.95, "RCVR 2", 0.15)
    c.line([(7.15, 0.95), (8.6, 0.95), (8.7, 1.1), (8.7, 2.25), (8.6, 2.4)], 0.025)
    for ang, lab, dx in ((200, "PM1/NB", -0.55), (150, "PM1/WB", -0.55), (100, "FM/TV", 0.0),
                         (62, "TV RMT", 0.35), (15, "PM2/NB", 0.6)):
        a = math.radians(ang)
        r0, r1 = 1.0, 1.15
        c.line([(ku + r0 * math.cos(a), kv - r0 * math.sin(a)), (ku + r1 * math.cos(a), kv - r1 * math.sin(a))], 0.03)
        c.text(ku + 1.45 * math.cos(a) + dx * 0.5, kv - 1.42 * math.sin(a), lab, 0.16)
    # POWER (separate section, outlined)
    c.line([(8.95, 0.15), (8.95, PANEL_H - 0.15)], 0.02, (48, 52, 62))
    header(c, 9.1, 11.8, 0.35, "POWER")
    pu, pv = POWER_SWITCH
    c.text(pu - 0.1, pv - 0.55, "INT", 0.15)
    c.text(pu + 0.75, pv + 0.05, "OFF", 0.15)
    c.text(pu - 0.1, pv + 0.6, "EXT", 0.15)
    for (u, v) in SCREWS:
        c.circle(u, v, 0.1, fill=(120, 122, 126))
    os.makedirs(OUT, exist_ok=True)
    path = os.path.abspath(os.path.join(OUT, "eq_lcru_panel.png"))
    c.im.save(path)
    print("wrote", path, c.im.size)


# Placards legible in the NASM close-ups (sizes in inches, horizontal text; equipment.py rotates the quad).
# name: (text, width, height, background, ink, style)
PLACARDS = {
    # LCRU port labels: printed cloth tags above the S-band connectors (NASM-NASM2020-00046)
    "eq_label_hga": ("HGA", 1.0, 0.42, (226, 218, 196), (178, 40, 36), "cloth"),
    "eq_label_lga": ("LGA", 1.0, 0.42, (226, 218, 196), (34, 44, 92), "cloth"),
    # pallet stowage placards: black labels with white letters (NASM-NASM2020-00057)
    "eq_placard_collection_bag": ("COLLECTION BAG", 2.6, 0.42, (18, 18, 20), (232, 232, 228), "label"),
    "eq_placard_tongs": ("TONGS", 1.1, 0.36, (18, 18, 20), (232, 232, 228), "label"),
    "eq_placard_extension_handle": ("EXTENSION HANDLE", 3.0, 0.36, (18, 18, 20), (232, 232, 228), "label"),
    "eq_placard_gnomon": ("GNOMON", 1.2, 0.34, (18, 18, 20), (232, 232, 228), "label"),
    "eq_placard_dust": ("DUST", 0.9, 0.34, (18, 18, 20), (232, 232, 228), "label"),
    # part number engraved on the pallet tool bracket (00057)
    "eq_partno_seb39105285": ("SEB39105285-301", 2.2, 0.24, (178, 180, 176), (52, 52, 54), "engraved"),
}


def make_placard(name, text, w, h, bg, ink, style, dpi=400):
    im = Image.new("RGB", (int(w * dpi), int(h * dpi)), bg)
    d = ImageDraw.Draw(im)
    cap = h * (0.48 if style != "engraved" else 0.62)
    path = None
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf"):
        if os.path.exists(p):
            path = p
            break
    size = int(cap * dpi / 0.72)
    fnt = ImageFont.truetype(path, size) if path else ImageFont.load_default()
    while path and d.textbbox((0, 0), text, font=fnt)[2] > w * dpi * 0.9:   # fit the width
        size -= 2
        fnt = ImageFont.truetype(path, size)
    if style == "label":   # thin light border of the embossed label tape
        d.rectangle([2, 2, im.width - 3, im.height - 3], outline=(60, 60, 62), width=3)
    if style == "cloth":   # faint weave and a stitched hem
        for y in range(0, im.height, 6):
            d.line([(0, y), (im.width, y)], fill=tuple(max(0, c - 10) for c in bg), width=1)
        d.rectangle([10, 10, im.width - 11, im.height - 11], outline=tuple(max(0, c - 40) for c in bg), width=2)
    d.text((im.width / 2, im.height / 2), text, font=fnt, fill=ink, anchor="mm")
    out = os.path.abspath(os.path.join(OUT, f"{name}.png"))
    im.save(out)
    print("wrote", out, im.size)


def make_placards():
    os.makedirs(OUT, exist_ok=True)
    for name, spec in PLACARDS.items():
        make_placard(name, *spec)


if __name__ == "__main__":
    make_panel()
    make_placards()
