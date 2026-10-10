"""Generate the Control and Display Console faceplate textures for the crew module.

Run: .venv/bin/python scripts/crew_textures.py      (writes textures/gen/console_*.png)

Layout and legends are read off the LRV-2 flight console photos (refs/images/7021041.jpg,
straight-on; 7021092.jpg) and A17 Systems Handbook Drawing 5.2 (PDF p.46): a 10 x 12 in
black faceplate with white legends. Panel coordinates are inches: u from the crew's left
edge (+Y side of the vehicle) to the right, v down from the top edge.

console_panel.png covers u in [U0, U1] (the extra strip on the left holds the vehicle
attitude indicator face, which hangs off the console's left side) and v in [0, 12].
crew.py maps every panel-parallel face with uv = ((u - U0) / (U1 - U0), 1 - v / 12).
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

U0, U1, V0, V1 = -2.6, 10.0, 0.0, 12.0
DPI = 240
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "textures", "gen")

BLACK = (14, 14, 15)
WHITE = (236, 236, 230)
DIAL = (22, 27, 38)       # blue-black instrument faces
METER = (30, 36, 48)
ORANGE = (220, 120, 40)

_FONTS = {}


def font(cap_in, condensed=False):
    """Bold sans font whose cap height is cap_in inches."""
    key = (round(cap_in, 3), condensed)
    if key not in _FONTS:
        names = (["DejaVuSansCondensed-Bold.ttf", "LiberationSansNarrow-Bold.ttf"] if condensed
                 else ["LiberationSans-Bold.ttf", "DejaVuSans-Bold.ttf"])
        roots = ["/usr/share/fonts/truetype/liberation", "/usr/share/fonts/truetype/dejavu"]
        for n in names:
            for r in roots:
                p = os.path.join(r, n)
                if os.path.exists(p):
                    _FONTS[key] = ImageFont.truetype(p, max(6, int(cap_in * DPI / 0.72)))
                    break
            if key in _FONTS:
                break
        else:
            _FONTS[key] = ImageFont.load_default()
    return _FONTS[key]


def px(u, v):
    return ((u - U0) * DPI, (v - V0) * DPI)


class Canvas:
    def __init__(self):
        self.im = Image.new("RGB", (int((U1 - U0) * DPI), int((V1 - V0) * DPI)), BLACK)
        self.d = ImageDraw.Draw(self.im)

    def text(self, u, v, s, cap=0.14, fill=WHITE, condensed=False):
        self.d.text(px(u, v), s, font=font(cap, condensed), fill=fill, anchor="mm")

    def line(self, pts, w=0.022, fill=WHITE):
        self.d.line([px(*p) for p in pts], fill=fill, width=max(1, int(w * DPI)))

    def rect(self, u0, v0, u1, v1, fill=None, outline=None, w=0.02):
        self.d.rectangle([px(u0, v0), px(u1, v1)], fill=fill, outline=outline,
                         width=max(1, int(w * DPI)))

    def circle(self, u, v, r, fill=None, outline=None, w=0.02):
        (x0, y0), (x1, y1) = px(u - r, v - r), px(u + r, v + r)
        self.d.ellipse([x0, y0, x1, y1], fill=fill, outline=outline, width=max(1, int(w * DPI)))

    def rtext(self, u, v, s, ang, cap=0.14, fill=WHITE):
        """Text centred at (u, v), rotated clockwise by ang degrees (tops point outward)."""
        f = font(cap)
        bb = f.getbbox(s)
        w, h = bb[2] - bb[0] + 8, bb[3] - bb[1] + 8
        t = Image.new("L", (w, h), 0)
        ImageDraw.Draw(t).text((w / 2, h / 2), s, font=f, fill=255, anchor="mm")
        t = t.rotate(-ang, resample=Image.BICUBIC, expand=True)
        x, y = px(u, v)
        self.im.paste(Image.new("RGB", t.size, fill), (int(x - t.size[0] / 2), int(y - t.size[1] / 2)), t)

    def bracket(self, u, v, s, half, cap=0.12, tick=0.07):
        """Label with ┌ ... ┐ bracket lines either side, as on the meter headings."""
        self.text(u, v, s, cap)
        tw = font(cap).getlength(s) / DPI / 2 + 0.05
        for sgn in (-1, 1):
            self.line([(u + sgn * tw, v), (u + sgn * half, v), (u + sgn * half, v + tick)], w=0.018)


# ---------------------------------------------------------------- instruments

HEADING_C, HEADING_R = (2.26, 2.80), 1.62


def heading(c):
    cu, cv = HEADING_C
    c.circle(cu, cv, HEADING_R, fill=DIAL)
    top = 330  # heading under the lubber index in 7021041
    for deg in range(0, 360, 2):
        a = math.radians(deg - top)
        r0 = 1.36 if deg % 10 == 0 else (1.44 if deg % 5 == 0 else 1.49)
        p0 = (cu + r0 * math.sin(a), cv - r0 * math.cos(a))
        p1 = (cu + 1.58 * math.sin(a), cv - 1.58 * math.cos(a))
        c.line([p0, p1], w=0.02 if deg % 10 == 0 else 0.013)
    for deg in range(0, 360, 30):
        a = math.radians(deg - top)
        lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(deg, str(deg))
        big = len(lab) == 1
        r = 1.13 if big else 1.17
        c.rtext(cu + r * math.sin(a), cv - r * math.cos(a), lab, deg - top, cap=0.22 if big else 0.13)
    # fixed lubber index
    c.d.polygon([px(cu, cv - 1.08), px(cu - 0.07, cv - 0.95), px(cu + 0.07, cv - 0.95)], fill=WHITE)
    for (lu, lv, lab, val) in ((cu, cv - 0.70, "BEARING", "144"), (cu - 0.60, cv + 0.10, "DISTANCE", "004"),
                               (cu + 0.58, cv + 0.10, "RANGE", "006")):
        c.text(lu, lv, lab, 0.12)
        c.rect(lu - 0.28, lv + 0.1, lu + 0.28, lv + 0.3, fill=(5, 5, 6), outline=(90, 90, 90), w=0.01)
        for i, ch in enumerate(val):
            c.rect(lu - 0.26 + i * 0.18, lv + 0.11, lu - 0.12 + i * 0.18, lv + 0.29, fill=(30, 30, 30))
            c.text(lu - 0.19 + i * 0.18, lv + 0.2, ch, 0.1)
        if lab != "BEARING":
            c.circle(lu + 0.07, lv + 0.27, 0.015, fill=WHITE)  # decimal point


SPEED_C = (7.53, 1.81)


def speed(c):
    cu, cv = SPEED_C
    c.rect(cu - 0.82, cv - 0.82, cu + 0.82, cv + 0.82, fill=(10, 10, 11))
    c.circle(cu, cv, 0.66, fill=DIAL)
    pc = (cu, cv + 0.18)
    for k in range(0, 21):
        a = math.radians(-62 + 124 * k / 20)
        r0 = 0.40 if k % 5 == 0 else 0.45
        c.line([(pc[0] + r0 * math.sin(a), pc[1] - r0 * math.cos(a)),
                (pc[0] + 0.52 * math.sin(a), pc[1] - 0.52 * math.cos(a))], w=0.014)
        if k % 5 == 0 and k:
            c.text(pc[0] + 0.33 * math.sin(a), pc[1] - 0.33 * math.cos(a), str(k), 0.07)
    c.text(cu, cv - 0.02, "KM/HR", 0.07)
    a = math.radians(-62)
    c.line([pc, (pc[0] + 0.48 * math.sin(a), pc[1] - 0.48 * math.cos(a))], w=0.03)
    c.circle(pc[0], pc[1], 0.1, fill=(5, 5, 5))


METERS = {  # name: (u0, u1, labels top->bottom, bottom label)
    "amp_hr": (2.92, 3.95, ["120", "100", "80", "60", "40", "20", "0"], "15"),
    "volts": (3.99, 4.98, ["100", "", "80", "", "60", "", "40", "", "20", "", "0"], None),
    "battery": (6.46, 7.49, ["180", "160", "140", "120", "100", "80", "60", "40", "20", "0"], None),
    "motor": (7.53, 8.52, ["500", "450", "400", "350", "300", "250", "200"], None),
}
METER_V = (5.39, 7.74)


def meters(c):
    v0, v1 = METER_V
    for name, (u0, u1, labs, bottom) in METERS.items():
        c.rect(u0 + 0.1, v0 + 0.1, u1 - 0.1, v1 - 0.1, fill=METER)
        cu = (u0 + u1) / 2
        top, bot = v0 + 0.33, v1 - (0.62 if bottom else 0.4)
        n = len(labs)
        for i, lab in enumerate(labs):
            vv = top + (bot - top) * i / (n - 1)
            for sgn in (-1, 1):
                c.line([(cu + sgn * 0.17, vv), (cu + sgn * 0.27, vv)], w=0.014)
            if lab:
                c.text(cu, vv, lab, 0.075)
            if i + 1 < n:
                for k in (1, 2, 3):
                    vk = vv + (bot - top) / (n - 1) * k / 4
                    for sgn in (-1, 1):
                        c.line([(cu + sgn * 0.22, vk), (cu + sgn * 0.27, vk)], w=0.01)
        # green-band brackets and pointer flags at the bottom
        c.line([(cu - 0.31, top + 0.1), (cu - 0.34, top + 0.1), (cu - 0.34, bot - 0.1), (cu - 0.31, bot - 0.1)], w=0.012)
        c.line([(cu + 0.31, top + 0.1), (cu + 0.34, top + 0.1), (cu + 0.34, bot - 0.1), (cu + 0.31, bot - 0.1)], w=0.012)
        if bottom:
            c.text(cu, v1 - 0.4, bottom, 0.075)
        pv = v1 - 0.3
        for sgn in (-1, 1):
            x = cu + sgn * 0.33
            c.d.polygon([px(x, pv - 0.05), px(x, pv + 0.05), px(x - sgn * 0.12, pv)], fill=ORANGE)
        if name == "volts":
            c.text(cu - 0.3, top + (bot - top) * 0.25, "V", 0.075)
            c.text(cu + 0.3, top + (bot - top) * 0.25, "V", 0.075)


def ssd_scale(c):
    """Sun shadow device scale (raised plate, same projection)."""
    u0, u1, v0, v1 = 4.12, 6.09, 1.36, 2.10
    c.rect(u0, v0, u1, v1, fill=(8, 8, 8), outline=(70, 70, 70), w=0.01)
    cu = 5.10
    for k in range(-15, 16):
        x = cu + k * 0.055
        c.line([(x, v1 - 0.12), (x, v1 - (0.28 if k % 5 == 0 else 0.2))], w=0.012)
    c.line([(cu - 15 * 0.055, v1 - 0.12), (cu + 15 * 0.055, v1 - 0.12)], w=0.012)
    for k in (-15, -10, -5, 0, 5, 10, 15):
        c.text(cu + k * 0.055, v0 + 0.33, str(abs(k)), 0.09)
    c.circle(cu - 0.35, v0 + 0.12, 0.05, fill=WHITE)
    c.circle(cu + 0.55, v0 + 0.12, 0.05, fill=WHITE)


def attitude(c):
    """Vehicle attitude indicator face (box on the console's left side)."""
    u0, u1, v0, v1 = -2.0, -0.2, 1.2, 3.9
    cu = (u0 + u1) / 2
    c.rect(u0, v0, u1, v1, fill=(12, 12, 13))
    c.text(cu, v0 + 0.55, "VEHICLE", 0.12)
    c.text(cu, v0 + 0.78, "ATTITUDE", 0.12)
    c.text(cu, v0 + 1.01, "INDICATOR", 0.12)
    c.text(u0 + 0.25, v0 + 1.32, "L", 0.11)
    c.text(cu, v0 + 1.32, "ROLL", 0.11)
    c.text(u1 - 0.25, v0 + 1.32, "R", 0.11)
    c.rect(u0 + 0.2, v0 + 1.5, u1 - 0.2, v1 - 0.35, fill=(225, 225, 220))
    pc = (cu, v0 + 0.9)
    for k in range(-25, 26, 5):
        a = math.radians(k * 1.2)
        r0, r1 = 1.0, 1.12 if k % 10 else 1.18
        c.line([(pc[0] + r0 * math.sin(a), pc[1] + r0 * math.cos(a)),
                (pc[0] + r1 * math.sin(a), pc[1] + r1 * math.cos(a))], w=0.015, fill=(20, 20, 20))
    c.line([(cu, v0 + 1.55), (cu, v1 - 0.4)], w=0.03, fill=(200, 60, 20))


# ---------------------------------------------------------------- legends

def legends(c):
    T = c.text
    T(2.18, 0.70, "HEADING", 0.16)
    T(7.53, 0.70, "SPEED", 0.16)
    T(9.01, 0.68, "NAV", 0.16)
    T(9.01, 0.95, "POWER", 0.16)
    T(7.00, 3.13, "GYRO", 0.16)
    T(7.00, 3.39, "TORQUING", 0.16)
    T(6.36, 3.87, "LEFT", 0.12)
    T(7.66, 3.87, "RIGHT", 0.12)
    T(7.00, 4.32, "OFF", 0.12)
    T(9.04, 3.13, "SYSTEM", 0.16)
    T(9.04, 3.39, "RESET", 0.16)
    T(8.99, 4.32, "OFF", 0.12)
    c.rect(4.98, 3.55, 5.06, 4.30, fill=(40, 40, 40))  # SSD slot

    # section dividers
    c.line([(0.05, 4.69), (9.95, 4.69)])
    c.line([(2.67, 4.69), (2.67, 11.95)])
    c.line([(2.67, 7.98), (9.95, 7.98)])
    c.line([(4.53, 7.98), (4.53, 11.95)])
    c.line([(7.82, 7.98), (7.82, 11.95)])

    # POWER
    T(1.69, 4.90, "POWER", 0.15)
    T(1.13, 5.51, "AUX", 0.12)
    for v, lab in ((6.48, "BUS A"), (7.34, "BUS B"), (8.19, "BUS C"), (9.04, "BUS D")):
        T(2.30, v, lab, 0.12)
    for v0, v1, lab in ((6.13, 7.70, "BAT 1"), (7.86, 9.39, "BAT 2")):
        c.line([(0.98, v0), (0.86, v0), (0.86, v1), (0.98, v1)], w=0.018)
        T(0.80, (v0 + v1) / 2, lab, 0.12)
        c.rect(0.55, (v0 + v1) / 2 - 0.1, 1.05, (v0 + v1) / 2 + 0.1, fill=BLACK)
        T(0.80, (v0 + v1) / 2, lab, 0.12)
    c.bracket(1.77, 9.59, "±15 VDC", 0.90, 0.12)
    T(1.77, 10.08, "PRIM", 0.12)
    T(1.77, 10.41, "OFF", 0.12)
    T(1.77, 10.76, "SEC", 0.12)

    # POWER / TEMPERATURE MONITOR
    T(6.26, 4.86, "POWER / TEMPERATURE  MONITOR", 0.16)
    c.bracket(3.42, 5.09, "AMP-HR", 0.48, 0.115)
    T(2.97, 5.29, "1", 0.11)
    T(3.86, 5.29, "2", 0.11)
    c.bracket(4.49, 5.09, "VOLTS", 0.46, 0.115)
    T(4.49, 5.29, "1  AMPS  2", 0.11)
    T(5.72, 5.09, "SELECT", 0.115)
    T(5.72, 5.28, "MOTOR TEMP", 0.115)
    T(5.72, 5.47, "FORWARD", 0.115)
    T(5.72, 6.21, "REAR", 0.115)
    T(5.72, 6.69, "BATTERY", 0.115)
    T(5.72, 6.91, "VOLTS X 1/2", 0.115)
    T(5.72, 7.63, "AMPS", 0.115)
    c.bracket(6.95, 5.09, "BATTERY", 0.50, 0.115)
    T(6.95, 5.29, "1    °F    2", 0.11)
    c.bracket(8.05, 5.09, "MOTOR", 0.50, 0.115)
    T(8.02, 5.29, "L    °F    R", 0.11)
    T(9.14, 6.46, "PWM", 0.12)
    T(9.14, 6.68, "SELECT", 0.12)
    T(9.14, 6.87, "1", 0.11)
    T(9.62, 7.24, "BOTH", 0.11)
    T(9.16, 7.61, "2", 0.11)

    # STEERING
    T(3.62, 8.19, "STEERING", 0.15)
    for v, lab in ((8.44, "FORWARD"), (8.66, "BUS A"), (9.36, "BUS C"), (9.85, "REAR"), (10.06, "BUS B"), (10.74, "BUS D")):
        T(3.70, v, lab, 0.115)
    T(3.72, 9.01, "OFF", 0.115)
    T(3.72, 10.40, "OFF", 0.115)

    # DRIVE POWER
    T(6.13, 8.17, "DRIVE  POWER", 0.15)
    for v in (8.42, 9.82):
        T(4.91, v, "LF" if v < 9 else "LR", 0.115)
        T(5.68, v, "RF" if v < 9 else "RR", 0.115)
        T(6.52, v - 0.02, "LF" if v < 9 else "LR", 0.115)
        T(7.39, v - 0.02, "RF" if v < 9 else "RR", 0.115)
    for v, lab in ((8.62, "BUS A"), (8.98, "OFF"), (9.34, "BUS C"), (10.02, "BUS B"), (10.35, "OFF"), (10.70, "BUS D")):
        T(6.95, v, lab, 0.115)

    # DRIVE ENABLE
    T(8.88, 8.13, "DRIVE  ENABLE", 0.15)
    for v, l1, l2 in ((8.40, "LF", "RF"), (9.79, "LR", "RR")):
        T(8.27, v, l1, 0.115)
        T(9.14, v, l2, 0.115)
    for v, lab in ((8.60, "PWM 1"), (8.95, "OFF"), (9.32, "PWM 2"), (10.00, "PWM 1"), (10.33, "OFF"), (10.68, "PWM 2")):
        T(8.71, v, lab, 0.115)

    # thin white border just inside the faceplate edge (the chamfer catches light)
    c.rect(0.02, 0.02, 9.98, 11.98, outline=(120, 120, 118), w=0.012)


def cw_flag():
    """Caution & warning flag: gold foil with black diagonal hatching (7021041)."""
    w, h = 1024, 256
    im = Image.new("RGB", (w, h), (196, 150, 52))
    d = ImageDraw.Draw(im)
    for x in range(-h, w + h, 64):
        d.polygon([(x, h), (x + 22, h), (x + 22 + h, 0), (x + h, 0)], fill=(25, 22, 18))
    return im


# ---------------------------------------------------------------- placards / decals (crew_*.png)

def placard(w_in, h_in, lines, bg, fg, dpi=300, border=None, font_cond=False):
    """Small placard image: lines = [(text, cap_height_in, v_centre_in)]."""
    im = Image.new("RGB", (int(w_in * dpi), int(h_in * dpi)), bg)
    d = ImageDraw.Draw(im)
    for text, cap, v in lines:
        f = font(cap * DPI / dpi * 1.0, font_cond)
        f = ImageFont.truetype(f.path, max(6, int(cap * dpi / 0.72))) if hasattr(f, "path") else f
        d.text((w_in * dpi / 2, v * dpi), text, font=f, fill=fg, anchor="mm")
    if border:
        d.rectangle([4, 4, im.size[0] - 5, im.size[1] - 5], outline=border, width=max(2, dpi // 60))
    return im


def hc_instructions():
    """Hand-controller housing decal 'LRV operation instructions' (DWG 5.2 top view). Wording from
    the Ops Handbook Fig. 1-11 hand-controller operation notes."""
    lines = [("HAND CONTROLLER OPERATION", 0.085, 0.16),
             ("FWD - FORWARD SPEED", 0.06, 0.40),
             ("AFT (REV SW UP) - REVERSE SPEED", 0.06, 0.58),
             ("LEFT / RIGHT - STEERING", 0.06, 0.76),
             ("PULL AFT - BRAKE   3 IN - PARK BRAKE", 0.06, 0.94),
             ("STEER LEFT - RELEASE PARK BRAKE", 0.06, 1.12),
             ("REVERSE INHIBIT SW DOWN FOR INHIBIT", 0.06, 1.30)]
    return placard(3.4, 1.5, lines, (226, 224, 214), (18, 18, 18), border=(18, 18, 18))


def armrest_pn():
    """Part number stamped on the armrest bracket (QTU 00055: '209-31725-2')."""
    return placard(1.6, 0.35, [("209-31725-2", 0.13, 0.18)], (150, 148, 142), (22, 22, 22))


def seat_pn(dash):
    """Part number inked on the seat-back velcro patch covers (QTU 00056: 209-31006-29 / -30)."""
    return placard(2.6, 0.32, [(f"209-31006-{dash}", 0.12, 0.16)], (170, 112, 48), (30, 22, 14))


def belt_placard():
    """Seat-belt hook plate marking (DWG 5.1 seat belt detail: outboard half / inboard half)."""
    return placard(1.6, 0.9, [("SEAT BELT", 0.1, 0.25), ("PULL TO", 0.08, 0.5), ("RELEASE", 0.08, 0.68)],
                   (170, 170, 166), (20, 20, 20))


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, im in (("crew_hc_instructions.png", hc_instructions()), ("crew_armrest_pn.png", armrest_pn()),
                     ("crew_seat_pn_29.png", seat_pn(29)), ("crew_seat_pn_30.png", seat_pn(30)),
                     ("crew_belt_placard.png", belt_placard())):
        im.save(os.path.abspath(os.path.join(OUT, name)))
    c = Canvas()
    heading(c)
    speed(c)
    meters(c)
    ssd_scale(c)
    attitude(c)
    legends(c)
    p = os.path.abspath(os.path.join(OUT, "console_panel.png"))
    c.im.save(p, optimize=True)
    q = os.path.abspath(os.path.join(OUT, "console_cwflag.png"))
    cw_flag().save(q)
    print(f"wrote {p} ({c.im.size[0]}x{c.im.size[1]}) and {q}")


if __name__ == "__main__":
    main()
