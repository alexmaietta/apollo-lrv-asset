"""Generate procedural source textures (run with the venv Python, before build.py).

Run: .venv/bin/python scripts/textures.py [--out textures/gen] [--only flag,...]

Outputs PNGs that materials.py loads by name. Everything is resolution-independent
and generated deterministically, so the textures can be regenerated at any size.
"""
import argparse
import math
import os

import sys

import struct
import zlib

import numpy as np

try:
    from PIL import Image
except ImportError:  # Blender's bundled Python: only the flag needs PIL
    Image = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "textures", "gen")


def _star_mask(h, w, cx, cy, r_out, r_in=None):
    """Boolean mask of a 5-point star centred at (cx, cy) in pixel coords, point up."""
    r_in = r_in or r_out * 0.382
    pts = []
    for k in range(10):
        a = math.pi / 2 + k * math.pi / 5
        r = r_out if k % 2 == 0 else r_in
        pts.append((cx + r * math.cos(a), cy - r * math.sin(a)))
    x0, x1 = int(cx - r_out) - 1, int(cx + r_out) + 2
    y0, y1 = int(cy - r_out) - 1, int(cy + r_out) + 2
    ys, xs = np.mgrid[y0:y1, x0:x1] + 0.5
    inside = np.zeros(xs.shape, bool)
    for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):
        cond = (ay > ys) != (by > ys)
        xint = ax + (ys - ay) * (bx - ax) / np.where(by - ay == 0, 1e-9, by - ay)
        inside ^= cond & (xs < xint)
    return (slice(max(y0, 0), min(y1, h)), slice(max(x0, 0), min(x1, w))), inside[
        max(0, -y0):inside.shape[0] - max(0, y1 - h), max(0, -x0):inside.shape[1] - max(0, x1 - w)]


def flag_us(hoist=1000, ss=3):
    """US flag per Executive Order 10834 proportions (A=1, B=1.9, C=7/13, D=0.76, K=0.0616)."""
    H, W = hoist * ss, int(round(hoist * 1.9)) * ss
    red, white, blue = (178, 34, 52), (255, 255, 255), (60, 59, 110)
    img = np.empty((H, W, 3), np.uint8)
    stripe = H / 13
    rows = (np.arange(H) // stripe).astype(int)
    img[:] = white
    img[rows % 2 == 0] = red
    ch, cw = int(round(H * 7 / 13)), int(round(H * 0.76))
    img[:ch, :cw] = blue
    E = F = 0.054 * H
    G = H_ = 0.063 * H
    K = 0.0616 * H
    for r in range(9):
        cols = 6 if r % 2 == 0 else 5
        for c in range(cols):
            cx = G + (2 * c + (r % 2)) * H_
            cy = E + r * F
            sl, m = _star_mask(H, W, cx, cy, K / 2)
            img[sl][m] = white
    # white sticker margin round the flag, uneven as on the Smithsonian decal (NASM2020-00054): narrow at
    # the hoist, wider at the fly end and along the bottom; fractions of the hoist (flag height)
    left, right, top, bottom = (int(round(H * k)) for k in STICKER_BORDER)
    sticker = np.empty((H + top + bottom, W + left + right, 3), np.uint8)
    sticker[:] = (242, 242, 238)
    sticker[top:top + H, left:left + W] = img
    im = Image.fromarray(sticker).resize((sticker.shape[1] // ss, sticker.shape[0] // ss), Image.LANCZOS)
    return im


STICKER_BORDER = (0.07, 0.19, 0.10, 0.30)   # left (hoist), right (fly), top, bottom


# ---------------------------------------------------------------- tileable PBR sets
#
# Each set is periodic (tiles seamlessly) and written as textures/gen/<set>/<map>.png:
# basecolor (sRGB), roughness, metallic (linear greyscale), normal (OpenGL, +Y up),
# opacity where needed. TILE_M is the real-world size of one tile in metres; the
# materials map UVs in metres, so textures land at true scale.

RES = 512 if os.environ.get("LRV_DRAFT") else 2048   # LRV_DRAFT=1: fast low-res sets for iteration
TILE_M = {}


def _rng(seed):
    return np.random.default_rng(seed)


def pnoise(n, feature_px, seed, octaves=4, persistence=0.5):
    """Periodic fractal noise in [0, 1] via FFT band-limited white noise."""
    rng = _rng(seed)
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.fftfreq(n)[None, :]
    f = np.sqrt(fx * fx + fy * fy)
    out = np.zeros((n, n))
    amp, fp, total = 1.0, feature_px, 0.0
    for _ in range(octaves):
        w = rng.standard_normal((n, n))
        f0 = 1.0 / max(fp, 1.0)
        band = np.exp(-((f - f0) ** 2) / (2 * (0.6 * f0) ** 2))
        layer = np.real(np.fft.ifft2(np.fft.fft2(w) * band))
        layer /= layer.std() + 1e-9
        out += amp * layer
        total += amp
        amp *= persistence
        fp /= 2.0
    out /= total
    return np.clip(0.5 + out * 0.18, 0, 1)


def streaks(n, seed, along=1, length_px=900, width_px=1.2):
    """Periodic anisotropic streaks (brushing/rolling marks) along axis `along` (1 = U)."""
    rng = _rng(seed)
    w = rng.standard_normal((n, n))
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.fftfreq(n)[None, :]
    fa, fc = (fx, fy) if along == 1 else (fy, fx)
    band = np.exp(-(fa * length_px) ** 2) * np.exp(-((np.abs(fc) - 0.0) * width_px) ** 2 / 50)
    out = np.real(np.fft.ifft2(np.fft.fft2(w) * band))
    out /= out.std() + 1e-9
    return np.clip(0.5 + out * 0.2, 0, 1)


def scratches(n, seed, count=220, max_len=0.35, depth=1.0):
    """Periodic fine scratches as a height map (0 = surface, negative = groove)."""
    rng = _rng(seed)
    h = np.zeros((n, n))
    for _ in range(count):
        x0, y0 = rng.random(2) * n
        ang = rng.normal(0.3, 0.5)
        ln = rng.random() ** 2 * max_len * n
        steps = int(ln)
        if steps < 2:
            continue
        t = np.linspace(0, ln, steps)
        xs = (x0 + np.cos(ang) * t).astype(int) % n
        ys = (y0 + np.sin(ang) * t + np.sin(t / 60) * 3).astype(int) % n
        h[ys, xs] -= depth * (0.3 + 0.7 * rng.random())
    k = np.fft.fft2(np.pad(np.ones((2, 2)), ((0, n - 2), (0, n - 2))))
    return np.real(np.fft.ifft2(np.fft.fft2(h) * k)) / 4


def normal_from_height(h, strength):
    """Tangent-space OpenGL normal map from a periodic height field (strength in px units)."""
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5 * strength
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5 * strength
    nx, ny, nz = -dx, dy, np.ones_like(h)   # image rows go down, OpenGL +Y up
    ln = np.sqrt(nx * nx + ny * ny + nz * nz)
    return np.stack([nx / ln, ny / ln, nz / ln], -1) * 0.5 + 0.5


def _to8(a):
    return (np.clip(a, 0, 1) * 255 + 0.5).astype(np.uint8)


def _srgb(lin):
    lin = np.clip(lin, 0, 1)
    return np.where(lin <= 0.0031308, lin * 12.92, 1.055 * lin ** (1 / 2.4) - 0.055)


def write_png(path, a8):
    """Minimal PNG writer (8-bit greyscale or RGB) using only zlib."""
    h, w = a8.shape[:2]
    ch = 1 if a8.ndim == 2 else a8.shape[2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a8.reshape(h, w * ch)], 1).tobytes()

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0 if ch == 1 else 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def save_set(name, out, tile_m, **maps):
    d = os.path.join(out, name)
    os.makedirs(d, exist_ok=True)
    for k, a in maps.items():
        a8 = _to8(_srgb(a)) if k == "basecolor" else _to8(a)
        # set-prefixed names: exporters flatten every texture into one folder
        write_png(os.path.join(d, f"{os.path.basename(name)}_{k}.png"), a8)
    TILE_M[name] = tile_m
    return d


def tint(var, colour, amount):
    """Base colour = linear colour modulated by a [0,1] variation field."""
    c = np.array(colour)[None, None, :]
    return c * (1 + (var[..., None] - 0.5) * 2 * amount)


def set_wire_weave(out, n=RES, seed=11):
    """Plain-woven zinc-coated piano wire on the bias (diamond cells), crimped over/under."""
    import params as P
    cells = 12                                    # cells across the tile along each diagonal family
    pitch_px = n / (cells * math.sqrt(2))
    tile_m = P.WIRE_PITCH * n / pitch_px
    r_px = P.WIRE_DIAMETER / 2 / P.WIRE_PITCH * pitch_px * 1.35   # slightly fat to read at distance
    y, x = np.mgrid[0:n, 0:n] + 0.5
    s = (x + y) / (pitch_px * math.sqrt(2))       # along family B / across family A
    t = (x - y) / (pitch_px * math.sqrt(2))
    t = t % (n / (pitch_px * math.sqrt(2)))
    s = s % (n / (pitch_px * math.sqrt(2)))
    a_amp = r_px * 0.9
    dA = (t - np.round(t)) * pitch_px             # distance to nearest A wire (runs along s)
    dB = (s - np.round(s)) * pitch_px
    parA = np.where(np.round(t).astype(int) % 2 == 0, 1, -1)
    parB = np.where(np.round(s).astype(int) % 2 == 0, 1, -1)
    zA = a_amp * np.cos(math.pi * s) * parA + np.sqrt(np.clip(r_px ** 2 - dA ** 2, 0, None))
    zB = -a_amp * np.cos(math.pi * t) * parB + np.sqrt(np.clip(r_px ** 2 - dB ** 2, 0, None))
    inA, inB = np.abs(dA) < r_px, np.abs(dB) < r_px
    h = np.where(inA & inB, np.maximum(zA, zB), np.where(inA, zA, np.where(inB, zB, -a_amp * 2)))
    alpha = np.clip(0.5 + (r_px - np.minimum(np.abs(dA), np.abs(dB))), 0, 1)  # 1px anti-aliased edge
    var = pnoise(n, 180, seed)
    zinc = tint(var, (0.20, 0.205, 0.20), 0.2)        # weathered zinc coat reads dark charcoal (QTU 00046-49)
    ao = np.clip((h + a_amp * 2) / (r_px + a_amp * 3), 0, 1) ** 0.6
    base = zinc * (0.55 + 0.45 * ao[..., None])
    rough = np.clip(0.5 + (var - 0.5) * 0.4 + (1 - ao) * 0.2, 0, 1)
    nrm = normal_from_height(h / r_px, 2.5)
    return save_set("wire_weave", out, tile_m, basecolor=base, roughness=rough, normal=nrm, opacity=alpha)


def set_fiberglass(out, name, colour, n=RES, seed=21, weave_amp=0.08):
    """Fender finish: burnished copper-coloured fibreglass with a granular, sparkly coat (7020254.jpg)."""
    grain = pnoise(n, 3.0, seed, octaves=2)
    stipple = pnoise(n, 14.0, seed + 1, octaves=3)
    weave = 0.5 + 0.5 * np.sin(np.mgrid[0:n, 0:n][1] * 2 * math.pi * 150 / n) * np.sin(
        np.mgrid[0:n, 0:n][0] * 2 * math.pi * 150 / n)   # ~2 mm glass-cloth weave
    mottle = pnoise(n, 400.0, seed + 2, octaves=4)
    base = tint(mottle, colour, 0.12) * (0.9 + 0.2 * grain[..., None])
    flake = (grain > 0.62).astype(float)
    rough = np.clip(0.42 - flake * 0.18 + (stipple - 0.5) * 0.2, 0, 1)
    metal = np.clip(0.25 + flake * 0.5, 0, 1)
    h = stipple * 1.0 + grain * 0.35 + weave * weave_amp
    return save_set(name, out, 0.30, basecolor=base, roughness=rough, metallic=metal,
                    normal=normal_from_height(h, 3.0))


def set_brushed(out, name, colour, rough0, n=RES, seed=31, tile_m=0.4, along=1):
    st = streaks(n, seed, along)
    var = pnoise(n, 500, seed + 1)
    sc = scratches(n, seed + 2, count=160)
    base = tint(var, colour, 0.06) * (0.92 + 0.16 * st[..., None])
    rough = np.clip(rough0 + (st - 0.5) * 0.25 + (var - 0.5) * 0.1 - sc * 0.1, 0.05, 1)
    h = st * 0.6 + sc * 0.8
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 1.5))


def set_titanium(out, n=RES, seed=41):
    """Chevron strip stock: rolled titanium, hairline scratches, handling marks (GRC-2017-C-00745.jpg)."""
    st = streaks(n, seed, 1, length_px=1500, width_px=1.0)
    sc = scratches(n, seed + 1, count=420, max_len=0.25)
    smudge = pnoise(n, 300, seed + 2)
    base = tint(smudge, (0.56, 0.55, 0.52), 0.12) * (0.95 + 0.1 * st[..., None])
    rough = np.clip(0.28 + (smudge - 0.5) * 0.35 + (st - 0.5) * 0.15 - sc * 0.15, 0.05, 1)
    return save_set("titanium", out, 0.25, basecolor=base, roughness=rough,
                    normal=normal_from_height(st * 0.5 + sc * 1.2, 1.5))


def set_paint(out, name, colour, rough0, n=RES, seed=51, tile_m=0.3, peel_px=10.0):
    peel = pnoise(n, peel_px, seed, octaves=3)
    var = pnoise(n, 450, seed + 1)
    base = tint(var, colour, 0.04)
    rough = np.clip(rough0 + (var - 0.5) * 0.12 + (peel - 0.5) * 0.08, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(peel, 1.2))


def set_cast(out, name, colour, rough0, n=RES, seed=61):
    """Bead-blasted / cast aluminium housings: isotropic fine pitting."""
    pit = pnoise(n, 4.0, seed, octaves=3)
    var = pnoise(n, 350, seed + 1)
    base = tint(var, colour, 0.08) * (0.92 + 0.16 * pit[..., None])
    rough = np.clip(rough0 + (pit - 0.5) * 0.3 + (var - 0.5) * 0.1, 0, 1)
    return save_set(name, out, 0.2, basecolor=base, roughness=rough, normal=normal_from_height(pit, 2.0))


def set_cloth(out, name, colour, rough0, n=RES, seed=71, tile_m=0.08, threads=96):
    """Teflon-coated beta-glass cloth / nylon fabric: plain weave."""
    y, x = np.mgrid[0:n, 0:n]
    p = n / threads
    u, v = x / p, y / p
    warp = np.abs(np.sin(math.pi * u)) * (0.6 + 0.4 * np.cos(math.pi * v) * np.sign(np.sin(math.pi * u)))
    weft = np.abs(np.sin(math.pi * v)) * (0.6 - 0.4 * np.cos(math.pi * u) * np.sign(np.sin(math.pi * v)))
    h = np.maximum(warp, weft)
    slub = pnoise(n, 60, seed)
    var = pnoise(n, 500, seed + 1)
    base = tint(var, colour, 0.05) * (0.86 + 0.14 * h[..., None]) * (0.97 + 0.06 * slub[..., None])
    rough = np.clip(rough0 + (0.5 - h) * 0.1 + (var - 0.5) * 0.06, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 0.9))


def set_dusty_cloth(out, name, colour, rough0, n=RES, seed=73, tile_m=0.4, threads=480):
    """Fabric thermal cover handled on the surface (traverse gravimeter, as17-137-20979): fine plain weave
    under large grey regolith smudges and finer speckle. The tile is larger than the cover face so the dust
    does not visibly repeat."""
    y, x = np.mgrid[0:n, 0:n]
    p = n / threads
    u, v = x / p, y / p
    warp = np.abs(np.sin(math.pi * u)) * (0.6 + 0.4 * np.cos(math.pi * v) * np.sign(np.sin(math.pi * u)))
    weft = np.abs(np.sin(math.pi * v)) * (0.6 - 0.4 * np.cos(math.pi * u) * np.sign(np.sin(math.pi * v)))
    h = np.maximum(warp, weft)
    var = pnoise(n, 500, seed + 1)
    big = pnoise(n, 420, seed + 2, octaves=4)                 # hand-sized smudges
    mid = pnoise(n, 110, seed + 3, octaves=3)                 # rubbed patches
    fine = pnoise(n, 10, seed + 4, octaves=2)                 # grain speckle
    dust = np.clip((big - 0.42) * 2.6, 0, 1) * 0.7 + np.clip((mid - 0.55) * 3.0, 0, 1) * 0.35
    dust = np.clip(dust + (fine - 0.5) * 0.25 * dust, 0, 1)
    regolith = np.array((0.15, 0.145, 0.135))[None, None, :]
    cloth = tint(var, colour, 0.06) * (0.86 + 0.14 * h[..., None])
    base = cloth * (1 - 0.75 * dust[..., None]) + regolith * 0.75 * dust[..., None]
    rough = np.clip(rough0 + (0.5 - h) * 0.08 + dust * 0.1, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough,
                    normal=normal_from_height(h * (1 - 0.5 * dust) + dust * 0.15, 0.9))


def set_webbing(out, name, colour, rough0, n=RES, seed=81, tile_m=0.05):
    """Nylon seat webbing: 2/2 twill, diagonal ribs, fuzzy fibres."""
    y, x = np.mgrid[0:n, 0:n]
    k = 64
    twill = 0.5 + 0.5 * np.sin(2 * math.pi * k * (x + y) / n)
    rib = 0.5 + 0.5 * np.sin(2 * math.pi * 2 * k * y / n)
    fuzz = pnoise(n, 2.5, seed, octaves=2)
    var = pnoise(n, 300, seed + 1)
    h = twill * 0.7 + rib * 0.2 + fuzz * 0.25
    base = tint(var, colour, 0.06) * (0.82 + 0.18 * h[..., None])
    rough = np.clip(rough0 + (fuzz - 0.5) * 0.1, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 1.6))


def set_foil(out, name, colour, rough0, n=RES, seed=91, tile_m=0.5):
    """Aluminised Kapton blanket outer layer (NASM2020-00053): large hand-formed crinkles with
    sharp creases, mostly mirror-glossy facets, a few duller handled patches."""
    big = pnoise(n, 260, seed, octaves=3, persistence=0.55)        # ~6 cm folds
    mid = pnoise(n, 70, seed + 1, octaves=3, persistence=0.5)      # ~1.7 cm crinkles
    ridge = 1 - np.abs(big - 0.5) * 2
    ridge2 = 1 - np.abs(mid - 0.5) * 2
    h = big * 2.0 + ridge ** 8 * 0.6 + ridge2 ** 12 * 0.15
    var = pnoise(n, 400, seed + 2)
    smudge = pnoise(n, 180, seed + 3)
    base = tint(var, colour, 0.08)
    rough = np.clip(0.05 + (smudge > 0.62) * 0.12 + ridge2 ** 10 * 0.08, 0.03, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 4.5))


def set_beta_cloth(out, name, colour, rough0, n=RES, seed=95, tile_m=0.4, threads=640):
    """Teflon-coated beta-glass cloth blanket: fine plain weave plus soft sags and wrinkles."""
    y, x = np.mgrid[0:n, 0:n]
    p = n / threads
    weave = np.abs(np.sin(math.pi * x / p)) * 0.5 + np.abs(np.sin(math.pi * y / p)) * 0.5
    sag = pnoise(n, 500, seed, octaves=3)
    wr = pnoise(n, 120, seed + 1, octaves=2)
    wrinkle = (1 - np.abs(wr - 0.5) * 2) ** 5
    h = sag * 3.0 + wrinkle * 0.8 + weave * 0.05
    var = pnoise(n, 300, seed + 2)
    base = tint(var, colour, 0.04) * (0.94 + 0.06 * weave[..., None])
    rough = np.clip(rough0 + (var - 0.5) * 0.08, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 4.0))


def set_plastic(out, name, colour, rough0, n=1024, seed=101, tile_m=0.1):
    """Moulded or anodised small parts: very fine texture, slight variation."""
    tex = pnoise(n, 3.0, seed, octaves=2)
    var = pnoise(n, 200, seed + 1)
    base = tint(var, colour, 0.04)
    rough = np.clip(rough0 + (tex - 0.5) * 0.08, 0, 1)
    return save_set(name, out, tile_m, basecolor=base, roughness=rough, normal=normal_from_height(tex, 0.6))


AUTO_RECIPES = {  # recipe -> generator(out, name, colour, rough)
    "cloth": set_cloth, "webbing": set_webbing, "foil": set_foil, "plastic": set_plastic,
    "beta": set_beta_cloth, "dusty_cloth": set_dusty_cloth,
    "paint": lambda o, nm, c, r: set_paint(o, nm, c, r, n=1024),
    "brushed": lambda o, nm, c, r: set_brushed(o, nm, c, r, n=1024, tile_m=0.25),
    "cast": lambda o, nm, c, r: set_cast(o, nm, c, r, n=1024),
    "fiberglass": lambda o, nm, c, r: set_fiberglass(o, nm, c, n=1024),
}


AUTO_VERSION = {"foil": 4, "beta": 3, "cloth": 2, "webbing": 2, "plastic": 2, "paint": 2, "brushed": 2, "cast": 2, "fiberglass": 2}   # bump to regenerate cached auto sets after a generator change


def make_auto_set(out, name, recipe, colour, rough, seed=0):
    """Generate a coloured tileable set for one material (used by materials.upgrade_all)."""
    d = AUTO_RECIPES[recipe](out, name, tuple(colour), rough)
    with open(os.path.join(d, "tile.txt"), "w") as f:
        f.write(str(TILE_M[name]))
    return d


def set_knit(out, name="hga_knit", colour=(0.20, 0.15, 0.10), n=1024, cells=32, pitch_m=0.0032, thread=0.16,
             seed=111):
    """HGA reflector: knitted gold-plated molybdenum wire mesh, open weave (NASM2020-00051).

    Opacity is a cut-out mask (threads opaque, holes clear) so it survives USD / Newton / Isaac.
    """
    y, x = np.mgrid[0:n, 0:n] + 0.5
    p = n / cells
    dx = np.abs((x / p) % 1.0 - 0.5)
    dy = np.abs((y / p + 0.5) % 1.0 - 0.5)
    half = 0.5 - thread / 2
    ax, ay = dx > half, dy > half
    alpha = np.clip(np.maximum((dx - half) * p + 0.5, (dy - half) * p + 0.5), 0, 1) * (ax | ay)
    loops = 0.5 + 0.5 * np.sin(2 * math.pi * x / p * 2) * ay + 0.5 * np.sin(2 * math.pi * y / p * 2) * ax
    var = pnoise(n, 120, seed)
    base = tint(var, colour, 0.15) * (0.8 + 0.2 * loops[..., None])
    rough = np.clip(0.45 + (var - 0.5) * 0.2, 0, 1)
    h = np.maximum(np.where(ax, 1 - (0.5 - dx) / (0.5 - half + 1e-6), 0), np.where(ay, 1 - (0.5 - dy) / (0.5 - half + 1e-6), 0))
    return save_set(name, out, cells * pitch_m, basecolor=base, roughness=rough, normal=normal_from_height(h, 1.5),
                    opacity=np.clip(alpha + 0.0, 0, 1))


def pbr_sets(out):
    """All tileable material sets (colours are linear, matching materials.py)."""
    set_wire_weave(out)
    import params as P
    fixed, ext = P.FENDER_COLOR[P.MISSION]
    set_fiberglass(out, "fiberglass_fender", fixed, seed=21)
    set_fiberglass(out, "fiberglass_fender_ext", ext, seed=22, weave_amp=0.6)  # weave shows through (00054)
    set_brushed(out, "aluminium", (0.80, 0.80, 0.80), 0.32, seed=31)
    set_brushed(out, "aluminium_spun", (0.85, 0.85, 0.84), 0.22, seed=32, tile_m=0.15)
    set_brushed(out, "aluminium_disc", (0.66, 0.66, 0.64), 0.42, seed=33, tile_m=0.3)
    set_brushed(out, "steel", (0.55, 0.55, 0.56), 0.35, seed=34, tile_m=0.2)
    set_titanium(out)
    set_knit(out)
    set_cast(out, "drive_housing", (0.70, 0.70, 0.70), 0.45, seed=61)
    set_cast(out, "floor_panel", (0.72, 0.72, 0.71), 0.45, seed=62)
    set_paint(out, "white_paint", (0.82, 0.82, 0.80), 0.45, seed=51)
    set_paint(out, "cream_paint", (0.86, 0.83, 0.72), 0.5, seed=52)
    set_paint(out, "black_paint", (0.02, 0.02, 0.02), 0.5, seed=53)
    set_paint(out, "yellow_paint", (0.85, 0.65, 0.05), 0.45, seed=54)
    with open(os.path.join(out, "tiles.json"), "w") as f:
        import json
        json.dump(TILE_M, f, indent=1)


GENERATORS = {"flag_us": flag_us}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    only = set(filter(None, a.only.split(",")))
    for name, gen in GENERATORS.items():
        if only and name not in only:
            continue
        path = os.path.join(a.out, f"{name}.png")
        gen().save(path, optimize=True)
        print("wrote", os.path.relpath(path))
    if not only or "pbr" in only:
        pbr_sets(a.out)
        print("wrote", len(TILE_M), "PBR sets")


if __name__ == "__main__":
    main()
