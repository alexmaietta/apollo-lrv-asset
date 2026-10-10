"""Single source of truth for LRV dimensions (SI units, metres / kg / radians).

Frame: X forward, Y left, Z up. Origin on the ground plane, midway between the axles.
Values marked PROVISIONAL are standard published figures not yet checked against the source documents.
"""
from math import radians

# --- overall (PROVISIONAL: LRV Operations Handbook summary figures) ---
WHEELBASE = 2.286          # 90 in
TRACK = 1.829              # 72 in, wheel centre to wheel centre
GROUND_CLEARANCE = 0.356   # 14 in, loaded
MASS_TOTAL = 210.0         # 462 lb Earth weight, empty

# --- wheel (PROVISIONAL) ---
WHEEL_RADIUS = 0.409       # 32.2 in diameter
WHEEL_WIDTH = 0.230        # 9 in
WIRE_DIAMETER = 0.00084    # 0.033 in zinc-coated piano wire
# Wheel details below are scaled off refs/images/7020255.jpg (outer face) and
# 7020263.jpg (tread close-up) — counts and ratios, not drawings.
WIRE_PITCH = 0.0065        # square weave, ~4 cells per tread strip width
WIRE_BIAS = radians(45)    # bias weave: reads as diamonds on the sidewall (7020255, Asnani 'bias tire')
TIRE_RIM_RADIUS = 0.255    # where the mesh clamps to the hub rims
TIRE_SHAPE_N = 2.6         # superellipse exponent of the tire cross-section
CHEVRON_COUNT = 36         # ~18 visible on each half in 7020255 (50% coverage, Handbook 4.2)
CHEVRON_STRIP = 0.025      # strip width, ~= gap width
CHEVRON_ANGLE = radians(48)  # leg sweep from the axle direction
CHEVRON_END_R = 0.352      # strips cover tread + shoulders only; sidewalls are bare mesh (7020255)
CHEVRON_RIVETS = 3         # riveted joints per leg (segmented strips)
WHEEL_REST_SPIN = {"lf": radians(-32), "rf": radians(21), "lr": radians(-58), "rr": radians(12)}  # cosmetic
HUB_RADIUS = 0.23          # white outer disc, ~0.56 R
HUB_CAP_RADIUS = 0.05     # white cap over the decoupling mechanism
HUB_DECOUPLE_R = 0.072     # dark decoupling plate behind the cap
DISC_BAND_W = 0.085        # cream painted band across the disc
DISC_STRIPE_W = 0.022      # black stripe on the band
BUMPSTOP_INNER = 0.25      # titanium inner frame: two hoops joined by flat bars (25.5 in dia, A17 HB 4.1)
BUMPSTOP_OUTER = 0.334
BUMPSTOP_WIDTH = 0.12
BUMPSTOP_BARS = 30
BUMPSTOP_BAR_W = 0.03      # bars ~= slot width

IN = 0.0254

# --- suspension: double wishbone on fore-aft torsion bars ---
# LRV Systems Handbook A15 Rev A, Drawing 4.3 (p.4-5/4-6). BL = lateral, WL = vertical (in).
# Wheel centre line sits at ~WL 100.50 in the drawing; heights below are relative to it.
LOWER_PIVOT_Y = 19.38 * IN             # BL 19.38
LOWER_PIVOT_DZ = (99.25 - 100.50) * IN  # WL 99.25
UPPER_PIVOT_Y = 20.45 * IN             # BL 20.45
UPPER_PIVOT_DZ = (104.43 - 100.50) * IN  # WL 104.43
KINGPIN_TO_WHEEL = 4.21 * IN           # kingpin axis to wheel centre, lateral
KINGPIN_Y = 36.0 * IN - KINGPIN_TO_WHEEL
KNUCKLE_LOWER_DZ = -2.8 * IN           # outer ball joints, scaled off the drawing
KNUCKLE_UPPER_DZ = +2.8 * IN
ARM_BASE = 13.72 * IN                  # fore-aft spread of each A-arm at the chassis
TORSION_BAR_LEN = 24.05 * IN
JOUNCE = 6.0 * IN                      # damper-limited wheel travel, Handbook 4.3
REBOUND = 4.0 * IN
STOW_FOLD = radians(135)               # arms rotate ~135 deg for LM stowage

# --- chassis: A17 Systems Handbook Drawing 3.1 (stations in inches) ---
STA_FWD_END, STA_FWD_HINGE = 15.50, 44.30   # forward section 29.0 long
STA_AFT_HINGE, STA_AFT_END = 97.30, 125.00  # centre 53.0, aft 27.5; overall 109.5
CENTER_WIDTH_IN = 55.45                     # centre section outer width (2 floor panels 27.65 wide)
END_SIDE_BL_IN = 17.7                       # end-section side members (outer face ~BL 18.7, 37.35 overall)
END_INNER_BL_IN = 7.0                       # inner longerons under the batteries / pallet, estimated
FRAME_W_IN, FRAME_H_IN = 1.95, 2.95         # box-section members (TEE/ELBOW details)
TORSION_BAR_UPPER_LEN = 15.22 * IN          # Drawing 4.3
MARK_W = 0.032                              # black alignment marks on the centre side members
MARK_PITCH = (0.10, 0.19, 0.10, 0.27)       # spacing pattern scaled from QTU NASM2020-00047

# --- steering: double Ackermann, front and rear linked ---
STEER_LIMIT = radians(50)        # inner wheel at full lock; outer 22 deg (Ops HB p.1-15) / 23 deg (A17 DWG 4.6)
TURN_RADIUS = 3.1                # wall-to-wall, PROVISIONAL

# Steering linkage, A17 Systems Handbook DWG 4.6 (PDF p.32). Per axle the steering motor and gear train
# drive a triangular sector ("steering rack" detail): pivot at the apex on the vehicle centre line, spur-gear
# teeth on the wide end, and the two tie-rod ball studs at the wide-end corners. The plan view puts the apex
# on the axle line pointing toward the vehicle end, the wide end toward the vehicle centre, and long, nearly
# lateral tie rods out to steering arms that trail from the knuckles toward the vehicle centre.
# DWG 4.6 gives no linkage dimensions, so the values below were fitted to the double-Ackermann law within
# those layout constraints (arm end >= 3 cm inboard of the kingpin, transmission angle >= 20 deg at full lock).
# Checked over the full travel: the outer wheel is within 1.8 deg of the angle Ackermann wants for the inner
# one (full lock: inner 50.0, outer 22.1 deg, cf. 22-23 in the handbooks). With the linear command
# sector = STEER_SECTOR_RATIO * virtual steer, each kingpin is within 2.1 deg of the exact law; the ratio puts
# the vehicle's turn centre (between the inner- and outer-wheel centres) on the exact one at moderate steer.
STEER_SECTOR_X = WHEELBASE / 2         # sector pivot |x|: on the axle line, y = 0
STEER_STUD = (0.130, 0.065)            # tie-rod studs from the pivot: toward the vehicle centre, +-lateral
STEER_ARM = (0.168, radians(10.25))    # knuckle arm: length, plan angle inboard of the fore-aft line
# Heights below the wheel centre. The tie rod is inclined like the lower A-arm (its inner ball sits on the
# line from the knuckle-arm ball through the four-bar's instant centre), so suspension travel gives no
# first-order bump steer; it passes under the end-section side members and torsion bars, over the lower arms.
STEER_ARM_DZ = -0.119                  # knuckle-arm ball (and the tie rod's outer end)
STEER_STUD_DZ = -0.029                 # sector studs / pivot plane, just under the frame
STEER_SECTOR_RATIO = 1.40              # sector angle per unit virtual (bicycle) steer angle
STEER_KINGPIN_STOP = radians(55)       # kingpin hard stops, outside the linkage's travel


def _rot2(v, a):
    from math import cos, sin
    return (cos(a) * v[0] - sin(a) * v[1], sin(a) * v[0] + cos(a) * v[1])


def _steer_plan(corner):
    """Rest-pose plan geometry in the front-axle frame (x mirrored for the rear): kingpin K, arm vector v,
    sector pivot O and stud S0, all (x, y)."""
    from math import cos, sin
    _, sy = CORNERS[corner]
    K = (WHEELBASE / 2, sy * KINGPIN_Y)
    v = (-STEER_ARM[0] * cos(STEER_ARM[1]), -sy * STEER_ARM[0] * sin(STEER_ARM[1]))
    O = (STEER_SECTOR_X, 0.0)
    S0 = (STEER_SECTOR_X - STEER_STUD[0], sy * STEER_STUD[1])
    return K, v, O, S0


def steer_geometry(corner):
    """World rest-pose points (x, y, z) of the linkage for a corner: sector pivot, tie-rod stud, arm end."""
    sx, _ = CORNERS[corner]
    K, v, O, S0 = _steer_plan(corner)
    w = lambda p, dz: (sx * p[0], p[1], WHEEL_RADIUS + dz)  # noqa: E731
    return w(O, STEER_STUD_DZ), w(S0, STEER_STUD_DZ), w((K[0] + v[0], K[1] + v[1]), STEER_ARM_DZ)


def steer_linkage(phi, corner):
    """Exact linkage kinematics: sector angle phi (rad; + = left turn on both sectors, the rear sector's axis
    is -Z) -> (kingpin angle, tie-rod yaw relative to the sector), both about world +Z, or None past toggle.

    Planar: the sector and kingpin axes are both vertical (rest suspension) and both tie-rod ends keep their
    heights, so each ball-jointed tie rod keeps a constant plan-view length. Rear corners are the front solution mirrored in x.
    """
    from math import acos, atan2, hypot, pi
    sx, _ = CORNERS[corner]
    K, v, O, S0 = _steer_plan(corner)
    A0 = (K[0] + v[0], K[1] + v[1])
    L = hypot(A0[0] - S0[0], A0[1] - S0[1])
    a = hypot(*v)
    beta = atan2(v[1], v[0])
    wrap = lambda t: (t + pi) % (2 * pi) - pi  # noqa: E731

    def solve(S):
        w = (S[0] - K[0], S[1] - K[1])
        nw = hypot(*w)
        return atan2(w[1], w[0]), (a * a + nw * nw - L * L) / (2 * a * nw)

    g0, _ = solve(S0)
    sgn = 1.0 if wrap(beta - g0) >= 0 else -1.0
    r = _rot2((S0[0] - O[0], S0[1] - O[1]), phi)
    S = (O[0] + r[0], O[1] + r[1])
    g, c = solve(S)
    if abs(c) > 1.0:
        return None
    theta = wrap(g - beta + sgn * acos(c))
    va = _rot2(v, theta)
    A = (K[0] + va[0], K[1] + va[1])
    psi = atan2(A[1] - S[1], A[0] - S[0]) - atan2(A0[1] - S0[1], A0[0] - S0[0])
    return sx * theta, sx * wrap(psi - phi)


def steer_sector_limit():
    """Sector angle at which the inner kingpin reaches STEER_LIMIT (the steering travel stop)."""
    lo, hi = 0.0, 1.2
    for _ in range(60):
        mid = (lo + hi) / 2
        r = steer_linkage(mid, "lf")
        if r is not None and r[0] < STEER_LIMIT:
            lo = mid
        else:
            hi = mid
    return lo


def steer_sector_angle(delta):
    """Sector command for a virtual (bicycle) steer angle; the same value goes to both sectors."""
    return STEER_SECTOR_RATIO * delta

# ---------------------------------------------------------------- physics
# Everything here is PROVISIONAL engineering estimate unless cited.
G_MOON = 1.62
MASS_LOADED = 700.0              # ~490 kg payload capacity + 210 kg vehicle

MASS = {  # kg per link; the centre chassis gets the remainder of MASS_TOTAL (make_usd)
    "wheel": 5.4,                # 12 lb Earth weight per wheel
    "steer": 8.0,                # traction drive, brake, knuckle
    "upright": 1.0,
    "lower_arm": 1.5,
    "upper_arm": 1.0,
    "damper_cyl": 0.3,
    "damper_rod": 0.2,
    "fender_ext": 0.3,
    "steer_sector": 1.2,         # sector plate + gear segment
    "tie_rod": 0.35,
}
MASS["chassis_fwd"] = 75.0       # frame + 2 batteries + DCE/SPU/gyro; empty CG is at STA 52.8 (Ops HB Fig 1-1)
MASS["chassis_aft"] = 12.0       # frame + pallet mounts

FOLD_LOCK_K = 2.0e5              # chassis hinge lock pins, N*m/rad (stiff stand-in for a pinned joint)
FOLD_LOCK_D = 2.0e3


def arm_length_lower():
    return KINGPIN_Y - LOWER_PIVOT_Y


def arm_angle(travel):
    """Lower-arm rotation for a given vertical travel of the upright."""
    from math import asin
    return asin(min(travel / arm_length_lower(), 1.0))


# Torsion bars: target ~4 kN/m wheel rate, lower bars carry 85% (Handbook 4.3).
# Spring targets are pre-wound so the arms sit at the modelled (loaded, Drawing 4.3)
# pose under the loaded vehicle in lunar gravity; the empty rover rides ~2 in higher.
WHEEL_RATE = 4000.0              # N/m
# Checked in Newton (2026-10-10): as-equipped (286 kg) rides +4.2 cm above the loaded design pose, loaded to
# 700 kg sits at +0.1 cm. Handbook clearance 17 in empty / 14 in loaded = 7.6 cm, of which ~3.2 cm is wire-tire
# deflection (rigid wheels here), leaving ~4.4 cm for the suspension: matches.
TORSION_K_LOWER = 0.85 * WHEEL_RATE * arm_length_lower() ** 2   # N*m/rad
TORSION_K_UPPER = 0.15 * WHEEL_RATE * arm_length_lower() ** 2
_unsprung = MASS["wheel"] + MASS["steer"] + MASS["upright"]
SPRING_PRELOAD_ANGLE = (MASS_LOADED / 4 - _unsprung) * G_MOON * arm_length_lower() / (WHEEL_RATE * arm_length_lower() ** 2)

DAMPER_C = 1300.0                # N*s/m, ~0.3 critical at the wheel
DAMPER_STROKE = (-0.08, 0.08)    # m about the modelled pose (8.65 in min length, Drawing 4.3)

STEER_K = 400.0                  # N*m/rad position servo
STEER_D = 40.0
STEER_MAX_TORQUE = 100.0
# The servo sits on the sector, which drives two kingpins at ~1/STEER_SECTOR_RATIO each: same stiffness and
# torque as seen from each kingpin (virtual work: tau_sector = 2 tau_kingpin / ratio, K_sector = 2 K / ratio^2).
SECTOR_K = 2 * STEER_K / STEER_SECTOR_RATIO ** 2
SECTOR_D = 2 * STEER_D / STEER_SECTOR_RATIO ** 2
SECTOR_MAX_TORQUE = 2 * STEER_MAX_TORQUE / STEER_SECTOR_RATIO
WHEEL_DRIVE_D = 200.0            # N*m*s/rad velocity servo gain
WHEEL_MAX_TORQUE = 60.0          # N*m at the wheel (80:1 harmonic drive)
WHEEL_MAX_SPEED = 14.0 / 3.6 / WHEEL_RADIUS  # 14 km/h level-ground design max (Ops HB), rad/s

# --- fenders ---
# angles from top-dead-centre, positive toward the vehicle's end (PROVISIONAL)
FENDER_FIXED = (radians(-94), radians(36))   # QTU side view NASM2020-00047: wraps to just below the axle line
FENDER_EXT = (radians(33), radians(88))      # riveted seam at ~35 deg, ends ~80-98 deg (00047)
FENDER_CLEARANCE = 0.022       # outer surface ~3.4 cm above the tread (00047, scaled by 32.2 in tire)
FENDER_WIDTH = 0.33
FENDER_CROWN = 0.012            # slight crown across the width
FENDER_SKIRT = 0.06             # side walls of the channel on both faces of the tire; ~5 cm visible below the bead (7020255)

CORNERS = {  # name: (x sign, y sign)
    "lf": (+1, +1),
    "rf": (+1, -1),
    "lr": (-1, +1),
    "rr": (-1, -1),
}


def wheel_center(corner):
    sx, sy = CORNERS[corner]
    return (sx * WHEELBASE / 2, sy * TRACK / 2, WHEEL_RADIUS)

# ---------------------------------------------------------------- vehicle stations
# A17 Systems Handbook coordinates (inches): STA increases aft (front axle STA 26.5,
# rear axle STA 116.5); BL lateral (LBL = left, +Y here); Z 100 = chassis bottom.
STA_FRONT_AXLE, STA_REAR_AXLE = 26.5, 116.5
STA_MID = (STA_FRONT_AXLE + STA_REAR_AXLE) / 2
CHASSIS_BOTTOM_Z = WHEEL_RADIUS + (100.0 - 100.5) * IN   # Drawing 4.3: wheel centre ~WL 100.5


def sta(s):
    """Station (in) -> x (m), forward positive."""
    return (STA_MID - s) * IN


def bl(b):
    """Buttock line (in), left positive -> y (m)."""
    return b * IN


def wl(z):
    """Vertical station (in, Z 100 = chassis bottom) -> z (m)."""
    return CHASSIS_BOTTOM_Z + (z - 100.0) * IN


COMPONENT_MODULES = ["crew", "equipment", "harness"]

# fibreglass fender colour (linear albedo), sampled from the Smithsonian NASM close-up NASM2020-00054 with
# the flag decal's white stripes as the white reference (albedo 0.8), so exposure and white balance cancel:
#   fixed fender: ochre, masked orange pixels on faces lit like the flag (85-90th percentile), B/R ~0.03
#   extension: oxblood maroon on the same surface as the flag, render-matched (flag-normalised ratio)
# The flight vehicles' fibreglass is the same ochre (Hasselblad frames read redder: film + sunlight + dust).
FENDER_COLOR = {"A17": ((0.42, 0.175, 0.012), (0.092, 0.034, 0.042))}
MISSION = "A17"
# Fender configurations per Mission variant (photo evidence):
#   a15 (LRV-1): flags on all four extensions (as15-86-11603 left front + rear, as15-86-11602 right front)
#   a17 (LRV-3): flags on the two front extensions only (as17-146-22367; left rear plain in as17-147-22526)
#   nasm: the Smithsonian NASM vehicle (NASM2020-00046/48/49/50, close-up 00054): flags on all four
#         extensions, only the left-front extension is the maroon part (the rest are the fixed fenders'
#         ochre), and light blue-grey flaps hang from the aft end of both front fixed fenders.
FENDER_EXT_MAROON = ("lf",)            # nasm only
REAR_FLAG_MISSIONS = "a15,nasm"
FLAP_SIZE = (0.125, 0.004)             # [m] drop below the fender end, plate thickness (00048/00049)


# ---------------------------------------------------------------- GCTA colour TV camera (RCA CTV)
# Apollo 17 press kit (NTRS 20090012049, LRV TV section): "zoomed from a focal length of 12.5mm to 75mm
# corresponding to a field of view from 9 to 54 degrees"; pan/tilt ~3 deg/s; iris f/2.2-f/22.
# GCTA final report (NTRS 19730010465) Table 1-2: 6:1 zoom, f/2.2-f/22, pan +214/-134, tilt +85/-45,
# SIT tube, field-sequential colour wheel, NTSC line/frame rates (525 lines, 29.97 fps).
# Format: 54 deg at 12.5 mm => 12.7 mm wide, i.e. the 1-inch tube's 12.7 x 9.525 mm (4:3) active area.
TV_CAMERA = {
    "focal_mm": 12.5, "focal_range_mm": (12.5, 75.0), "aperture_mm": (12.7, 9.525),
    "fstop": 2.2, "fstop_range": (2.2, 22.0), "focus_m": 10.0, "clip_m": (0.05, 5000.0),
    "resolution": (640, 480), "frame_rate": 29.97, "pan_tilt_rate_deg_s": 3.0,
    "sensor": "RCA silicon intensifier target (SIT) tube, field-sequential colour",
    "source": "Apollo 17 press kit (NTRS 20090012049); GCTA final report Table 1-2 (NTRS 19730010465)",
}


# default selection for variant sets created from tagged parts (geo.add_mesh(variant="Set:option"))
VARIANT_DEFAULTS = {"Mission": "a17", "LCRU": "radiator_open", "SeatBelts": "stowed"}  # A17 surface configuration


# ---------------------------------------------------------------- chassis / drive / fender detail
# Lightening holes in the end-section cross members and inner longerons: positions along the member
# (m from its centre / start) and radii, scaled from 7021092.jpg (forward cross member: round holes
# ~0.55-0.6 of the 2.95 in web height, riveted flanges top and bottom, riveted corner plates).
CROSS_MEMBER_HOLES = ((0.0, 0.019), (0.085, 0.022), (-0.085, 0.022), (0.27, 0.024), (-0.27, 0.024),
                      (0.38, 0.019), (-0.38, 0.019))
LONGERON_HOLE_R = 0.017
FRAME_RIVET_PITCH = 1.0 * IN     # rivet rows along the box-member flanges (QTU NASM2020-00047/49)
FRAME_RIVET_R = 0.0024

# Traction drive (A17 Systems Handbook Drawing 4.4: 10.18 in long, 3.25 in motor diameter; wheel-hub
# interface ~4.2 in outboard of the suspension-arm studs, i.e. at the wheel centre plane) and brake
# (Drawing 4.5: band brake ring with 7 bolts, brake arm under a rubber dust boot, cable + return spring
# to a bracket on the motor). Axial offsets are from the wheel centre plane, negative = inboard.
DRIVE_LEN = 10.18 * IN
DRIVE_MOTOR_R = 3.25 * IN / 2
DRIVE_HUB_R = 0.088              # rotating wheel-hub interface flange (wheel link)
DRIVE_BRAKE_R = 0.071            # stationary brake housing / dust shield
DRIVE_INBOARD_END = 0.213        # m inboard of the wheel centre plane (end cap)

# Fender extension stops: LRV-3 addition after the A15/A16 extension losses (A17 Mission Report
# App. A.3.2). No drawing found; modelled as small riveted blocks at the deployed end of the rails.
FENDER_STOP_SIZE = (0.022, 0.010, 0.016)   # along arc, off the skirt, radial

# Apollo 17 EVA-2 right-rear fender repair (A17 Mission Report sec. 9.1 / Fig 9-1, 10.x; as17-137-20979):
# four laminated maps taped into one stiff sheet, clamped to the fixed fender with two utility clamps
# from the LM optical alignment telescope. Sheet size estimated from the photos (2 x 2 maps).
REPAIR_SHEET_START = radians(3)    # leading edge just aft of top-dead-centre (red fender visible ahead of it)
REPAIR_TAIL_LEN = 0.26             # straight run past the fixed fender's aft edge
REPAIR_FLAP = 0.045                # side flaps folded down over both skirts
REPAIR_CLAMP_ANGLE = radians(32)   # at the fixed fender's aft edge (as17-137-20979)
