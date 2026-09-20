#!/usr/bin/env python3
"""
Render the overboard sequence as an animated GIF for the README.

Everything shown -- the depth profile and the moment the beacon fires --
comes from beacon-data.json, which is emitted by tools/export_data.c using
the real bb_beacon.c state machine. Nothing here re-simulates anything;
this file only draws.

The watch is an original rendering (case, bezel, crown, lugs, analog dial).
It is not modelled on any real product.

    make data && python3 tools/make_gif.py
"""
import json, math, os, sys
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "docs/demo/beacon-data.json")

BOTTOM_M   = 6.0              # lake depth
THRESH_CM  = 100              # recommended trigger, not the 10 ft in the spec
PATTERN    = "DOUBLE_TAP_FAST"
FPS        = 20
DURATION_S = 14.0

W, H  = 720, 340              # output size
SS    = 2                     # supersample, for smooth circles
HEAD  = 30
COLW  = 430                   # width of the water column zone
SURF  = 70
BED   = 300
SPAN  = 6.35

PAPER=(255,255,255); INK=(22,33,43); MUTED=(112,126,138)
SKY=(216,230,237); DECK=(58,66,74); BEDC=(43,39,31)
LEDW=(255,253,240)


def F(sz, bold=False):
    names = (["DejaVuSans-Bold.ttf"] if bold else ["DejaVuSans.ttf"])
    for base in ("/usr/share/fonts/truetype/dejavu/", ""):
        for n in names:
            try: return ImageFont.truetype(base + n, sz)
            except Exception: pass
    return ImageFont.load_default()


def gradient(w, h, top, bot):
    g = Image.new("RGB", (1, h)); d = ImageDraw.Draw(g)
    for i in range(h):
        f = (i / max(h - 1, 1)) ** 0.85
        d.point((0, i), tuple(int(top[c] + (bot[c] - top[c]) * f) for c in range(3)))
    return g.resize((w, h))


def led_on(ms, pulses):
    per = sum(a + b for a, b in pulses); p = ms % per; acc = 0
    for on, off in pulses:
        if p < acc + on: return True
        acc += on
        if p < acc + off: return False
        acc += off
    return False


def watch(d, cx, cy, R, lit, depth, elapsed, armed, fs):
    """Original sport watch. Ticks and hands invert when the backlight floods."""
    dR = R - R * 0.17
    if lit:
        for r, a in ((int(R*2.1), 26), (int(R*1.5), 52), (int(R*1.1), 88)):
            d.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(255, 232, 150, a))
    # band
    d.rounded_rectangle([cx-R*.52, cy-R-R*.55, cx+R*.52, cy-R*.16], R*.1, fill=(43,50,56))
    d.rounded_rectangle([cx-R*.52, cy+R*.16, cx+R*.52, cy+R+R*.55], R*.1, fill=(43,50,56))
    # crown + pusher
    d.rounded_rectangle([cx+R-R*.03, cy-R*.13, cx+R+R*.13, cy+R*.13], R*.04, fill=(125,134,141))
    d.rounded_rectangle([cx+R*.93, cy+R*.34, cx+R*1.05, cy+R*.51], R*.03, fill=(93,102,109))
    # case + bezel
    d.ellipse([cx-R, cy-R, cx+R, cy+R], fill=(105,114,121))
    d.ellipse([cx-R*.955, cy-R*.955, cx+R*.955, cy+R*.955], fill=(32,38,43))
    # dial
    if lit:
        for i in range(int(dR), 0, -1):
            f = i / dR
            col = (int(255-9*f), int(254-44*f), int(246-150*f))
            d.ellipse([cx-i, cy-i, cx+i, cy+i], fill=col)
    else:
        d.ellipse([cx-dR, cy-dR, cx+dR, cy+dR], fill=(10,14,17))
    fg  = (48,33,6) if lit else (205,218,226)
    hi  = (36,25,4) if lit else (244,249,252)
    acc = (143,60,16) if lit else (232,166,60)
    for i in range(60):
        a = i*math.pi/30 - math.pi/2
        maj = (i % 5 == 0)
        r1, r2 = dR-dR*.06, dR-(dR*.19 if maj else dR*.11)
        d.line([cx+math.cos(a)*r1, cy+math.sin(a)*r1,
                cx+math.cos(a)*r2, cy+math.sin(a)*r2],
               fill=hi if maj else fg, width=max(1, int(dR*(.055 if maj else .022))))
    d.text((cx, cy+dR*.46), f"{depth:.2f} m", fg, fs["dial"], anchor="mm")
    if armed:
        d.text((cx, cy-dR*.45), "RECOVERY", fg if lit else acc, fs["rec"], anchor="mm")
    mins = 609 + elapsed/60.0
    for ang, ln, wd, col in (((mins/60 % 12)*30, dR*.48, dR*.105, hi),
                             ((mins % 60)*6,     dR*.72, dR*.075, hi),
                             ((elapsed % 60)*6,  dR*.80, dR*.03,  acc)):
        t = ang*math.pi/180 - math.pi/2
        d.line([cx-math.cos(t)*dR*.12, cy-math.sin(t)*dR*.12,
                cx+math.cos(t)*ln,     cy+math.sin(t)*ln],
               fill=col, width=max(1, int(wd)))
    d.ellipse([cx-dR*.045, cy-dR*.045, cx+dR*.045, cy+dR*.045], fill=acc)


def main():
    data = json.load(open(DATA))
    sink = data["sink"]; dt = sink["dt_s"]
    prof = [min(x, BOTTOM_M) for x in sink["depth_m"]]
    grid = {(r[0], r[1], r[2]): (r[3], r[4], r[5]) for r in data["grid"]}
    fired, fire_t, fire_d = grid[(BOTTOM_M, THRESH_CM, 0)]
    if not fired: raise SystemExit("scenario does not fire")

    pulses = {"SOS":[(200,200)]*2+[(200,600)]+[(600,200)]*2+[(600,600)]+
                    [(200,200)]*2+[(200,1400)],
              "DOUBLE_TAP":[(120,120),(120,2640)],
              "DOUBLE_TAP_FAST":[(120,120),(120,1140)],
              "STROBE_1HZ":[(500,500)],
              "RESCUE_SLOW":[(80,1920)]}[PATTERN]
    pinfo = next(p for p in data["patterns"] if p["name"] == PATTERN)
    duty, rt_h = pinfo["duty_permille"], pinfo["runtime_s"]/3600.0

    S = SS
    pxm = (BED-SURF)*S/SPAN
    grad = gradient(COLW*S, (BED-SURF)*S, (58,122,140), (11,32,42))
    fs = {"title":F(13*S,True), "cap":F(10*S), "hud":F(13*S,True), "hudl":F(9*S),
          "dial":F(11*S,True), "rec":F(9*S,True)}
    frames = []

    for k in range(int(DURATION_S*FPS)):
        t = k/FPS
        depth = prof[min(int(t/dt), len(prof)-1)]
        lit = t >= fire_t and led_on(int((t-fire_t)*1000), pulses)

        im = Image.new("RGB", (W*S, H*S), PAPER)
        d = ImageDraw.Draw(im, "RGBA")

        d.text((14*S, 8*S), "COROS Water Recovery Mode - watch lost overboard", INK, fs["title"])

        d.rectangle([0, HEAD*S, COLW*S, SURF*S], fill=SKY)
        d.rectangle([0, HEAD*S, COLW*S, (HEAD+12)*S], fill=DECK)
        d.text((14*S, (HEAD+16)*S), "boat deck", (96,112,122), fs["cap"])
        im.paste(grad, (0, SURF*S))
        d = ImageDraw.Draw(im, "RGBA")
        d.rectangle([0, BED*S, COLW*S, H*S], fill=BEDC)
        d.line([0, SURF*S, COLW*S, SURF*S], fill=(150,196,208), width=2*S)

        for m in range(1, int(SPAN)+1):
            y = SURF*S + m*pxm
            d.line([0, y, 7*S, y], fill=(120,160,174), width=S)
            d.text((11*S, y-5*S), f"{m} m", (118,158,172), fs["cap"])
        ty = SURF*S + (THRESH_CM/100)*pxm
        for x in range(46*S, COLW*S-16*S, 11*S):
            d.line([x, ty, x+5*S, ty], fill=(132,178,192), width=S)
        d.text((46*S, ty-13*S), f"trigger depth  {THRESH_CM/100:.1f} m", (162,200,212), fs["cap"])

        wx = COLW*S//2
        wy = SURF*S + depth*pxm if depth > 0 else \
             (HEAD+12)*S + (SURF-HEAD-12)*S*(t/2.55 if t < 2.55 else 1)
        wy = min(wy, BED*S - 8*S)
        if lit:
            for r, a in ((48*S,34),(30*S,62),(17*S,115)):
                d.ellipse([wx-r, wy-r, wx+r, wy+r], fill=(255,238,168,a))
        d.ellipse([wx-7*S, wy-7*S, wx+7*S, wy+7*S],
                  fill=LEDW if lit else (36,47,56),
                  outline=LEDW if lit else (174,190,203), width=int(1.5*S))

        # close-up panel
        d.rectangle([COLW*S, HEAD*S, W*S, H*S], fill=PAPER)
        d.line([COLW*S, HEAD*S, COLW*S, H*S], fill=(216,224,229), width=int(1.5*S))
        cx, cy, R = (COLW + (W-COLW)/2)*S, 136*S, 63*S
        watch(d, cx, cy, R, lit, depth, t, t >= fire_t, fs)

        rx, ry, RA = (COLW+18)*S, 254*S, (W-18)*S
        for lbl, val, col in (("ELAPSED", f"{t:.2f} s", INK),
                              ("DEPTH",   f"{depth:.2f} m", INK),
                              ("STATE",
                               "IN AIR" if t < 2.55 else ("SINKING" if t < fire_t else "BEACON ON"),
                               (188,128,6) if t >= fire_t else INK)):
            d.text((rx, ry), lbl, MUTED, fs["hudl"])
            d.text((RA, ry-2*S), val, col, fs["hud"], anchor="ra")
            ry += 22*S

        d.text((14*S, (H-22)*S),
               f"{PATTERN.replace('_',' ').title()} - {duty/10:.0f}% duty - "
               f"~{rt_h:.0f} h of beacon on the charge remaining", MUTED, fs["cap"])
        d.text((W*S-22*S, (H-22)*S), "bb_beacon.c - real time", MUTED, fs["cap"], anchor="ra")

        frames.append(im.resize((W, H), Image.LANCZOS))

    tmp = os.environ.get("FRAMEDIR", "/tmp/gifframes")
    os.makedirs(tmp, exist_ok=True)
    for i, f in enumerate(frames):
        f.save(f"{tmp}/f{i:04d}.png")
    print(f"wrote {len(frames)} frames to {tmp}")
    print(f"beacon fires at {fire_t:.2f} s at {fire_d:.2f} m (from the C firmware)")


if __name__ == "__main__":
    main()
