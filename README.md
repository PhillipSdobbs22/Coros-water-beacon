[README.md](https://github.com/user-attachments/files/32447310/README.md)
# Water Recovery Mode — depth-triggered backlight beacon

[![build](https://github.com/PhillipSdobbs22/coros-water-beacon/actions/workflows/ci.yml/badge.svg)](https://github.com/PhillipSdobbs22/coros-water-beacon/actions/workflows/ci.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![language: C11](https://img.shields.io/badge/language-C11-555.svg)

<p align="center"><img src="docs/overboard.gif" alt="Animation: a watch detaches on a boat deck, sinks into a lake, crosses the trigger depth at 1.57 m, and the backlight floods the dial in a repeating double flash" width="720"></p>

<p align="center"><strong><a href="https://phillipsdobbs22.github.io/coros-water-beacon/demo/">Try the interactive version &rarr;</a></strong><br>
<sub>Drag the water depth and trigger depth and watch the beacon go quiet.</sub></p>

Firmware and a simulation test bench for a feature I wanted to exist after
I lost a watch in a lake on 19 September 2026: **if the watch goes into the
water and sinks, light the backlight and flash it until someone finds it or
the battery dies.**

I wrote the feature up, then built it to find out whether it would actually
have worked.

It would not have — not as I originally specified it. That turned out to be
the most useful thing the exercise produced.

---

## The finding

My feature request specified a user-set depth threshold and used **10 feet**
as the example. That example is the bug.

Running the real state machine against a range of water depths:

| Water depth | 10 ft threshold | 5 ft threshold | 3.3 ft threshold |
|---|---|---|---|
| 1.0 m | **silent** | **silent** | fires at 6.75 s |
| 1.5 m | **silent** | **silent** | fires at 5.12 s |
| 2.0 m | **silent** | fires at 6.12 s | fires at 5.12 s |
| 2.5 m | **silent** | fires at 6.12 s | fires at 5.12 s |
| 3.0 m | **silent** | fires at 6.12 s | fires at 5.12 s |
| 3.5 m | fires at 8.75 s | fires at 6.12 s | fires at 5.12 s |
| 10.0 m | fires at 8.75 s | fires at 6.12 s | fires at 5.12 s |

A watch that never reaches 10 feet never arms the beacon. Lakes, ponds,
swim areas, the water off a dock — a large share of real watch losses
happen in water shallower than the trigger.

And that is backwards, because **shallow water is where a lost watch is
still recoverable.** At 30 m it is gone regardless. At 2 m a flashing light
is the difference between finding it and not. The feature as specified is
silent in exactly the cases where it would do the most good.

### Why the obvious fix doesn't work, and what does

Dropping the threshold to ~1 m fixes the coverage and breaks something
else: a swimmer who duck-dives a metre would trip the beacon on every
swim.

Pairing the low threshold with a wrist-off requirement resolves both. A
watch that is deep *and* not on a body is not being worn — it is lost:

| Case | Expected | Result |
|---|---|---|
| Swimmer duck-dives 2 m, watch worn | quiet | quiet ✓ |
| Snorkeler at 3 m, watch worn | quiet | quiet ✓ |
| Watch detaches in 1.2 m of lake | fire | fire ✓ |
| Watch detaches in 2.5 m of lake | fire | fire ✓ |

That combination is the actual design recommendation, and I would not have
arrived at it by reasoning about the feature. It came out of running it.

---

## What's here

```
bb_beacon.h / bb_beacon.c    the firmware — integer math, no allocation, no blocking
sim_world.h / sim_world.c    test bench physics: buoyancy, drag, barometer noise
sim_main.c                   scenarios and the check suites
tools/export_data.c          dumps firmware behaviour as JSON for the demo
tools/make_gif.py            renders the animation above from that JSON
docs/demo/                   the interactive demo (GitHub Pages)
docs/state-machine.svg       the state machine
simulation_output.txt        full output of a run
```

The animation and the interactive demo are both driven by `export_data.c`,
which links the same `bb_beacon.c` that would run on the watch. Neither one
re-implements the state machine in Python or JavaScript, so neither can drift
away from what the firmware actually does — if the C changes, `make data`
changes what they show.

<p align="center"><img src="docs/state-machine.svg" alt="Beacon state machine" width="620"></p>

The firmware half compiles for a watch: **no floating point, no dynamic
allocation, no blocking calls**, two periodic entry points that are both
O(1) and ISR-safe.

```c
bb_init(&ctx, &cfg, now_ms);
bb_set_surface_ref(&ctx, baro_pa);            /* at the surface, pre-activity */

/* barometer callback, 4–8 Hz */
bb_on_sample(&ctx, now_ms, baro_pa, wet_sensor, wrist_on);

/* display timer, 100 Hz */
bb_output_t o = bb_on_tick(&ctx, now_ms);
backlight_set(o.brightness_pct);
```

`bb_on_tick()` is the only thing that touches the LED, and it also does the
charge accounting — so the battery estimate and the flash pattern can never
drift apart.

The test bench half is ordinary desktop C with floating point and printf.
None of it ships.

## Build and run

```
make run     # build and run every check
make data    # regenerate docs/demo/beacon-data.json from the firmware
```

C11, no dependencies beyond libm for the test bench. CI builds it on Linux
and macOS with `-Wall -Wextra -Wshadow -Wconversion -Wsign-conversion
-Werror` and runs every check. Rebuilding the animation additionally needs
Python with Pillow, plus ffmpeg for the palette pass:
`python3 tools/make_gif.py`.

---

## Results

**Deep-water case** — sea water, 18 m bottom, 10 ft threshold:

```
 2.00 s  strap fails
 2.55 s  splash, entering at 5.0 m/s
 7.87 s  true depth crosses 3.05 m
 8.88 s  BEACON ON at 3.60 m
34.88 s  on the bottom at 18 m
```

**1.00 s detection latency** — 55 cm of extra depth at the 0.55 m/s
terminal sink rate. 750 ms of that is deliberate debounce; the rest is
barometer filter lag. The beacon is lit 26 seconds before the watch stops
moving.

**Battery** is a flash-pattern problem, not an efficiency problem. On
105 mAh — a realistic *remaining* charge, not a full cell:

| Pattern | Duty | Avg current | Runtime |
|---|---|---|---|
| 1 Hz strobe | 50% | 12.35 mA | 8 h 30 m |
| SOS | 44% | 10.93 mA | 9 h 36 m |
| **Double-tap fast** (default) | 16% | 4.19 mA | **25 h 03 m** |
| Double-tap | 8% | 2.27 mA | 46 h 15 m |
| Slow rescue | 4% | 1.31 mA | 80 h 09 m |

Same LED, same brightness. **Nearly 10× the search window from duty cycle
alone.**

The default is the 16% pattern rather than the 8% one because of what the
gap costs. Double-tap leaves 2.6 seconds of darkness between flashes, and a
searcher panning a torch across open water may hold any one spot for about
a second — long enough to sweep right past a watch that happens to be dark
just then. Halving the gap doubles the duty and still leaves 25 hours. That
trade is worth making; 46 hours of a beacon nobody catches is worth less
than 25 hours of one they do.

**Reserve mode** is where the runtime actually comes from. Dropping to a
slow flash at 25% remaining turns the last quarter of the battery from
2 h 24 m into 25 h 41 m. A dim beacon on day three beats a bright one that
died on night one.

**False-trigger rejection** — shower, hand-washing, lap swimming, a 2.5 m
duck-dive, and a 3.1 m excursion lasting 0.4 s all stay quiet. A genuine
descent fires.

---

## A bug worth showing

The verification pass caught this one, and it is the kind of bug that
survives code review because the code looks correct:

```c
/* before */
if (!reserve && bb_battery_pct(c) <= c->cfg.reserve_pct) {
```

`bb_battery_pct()` returns an integer floor, so `<= 25%` fires anywhere in
25.00–25.99%. The reserve phase ran up to **4% long**, which meant the
runtime figure a settings screen would show the user was wrong by the same
margin. Nothing crashed. Nothing looked wrong.

```c
/* after — compare raw charge */
uint64_t left = c->budget_uams - c->used_uams;
if (!reserve && left * 100ull <= c->budget_uams * (uint64_t)c->cfg.reserve_pct) {
```

Predicted vs. simulated runtime now agrees to 0.01%. The whole point of
modelling the battery was to produce a number honest enough to show a user
before they get in the water, and a 4% lie would have defeated that.

---

## Decisions I made that are arguable

I would rather document a tradeoff than pretend it was obvious.

**Arming policy.** Depth-only fires even while the watch is worn, which is
right for divers — the dangerous moment is the one *after* the strap fails,
and a beacon that waits to detect detachment loses the descent. Depth-plus-
wrist-off makes false triggers during intentional diving near-impossible
and is what makes a low threshold viable, but it leans on the optical
sensor in cold moving water, which is where that sensor is least reliable.
The code implements both and defaults to depth-only; the shallow-water
result above argues for changing that default.

**Once lit, it latches.** Only a long press or a flat battery stops it. A
beacon that can be silenced by a bump is not a beacon. The open question is
whether it should auto-cancel on resurfacing — convenient, but a watch that
washes into the shallows would go dark.

**The default flash pattern** is the paired double-tap, on the reasoning
that an irregular paired flash is easier to pick out of a moving,
reflective background than a steady strobe, at an eighth of the charge.
That reasoning is plausible and **this simulation cannot settle it.** It
needs a pool test against the 1 Hz strobe in turbid water. The default is a
hypothesis, not a finding, and the README says so because the code can't.

---

## How this was built

This was built with [Claude Code](https://claude.com/claude-code) in a
single working session, and I think the division of labour is worth being
specific about, because "AI wrote it" and "AI figured it out" are very
different claims and only one of them is true here.

**What the tooling did well.** It turned a written feature request into
compiling, warning-clean C — firmware module and physics test bench — far
faster than I would have. When I asked for verification as an explicit step
rather than trusting the output, it found the reserve-handoff bug above and
a second one where two `printf` arguments shared a character buffer and
silently printed the same string twice. Both were real, and both were in
code it had written itself.

**What it did not do.** It did not know that the feature was broken. It
modelled an 18 m Gulf bottom with sea water because that is what I
described, and every check passed. The shallow-water finding — the single
most valuable output of this project — only appeared when I corrected the
scenario to *a lake*, because I was the one who knew where the watch
actually went in and what lakes are like. The model then confirmed the
problem in about ninety seconds.

That is the honest shape of it. The tooling compressed the implementation
and was a genuinely good reviewer of its own work. The domain knowledge,
the decision to test the feature rather than just build it, and the
correction that exposed the real flaw were mine. A faster way to build the
wrong thing is not an improvement, and the part that kept this from being
the wrong thing was not automated.

Commits carry AI co-authorship attribution rather than hiding it.

---

## Why this problem

I spent 2001–2011 as a Navy deep sea diver and 2004–2011 as an aviation
water survival instructor, and directed Navy aviation water survival
training centers from 2014 to 2021, retiring in 2026 after 26 years.
Equipment that has to work when someone is in the water and things have
gone wrong is most of what I did.

I have also been a COROS ambassador since 2018 as an ultra runner, and an
Ironman finisher back in 2005 — I use these watches hard, which is how one
ended up at the bottom of a lake.

Finding small objects in dark water is a problem I have a lot of direct
experience with. That is why this is the feature I chose to build rather
than just ask for.

---

## Known limits

- **Terminal velocity is estimated,** not measured — computed from assumed
  mass, volume, and drag coefficient for a tumbling 46 mm watch. A heavier
  metal bracelet sinks faster; a trapped air bubble under a loose band may
  not sink at all.
- **The barometer noise model is ±40 Pa uniform.** Real parts drift with
  temperature, and the thermal shock of a warm deck into cool water is a
  bigger error source than the noise floor.
- **Sensor pressure rating is unverified.** Watch barometers are commonly
  rated near 10 bar. If the part saturates, depth stops reading past
  roughly 90 m — fine for triggering, wrong for display.
- **LED current is modelled as linear in PWM duty.** Close enough for a
  budget; the real backlight has a forward-voltage knee and the boost
  converter has an efficiency curve.
- **Backlight colour is not modelled and probably should be.** Water
  absorbs red far faster than green-cyan, so a white backlight loses much
  of its output within a few metres of turbid water. If the display can
  bias the beacon green, that may buy more range than any change to the
  flash pattern.

This is a concept demonstrator and an argument, not production firmware.
It is not affiliated with or endorsed by COROS.

## License

MIT — see [LICENSE](LICENSE).
