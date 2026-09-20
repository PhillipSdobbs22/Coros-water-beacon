/* ============================================================================
 * sim_main.c -- test bench for the Water Recovery beacon.
 *
 * Scenario: a watch detaches from the wrist on a boat deck in the Gulf,
 * falls 1.5 m to the water, and sinks to an 18 m bottom.  We drive the
 * real firmware state machine with modelled barometer readings and watch
 * what it does.
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include "bb_beacon.h"
#include "sim_world.h"

#define TICK_MS    10u     /* display timer -- 100 Hz                     */
#define SIM_DT     0.001   /* physics step, seconds                       */

static void hr(void)
{
    for (int i = 0; i < 74; i++) putchar('-');
    putchar('\n');
}

static void rule(const char *title)
{
    printf("\n");
    for (int i = 0; i < 74; i++) putchar('=');
    printf("\n %s\n", title);
    for (int i = 0; i < 74; i++) putchar('=');
    printf("\n");
}

static const char *hms(uint32_t s, char *buf, size_t n)
{
    snprintf(buf, n, "%3uh %02um %02us", s / 3600u, (s / 60u) % 60u, s % 60u);
    return buf;
}

/* ===========================================================================
 * 1. Depth math -- verify the integer conversion against hand calculations
 * ========================================================================*/
static int check_depth_math(void)
{
    struct { int32_t dp; bb_water_t w; int32_t want_cm; const char *why; } t[] = {
        {      0, BB_WATER_SALT,     0, "at the surface"                  },
        {   -500, BB_WATER_SALT,     0, "above the surface clamps to 0"   },
        {  10052, BB_WATER_SALT,   100, "1 m of sea water = 10052 Pa"     },
        {   9789, BB_WATER_FRESH,  100, "1 m of fresh water = 9789 Pa"    },
        {  30659, BB_WATER_SALT,   305, "10 ft trigger depth"             },
        { 100520, BB_WATER_SALT,  1000, "10 m"                            },
        { 301560, BB_WATER_SALT,  3000, "30 m -- no int32 overflow"       },
        {  10052, BB_WATER_FRESH,  103, "salt reading in fresh: +2.7%"    },
    };
    int fails = 0;
    printf("  %-34s %8s %8s  %s\n", "case", "want", "got", "");
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        int32_t got = bb_depth_from_pa(101325 + t[i].dp, 101325, t[i].w);
        int ok = (got == t[i].want_cm);
        if (!ok) fails++;
        printf("  %-34s %6d cm %6d cm  [%s]\n",
               t[i].why, t[i].want_cm, got, ok ? "PASS" : "FAIL");
    }
    return fails;
}

/* ===========================================================================
 * 2. Pre-dive prediction table
 * ========================================================================*/
static void print_runtime_table(uint32_t mah, const char *label)
{
    char b[32];
    printf("\n  Battery budget: %u mAh (%s)\n", mah, label);
    printf("  %-16s %6s %8s %9s   %s\n",
           "pattern", "duty", "avg mA", "runtime", "notes");
    hr();

    static const char *note[BB_PATTERN_COUNT] = {
        "unmistakably a signal; costs the most",
        "longest window, but a 2.6 s gap is easy to miss",
        "DEFAULT - the 1.1 s gap a sweeping searcher can catch",
        "brightest average, shortest window",
        "cheapest pattern that still reads as deliberate"
    };

    for (int p = 0; p < BB_PATTERN_COUNT; p++) {
        bb_config_t cfg;
        bb_config_defaults(&cfg);
        cfg.capacity_mah    = mah;
        cfg.pattern         = (bb_pattern_id_t)p;
        cfg.reserve_pattern = (bb_pattern_id_t)p;   /* isolate the pattern */
        cfg.reserve_brightness_pct = 100;

        uint16_t duty = bb_pattern_duty_permille((bb_pattern_id_t)p);
        uint32_t avg_ua = (cfg.led_full_ua * duty) / 1000u + cfg.idle_ua;
        uint32_t rt = bb_predict_runtime_s(&cfg);

        printf("  %-16s %5u%% %6u.%02u %12s   %s\n",
               bb_pattern_name((bb_pattern_id_t)p),
               duty / 10u, avg_ua / 1000u, (avg_ua % 1000u) / 10u,
               hms(rt, b, sizeof b), note[p]);
    }
}

/* ===========================================================================
 * 3. The overboard run
 * ========================================================================*/
#define TRACE_BUCKET_MS 50u
#define TRACE_BUCKETS   400u   /* 20 s of flash trace */

static void run_overboard(void)
{
    sim_world_t w;
    sim_defaults(&w);

    bb_config_t cfg;
    bb_config_defaults(&cfg);
    cfg.trigger_depth_cm = 305;          /* 10 ft                          */
    cfg.pattern          = BB_PATTERN_DOUBLE_TAP_FAST;
    /* The budget is the charge LEFT when the watch goes in, not a fresh
       battery.  Four hours into a fishing trip, 35% is realistic. */
    cfg.capacity_mah     = 105;

    bb_ctx_t bb;
    bb_init(&bb, &cfg, 0);
    bb_set_surface_ref(&bb, sim_read_pressure_pa(&w));

    printf("\n  Setup\n");
    printf("    threshold ......... %d cm (%.1f ft)\n",
           cfg.trigger_depth_cm, cfg.trigger_depth_cm / 30.48);
    printf("    debounce .......... %u samples @ %u Hz = %u ms\n",
           cfg.confirm_samples, BB_SAMPLE_HZ_DESCENT,
           cfg.confirm_samples * 1000u / BB_SAMPLE_HZ_DESCENT);
    printf("    pattern ........... %s (%u.%u%% duty)\n",
           bb_pattern_name(cfg.pattern),
           bb_pattern_duty_permille(cfg.pattern) / 10u,
           bb_pattern_duty_permille(cfg.pattern) % 10u);
    printf("    water column ...... %.1f m, sea water\n", w.env.bottom_m);
    printf("    surface ref ....... %d Pa\n", bb.surface_pa);

    printf("\n  Timeline\n");
    printf("  %8s  %-15s %9s  %s\n", "t", "state", "depth", "event");
    hr();

    bb_state_t last = bb_state(&bb);
    uint32_t now_ms = 0;
    uint32_t next_tick = 0;
    uint32_t next_sample = 0;
    double splash_t = -1.0, thresh_t = -1.0, trigger_t = -1.0, bottom_t = -1.0;
    bool was_in_water = false, was_on_bottom = false;
    double trigger_depth_actual = 0.0;
    double v_terminal = 0.0;

    static char trace[TRACE_BUCKETS + 1];
    memset(trace, ' ', sizeof trace);
    trace[TRACE_BUCKETS] = '\0';
    uint32_t beacon_start_ms = 0;
    bool tracing = false;

    printf("  %8.2f  %-15s %8.2fm  worn on wrist, on deck\n",
           0.0, bb_state_name(last), 0.0);

    for (uint32_t step = 0; step < 60000; step++) {   /* 60 s at 1 ms */
        double t = step * SIM_DT;

        if (!w.released && t >= 2.0) {
            w.released = true;
            printf("  %8.2f  %-15s %8s   STRAP FAILS - watch leaves the wrist\n",
                   t, bb_state_name(bb_state(&bb)), "-");
        }

        sim_step(&w, SIM_DT);
        now_ms = (uint32_t)(t * 1000.0);

        if (!was_in_water && w.in_water) {
            was_in_water = true; splash_t = t;
            printf("  %8.2f  %-15s %8.2fm  SPLASH - enters the water at %.1f m/s\n",
                   t, bb_state_name(bb_state(&bb)), 0.0, w.v_ms);
        }

        /* --- barometer, at whatever rate the firmware is asking for --- */
        if (now_ms >= next_sample) {
            uint8_t hz = bb_sample_rate_hz(&bb);
            next_sample = now_ms + 1000u / hz;
            bb_on_sample(&bb, now_ms, sim_read_pressure_pa(&w),
                         sim_read_water_contact(&w), sim_read_wrist_on(&w));
        }

        if (thresh_t < 0.0 && w.depth_m * 100.0 >= cfg.trigger_depth_cm) {
            thresh_t = t;
            printf("  %8.2f  %-15s %8.2fm  true depth crosses threshold\n",
                   t, bb_state_name(bb_state(&bb)), w.depth_m);
        }

        /* --- display tick --- */
        bb_output_t out = { false, 0 };
        if (now_ms >= next_tick) {
            next_tick = now_ms + TICK_MS;
            out = bb_on_tick(&bb, now_ms);
        }

        bb_state_t s = bb_state(&bb);
        if (s != last) {
            const char *msg = "";
            if (s == BB_STATE_WET_SHALLOW) msg = "wet sensor + depth: submerged";
            else if (s == BB_STATE_CONFIRMING) msg = "past threshold, debouncing";
            else if (s == BB_STATE_BEACON) {
                msg = "*** BEACON ON - max brightness ***";
                trigger_t = t; trigger_depth_actual = w.depth_m;
                beacon_start_ms = now_ms; tracing = true;
            }
            printf("  %8.2f  %-15s %8.2fm  %s\n", t, bb_state_name(s), w.depth_m, msg);
            last = s;
        }

        if (tracing) {
            uint32_t b = (now_ms - beacon_start_ms) / TRACE_BUCKET_MS;
            if (b < TRACE_BUCKETS) {
                if (trace[b] != '#') trace[b] = out.led_on ? '#' : '.';
            }
        }

        if (w.in_water && !w.on_bottom) v_terminal = w.v_ms;
        if (!was_on_bottom && w.on_bottom) {
            was_on_bottom = true; bottom_t = t;
            printf("  %8.2f  %-15s %8.2fm  ON THE BOTTOM - beacon %s\n",
                   t, bb_state_name(s), w.depth_m,
                   bb_is_beaconing(&bb) ? "already running" : "NOT running");
        }
    }

    printf("\n  Result\n");
    printf("    splash at ................. %.2f s\n", splash_t);
    printf("    terminal sink rate ........ %.2f m/s\n", v_terminal);
    printf("    true threshold crossing ... %.2f s (%.2f m)\n",
           thresh_t, cfg.trigger_depth_cm / 100.0);
    printf("    beacon lit at ............. %.2f s\n", trigger_t);
    printf("    detection latency ......... %.2f s  (%.0f cm of extra depth)\n",
           trigger_t - thresh_t,
           (trigger_depth_actual - cfg.trigger_depth_cm / 100.0) * 100.0);
    printf("    depth when it lit up ...... %.2f m\n", trigger_depth_actual);
    printf("    reached bottom at ......... %.2f s, %.1f m\n", bottom_t, w.depth_m);
    printf("    beacon was lit %.1f s before the watch stopped moving.\n",
           bottom_t - trigger_t);

    char b[32];
    printf("    predicted search window ... %s on the charge remaining\n",
           hms(bb_predict_runtime_s(&cfg), b, sizeof b));

    printf("\n  Backlight trace, first 20 s after trigger  ('#' lit, '.' dark)\n");
    printf("  each column = %u ms\n\n", TRACE_BUCKET_MS);
    for (uint32_t r = 0; r < TRACE_BUCKETS; r += 80) {
        printf("  %5.1fs |", (double)(r * TRACE_BUCKET_MS) / 1000.0);
        for (uint32_t i = r; i < r + 80 && i < TRACE_BUCKETS; i++) putchar(trace[i]);
        printf("|\n");
    }
}

/* ===========================================================================
 * 4. False-trigger suite
 *
 * A beacon that cries wolf gets switched off, and a beacon that is switched
 * off saves nothing.  These are the everyday events a wrist-worn depth
 * trigger has to survive.
 * ========================================================================*/
typedef struct {
    const char *name;
    double      depth_m;       /* sustained depth                          */
    double      hold_s;
    bool        wet;
    bool        wrist_on;
    bb_arm_policy_t policy;
    bool        want_beacon;
    const char *why;
} case_t;

static bool run_case(const case_t *c)
{
    bb_config_t cfg;
    bb_config_defaults(&cfg);
    cfg.trigger_depth_cm = 305;
    cfg.arm_policy = c->policy;

    bb_ctx_t bb;
    bb_init(&bb, &cfg, 0);
    bb_set_surface_ref(&bb, 101325);

    uint32_t now = 0;
    uint32_t next_sample = 0;
    uint32_t end = (uint32_t)(c->hold_s * 1000.0);
    sim_world_t w; sim_defaults(&w);
    w.env.surface_pa = 101325.0;

    while (now <= end) {
        if (now >= next_sample) {
            next_sample = now + 1000u / bb_sample_rate_hz(&bb);
            w.depth_m = c->depth_m;
            bb_on_sample(&bb, now, sim_read_pressure_pa(&w),
                         c->wet, c->wrist_on);
        }
        bb_on_tick(&bb, now);
        now += TICK_MS;
    }
    return bb_is_beaconing(&bb);
}

static int run_false_trigger_suite(void)
{
    static const case_t cases[] = {
      { "wave over the deck",     0.00,  3.0, true,  true,  BB_ARM_DEPTH_ONLY,
        false, "wet, but no depth at all" },
      { "hand washing",           0.05,  30.0, true,  true,  BB_ARM_DEPTH_ONLY,
        false, "wet sensor trips, 5 cm is nowhere near 3 m" },
      { "shower, 10 minutes",     0.10, 600.0, true,  true,  BB_ARM_DEPTH_ONLY,
        false, "long wet exposure must not accumulate toward a trigger" },
      { "lap swimming",           0.80, 1800.0, true, true,  BB_ARM_DEPTH_ONLY,
        false, "surface swimming stays far above threshold" },
      { "surf duck-dive, 2.5 m",  2.50,   4.0, true,  true,  BB_ARM_DEPTH_ONLY,
        false, "below threshold, and brief" },
      { "3.1 m for 0.4 s",        3.10,   0.4, true,  false, BB_ARM_DEPTH_ONLY,
        false, "past threshold but shorter than the debounce window" },
      { "snorkel dive to 5 m",    5.00,  20.0, true,  true,  BB_ARM_DEPTH_ONLY,
        true,  "genuine depth: diver wants the beacon" },
      { "same dive, wrist-off",   5.00,  20.0, true,  true,  BB_ARM_DEPTH_WRIST_OFF,
        false, "watch is still on a body, so it is not lost" },
      { "lost overboard, 5 m",    5.00,  20.0, true,  false, BB_ARM_DEPTH_WRIST_OFF,
        true,  "depth AND nobody wearing it: the real case" },
    };

    int fails = 0;
    printf("  %-24s %-8s %-9s  %s\n", "scenario", "expect", "result", "reasoning");
    hr();

    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        bool got = run_case(&cases[i]);
        bool ok  = (got == cases[i].want_beacon);
        if (!ok) fails++;
        printf("  %-24s %-8s %-9s  %s\n",
               cases[i].name,
               cases[i].want_beacon ? "FIRE" : "quiet",
               ok ? (got ? "FIRE  OK" : "quiet OK") : "** FAIL **",
               cases[i].why);
    }
    return fails;
}

/* ===========================================================================
 * 5. Long-run battery check -- does the closed-form prediction match what
 *    the state machine actually burns?  If these disagree, the number the
 *    settings screen shows the user is a lie.
 * ========================================================================*/
static void run_battery_verification(void)
{
    printf("  %-16s %13s %13s %8s   %s\n",
           "pattern", "predicted", "simulated", "error",
           "last 25%: reserve vs primary");
    hr();

    for (int p = 0; p < BB_PATTERN_COUNT; p++) {
        bb_config_t cfg;
        bb_config_defaults(&cfg);
        cfg.capacity_mah = 105;                 /* 35% of 300 mAh */
        cfg.pattern = (bb_pattern_id_t)p;

        bb_ctx_t bb;
        bb_init(&bb, &cfg, 0);
        bb.state = BB_STATE_BEACON;             /* jump straight to beacon */

        uint32_t now = 0;
        uint32_t reserve_at = 0;
        while (bb_state(&bb) != BB_STATE_DEPLETED && now < 400u*3600u*1000u) {
            now += TICK_MS;
            bb_on_tick(&bb, now);
            if (!reserve_at && bb_state(&bb) == BB_STATE_BEACON_RESERVE) {
                reserve_at = now;
            }
        }

        uint32_t sim_s  = now / 1000u;
        uint32_t pred_s = bb_predict_runtime_s(&cfg);
        double err = pred_s ? 100.0 * ((double)sim_s - pred_s) / pred_s : 0.0;

        /* What would that same last 25% have bought at the primary
           pattern?  That difference is what reserve mode is worth. */
        bb_config_t flat = cfg;
        flat.reserve_pattern = cfg.pattern;
        flat.reserve_brightness_pct = cfg.brightness_pct;
        uint32_t reserve_s = sim_s - (reserve_at / 1000u);
        uint32_t flat_reserve_s = bb_predict_runtime_s(&flat) * 25u / 100u;

        char b1[32], b2[32], b3[32], b4[32];
        printf("  %-16s %13s %13s %7.2f%%   %s vs %s\n",
               bb_pattern_name((bb_pattern_id_t)p),
               hms(pred_s, b1, sizeof b1),
               hms(sim_s,  b2, sizeof b2),
               err,
               hms(reserve_s, b3, sizeof b3),
               hms(flat_reserve_s, b4, sizeof b4));
    }
    printf("\n  Reserve mode: at %u%% remaining the beacon drops to %s at %u%%\n",
           25u, bb_pattern_name(BB_PATTERN_RESCUE_SLOW), 70u);
    printf("  brightness. The last quarter of the battery buys several times\n");
    printf("  more search time than it would at the primary pattern.\n");
}


/* ===========================================================================
 * 6. SHALLOW WATER VIABILITY
 *
 * The feature request specifies a user-set depth threshold with 10 ft as
 * the example.  That example is the whole problem.  A watch lost in a lake
 * or off a dock never reaches 10 ft, so the beacon never arms -- and
 * shallow water is exactly where a lost watch is still recoverable.
 *
 * This sweep runs the real state machine against a range of bottom depths
 * and threshold settings and reports where the beacon actually fires.
 * ========================================================================*/
static bool sink_to_bottom(double bottom_m, int32_t threshold_cm,
                           bb_water_t water, bb_arm_policy_t policy,
                           double *fire_t, double *fire_depth)
{
    sim_world_t w;
    sim_defaults(&w);
    w.env.bottom_m  = bottom_m;
    w.env.rho_water = (water == BB_WATER_SALT) ? 1025.0 : 998.2;

    bb_config_t cfg;
    bb_config_defaults(&cfg);
    cfg.trigger_depth_cm = threshold_cm;
    cfg.water            = water;
    cfg.arm_policy       = policy;

    bb_ctx_t bb;
    bb_init(&bb, &cfg, 0);
    bb_set_surface_ref(&bb, sim_read_pressure_pa(&w));

    uint32_t next_tick = 0, next_sample = 0;
    *fire_t = -1.0; *fire_depth = -1.0;

    /* 90 s: long enough for the watch to land and the debounce to finish */
    for (uint32_t step = 0; step < 90000; step++) {
        double t = step * SIM_DT;
        if (!w.released && t >= 2.0) w.released = true;
        sim_step(&w, SIM_DT);
        uint32_t now_ms = (uint32_t)(t * 1000.0);

        if (now_ms >= next_sample) {
            next_sample = now_ms + 1000u / bb_sample_rate_hz(&bb);
            bb_on_sample(&bb, now_ms, sim_read_pressure_pa(&w),
                         sim_read_water_contact(&w), sim_read_wrist_on(&w));
        }
        if (now_ms >= next_tick) {
            next_tick = now_ms + TICK_MS;
            bb_on_tick(&bb, now_ms);
        }
        if (*fire_t < 0.0 && bb_is_beaconing(&bb)) {
            *fire_t = t; *fire_depth = w.depth_m;
            return true;
        }
    }
    return false;
}

static void run_shallow_water_sweep(void)
{
    static const struct { int32_t cm; const char *label; } thr[] = {
        { 305, "305 cm (10 ft, the value in the feature request)" },
        { 152, "152 cm (5 ft)" },
        { 100, "100 cm (3.3 ft)" },
    };
    static const double bottoms[] = { 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 5.0, 10.0 };

    printf("  Fresh water (lake). Does the beacon ever light up?\n");

    for (size_t k = 0; k < sizeof thr / sizeof thr[0]; k++) {
        printf("\n  threshold = %s\n", thr[k].label);
        printf("  %12s  %-10s %10s  %s\n",
               "lake depth", "beacon", "lit at", "");
        hr();
        for (size_t i = 0; i < sizeof bottoms / sizeof bottoms[0]; i++) {
            double ft, fd;
            bool fired = sink_to_bottom(bottoms[i], thr[k].cm,
                                        BB_WATER_FRESH, BB_ARM_DEPTH_ONLY,
                                        &ft, &fd);
            char when[48];
            if (fired) snprintf(when, sizeof when, "%.2f s at %.2f m", ft, fd);
            else       snprintf(when, sizeof when, "%s", "-- never --");
            printf("  %10.1f m  %-10s %s\n",
                   bottoms[i], fired ? "FIRES" : "SILENT", when);
        }
    }

    printf("\n  The 100 cm threshold fires in water the 305 cm threshold\n");
    printf("  cannot reach. But a 100 cm trigger on depth alone would also\n");
    printf("  fire every time the wearer swims down a metre. Pairing the\n");
    printf("  low threshold with the wrist-off requirement resolves that:\n\n");

    static const struct {
        const char *name; double depth; bool wrist; bool want;
    } combo[] = {
        { "swimmer duck-dives 2 m, watch worn", 2.0, true,  false },
        { "snorkeler at 3 m, watch worn",       3.0, true,  false },
        { "watch detaches in 1.2 m of lake",    1.2, false, true  },
        { "watch detaches in 2.5 m of lake",    2.5, false, true  },
    };
    printf("  threshold = 100 cm, policy = DEPTH + WRIST-OFF\n");
    printf("  %-38s %-8s %s\n", "case", "expect", "result");
    hr();
    for (size_t i = 0; i < sizeof combo / sizeof combo[0]; i++) {
        bb_config_t cfg; bb_config_defaults(&cfg);
        cfg.trigger_depth_cm = 100;
        cfg.water = BB_WATER_FRESH;
        cfg.arm_policy = BB_ARM_DEPTH_WRIST_OFF;
        bb_ctx_t bb; bb_init(&bb, &cfg, 0);
        bb_set_surface_ref(&bb, 101325);

        sim_world_t w; sim_defaults(&w);
        w.env.surface_pa = 101325.0; w.env.rho_water = 998.2;
        uint32_t now = 0, next_sample = 0;
        while (now <= 20000u) {
            if (now >= next_sample) {
                next_sample = now + 1000u / bb_sample_rate_hz(&bb);
                w.depth_m = combo[i].depth;
                bb_on_sample(&bb, now, sim_read_pressure_pa(&w),
                             true, combo[i].wrist);
            }
            bb_on_tick(&bb, now);
            now += TICK_MS;
        }
        bool got = bb_is_beaconing(&bb);
        printf("  %-38s %-8s %s\n", combo[i].name,
               combo[i].want ? "FIRE" : "quiet",
               (got == combo[i].want) ? (got ? "FIRE  OK" : "quiet OK")
                                      : "** FAIL **");
    }
}

/* ========================================================================*/
int main(void)
{
    int fails = 0;

    printf("\n  COROS Water Recovery Mode -- Emergency Backlight Beacon\n");
    printf("  firmware simulation, %s %s\n", __DATE__, __TIME__);

    rule("1. DEPTH MATH (integer pressure -> depth)");
    fails += check_depth_math();

    rule("2. PRE-DIVE RUNTIME PREDICTION");
    print_runtime_table(300, "full charge");
    print_runtime_table(105, "35% left -- four hours into a fishing trip");

    rule("3. OVERBOARD SCENARIO");
    run_overboard();

    rule("4. FALSE-TRIGGER REJECTION");
    fails += run_false_trigger_suite();

    rule("5. BATTERY MODEL VERIFICATION");
    run_battery_verification();

    rule("6. SHALLOW WATER VIABILITY (the lake case)");
    run_shallow_water_sweep();

    rule(fails == 0 ? "ALL CHECKS PASSED" : "FAILURES PRESENT");
    if (fails) printf("  %d check(s) failed\n", fails);
    printf("\n");
    return fails ? 1 : 0;
}
