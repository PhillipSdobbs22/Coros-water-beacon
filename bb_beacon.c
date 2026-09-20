/* ============================================================================
 * bb_beacon.c -- Emergency Backlight Flash / Water Recovery Mode
 * Reference implementation. Integer math only.
 * ==========================================================================*/

#include "bb_beacon.h"

/* ---------------------------------------------------------------------------
 * Pressure -> depth
 *
 * Hydrostatic: dP = rho * g * h.  Using rho*g in pascals per metre lets the
 * whole conversion stay in 32-bit integers:
 *
 *      depth_cm = (dP_pa * 100 + half) / pa_per_m
 *
 * Fresh water  998.2 kg/m^3 * 9.807 = 9789 Pa/m
 * Sea water   1025.0 kg/m^3 * 9.807 = 10052 Pa/m
 *
 * Range check: 30 m of sea water is 301 560 Pa.  Times 100 is 3.0e7, which
 * is well inside int32 (2.1e9).  No overflow, no saturation logic needed.
 * ------------------------------------------------------------------------*/
#define PA_PER_M_FRESH  9789
#define PA_PER_M_SALT  10052

int32_t bb_depth_from_pa(int32_t pressure_pa, int32_t surface_pa,
                         bb_water_t water)
{
    int32_t dp = pressure_pa - surface_pa;
    int32_t k  = (water == BB_WATER_SALT) ? PA_PER_M_SALT : PA_PER_M_FRESH;

    if (dp <= 0) {
        return 0;                       /* at or above the surface */
    }
    return (dp * 100 + k / 2) / k;
}

/* ---------------------------------------------------------------------------
 * Flash pattern tables
 *
 * Every pattern is a list of (on_ms, off_ms) pulses that repeats.  Keeping
 * them as data rather than code means a new pattern is a table entry, and
 * the duty cycle -- which is what actually determines runtime -- can be
 * computed rather than asserted.
 * ------------------------------------------------------------------------*/
typedef struct { uint16_t on_ms; uint16_t off_ms; } bb_pulse_t;

typedef struct {
    const char       *name;
    const bb_pulse_t *pulses;
    uint8_t           n;
} bb_pattern_t;

/* ... --- ...   International distress. Slow, but unambiguously man-made:
   a searcher who sees it knows it is not a reflection or a boat light. */
static const bb_pulse_t k_sos[] = {
    {200,200},{200,200},{200,600},        /* S */
    {600,200},{600,200},{600,600},        /* O */
    {200,200},{200,200},{200,1400}        /* S + word gap */
};

/* Two fast blinks, then a long dark gap.  Low duty, but the paired-flash
   cadence is far easier to pick out of a moving, reflective background
   than a steady strobe -- and it costs about a sixth of the charge. */
static const bb_pulse_t k_double[] = {
    {120,120},{120,2640}
};

/* The same pair on a 1.5 s period instead of 3 s. The long gap is what
   makes DOUBLE_TAP cheap, and also what makes it easy to miss: a searcher
   panning a torch across open water may hold any one spot for only a
   second. Halving the gap doubles the duty and halves the search window --
   a deliberate trade of endurance for the chance of being seen at all. */
static const bb_pulse_t k_double_fast[] = {
    {120,120},{120,1140}
};

/* 50% duty.  Highest average luminance and therefore the greatest range
   in murky water, at the price of the shortest search window. */
static const bb_pulse_t k_strobe[] = {
    {500,500}
};

/* Aviation anti-collision cadence: one short, bright pulse every 2 s.
   The cheapest pattern that still reads as a deliberate signal. */
static const bb_pulse_t k_rescue[] = {
    {80,1920}
};

static const bb_pattern_t k_patterns[BB_PATTERN_COUNT] = {
    [BB_PATTERN_SOS]         = { "SOS",         k_sos,    9 },
    [BB_PATTERN_DOUBLE_TAP]  = { "DOUBLE_TAP",  k_double, 2 },
    [BB_PATTERN_DOUBLE_TAP_FAST] = { "DOUBLE_TAP_FAST", k_double_fast, 2 },
    [BB_PATTERN_STROBE_1HZ]  = { "STROBE_1HZ",  k_strobe, 1 },
    [BB_PATTERN_RESCUE_SLOW] = { "RESCUE_SLOW", k_rescue, 1 }
};

static const bb_pattern_t *pattern_of(bb_pattern_id_t p)
{
    if ((unsigned)p >= (unsigned)BB_PATTERN_COUNT) {
        p = BB_PATTERN_DOUBLE_TAP;
    }
    return &k_patterns[p];
}

static uint32_t pattern_period_ms(const bb_pattern_t *pt)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < pt->n; i++) {
        sum += (uint32_t)pt->pulses[i].on_ms + pt->pulses[i].off_ms;
    }
    return sum ? sum : 1u;
}

static uint32_t pattern_on_ms(const bb_pattern_t *pt)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < pt->n; i++) {
        sum += pt->pulses[i].on_ms;
    }
    return sum;
}

uint16_t bb_pattern_duty_permille(bb_pattern_id_t p)
{
    const bb_pattern_t *pt = pattern_of(p);
    return (uint16_t)((pattern_on_ms(pt) * 1000u) / pattern_period_ms(pt));
}

const char *bb_pattern_name(bb_pattern_id_t p) { return pattern_of(p)->name; }

/* ---------------------------------------------------------------------------
 * Config
 * ------------------------------------------------------------------------*/
void bb_config_defaults(bb_config_t *cfg)
{
    if (!cfg) return;

    cfg->enabled                = true;
    cfg->trigger_depth_cm       = 305;              /* 10 ft               */
    cfg->water                  = BB_WATER_SALT;
    cfg->arm_policy             = BB_ARM_DEPTH_ONLY;
    cfg->confirm_samples        = 6;                /* 750 ms at 8 Hz      */
    cfg->wet_depth_cm           = 40;               /* deeper than splash  */

    cfg->pattern                = BB_PATTERN_DOUBLE_TAP_FAST;
    cfg->reserve_pattern        = BB_PATTERN_RESCUE_SLOW;
    cfg->brightness_pct         = 100;
    cfg->reserve_brightness_pct = 70;
    cfg->reserve_pct            = 25;

    /* Representative of a mid-size COROS-class watch. */
    cfg->capacity_mah           = 300;
    cfg->led_full_ua            = 24000;            /* 24 mA backlight     */
    cfg->idle_ua                = 350;              /* MCU + baro + wet    */
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------*/
void bb_init(bb_ctx_t *c, const bb_config_t *cfg, uint32_t now_ms)
{
    if (!c || !cfg) return;

    c->cfg          = *cfg;
    c->state        = cfg->enabled ? BB_STATE_DRY : BB_STATE_DISABLED;
    c->surface_pa   = 101325;          /* until bb_set_surface_ref() */
    c->depth_cm     = 0;
    c->max_depth_cm = 0;
    c->filt_pa      = 0;
    c->filt_primed  = false;
    c->confirm_count = 0;
    c->trigger_ms   = 0;
    c->last_tick_ms = now_ms;
    c->pattern_ms   = 0;
    c->pulse_idx    = 0;

    /* 1 mAh = 3.6e6 uA*s = 3.6e9 uA*ms */
    c->budget_uams = (uint64_t)cfg->capacity_mah * 3600000ull * 1000ull;
    c->used_uams   = 0;
}

void bb_set_surface_ref(bb_ctx_t *c, int32_t pressure_pa)
{
    if (!c) return;
    c->surface_pa  = pressure_pa;
    c->filt_pa     = pressure_pa;
    c->filt_primed = true;
}

void bb_cancel(bb_ctx_t *c)
{
    if (!c) return;
    if (c->state == BB_STATE_BEACON || c->state == BB_STATE_BEACON_RESERVE) {
        c->state = BB_STATE_CANCELLED;
    }
}

/* ---------------------------------------------------------------------------
 * Sensor path
 * ------------------------------------------------------------------------*/
static bool state_is_latched(bb_state_t s)
{
    return s == BB_STATE_BEACON || s == BB_STATE_BEACON_RESERVE ||
           s == BB_STATE_DEPLETED || s == BB_STATE_CANCELLED;
}

static void enter_beacon(bb_ctx_t *c, uint32_t now_ms)
{
    c->state      = BB_STATE_BEACON;
    c->trigger_ms = now_ms;
    c->pattern_ms = 0;
    c->pulse_idx  = 0;
}

void bb_on_sample(bb_ctx_t *c, uint32_t now_ms, int32_t pressure_pa,
                  bool water_contact, bool wrist_on)
{
    if (!c) return;

    if (!c->cfg.enabled) {
        c->state = BB_STATE_DISABLED;
        return;
    }

    /* First-order IIR, alpha = 1/4.  Barometer noise on these parts is
       roughly +/-30 Pa RMS, which is +/-3 cm of water -- small next to a
       3 m threshold, but the filter keeps a single spike from starting
       the debounce counter for free. */
    if (!c->filt_primed) {
        c->filt_pa     = pressure_pa;
        c->filt_primed = true;
    } else {
        c->filt_pa += (pressure_pa - c->filt_pa) / 4;
    }

    c->depth_cm = bb_depth_from_pa(c->filt_pa, c->surface_pa, c->cfg.water);
    if (c->depth_cm > c->max_depth_cm) {
        c->max_depth_cm = c->depth_cm;
    }

    /* Once it fires, it stays fired.  Only the user or a flat battery
       stops it -- that is the whole point of a recovery beacon. */
    if (state_is_latched(c->state)) {
        return;
    }

    bool wet = water_contact || (c->depth_cm >= c->cfg.wet_depth_cm);

    bool policy_ok = true;
    if (c->cfg.arm_policy == BB_ARM_DEPTH_WRIST_OFF) {
        policy_ok = !wrist_on;
    }

    bool past = wet && policy_ok && (c->depth_cm >= c->cfg.trigger_depth_cm);

    switch (c->state) {

    case BB_STATE_DRY:
        if (wet) {
            c->state = BB_STATE_WET_SHALLOW;
        }
        break;

    case BB_STATE_WET_SHALLOW:
        if (!wet) {
            c->state = BB_STATE_DRY;
        } else if (past) {
            c->state         = BB_STATE_CONFIRMING;
            c->confirm_count = 1;
            if (c->confirm_count >= c->cfg.confirm_samples) {
                enter_beacon(c, now_ms);
            }
        }
        break;

    case BB_STATE_CONFIRMING:
        if (past) {
            if (c->confirm_count < 255) {
                c->confirm_count++;
            }
            if (c->confirm_count >= c->cfg.confirm_samples) {
                enter_beacon(c, now_ms);
            }
        } else {
            /* Came back up, or the wrist sensor re-acquired.  Reset the
               counter completely -- no partial credit, or a watch bobbing
               at the threshold would eventually trip it. */
            c->confirm_count = 0;
            c->state = wet ? BB_STATE_WET_SHALLOW : BB_STATE_DRY;
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Display path
 * ------------------------------------------------------------------------*/
static uint32_t led_current_ua(const bb_ctx_t *c, uint8_t brightness_pct)
{
    /* Backlight current is close enough to linear in PWM duty for a
       budget model; the LED's own I-V curve does not move with PWM. */
    return (c->cfg.led_full_ua * brightness_pct) / 100u;
}

bb_output_t bb_on_tick(bb_ctx_t *c, uint32_t now_ms)
{
    bb_output_t out = { false, 0 };
    if (!c) return out;

    uint32_t dt = now_ms - c->last_tick_ms;   /* wraps correctly */
    c->last_tick_ms = now_ms;

    if (c->state != BB_STATE_BEACON && c->state != BB_STATE_BEACON_RESERVE) {
        return out;
    }

    bool reserve = (c->state == BB_STATE_BEACON_RESERVE);
    uint8_t bright = reserve ? c->cfg.reserve_brightness_pct
                             : c->cfg.brightness_pct;
    const bb_pattern_t *pt = pattern_of(reserve ? c->cfg.reserve_pattern
                                                : c->cfg.pattern);

    /* --- where are we in the pattern? --- */
    uint32_t period = pattern_period_ms(pt);
    c->pattern_ms = (c->pattern_ms + dt) % period;

    uint32_t acc = 0;
    bool on = false;
    for (uint8_t i = 0; i < pt->n; i++) {
        uint32_t on_ms  = pt->pulses[i].on_ms;
        uint32_t off_ms = pt->pulses[i].off_ms;
        if (c->pattern_ms < acc + on_ms) {
            on = true;
            c->pulse_idx = i;
            break;
        }
        acc += on_ms;
        if (c->pattern_ms < acc + off_ms) {
            on = false;
            c->pulse_idx = i;
            break;
        }
        acc += off_ms;
    }

    /* --- charge accounting --- */
    uint64_t spend = (uint64_t)c->cfg.idle_ua * dt;
    if (on) {
        spend += (uint64_t)led_current_ua(c, bright) * dt;
    }
    c->used_uams += spend;

    if (c->used_uams >= c->budget_uams) {
        c->used_uams = c->budget_uams;
        c->state = BB_STATE_DEPLETED;
        return out;                       /* LED off, nothing left */
    }

    /* --- reserve transition ---
       Compare raw charge, not bb_battery_pct().  The percentage is an
       integer floor, so "<= 25%" can fire anywhere in 25.00..25.99%,
       which stretches the reserve phase by up to 4% and makes the
       runtime estimate on the settings screen wrong by the same amount.
       3.78e11 * 100 is 3.78e13 -- comfortably inside uint64. */
    uint64_t left = c->budget_uams - c->used_uams;
    if (!reserve && left * 100ull <= c->budget_uams * (uint64_t)c->cfg.reserve_pct) {
        c->state      = BB_STATE_BEACON_RESERVE;
        c->pattern_ms = 0;
        c->pulse_idx  = 0;
        /* Fall through with this tick's LED state; the next tick picks up
           the reserve pattern. */
    }

    out.led_on         = on;
    out.brightness_pct = on ? bright : 0;
    return out;
}

/* ---------------------------------------------------------------------------
 * Introspection
 * ------------------------------------------------------------------------*/
bb_state_t bb_state(const bb_ctx_t *c) { return c ? c->state : BB_STATE_DISABLED; }
int32_t bb_depth_cm(const bb_ctx_t *c) { return c ? c->depth_cm : 0; }

bool bb_is_beaconing(const bb_ctx_t *c)
{
    return c && (c->state == BB_STATE_BEACON ||
                 c->state == BB_STATE_BEACON_RESERVE);
}

uint8_t bb_battery_pct(const bb_ctx_t *c)
{
    if (!c || c->budget_uams == 0) return 0;
    uint64_t left = c->budget_uams - c->used_uams;
    return (uint8_t)((left * 100ull) / c->budget_uams);
}

uint8_t bb_sample_rate_hz(const bb_ctx_t *c)
{
    if (!c) return BB_SAMPLE_HZ_NOMINAL;
    switch (c->state) {
    case BB_STATE_WET_SHALLOW:
    case BB_STATE_CONFIRMING:
        return BB_SAMPLE_HZ_DESCENT;
    default:
        return BB_SAMPLE_HZ_NOMINAL;
    }
}

const char *bb_state_name(bb_state_t s)
{
    static const char *n[BB_STATE_COUNT] = {
        "DISABLED", "DRY", "WET_SHALLOW", "CONFIRMING",
        "BEACON", "BEACON_RESERVE", "DEPLETED", "CANCELLED"
    };
    return ((unsigned)s < (unsigned)BB_STATE_COUNT) ? n[s] : "?";
}

/* Predicted runtime, main budget then reserve budget.  This is the number
   the settings screen should show the user before they get in the water:
   "DOUBLE_TAP -- about 14 h of beacon at 100% charge". */
uint32_t bb_predict_runtime_s(const bb_config_t *cfg)
{
    if (!cfg) return 0;

    uint64_t cap_uas = (uint64_t)cfg->capacity_mah * 3600000ull;
    uint64_t res_uas = (cap_uas * cfg->reserve_pct) / 100ull;
    uint64_t main_uas = cap_uas - res_uas;

    uint32_t d1 = bb_pattern_duty_permille(cfg->pattern);
    uint32_t d2 = bb_pattern_duty_permille(cfg->reserve_pattern);

    uint64_t i1 = ((uint64_t)cfg->led_full_ua * cfg->brightness_pct / 100ull)
                      * d1 / 1000ull + cfg->idle_ua;
    uint64_t i2 = ((uint64_t)cfg->led_full_ua * cfg->reserve_brightness_pct / 100ull)
                      * d2 / 1000ull + cfg->idle_ua;

    uint64_t t = 0;
    if (i1) t += main_uas / i1;
    if (i2) t += res_uas  / i2;
    return (uint32_t)t;
}
