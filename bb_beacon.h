/* ============================================================================
 * bb_beacon.h -- Emergency Backlight Flash / Water Recovery Mode
 *
 * COROS feature request: "Emergency Backlight Flash for Watch Recovery in
 * Dark Water".  Depth-triggered beacon that turns the watch into a visible
 * strobe if it is lost overboard or detaches and sinks.
 *
 * Firmware-style reference implementation:
 *   - no floating point          (barometer math is integer / fixed-point)
 *   - no dynamic allocation      (single static context, caller-owned)
 *   - no blocking calls          (two periodic entry points, both O(1))
 *   - no assumptions about RTOS  (caller supplies a monotonic ms clock)
 *
 * Integration model
 * -----------------
 *   bb_init()             once, at boot / when the user enables the mode
 *   bb_set_surface_ref()  when the activity starts, at the surface
 *   bb_on_sample()        from the barometer sample callback  (4-8 Hz)
 *   bb_on_tick()          from the display timer ISR           (100 Hz)
 *   bb_cancel()           from the button handler (long press)
 *
 * bb_on_tick() returns the brightness the backlight driver should apply
 * right now.  It is the only thing that touches the LED, and it is cheap
 * enough to live in an ISR.
 * ==========================================================================*/

#ifndef BB_BEACON_H
#define BB_BEACON_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Tunables / limits
 * ---------------------------------------------------------------------- */
#define BB_MAX_PULSES            16u   /* max pulses in one pattern period  */
#define BB_SAMPLE_HZ_NOMINAL      4u   /* barometer rate while armed-idle   */
#define BB_SAMPLE_HZ_DESCENT      8u   /* boosted rate once water detected  */

/* -------------------------------------------------------------------------
 * Water type -- changes the pressure-to-depth constant by ~2.7%
 * (fresh 9789 Pa/m vs salt 10052 Pa/m).  At a 3 m threshold that is 8 cm:
 * irrelevant for triggering, but it matters if depth is ever displayed.
 * ---------------------------------------------------------------------- */
typedef enum {
    BB_WATER_FRESH = 0,
    BB_WATER_SALT  = 1
} bb_water_t;

/* -------------------------------------------------------------------------
 * Arming policy
 *
 * DEPTH_ONLY      trigger on depth alone. Correct for divers, who want the
 *                 beacon even while the watch is still on the wrist.
 * DEPTH_WRIST_OFF also require the wrist sensor to report "not on a body".
 *                 Correct for the overboard case, and it makes false
 *                 triggers during an intentional dive essentially
 *                 impossible. Costs nothing in the lost-watch scenario --
 *                 a watch on the sea floor is by definition wrist-off.
 * ---------------------------------------------------------------------- */
typedef enum {
    BB_ARM_DEPTH_ONLY     = 0,
    BB_ARM_DEPTH_WRIST_OFF = 1
} bb_arm_policy_t;

/* -------------------------------------------------------------------------
 * Flash patterns.  Duty cycle is the whole ballgame: it sets both how far
 * away the watch can be seen and how long the search window lasts.
 * ---------------------------------------------------------------------- */
typedef enum {
    BB_PATTERN_SOS = 0,      /* ...---...   unmistakably man-made          */
    BB_PATTERN_DOUBLE_TAP,   /* blink-blink, long gap. low duty, high
                                salience -- irregular motion is what the
                                human visual periphery actually locks on   */
    BB_PATTERN_DOUBLE_TAP_FAST, /* same pair, gap cut from 2.6 s to 1.1 s.
                                A searcher sweeping a light across the water
                                may only look at any one spot for a second
                                or two, so a 2.6 s gap can be missed
                                entirely. Costs 2x the charge.             */
    BB_PATTERN_STROBE_1HZ,   /* 50% duty. brightest average, shortest life */
    BB_PATTERN_RESCUE_SLOW,  /* 80 ms every 2 s, aviation-strobe cadence   */
    BB_PATTERN_COUNT
} bb_pattern_id_t;

/* -------------------------------------------------------------------------
 * State machine
 * ---------------------------------------------------------------------- */
typedef enum {
    BB_STATE_DISABLED = 0,   /* feature off in settings                    */
    BB_STATE_DRY,            /* armed and waiting, watch out of the water  */
    BB_STATE_WET_SHALLOW,    /* submerged, above the depth threshold       */
    BB_STATE_CONFIRMING,     /* past threshold, debounce in progress       */
    BB_STATE_BEACON,         /* strobing at full brightness                */
    BB_STATE_BEACON_RESERVE, /* strobing on the low-battery reserve budget */
    BB_STATE_DEPLETED,       /* battery exhausted                          */
    BB_STATE_CANCELLED,      /* user stopped it (long press)               */
    BB_STATE_COUNT
} bb_state_t;

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */
typedef struct {
    bool            enabled;              /* master switch in settings      */
    int32_t         trigger_depth_cm;     /* e.g. 305 cm == 10 ft           */
    bb_water_t      water;
    bb_arm_policy_t arm_policy;

    /* Debounce: consecutive past-threshold samples required before the
       beacon fires. At 8 Hz, 6 samples == 750 ms of continuous depth. */
    uint8_t         confirm_samples;

    /* Wet detection. Anything shallower than this is splash, rain, a
       hand wash, or a shower -- never a submerged watch. */
    int32_t         wet_depth_cm;

    bb_pattern_id_t pattern;
    bb_pattern_id_t reserve_pattern;      /* used below reserve_pct         */
    uint8_t         brightness_pct;       /* 100 = max                      */
    uint8_t         reserve_brightness_pct;
    uint8_t         reserve_pct;          /* switch point, % of capacity    */

    /* Battery model -- lets the watch tell the user, before the dive,
       how many hours of beacon it is actually carrying. */
    uint32_t        capacity_mah;
    uint32_t        led_full_ua;          /* backlight at 100% brightness   */
    uint32_t        idle_ua;              /* MCU + baro while beaconing     */
} bb_config_t;

/* Sensible defaults: 10 ft threshold, salt water, double-tap pattern. */
void bb_config_defaults(bb_config_t *cfg);

/* -------------------------------------------------------------------------
 * What the display layer should do right now
 * ---------------------------------------------------------------------- */
typedef struct {
    bool     led_on;
    uint8_t  brightness_pct;  /* 0 when led_on == false                    */
} bb_output_t;

/* -------------------------------------------------------------------------
 * Context.  Caller owns the storage; nothing here is allocated.
 * ---------------------------------------------------------------------- */
typedef struct {
    bb_config_t cfg;
    bb_state_t  state;

    int32_t  surface_pa;        /* reference taken at the surface          */
    int32_t  depth_cm;          /* latest filtered depth                   */
    int32_t  max_depth_cm;      /* deepest seen -- useful for recovery     */
    int32_t  filt_pa;           /* IIR-filtered pressure                   */
    bool     filt_primed;

    uint8_t  confirm_count;
    uint32_t trigger_ms;        /* when the beacon started                 */
    uint32_t last_tick_ms;
    uint32_t pattern_ms;        /* position inside the pattern period      */
    uint8_t  pulse_idx;

    /* Charge accounting in microamp-MILLISECONDS, so that a 10 ms tick at
       350 uA does not truncate to zero. 300 mAh == 1.08e12 uA*ms. */
    uint64_t budget_uams;
    uint64_t used_uams;
} bb_ctx_t;

/* -------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------- */
void bb_init(bb_ctx_t *c, const bb_config_t *cfg, uint32_t now_ms);

/* Take the surface pressure reference. Call at the start of the activity,
   while the watch is out of the water. Without this the depth math is
   referenced to a stale sea-level assumption. */
void bb_set_surface_ref(bb_ctx_t *c, int32_t pressure_pa);

/* Barometer callback. water_contact comes from the capacitive wet sensor,
   wrist_on from the optical HR / skin-contact detector. */
void bb_on_sample(bb_ctx_t *c, uint32_t now_ms, int32_t pressure_pa,
                  bool water_contact, bool wrist_on);

/* Display timer callback. Returns the brightness to drive right now. */
bb_output_t bb_on_tick(bb_ctx_t *c, uint32_t now_ms);

/* Long-press cancel. Irreversible for this activity, by design: a beacon
   that can be stopped by a bump is not a beacon. */
void bb_cancel(bb_ctx_t *c);

/* -------------------------------------------------------------------------
 * Introspection -- UI, logging, and the pre-dive "you have N hours of
 * beacon" estimate.
 * ---------------------------------------------------------------------- */
bb_state_t  bb_state(const bb_ctx_t *c);
const char *bb_state_name(bb_state_t s);
const char *bb_pattern_name(bb_pattern_id_t p);
int32_t     bb_depth_cm(const bb_ctx_t *c);
uint8_t     bb_battery_pct(const bb_ctx_t *c);
bool        bb_is_beaconing(const bb_ctx_t *c);

/* Required barometer sample rate right now, in Hz. Drops back to the
   nominal rate when dry to save power. */
uint8_t     bb_sample_rate_hz(const bb_ctx_t *c);

/* Average duty cycle of a pattern, in tenths of a percent (450 == 45.0%). */
uint16_t    bb_pattern_duty_permille(bb_pattern_id_t p);

/* Predicted beacon runtime in seconds for a given config, computed from
   the duty cycle and the battery budget. Pure function -- safe to call
   from the settings screen before the activity starts. */
uint32_t    bb_predict_runtime_s(const bb_config_t *cfg);

/* Pressure -> depth, exposed for test. */
int32_t     bb_depth_from_pa(int32_t pressure_pa, int32_t surface_pa,
                             bb_water_t water);

#endif /* BB_BEACON_H */
