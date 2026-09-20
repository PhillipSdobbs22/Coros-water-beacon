#include "sim_world.h"
#include <math.h>

#define G 9.80665

void sim_defaults(sim_world_t *w)
{
    /* 46 mm multisport watch, titanium bezel, nylon band. */
    w->watch.mass_kg      = 0.060;
    w->watch.volume_m3    = 30.0e-6;      /* ~30 cm^3 -> rho ~2000 kg/m^3 */
    w->watch.frontal_a_m2 = 1.66e-3;      /* pi * (23 mm)^2               */
    w->watch.cd           = 1.10;         /* tumbling disc                */

    w->env.rho_water     = 1025.0;        /* sea water                    */
    w->env.bottom_m      = 18.0;          /* nearshore Gulf bottom        */
    w->env.surface_pa    = 101120.0;      /* a mild low that morning      */
    w->env.baro_noise_pa = 40.0;

    w->t         = 0.0;
    w->depth_m   = -1.5;                  /* worn, 1.5 m above the water  */
    w->v_ms      = 0.0;
    w->released  = false;
    w->in_water  = false;
    w->on_bottom = false;
    w->rng       = 0x13572468u;
}

void sim_step(sim_world_t *w, double dt)
{
    w->t += dt;

    if (!w->released || w->on_bottom) {
        return;
    }

    double a;
    if (w->depth_m < 0.0) {
        /* Free fall through air -- drag is negligible over 1.5 m. */
        a = G;
    } else {
        w->in_water = true;
        double weight    = w->watch.mass_kg * G;
        double buoyancy  = w->env.rho_water * w->watch.volume_m3 * G;
        double drag      = 0.5 * w->env.rho_water * w->watch.cd *
                           w->watch.frontal_a_m2 * w->v_ms * fabs(w->v_ms);
        a = (weight - buoyancy - drag) / w->watch.mass_kg;
    }

    w->v_ms   += a * dt;
    w->depth_m += w->v_ms * dt;

    if (w->depth_m >= w->env.bottom_m) {
        w->depth_m   = w->env.bottom_m;
        w->v_ms      = 0.0;
        w->on_bottom = true;
    }
}

/* Uniform-ish noise from a small LCG -- deterministic, so runs repeat. */
static double noise(sim_world_t *w, double peak)
{
    w->rng = w->rng * 1664525u + 1013904223u;
    double u = (double)((w->rng >> 8) & 0xFFFFu) / 65535.0;  /* 0..1 */
    return (u * 2.0 - 1.0) * peak;
}

int32_t sim_read_pressure_pa(sim_world_t *w)
{
    double d = w->depth_m > 0.0 ? w->depth_m : 0.0;
    double p = w->env.surface_pa + w->env.rho_water * G * d;
    return (int32_t)(p + noise(w, w->env.baro_noise_pa) + 0.5);
}

bool sim_read_water_contact(const sim_world_t *w)
{
    return w->depth_m > 0.0;
}

bool sim_read_wrist_on(const sim_world_t *w)
{
    return !w->released;
}
