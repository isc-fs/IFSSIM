/* How fast does an FMU step, called the way the simulator calls it?
 *
 * Loads the FMU's binary exactly as reload_probe (and FSDSFmi3.cpp) does --
 * Co-Simulation, eventModeUsed false -- and steps it at the simulator's
 * 1/60 s communication step for a stretch of simulated time, timing the wall
 * clock. The answer is a REAL-TIME FACTOR: simulated seconds per wall second.
 * The simulator needs it comfortably above 1 for the WHOLE car, so a part's
 * factor is a share of a budget, not a pass mark on its own.
 *
 * Optionally drives one Float64 input (by value reference) from 0 to a value
 * at a time, so a mechanism that only does work under load is timed under it.
 *
 *   step_timing <dylib> <token> [seconds] [inputVR inputValue stepTime]
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

typedef void* fmi3Instance; typedef void* fmi3InstanceEnvironment; typedef const char* fmi3String;
typedef bool fmi3Boolean; typedef uint32_t fmi3ValueReference; typedef int fmi3Status; typedef double fmi3Float64;
typedef void (*fmi3LogMessageCallback)(fmi3InstanceEnvironment, fmi3Status, fmi3String, fmi3String);
typedef fmi3Instance (*PFN_Instantiate)(fmi3String, fmi3String, fmi3String, fmi3Boolean, fmi3Boolean, fmi3Boolean,
    fmi3Boolean, const fmi3ValueReference[], size_t, fmi3InstanceEnvironment, fmi3LogMessageCallback, void*);
typedef void (*PFN_Free)(fmi3Instance);
typedef fmi3Status (*PFN_Enter)(fmi3Instance, fmi3Boolean, fmi3Float64, fmi3Float64, fmi3Boolean, fmi3Float64);
typedef fmi3Status (*PFN_Exit)(fmi3Instance);
typedef fmi3Status (*PFN_Step)(fmi3Instance, fmi3Float64, fmi3Float64, fmi3Boolean, fmi3Boolean*, fmi3Boolean*, fmi3Boolean*, fmi3Float64*);
typedef fmi3Status (*PFN_SetF64)(fmi3Instance, const fmi3ValueReference[], size_t, const fmi3Float64[], size_t);

static int g_err = 0;
static void on_log(fmi3InstanceEnvironment e, fmi3Status s, fmi3String c, fmi3String m) {
    (void)e; (void)c; if (s >= 2) { g_err++; if (g_err < 5) printf("  [fmu] status %d: %s\n", s, m ? m : ""); }
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: step_timing <dylib> <token> [seconds] [vr value t]\n"); return 2; }
    double T = argc > 3 ? atof(argv[3]) : 10.0;
    int hasIn = argc > 6;
    fmi3ValueReference vr = hasIn ? (fmi3ValueReference)atoi(argv[4]) : 0;
    double uval = hasIn ? atof(argv[5]) : 0.0, ut = hasIn ? atof(argv[6]) : 0.0;
    void* h = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!h) { printf("  [FAIL] dlopen: %s\n", dlerror()); return 1; }
    PFN_Instantiate inst = (PFN_Instantiate)dlsym(h, "fmi3InstantiateCoSimulation");
    PFN_Free freei = (PFN_Free)dlsym(h, "fmi3FreeInstance");
    PFN_Enter enter = (PFN_Enter)dlsym(h, "fmi3EnterInitializationMode");
    PFN_Exit exit_ = (PFN_Exit)dlsym(h, "fmi3ExitInitializationMode");
    PFN_Step step = (PFN_Step)dlsym(h, "fmi3DoStep");
    PFN_SetF64 setf = (PFN_SetF64)dlsym(h, "fmi3SetFloat64");
    fmi3Instance c = inst("timing", argv[2], NULL, false, false, false, false, NULL, 0, NULL, on_log, NULL);
    if (!c || enter(c, false, 0.0, 0.0, true, 1e5) || exit_(c)) { printf("  [FAIL] could not initialise\n"); return 1; }
    const double dt = 1.0 / 60.0;
    int n = (int)(T / dt + 0.5), done = 0;
    fmi3Boolean ev, term, disc; fmi3Float64 last;
    double t0 = now(), worst = 0;
    for (int k = 0; k < n; k++) {
        double t = k * dt;
        if (hasIn) { fmi3Float64 u = t >= ut ? uval : 0.0; setf(c, &vr, 1, &u, 1); }
        double a = now();
        if (step(c, t, dt, true, &ev, &term, &disc, &last)) { printf("  [FAIL] doStep at t=%.3f\n", t); break; }
        double w = now() - a; if (w > worst) worst = w;
        done++;
    }
    double wall = now() - t0;
    freei(c); dlclose(h);
    printf("  %d steps of 1/60 s (%.1f s simulated) in %.3f s wall\n", done, done * dt, wall);
    printf("  real-time factor %.1fx   mean %.3f ms per 1/60 s step   worst %.3f ms (budget 16.7)\n",
           done * dt / wall, 1e3 * wall / done, 1e3 * worst);
    return (done == n && g_err == 0) ? 0 : 1;
}
