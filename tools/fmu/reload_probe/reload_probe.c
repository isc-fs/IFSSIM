/* Does this FMU survive instantiate -> free -> instantiate in ONE process?
 *
 * inspect_fmu.py raises a WARN because Simulink declares
 * canBeInstantiatedOnlyOncePerProcess = true: its generated code is not
 * reentrant. That is an honest statement about the code, and it is NOT the
 * same claim as "cannot be reloaded". IFSSIM never needs two cars at once; it
 * needs to load a car, drop it, and load one again in the same editor session.
 *
 * The inspector's own note is the standard this answers: sequential reload
 * must be PROVEN with instantiate/free/instantiate, not assumed and not
 * waved away by overriding the flag. So this dlopens the FMU exactly as
 * FSDSFmi3.cpp does and runs the cycle for real.
 *
 * Two cycles, not one. A single instantiate/free proves nothing about state
 * left behind; the failure this is looking for shows up on the SECOND
 * instantiate, after the first instance's statics have been touched.
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef void*          fmi3Instance;
typedef void*          fmi3InstanceEnvironment;
typedef const char*    fmi3String;
typedef bool           fmi3Boolean;
typedef uint32_t       fmi3ValueReference;
typedef int            fmi3Status;
typedef double         fmi3Float64;

typedef void (*fmi3LogMessageCallback)(fmi3InstanceEnvironment, fmi3Status, fmi3String, fmi3String);
typedef void (*fmi3IntermediateUpdateCallback)(void);

typedef fmi3Instance (*PFN_Instantiate)(
    fmi3String, fmi3String, fmi3String,
    fmi3Boolean, fmi3Boolean, fmi3Boolean, fmi3Boolean,
    const fmi3ValueReference[], size_t,
    fmi3InstanceEnvironment, fmi3LogMessageCallback, fmi3IntermediateUpdateCallback);
typedef void       (*PFN_FreeInstance)(fmi3Instance);
typedef fmi3Status (*PFN_EnterInit)(fmi3Instance, fmi3Boolean, fmi3Float64, fmi3Float64, fmi3Boolean, fmi3Float64);
typedef fmi3Status (*PFN_ExitInit)(fmi3Instance);
typedef fmi3Status (*PFN_DoStep)(fmi3Instance, fmi3Float64, fmi3Float64, fmi3Boolean,
                                 fmi3Boolean*, fmi3Boolean*, fmi3Boolean*, fmi3Float64*);

static int g_errors = 0;
static void on_log(fmi3InstanceEnvironment env, fmi3Status s, fmi3String cat, fmi3String msg) {
    (void)env; (void)cat;
    if (s >= 2) { g_errors++; printf("        [fmu] status %d: %s\n", s, msg ? msg : "(null)"); }
}

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: reload_probe <dylib> <token>\n"); return 2; }
    const char* lib   = argv[1];
    const char* token = argv[2];

    void* h = dlopen(lib, RTLD_NOW | RTLD_LOCAL);
    if (!h) { printf("  [FAIL] dlopen: %s\n", dlerror()); return 1; }

    PFN_Instantiate  inst  = (PFN_Instantiate)  dlsym(h, "fmi3InstantiateCoSimulation");
    PFN_FreeInstance freei = (PFN_FreeInstance) dlsym(h, "fmi3FreeInstance");
    PFN_EnterInit    enter = (PFN_EnterInit)    dlsym(h, "fmi3EnterInitializationMode");
    PFN_ExitInit     exit_ = (PFN_ExitInit)     dlsym(h, "fmi3ExitInitializationMode");
    PFN_DoStep       step  = (PFN_DoStep)       dlsym(h, "fmi3DoStep");
    if (!inst || !freei || !enter || !exit_ || !step) {
        printf("  [FAIL] missing entry points\n"); dlclose(h); return 1;
    }

    int ok = 1, failed_cycle = 0;
    for (int cycle = 1; cycle <= 3; cycle++) {
        /* Argument order is visible, loggingOn, eventModeUsed, earlyReturnAllowed.
         * eventModeUsed MUST be false to match FSDSFmi3.cpp:138, whose own
         * comment is the reason: "the FMU may advertise hasEventMode, but
         * opting in changes the calling protocol and is not something to
         * enable by accident". This probe passed true, so it was proving
         * reload for a protocol the platform does not use -- the right answer
         * about the wrong thing. */
        fmi3Instance c = inst("probe", token, NULL,
                              false, false, false, false,
                              NULL, 0, NULL, on_log, NULL);
        if (!c) {
            printf("  [FAIL] cycle %d: fmi3InstantiateCoSimulation returned NULL\n", cycle);
            ok = 0; failed_cycle = cycle; break;
        }
        /* Instantiating is the cheap half. Actually stepping is what touches
         * the generated code's statics, which is where non-reentrancy bites. */
        if (enter(c, false, 0.0, 0.0, true, 1e5) != 0) {
            printf("  [FAIL] cycle %d: EnterInitializationMode\n", cycle); ok = 0; failed_cycle = cycle; freei(c); break;
        }
        if (exit_(c) != 0) {
            printf("  [FAIL] cycle %d: ExitInitializationMode\n", cycle); ok = 0; failed_cycle = cycle; freei(c); break;
        }
        fmi3Boolean ev = false, term = false, disc = false; fmi3Float64 last = 0.0;
        int bad = 0;
        for (int k = 0; k < 60; k++) {
            if (step(c, k / 60.0, 1.0 / 60.0, true, &ev, &term, &disc, &last) != 0) { bad = 1; break; }
        }
        if (bad) { printf("  [FAIL] cycle %d: fmi3DoStep\n", cycle); ok = 0; failed_cycle = cycle; freei(c); break; }
        freei(c);
        printf("  [ok  ] cycle %d: instantiate, init, 60 steps, free\n", cycle);
    }

    dlclose(h);
    if (ok && g_errors == 0) { printf("\n  Sequential reload WORKS: 3 full cycles in one process.\n"); return 0; }
    if (ok) { printf("\n  Cycles completed but the FMU logged %d error(s).\n", g_errors); return 1; }

    /* WHICH cycle failed is the whole diagnosis, and conflating them is how a
     * bad argument gets reported as a reload defect. Cycle 1 failing says
     * nothing about reload -- the FMU never loaded once. Only a failure on a
     * LATER cycle, after a successful first, is evidence about reentrancy. */
    if (failed_cycle <= 1) {
        printf("\n  The FIRST cycle failed, so this says nothing about reload:\n"
               "  the FMU never came up once. Check the instantiation token and\n"
               "  the library path before reading anything into it.\n");
    } else {
        printf("\n  Cycle 1 succeeded and cycle %d did not. THAT is a reload\n"
               "  failure: the multi-instance flag is a real constraint here and\n"
               "  the platform cannot reload the car in one session.\n", failed_cycle);
    }
    return 1;
}
