// Motor test target entry point.
//
// PARTIAL (chunk 7.4a): this build brings the pins up and proves they are
// where we think they are, then sits there reporting its position. It does
// not read commands yet - the command loop arrives in 7.4b. Everything it
// needs for that (line reader, parser, formatter, motion maths) is already
// written and tested on the laptop.
//
// Deliberately absent, and staying absent:
//   no radio          the test procedure says to disconnect it
//   no ENA, no ALM    not wired on this board; the Pico cannot disable a
//                     drive and cannot tell when one faults
//   no limit switches none fitted; the soft limits are all there is
//   no encoder        position here is what we believe, not what happened
//
// Safe to run with the driver supply OFF, which is how bring-up starts.
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "config.hpp"
#include "motor_reply.hpp"
#include "stepper.hpp"

// How often the idle heartbeat goes out (spec 5.4 uses 5 s for the receiver;
// same cadence here so one serial monitor looks familiar after the other).
constexpr uint32_t MSTAT_PERIOD_MS = 5000;

static Stepper g_az(AZ_SPEC, PIN_AZ_PUL, PIN_AZ_DIR);
static Stepper g_el(EL_SPEC, PIN_EL_PUL, PIN_EL_DIR);

// This target lives on the laptop's USB port, so it waits for a program to
// open the port rather than printing its banner into nothing (spec 5.3).
static void wait_for_usb_forever()
{
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(200);  // let the host finish opening before the first line
}

// Print a millidegree value as plain degrees, without pulling in floating
// point printf. 45006 comes out as "45.006".
static void print_mdeg(int32_t mdeg)
{
    const long whole = mdeg / 1000;
    long frac        = mdeg % 1000;
    if (frac < 0) {
        frac = -frac;
    }
    // The sign lives on the whole part, except between -1 and 0 where the
    // whole part is 0 and would lose it.
    if (mdeg < 0 && whole == 0) {
        printf("-0.%03ld", frac);
    } else {
        printf("%ld.%03ld", whole, frac);
    }
}

// Steps per degree, to three decimals, again without floating point:
// asking for 1000 degrees gives the answer already scaled by 1000.
static void print_steps_per_deg(const AxisSpec &axis)
{
    const int32_t milli = mdeg_to_steps(axis, 1000000);
    printf("%ld.%03ld", static_cast<long>(milli / 1000),
           static_cast<long>(milli % 1000));
}

// Spec 5.3 step 6: on any failure, say so every 2 seconds forever. Never die
// quietly, and never carry on as if nothing happened.
static void fatal_forever(const char *reason)
{
    for (;;) {
        printf("[fatal] %s\n", reason);
        sleep_ms(2000);
    }
}

int main()
{
    stdio_init_all();
    wait_for_usb_forever();

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);

    g_az.init();
    g_el.init();

    // Read the pins back. This proves they are configured and sitting at the
    // idle levels; it cannot tell whether a driver is plugged in.
    if (!g_az.pins_ok()) {
        fatal_forever("azimuth pins did not read back idle HIGH");
    }
    if (!g_el.pins_ok()) {
        fatal_forever("elevation pins did not read back idle HIGH");
    }

    printf("[motor] pins AZ PUL=GP%u DIR=GP%u  EL PUL=GP%u DIR=GP%u  idle HIGH OK\n",
           PIN_AZ_PUL, PIN_AZ_DIR, PIN_EL_PUL, PIN_EL_DIR);

    printf("[motor] active low: PUL rests HIGH and is pulled LOW to step\n");

    printf("[motor] ready az=");
    print_steps_per_deg(AZ_SPEC);
    printf(" st/deg el=");
    print_steps_per_deg(EL_SPEC);
    printf(" st/deg\n");

    printf("[motor] limits az ");
    print_mdeg(AZ_SPEC.min_mdeg);
    printf("..");
    print_mdeg(AZ_SPEC.max_mdeg);
    printf("  el ");
    print_mdeg(EL_SPEC.min_mdeg);
    printf("..");
    print_mdeg(EL_SPEC.max_mdeg);
    printf("\n");

    printf("[motor] NOT ZEROED - send ZERO before any MOVE\n");
    printf("[motor] chunk 7.4a: no command loop yet, heartbeat only\n");

    // Scheduled on absolute time so the period holds once there is real work
    // in this loop (spec 5.2.3).
    char line[MAX_REPLY_LEN];
    absolute_time_t next = get_absolute_time();
    for (;;) {
        next = delayed_by_ms(next, MSTAT_PERIOD_MS);
        sleep_until(next);

        // Built whole, then sent in one call, so nothing can land mid-line
        // (spec line 168).
        const size_t n = format_mstat(line, sizeof(line),
                                      to_ms_since_boot(get_absolute_time()) / 1000,
                                      g_az.pos_mdeg(), g_el.pos_mdeg(), false);
        if (n > 0) {
            fputs(line, stdout);
        }
    }
}
