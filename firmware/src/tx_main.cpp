// Transmitter entry point, shared by gs_tx, gs_tx_diag and gs_tx_gps.
//
// PLACEHOLDER (chunk 2.2): there is no radio in here yet. What it does do is the
// USB half of the startup sequence (spec 5.3 steps 1-3) plus a heartbeat, which
// is enough to test reading a Pico over USB serial from the laptop with nothing
// but a USB cable. The SPI self-test, radio.begin(), nmea_build and the real
// transmit loop arrive with radio.cpp.
//
// Which of the three transmitter builds this is comes from the compile
// definitions set per target in CMakeLists.txt - never from editing this file.
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "config.hpp"

#if GS_TX_USE_REAL_GPS
static const char *const TX_SOURCE = "GGA from a real GPS on UART0";
#elif GS_TX_MODE_DIAG
static const char *const TX_SOURCE = "diagnostic counter";
#else
static const char *const TX_SOURCE = "simulated GGA";
#endif

// How often the placeholder says "still here". The real cadences
// (TX_PERIOD_MS, STAT_PERIOD_MS) belong to the real loops, so this smoke-test
// heartbeat gets its own number rather than borrowing one of theirs.
constexpr uint32_t SMOKE_PERIOD_MS = 1000;

// A transmitter normally runs off a power bank with nothing watching, so it
// gives up after USB_WAIT_TX_MS and gets on with its job. Returns whether a
// host actually turned up, which is worth printing during a bench test.
// This moves into the shared startup path once radio.cpp exists.
static bool wait_for_usb(uint32_t timeout_ms)
{
    const absolute_time_t until = make_timeout_time_ms(timeout_ms);
    while (!stdio_usb_connected() && !time_reached(until)) {
        sleep_ms(50);
    }
    const bool connected = stdio_usb_connected();
    sleep_ms(200);  // let the host finish opening before the first line
    return connected;
}

int main()
{
    stdio_init_all();
    const bool usb = wait_for_usb(USB_WAIT_TX_MS);

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);
    printf("[tx] placeholder build: no radio touched, USB serial only\n");
    printf("[tx] would send: %s\n", TX_SOURCE);
    printf("[tx] USB host %s within %u ms\n",
           usb ? "connected" : "did not appear",
           static_cast<unsigned>(USB_WAIT_TX_MS));

    const ConfigLoRa_t cfg = lora_config();
    printf("[tx] would use %.1f MHz  BW %.1f kHz  SF%u  CR 4/%u  sync 0x%02X  %d dBm\n",
           static_cast<double>(cfg.frequency),
           static_cast<double>(cfg.bandwidth),
           static_cast<unsigned>(cfg.spreadingFactor),
           static_cast<unsigned>(cfg.codingRate),
           static_cast<unsigned>(cfg.syncWord),
           static_cast<int>(cfg.power));
    printf("[tx] real cadence will be one packet every %u ms\n",
           static_cast<unsigned>(TX_PERIOD_MS));

#if GS_TX_USE_REAL_GPS
    printf("[tx] GPS on UART0  TX GP%u  RX GP%u  @ %u baud\n",
           PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD);
#endif

    printf("[tx] heartbeat every %u ms\n", static_cast<unsigned>(SMOKE_PERIOD_MS));

    // Scheduled on absolute time, not sleep-after-work, so the period stays put
    // once there is real work in this loop (spec 5.2.3).
    uint32_t n = 0;
    absolute_time_t next = get_absolute_time();
    while (true) {
        next = delayed_by_ms(next, SMOKE_PERIOD_MS);
        sleep_until(next);
        printf("[tx] alive n=%u up=%u ms\n",
               static_cast<unsigned>(++n),
               static_cast<unsigned>(to_ms_since_boot(get_absolute_time())));
    }
}
