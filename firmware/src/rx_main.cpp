// Receiver entry point.
//
// PLACEHOLDER (chunk 2.2): there is no radio in here yet. What it does do is the
// USB half of the startup sequence (spec 5.3 steps 1-3) plus a heartbeat, which
// is enough to test reading a Pico over USB serial from the laptop with nothing
// but a USB cable. The SPI self-test, radio.begin() and the real listen loop
// arrive with radio.cpp.
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "config.hpp"

// How often the placeholder says "still here". The real cadences
// (TX_PERIOD_MS, STAT_PERIOD_MS) belong to the real loops, so this smoke-test
// heartbeat gets its own number rather than borrowing one of theirs.
constexpr uint32_t SMOKE_PERIOD_MS = 1000;

// The receiver lives on the laptop, so it waits forever for a serial program to
// open the port - otherwise the banner is printed into a port nobody has opened
// yet and you are left staring at a blank terminal. This moves into the shared
// startup path once radio.cpp exists.
static void wait_for_usb_forever()
{
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(200);  // let the host finish opening before the first line
}

int main()
{
    stdio_init_all();
    wait_for_usb_forever();

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);
    printf("[rx] placeholder build: no radio touched, USB serial only\n");
    const ConfigLoRa_t cfg = lora_config();
    printf("[rx] would use %.1f MHz  BW %.1f kHz  SF%u  CR 4/%u  sync 0x%02X\n",
           static_cast<double>(cfg.frequency),
           static_cast<double>(cfg.bandwidth),
           static_cast<unsigned>(cfg.spreadingFactor),
           static_cast<unsigned>(cfg.codingRate),
           static_cast<unsigned>(cfg.syncWord));
    printf("[rx] SPI0  SCK GP%u  MOSI GP%u  MISO GP%u  CS GP%u  RST GP%u  DIO0 GP%u\n",
           PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_RST, PIN_DIO0);
    printf("[rx] heartbeat every %u ms\n", static_cast<unsigned>(SMOKE_PERIOD_MS));

    // Scheduled on absolute time, not sleep-after-work, so the period stays put
    // once there is real work in this loop (spec 5.2.3).
    uint32_t n = 0;
    absolute_time_t next = get_absolute_time();
    while (true) {
        next = delayed_by_ms(next, SMOKE_PERIOD_MS);
        sleep_until(next);
        printf("[rx] alive n=%u up=%u ms\n",
               static_cast<unsigned>(++n),
               static_cast<unsigned>(to_ms_since_boot(get_absolute_time())));
    }
}
