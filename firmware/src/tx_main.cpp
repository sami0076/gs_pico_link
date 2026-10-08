// Transmitter entry point, shared by gs_tx, gs_tx_diag and gs_tx_gps.
//
// PLACEHOLDER (chunk 2.2): this exists so the build system has something real
// to compile and so config.hpp is actually type-checked by a compiler. It
// prints the settings it would use and then idles. The real transmit loop
// arrives with radio.cpp, nmea_build.cpp and gps_uart.cpp.
//
// Which of the three transmitter builds this is comes from the compile
// definitions set per target in CMakeLists.txt - never from editing this file.
#include <stdio.h>

#include "pico/stdlib.h"

#include "config.hpp"

#if GS_TX_USE_REAL_GPS
static const char *const TX_SOURCE = "GGA from a real GPS on UART0";
#elif GS_TX_MODE_DIAG
static const char *const TX_SOURCE = "diagnostic counter";
#else
static const char *const TX_SOURCE = "simulated GGA";
#endif

int main()
{
    stdio_init_all();

    const ConfigLoRa_t cfg = lora_config();

    printf("[tx] gs_pico_link transmitter (build placeholder)\n");
    printf("[tx] sending: %s\n", TX_SOURCE);
    printf("[tx] %.1f MHz  BW %.1f kHz  SF%u  CR 4/%u  sync 0x%02X  %d dBm\n",
           static_cast<double>(cfg.frequency),
           static_cast<double>(cfg.bandwidth),
           static_cast<unsigned>(cfg.spreadingFactor),
           static_cast<unsigned>(cfg.codingRate),
           static_cast<unsigned>(cfg.syncWord),
           static_cast<int>(cfg.power));
    printf("[tx] one packet every %u ms\n", static_cast<unsigned>(TX_PERIOD_MS));

#if GS_TX_USE_REAL_GPS
    printf("[tx] GPS on UART0  TX GP%u  RX GP%u  @ %u baud\n",
           PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD);
#endif

    while (true) {
        tight_loop_contents();
    }
}
