// Receiver entry point.
//
// PLACEHOLDER (chunk 2.2): this exists so the build system has something real
// to compile and so config.hpp is actually type-checked by a compiler. It
// prints the settings it would use and then idles. The real listen loop arrives
// with radio.cpp and rx_format.cpp.
#include <stdio.h>

#include "pico/stdlib.h"

#include "config.hpp"

int main()
{
    stdio_init_all();

    const ConfigLoRa_t cfg = lora_config();

    printf("[rx] gs_pico_link receiver (build placeholder)\n");
    printf("[rx] %.1f MHz  BW %.1f kHz  SF%u  CR 4/%u  sync 0x%02X\n",
           static_cast<double>(cfg.frequency),
           static_cast<double>(cfg.bandwidth),
           static_cast<unsigned>(cfg.spreadingFactor),
           static_cast<unsigned>(cfg.codingRate),
           static_cast<unsigned>(cfg.syncWord));
    printf("[rx] SPI0  SCK GP%u  MOSI GP%u  MISO GP%u  CS GP%u  RST GP%u  DIO0 GP%u\n",
           PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_RST, PIN_DIO0);

    while (true) {
        tight_loop_contents();
    }
}
