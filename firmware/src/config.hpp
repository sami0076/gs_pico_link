// Every wire and every radio setting for gs_pico_link, in one place, so the two
// Picos can never disagree about how they are connected or how they talk.
#pragma once

#include <RadioLib.h>

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// ============================================================================
// 1. Wiring
// ============================================================================
// Two different numbers describe the same hole on the board, and mixing them up
// is the most common wiring mistake:
//
//   GPIO number     what the code uses  (GP18)
//   physical pin    what you count on the board  (pin 24)
//
// The constants below are GPIO numbers. The physical pin is in the comment so
// you can count it out on the board while wiring.
//
//   RFM95W        Pico 2 W        physical pin
//   ----------    ------------    ------------
//   VIN           3V3(OUT)        36
//   GND           GND             38
//   SCK           GP18            24
//   MOSI          GP19            25
//   MISO          GP16            21
//   CS            GP17            22
//   RST           GP20            26
//   G0 (DIO0)     GP21            27
//   G1 (DIO1)     GP22            29
//   EN, G2-G5     not connected
//
// Verified against the official Pico 2 W pinout (Figure 2 of the datasheet) and
// the RP2350 GPIO function table in the SDK headers. See docs/HARDWARE_BRINGUP.md.

// Not constexpr: the SDK's spi0 macro casts a fixed address, which C++ won't
// evaluate at compile time. "inline const" gives us the same safety here.
inline spi_inst_t *const LORA_SPI = spi0;

constexpr uint PIN_MISO = 16;  // pin 21, SPI0 RX   - data from radio to Pico
constexpr uint PIN_CS   = 17;  // pin 22            - RadioLib drives this itself
constexpr uint PIN_SCK  = 18;  // pin 24, SPI0 SCK  - clock
constexpr uint PIN_MOSI = 19;  // pin 25, SPI0 TX   - data from Pico to radio
constexpr uint PIN_RST  = 20;  // pin 26            - reset, active LOW
constexpr uint PIN_DIO0 = 21;  // pin 27            - "packet finished" interrupt
constexpr uint PIN_DIO1 = 22;  // pin 29            - wired but unused for now

// GPS module. Only the gs_tx_gps build uses these.
// Note the crossover: the GPS's TX goes to the Pico's RX, and vice versa.
inline uart_inst_t *const GPS_UART = uart0;

constexpr uint PIN_GPS_TX = 0;  // pin 1, UART0 TX -> GPS RX
constexpr uint PIN_GPS_RX = 1;  // pin 2, UART0 RX <- GPS TX
constexpr uint GPS_BAUD   = 9600;

// ============================================================================
// 2. Radio settings
// ============================================================================
// These must be IDENTICAL on both ends. If even one differs, the receiver hears
// nothing at all - there is no warning and no partial reception. They match last
// year's flight link, so this receiver can still pick up last year's hardware.
inline SX1276::ConfigLoRa_t lora_config()
{
    SX1276::ConfigLoRa_t cfg;
    cfg.frequency       = 915.0f;  // MHz
    cfg.bandwidth       = 125.0f;  // kHz
    cfg.spreadingFactor = 7;
    cfg.codingRate      = 5;       // coding rate 4/5; RadioLib wants the denominator
    cfg.syncWord        = 0x12;    // only radios sharing this value talk to each other
    cfg.preambleLength  = 8;       // symbols
    // 10 dBm is plenty across a bench. Last year's rocket flew at 20. Turning it
    // up at close range can actually make reception worse, not better.
    cfg.power           = 10;      // dBm
    return cfg;
}

// ============================================================================
// 3. Timing
// ============================================================================
constexpr uint32_t TX_PERIOD_MS   = 1000;  // one packet per second
constexpr uint32_t STAT_PERIOD_MS = 5000;  // receiver heartbeat line

// The receiver sits on the laptop, so it waits forever for a serial program to
// open the port - otherwise its startup messages scroll past before you look.
// The transmitter runs off a power bank with nothing watching, so it gives up
// and gets on with transmitting.
constexpr uint32_t USB_WAIT_TX_MS = 3000;
