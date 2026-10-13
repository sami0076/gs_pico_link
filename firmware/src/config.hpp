// Every wire and every radio setting for gs_pico_link, in one place, so the two
// Picos can never disagree about how they are connected or how they talk.
#pragma once

#include <RadioLib.h>

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// Steps per degree, travel limits and pulse timing for the motors. Kept in its
// own header because it must stay free of Pico headers to be host-testable.
#include "motor_config.hpp"

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
// The radio is not wired straight to the Pico. It plugs into the team's
// gs_pcb_v1 breakout board, which decides which GPIO reaches which radio pin.
// So the board's pinout sheet is the authority here, not the RFM95W datasheet.
//
// Source: "GS 25: Pico Breakout Pinout", RocketryVT/Ground-Station.
//
//   RFM95W        Pico 2 W        physical pin
//   ----------    ------------    ------------
//   SCK           GP18            24
//   MOSI          GP19            25
//   MISO          GP20            26
//   CS            GP21            27
//   RST           GP22            29
//   G0 (DIO0)     GP17            22
//   EN            GP16            21
//   G1 (DIO1)     not routed on this board
//
// Power is not listed above because the Pico does not supply it. Every breakout
// is fed by the board's own 3V3 regulator module, which runs off a 5V regulator
// on the battery.
//
// The board calls this radio "LoRa0" and puts it on the right-hand SPI header.
// There is a second radio ("LoRa1", the 433 MHz backup) on SPI1 that this
// project does not use. See docs/HARDWARE_BRINGUP.md.

// Not constexpr: the SDK's spi0 macro casts a fixed address, which C++ won't
// evaluate at compile time. "inline const" gives us the same safety here.
inline spi_inst_t *const LORA_SPI = spi0;

constexpr uint PIN_SCK  = 18;  // pin 24, SPI0 SCK  - clock
constexpr uint PIN_MOSI = 19;  // pin 25, SPI0 TX   - data from Pico to radio
constexpr uint PIN_MISO = 20;  // pin 26, SPI0 RX   - data from radio to Pico
constexpr uint PIN_CS   = 21;  // pin 27, SPI0 CSn  - RadioLib drives this itself
constexpr uint PIN_RST  = 22;  // pin 29            - reset, active LOW
constexpr uint PIN_DIO0 = 17;  // pin 22            - "packet finished" interrupt
constexpr uint PIN_EN   = 16;  // pin 21            - radio enable

// GPS module. Only the gs_tx_gps build uses these.
// Note the crossover: the GPS's TX goes to the Pico's RX, and vice versa.
//
// UART0 can appear on two different pin pairs. The board puts it on GP12/GP13
// because GP0/GP1 are taken by the I2C sensors.
inline uart_inst_t *const GPS_UART = uart0;

constexpr uint PIN_GPS_TX = 12;  // pin 16, UART0 TX -> GPS RX
constexpr uint PIN_GPS_RX = 13;  // pin 17, UART0 RX <- GPS TX
constexpr uint GPS_BAUD   = 9600;

// Stepper motor drivers. Only the gs_motor build uses these.
//
// Two StepperOnline CL57TE drives, one per axis. Each takes two signals:
//   PUL - one pulse per step
//   DIR - which way to turn
//
// The board names them "Servo 1" (top) and "Servo 2" (bottom). Servo 1 is the
// azimuth axis and Servo 2 is elevation.
//
// THESE SIGNALS ARE ACTIVE LOW. The driver's PUL+ and DIR+ are tied to +5V on
// the board, so the Pico sinks current through an optocoupler instead of
// driving the line high. In practice that means:
//
//   PUL idles HIGH and is pulled LOW to make one step.
//   DIR LOW means the positive direction.
//
// Do not switch these pins to open-drain or to inputs to "fix" a signal
// problem. Pico GPIOs are not 5V tolerant, and releasing the pin lets the 5V
// rail push current back into it through the optocoupler.
//
//   driver        Pico 2 W    physical pin   wire
//   ------------  ----------  -------------  -----
//   Servo 1 PUL-  GP4         6              red
//   Servo 1 DIR-  GP5         7              red
//   Servo 2 PUL-  GP6         9              blue
//   Servo 2 DIR-  GP7         10             blue
//   all PUL+/DIR+ +5V, top two pins of each screw terminal, green
//
// ENA and ALM are not connected. The Pico cannot disable a drive and cannot
// tell when one faults - watch the red LED on the driver instead.
constexpr uint PIN_AZ_PUL = 4;  // pin 6             - azimuth step
constexpr uint PIN_AZ_DIR = 5;  // pin 7             - azimuth direction
constexpr uint PIN_EL_PUL = 6;  // pin 9             - elevation step
constexpr uint PIN_EL_DIR = 7;  // pin 10            - elevation direction

// ============================================================================
// 2. Radio settings
// ============================================================================
// These must be IDENTICAL on both ends. If even one differs, the receiver hears
// nothing at all - there is no warning and no partial reception. They match last
// year's flight link, so this receiver can still pick up last year's hardware.
inline ConfigLoRa_t lora_config()
{
    ConfigLoRa_t cfg;
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
