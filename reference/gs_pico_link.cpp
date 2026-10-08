// gs_pico_link.cpp
// ----------------------------------------------------------------------------
// One firmware, two roles. Flip NODE_ROLE below and re-flash:
//
//   ROLE_RX  - ground-station receiver. Listens on LoRa and prints every packet
//              to the laptop over USB as one line:
//                  RX,<rssi_dBm>,<snr_dB>,<len>,<payload text>
//              It does NOT parse anything. The laptop does all the parsing.
//
//   ROLE_TX  - test transmitter (stand-in for the rocket). Once per second it
//              sends one NMEA $GPGGA sentence. By default the sentence is
//              simulated (a fake rocket climbing near the launch site). Set
//              USE_REAL_GPS to 1 to forward GGA sentences from a real GPS
//              module on UART0 instead.
//
// Hardware: Raspberry Pi Pico 2 W + Adafruit RFM95W (SX1276) 915 MHz breakout.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

#include <RadioLib.h>
#include <hal/RPiPico/PicoHal.h>

// ============================================================================
// 1. Choose the role for THIS board
// ============================================================================
#define ROLE_RX 0
#define ROLE_TX 1
#define NODE_ROLE ROLE_RX          // <-- change to ROLE_TX for the transmitter

#define USE_REAL_GPS 0             // TX only: 1 = forward GGA from a GPS on UART0

// ============================================================================
// 2. Pins (GPIO numbers, NOT physical pin numbers) - see wiring table
// ============================================================================
#define LORA_SPI   spi0
#define PIN_MISO   16   // physical pin 21  -> RFM95W MISO
#define PIN_CS     17   // physical pin 22  -> RFM95W CS
#define PIN_SCK    18   // physical pin 24  -> RFM95W SCK
#define PIN_MOSI   19   // physical pin 25  -> RFM95W MOSI
#define PIN_RST    20   // physical pin 26  -> RFM95W RST
#define PIN_DIO0   21   // physical pin 27  -> RFM95W G0  ("packet done" interrupt)
#define PIN_DIO1   22   // physical pin 29  -> RFM95W G1

#define GPS_UART      uart0
#define PIN_GPS_TX    0   // physical pin 1  -> GPS RX  (only if USE_REAL_GPS)
#define PIN_GPS_RX    1   // physical pin 2  <- GPS TX
#define GPS_BAUD      9600

// ============================================================================
// 3. Radio settings - must be IDENTICAL on both ends.
//    These match last year's 915 MHz link (board_profile.hpp, Lora915).
// ============================================================================
#define LORA_FREQ_MHZ   915.0f
#define LORA_BW_KHZ     125.0f
#define LORA_SF         7
#define LORA_CR         5       // coding rate 4/5 (RadioLib takes the denominator)
#define LORA_SYNC_WORD  0x12    // "private network" sync word
#define LORA_PREAMBLE   8
// TX power only matters on the transmitter. 10 dBm is plenty on a bench;
// the rocket used 20 dBm. Too much power at 1 m can actually cause errors.
#define LORA_TX_DBM     10

// ============================================================================
// RadioLib objects
// ============================================================================
// PicoHal's init() calls stdio_init_all() again when radio.begin() runs. We
// already start USB serial at the top of main(), so this small subclass only
// starts SPI and avoids initialising USB twice.
class GsHal : public PicoHal {
public:
    using PicoHal::PicoHal;
    void init() override { spiBegin(); }
};

GsHal* hal = new GsHal(LORA_SPI, PIN_MISO, PIN_MOSI, PIN_SCK);
SX1276 radio = new Module(hal, PIN_CS, PIN_DIO0, PIN_RST, PIN_DIO1);

// ----------------------------------------------------------------------------
// Wait (up to timeout) for a serial monitor so the startup messages are seen.
// ----------------------------------------------------------------------------
static void wait_for_usb(uint32_t timeout_ms)
{
    absolute_time_t until = make_timeout_time_ms(timeout_ms);
    while (!stdio_usb_connected() && !time_reached(until)) sleep_ms(50);
    sleep_ms(200);
}

// ----------------------------------------------------------------------------
// Raw SPI self-test, before RadioLib touches anything.
// The SX1276 has a read-only "version" register at address 0x42 that always
// reads 0x12. If we read 0x12, power + SPI wiring + CS + RST are good.
// ----------------------------------------------------------------------------
static bool spi_self_test()
{
    spi_init(LORA_SPI, 1000 * 1000);
    spi_set_format(LORA_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);  gpio_put(PIN_CS, 1);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT);

    // SX1276 reset is active LOW: pull low briefly, release, wait for boot.
    gpio_put(PIN_RST, 0); sleep_ms(1);
    gpio_put(PIN_RST, 1); sleep_ms(10);

    uint8_t addr = 0x42 & 0x7F;   // MSB = 0 means "read"
    uint8_t value = 0;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(LORA_SPI, &addr, 1);
    spi_read_blocking(LORA_SPI, 0x00, &value, 1);
    gpio_put(PIN_CS, 1);

    printf("[spi] RegVersion (0x42) = 0x%02X  ", value);
    if (value == 0x12) {
        printf("-> OK, SX1276 found\n");
        return true;
    }
    if (value == 0x00) printf("-> reads 0x00: check MISO wire, VIN/GND, CS\n");
    else if (value == 0xFF) printf("-> reads 0xFF: chip not powered or held in reset (check VIN, GND, RST)\n");
    else printf("-> unexpected: check every SPI wire against the table\n");
    return false;
}

// ----------------------------------------------------------------------------
// Bring up the radio with the shared settings.
// ----------------------------------------------------------------------------
static bool radio_begin()
{
    int16_t state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                                LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE);
    if (state != RADIOLIB_ERR_NONE) {
        printf("[radio] begin failed, code %d\n", state);
        return false;
    }
    printf("[radio] ready: %.1f MHz  BW %.0f kHz  SF%d  CR4/%d  sync 0x%02X\n",
           LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD);
    return true;
}

// ============================================================================
// RECEIVER
// ============================================================================
#if NODE_ROLE == ROLE_RX

// Set from the DIO0 interrupt when a packet has finished arriving.
// "volatile" because it is changed inside an interrupt.
static volatile bool g_packet_ready = false;
static void on_packet() { g_packet_ready = true; }

static void run_receiver()
{
    radio.setPacketReceivedAction(on_packet);
    int16_t state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        printf("[radio] startReceive failed, code %d\n", state);
        return;
    }
    printf("[rx] listening...\n");

    uint8_t buf[256];
    uint32_t good = 0, crc_errors = 0;
    absolute_time_t last_status = get_absolute_time();

    for (;;) {
        if (g_packet_ready) {
            g_packet_ready = false;

            size_t len = radio.getPacketLength();
            if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
            state = radio.readData(buf, len);

            if (state == RADIOLIB_ERR_NONE) {
                good++;
                buf[len] = '\0';
                // Strip trailing CR/LF so each packet is exactly one USB line.
                while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) buf[--len] = '\0';
                printf("RX,%.1f,%.2f,%u,%s\n",
                       radio.getRSSI(), radio.getSNR(), (unsigned)len, (char*)buf);
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                crc_errors++;
                printf("ERR,crc,%.1f,%.2f\n", radio.getRSSI(), radio.getSNR());
            } else {
                printf("ERR,read,%d\n", state);
            }
            radio.startReceive();   // re-arm for the next packet
        }

        // Heartbeat every 5 s so you can tell "no packets" from "Pico frozen".
        if (absolute_time_diff_us(last_status, get_absolute_time()) > 5'000'000) {
            last_status = get_absolute_time();
            printf("STAT,%lu,%lu,%lu\n", (unsigned long)(to_ms_since_boot(last_status) / 1000),
                   (unsigned long)good, (unsigned long)crc_errors);
        }
        sleep_ms(1);
    }
}
#endif

// ============================================================================
// TRANSMITTER (test stand-in for the rocket)
// ============================================================================
#if NODE_ROLE == ROLE_TX

// NMEA checksum = XOR of every character between '$' and '*'.
static uint8_t nmea_checksum(const char* body)
{
    uint8_t cs = 0;
    while (*body) cs ^= (uint8_t)*body++;
    return cs;
}

// Decimal degrees -> NMEA "ddmm.mmmm" (lat) / "dddmm.mmmm" (lon) + hemisphere.
static void to_nmea_coord(double deg, bool is_lat, char* out, size_t n, char* hemi)
{
    *hemi = is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    deg = fabs(deg);
    int whole = (int)deg;
    double minutes = (deg - whole) * 60.0;
    if (is_lat) snprintf(out, n, "%02d%07.4f", whole, minutes);
    else        snprintf(out, n, "%03d%07.4f", whole, minutes);
}

// Build a $GPGGA sentence for a simulated flight.
static void make_fake_gga(uint32_t t_s, char* out, size_t n)
{
    // Fake launch site (edit to your test location). Altitudes are metres MSL.
    const double pad_lat = 37.2296, pad_lon = -80.4139, pad_alt = 634.0;

    // Simple profile: 60 s climb to 3000 m AGL, drifting north-east, then descent.
    uint32_t cycle = t_s % 180;
    double agl = cycle < 60 ? 50.0 * cycle : fmax(0.0, 3000.0 - 25.0 * (cycle - 60));
    double lat = pad_lat + 0.00002 * cycle;
    double lon = pad_lon + 0.00003 * cycle;

    char la[16], lo[16]; char ns, ew;
    to_nmea_coord(lat, true,  la, sizeof la, &ns);
    to_nmea_coord(lon, false, lo, sizeof lo, &ew);

    uint32_t hh = (t_s / 3600) % 24, mm = (t_s / 60) % 60, ss = t_s % 60;
    char body[100];
    snprintf(body, sizeof body, "GPGGA,%02lu%02lu%02lu.00,%s,%c,%s,%c,1,09,0.9,%.1f,M,-33.0,M,,",
             (unsigned long)hh, (unsigned long)mm, (unsigned long)ss,
             la, ns, lo, ew, pad_alt + agl);
    snprintf(out, n, "$%s*%02X", body, nmea_checksum(body));
}

#if USE_REAL_GPS
// Read characters from the GPS; return true when a full $..GGA line is ready.
static bool read_gps_gga(char* out, size_t n)
{
    static char line[100];
    static size_t len = 0;
    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);
        if (c == '$') len = 0;
        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            bool is_gga = len > 6 && strncmp(line + 3, "GGA", 3) == 0;   // $GPGGA / $GNGGA
            len = 0;
            if (is_gga) { strncpy(out, line, n); out[n - 1] = '\0'; return true; }
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
    return false;
}
#endif

static void run_transmitter()
{
#if USE_REAL_GPS
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(PIN_GPS_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_GPS_RX, GPIO_FUNC_UART);
    printf("[tx] forwarding GGA from GPS on UART0 @ %d baud\n", GPS_BAUD);
#else
    printf("[tx] sending simulated GGA once per second\n");
#endif

    char sentence[100];
    uint32_t count = 0;
    for (;;) {
#if USE_REAL_GPS
        if (!read_gps_gga(sentence, sizeof sentence)) { sleep_ms(5); continue; }
#else
        make_fake_gga(to_ms_since_boot(get_absolute_time()) / 1000, sentence, sizeof sentence);
#endif
        int16_t state = radio.transmit(sentence);   // blocks until sent (~175 ms for a GGA sentence)
        if (state == RADIOLIB_ERR_NONE) printf("TX,%lu,%s\n", (unsigned long)++count, sentence);
        else                            printf("[tx] transmit failed, code %d\n", state);
#if !USE_REAL_GPS
        sleep_ms(1000);
#endif
    }
}
#endif

// ============================================================================
int main()
{
    stdio_init_all();
#if NODE_ROLE == ROLE_RX
    wait_for_usb(24u * 3600u * 1000u);  // RX lives on the laptop: wait until a terminal/script opens the port
#else
    wait_for_usb(3000);                 // TX on a power bank: give up waiting after 3 s and run
#endif

    printf("\n=== gs_pico_link (%s) ===\n", NODE_ROLE == ROLE_RX ? "RX / ground station" : "TX / test rocket");

    if (!spi_self_test() || !radio_begin()) {
        // Keep complaining instead of silently dying, so the problem is visible.
        for (;;) { printf("[fatal] radio not working - fix wiring and reset\n"); sleep_ms(2000); }
    }

#if NODE_ROLE == ROLE_RX
    run_receiver();
#else
    run_transmitter();
#endif
    return 0;
}
