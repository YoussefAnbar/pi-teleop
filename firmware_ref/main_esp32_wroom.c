/*
 * ESP32-WROOM encoder reader for Pi teleop.
 *
 * Hardware:
 *   - ESP32-WROOM dev board with Silicon Labs CP2102 USB-UART bridge.
 *   - Pi sees this as /dev/ttyUSB0.
 *   - Data exits on UART0 (pins TX=GPIO1, RX=GPIO3), routed through CP2102 to USB.
 *   - Up to 3x MCP3008 SPI ADCs, 8 channels each = 24 encoder channels total.
 *
 * SPI pin defaults (VSPI — verify against your physical wiring before flashing):
 *   MISO = GPIO19,  MOSI = GPIO23,  SCLK = GPIO18
 *   CS chip 0 = GPIO5,  CS chip 1 = GPIO17,  CS chip 2 = GPIO16
 *
 * Packet format (102 bytes, 100 Hz):
 *   [0xAA 0x55]       2 B  header
 *   [uint16 LE]       2 B  sequence (wraps at 65535)
 *   [24x float32 LE] 96 B  channel values
 *   [uint16 LE]       2 B  CRC16-CCITT over (sequence + values)
 *
 * DEBUG_RAW=0 (default, PRODUCTION):
 *   Boot calibrates resting center over 100 ADC samples. Keep arm still.
 *   Values sent: centered deflection in [-1.5, 1.5]. 0.0 = rest position.
 *
 * DEBUG_RAW=1 (BRING-UP only):
 *   No calibration. Values sent: raw ADC voltage in [0.0, 3.3 V].
 *   Use this to verify pot wiring with a serial monitor before enabling motion.
 *
 * Build:  ESP-IDF v5.x  (idf.py build)
 * Flash:  idf.py flash   (connect laptop USB to the ESP32's CP2102 port)
 *
 * NOTE: this file is for ESP32-WROOM (no native USB).
 *       For XIAO ESP32-S3 (native USB), see main_mcp3008.c.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define DEBUG_RAW       0       /* 0 = production (centred), 1 = raw volt debug */

/* ---- SPI pin assignments (VSPI / SPI2_HOST on ESP32-WROOM) ---- *
 * Change these to match your wiring.  Do NOT share with UART0 pins (1, 3). */
#define PIN_SPI_MISO    GPIO_NUM_19
#define PIN_SPI_MOSI    GPIO_NUM_23
#define PIN_SPI_SCLK    GPIO_NUM_18
#define PIN_CS_CHIP0    GPIO_NUM_5
#define PIN_CS_CHIP1    GPIO_NUM_17
#define PIN_CS_CHIP2    GPIO_NUM_16

/* ---- SPI config ---- */
#define SPI_HOST_ID     SPI2_HOST       /* VSPI on ESP32 classic */
#define SPI_CLOCK_HZ    1000000         /* 1 MHz; MCP3008 max is 3.6 MHz at 5 V */

/* ---- ADC config ---- */
#define NUM_CHIPS       3
#define TOTAL_CHANNELS  24              /* 3 × 8 */
#define ADC_MAX         1023
#define ADC_VREF_VOLTS  3.3f
#define DEADZONE        5               /* ADC counts around centre treated as 0 */
#define SMOOTHING       0.05f           /* IIR α: higher = more responsive, noisier */
#define OUTPUT_MAX      1.5f            /* peak magnitude sent to Pi */

/* ---- Packet / timing ---- */
#define SEND_RATE_HZ    100
#define FLOAT_COUNT     24
#define HEADER_0        0xAA
#define HEADER_1        0x55

/* ---- UART (CP2102 bridge) ---- */
#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define UART_TX_BUF_SZ  512             /* bytes; must be > sizeof(packet_t) */

/* ================================================================ */

static int      adc_center[TOTAL_CHANNELS];
static float    smooth_adc[TOTAL_CHANNELS];
static uint16_t packet_sequence = 0;

static spi_device_handle_t spi_dev[NUM_CHIPS];

typedef struct __attribute__((packed)) {
    uint8_t  header[2];
    uint16_t sequence;
    float    values[FLOAT_COUNT];
    uint16_t crc;
} packet_t;

/* ---- CRC16-CCITT (identical polynomial to Pi-side Python) ---- */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- MCP3008 SPI single-ended read ---- */
static int mcp3008_read(spi_device_handle_t dev, int channel)
{
    /* TX: [0x01, 0x80|(ch<<4), 0x00]  RX: result = (rx[1]&0x03)<<8 | rx[2] */
    uint8_t tx[3] = { 0x01, (uint8_t)(0x80 | (channel << 4)), 0x00 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t t = {
        .length    = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (spi_device_transmit(dev, &t) != ESP_OK) return -1;
    return ((rx[1] & 0x03) << 8) | rx[2];
}

/* Fail-closed: returns false if any channel read fails. */
static bool read_all_channels(int out[TOTAL_CHANNELS])
{
    int idx = 0;
    for (int chip = 0; chip < NUM_CHIPS; chip++) {
        for (int ch = 0; ch < 8; ch++) {
            int val = mcp3008_read(spi_dev[chip], ch);
            if (val < 0) return false;
            out[idx++] = val;
        }
    }
    return true;
}

/* Centred, deadzone'd, normalised output for one ADC channel. */
static float adc_to_output(int adc_val, int center)
{
    int offset = adc_val - center;
    if (offset > -DEADZONE && offset < DEADZONE) offset = 0;
    float norm = (float)offset / (float)(ADC_MAX / 2);
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm * OUTPUT_MAX;
}

/* Build one 102-byte packet and send it over UART0. */
static void send_packet(const float vals[FLOAT_COUNT])
{
    packet_t pkt;
    pkt.header[0] = HEADER_0;
    pkt.header[1] = HEADER_1;
    pkt.sequence  = packet_sequence++;
    memcpy(pkt.values, vals, sizeof(pkt.values));
    pkt.crc = crc16_ccitt(
        (const uint8_t *)&pkt.sequence,
        sizeof(pkt.sequence) + sizeof(pkt.values)
    );
    uart_write_bytes(UART_PORT, (const char *)&pkt, sizeof(pkt));
}

/* ---- Initialisation ---- */

static void init_uart(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &cfg);
    /* TX buffer only; we never read from the USB-serial port in this firmware. */
    uart_driver_install(UART_PORT, 256, UART_TX_BUF_SZ, 0, NULL, 0);
}

static void init_spi(void)
{
    spi_bus_config_t bus_cfg = {
        .miso_io_num   = PIN_SPI_MISO,
        .mosi_io_num   = PIN_SPI_MOSI,
        .sclk_io_num   = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SPI_DMA_DISABLED));

    const int cs_pins[NUM_CHIPS] = { PIN_CS_CHIP0, PIN_CS_CHIP1, PIN_CS_CHIP2 };
    for (int i = 0; i < NUM_CHIPS; i++) {
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = SPI_CLOCK_HZ,
            .mode           = 0,        /* MCP3008: CPOL=0, CPHA=0 */
            .spics_io_num   = cs_pins[i],
            .queue_size     = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &dev_cfg, &spi_dev[i]));
    }
}

/*
 * Sample 100 frames at rest to establish per-channel ADC zero references.
 * The arm operator must keep encoders still during this ~1-second window.
 * Returns false only if SPI is completely non-functional.
 */
static bool calibrate(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));     /* settle time */

    long sum[TOTAL_CHANNELS] = { 0 };
    int  raw[TOTAL_CHANNELS];
    int  good = 0;

    for (int attempt = 0; attempt < 200 && good < 100; attempt++) {
        if (read_all_channels(raw)) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) sum[i] += raw[i];
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (good == 0) return false;

    for (int i = 0; i < TOTAL_CHANNELS; i++) {
        adc_center[i] = (int)(sum[i] / good);
    }
    return true;
}

/* ================================================================ */

void app_main(void)
{
    /* Silence all ESP-IDF log output so UART0 carries only binary packets.
     * Any text on UART0 would corrupt the packet stream seen by the Pi. */
    esp_log_level_set("*", ESP_LOG_NONE);

    init_uart();
    init_spi();

    const TickType_t period  = pdMS_TO_TICKS(1000 / SEND_RATE_HZ);
    TickType_t       next_wake = xTaskGetTickCount();

#if DEBUG_RAW
    /* ----------------------------------------------------------------
     * DEBUG mode: raw ADC voltages, no calibration required.
     * Use a serial monitor on the Pi:
     *   python3 -m pi_teleop.tools.test_encoder_input
     * to verify each pot moves the expected channel before enabling motion.
     * ---------------------------------------------------------------- */
    int raw_dbg[TOTAL_CHANNELS];
    while (1) {
        float vals[FLOAT_COUNT] = { 0.0f };
        if (read_all_channels(raw_dbg)) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                vals[i] = ((float)raw_dbg[i] / (float)ADC_MAX) * ADC_VREF_VOLTS;
            }
        }
        send_packet(vals);
        vTaskDelayUntil(&next_wake, period);
    }

#else
    /* ----------------------------------------------------------------
     * PRODUCTION mode: calibrate once at boot, stream centred output.
     * Block here until calibration succeeds — never enter the control
     * loop with bad centre values; that would command saturated targets.
     * ---------------------------------------------------------------- */
    while (!calibrate()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bool first_read = true;
    int  raw[TOTAL_CHANNELS];

    while (1) {
        /* Fail-closed: skip frame on any SPI error rather than send garbage. */
        if (!read_all_channels(raw)) {
            vTaskDelayUntil(&next_wake, period);
            continue;
        }

        /* IIR low-pass smoothing */
        if (first_read) {
            for (int i = 0; i < TOTAL_CHANNELS; i++) smooth_adc[i] = (float)raw[i];
            first_read = false;
        } else {
            for (int i = 0; i < TOTAL_CHANNELS; i++) {
                smooth_adc[i] = SMOOTHING * (float)raw[i]
                              + (1.0f - SMOOTHING) * smooth_adc[i];
            }
        }

        float vals[FLOAT_COUNT] = { 0.0f };
        for (int i = 0; i < TOTAL_CHANNELS; i++) {
            vals[i] = adc_to_output((int)smooth_adc[i], adc_center[i]);
        }
        /* vals[TOTAL_CHANNELS..FLOAT_COUNT-1] remain 0.0 (spare channels) */

        send_packet(vals);
        vTaskDelayUntil(&next_wake, period);
    }
#endif
}
