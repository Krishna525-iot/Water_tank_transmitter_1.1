/* ====================================================================
 * rf.c  —  TRANSMITTER  433 MHz OOK/ASK driver
 *
 * v6.4 — FS1000A DATA pin confirmed as PD1 (RF_connector_Pin)
 *
 * Pin map:
 *   FS1000A DATA → PD1  (GPIOD GPIO_PIN_1 = RF_connector_Pin)
 *   main.h defines: RF_connector_Pin = GPIO_PIN_1, _GPIO_Port = GPIOD
 *
 *   CRITICAL: PD0/PD1 are OSC_IN/OSC_OUT by default on STM32F1.
 *   __HAL_AFIO_REMAP_PD01_ENABLE() MUST be called in MX_GPIO_Init()
 *   before HAL_GPIO_Init() for GPIOD to use PD1 as GPIO output.
 *
 *   Radio_ConfigPin(RADIO_MODE_RF433) in main.c sets PD1 to
 *   OUTPUT_PP HIGH-speed before RF_Init() is called.
 *
 * TIM3: reconfigured to 1 MHz inside RF_Init() for OOK bit timing.
 *   64 MHz TIM3 clock / (63+1) = 1 MHz → 1 µs / count.
 * ==================================================================== */

#include "rf.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ── Board-specific ─────────────────────────────────────────────────── *
 * TIM3 clock = 64 MHz (APB1=32MHz, HAL doubles when APB1≠AHB)         *
 * Prescaler 63 → 64MHz/(63+1) = 1MHz → 1µs/count                     */
#define RF_TIM3_PRESCALER   63u

extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

static void uart_ln(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s,      (uint16_t)strlen(s), 500u);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2u,                  500u);
}

/* ── Module state ───────────────────────────────────────────────────── */
static bool     s_hwOk         = false;
static uint32_t s_txSeq        = 0u;
static uint32_t s_lastDataTick = 0u;
static uint32_t s_lastHiTick   = 0u;

static char dbg[88];

/* ── TIM3 µs delay ──────────────────────────────────────────────────── */
static void rf_us(uint32_t us)
{
    while (us > 60000u)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0u);
        while (__HAL_TIM_GET_COUNTER(&htim3) < 60000u) { }
        us -= 60000u;
    }
    if (us > 0u)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0u);
        while (__HAL_TIM_GET_COUNTER(&htim3) < (uint16_t)us) { }
    }
}

/* ── CRC-8  poly 0x07, init 0x00 ───────────────────────────────────── */
uint8_t RF_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;
    while (len--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80u) ? ((crc << 1u) ^ 0x07u) : (crc << 1u);
    }
    return crc;
}

/* ── Low-level bit output on RF_connector_Pin (PD1) ────────────────── *
 *  Writes to RF_connector_GPIO_Port / RF_connector_Pin (GPIOD/PD1).   *
 *  Drives the FS1000A DATA input.                                      */
static void rf_bit(uint8_t val)
{
    if (val)
    {
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_SET);
        rf_us(RF_BIT_ONE_HIGH_US);
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_RESET);
        rf_us(RF_BIT_ONE_LOW_US);
    }
    else
    {
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_SET);
        rf_us(RF_BIT_ZERO_HIGH_US);
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_RESET);
        rf_us(RF_BIT_ZERO_LOW_US);
    }
}

static void rf_byte(uint8_t b)
{
    for (int8_t i = 7; i >= 0; i--)
        rf_bit((b >> (uint8_t)i) & 1u);
}

/* ── RF_SendPacket ──────────────────────────────────────────────────── *
 *  Frame: 5×0xAA preamble + 0x2D 0xD4 sync + len + payload + CRC-8   *
 *  Transmitted RF_TX_REPEATS times with 20 ms gap between repeats.    */
void RF_SendPacket(const char *payload, uint8_t len)
{
    if (!s_hwOk || payload == NULL || len == 0u || len > RF_MAX_PAYLOAD)
        return;

    uint8_t crc = RF_CRC8((const uint8_t *)payload, len);

    for (uint8_t rep = 0u; rep < RF_TX_REPEATS; rep++)
    {
        for (uint8_t i = 0u; i < RF_PREAMBLE_BYTES; i++)
            rf_byte(0xAAu);

        rf_byte(RF_SYNC_BYTE1);
        rf_byte(RF_SYNC_BYTE2);
        rf_byte(len);

        for (uint8_t i = 0u; i < len; i++)
            rf_byte((uint8_t)payload[i]);

        rf_byte(crc);

        /* Data line LOW between repetitions (no carrier) */
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_RESET);
        rf_us(RF_INTER_PKT_GAP_US);
    }

    snprintf(dbg, sizeof(dbg),
             "[RF TX] sent %u×  len=%u  CRC=0x%02X  \"%s\"",
             (unsigned)RF_TX_REPEATS, (unsigned)len,
             (unsigned)crc, payload);
    uart_ln(dbg);
}

/* ── RF_Init ────────────────────────────────────────────────────────── */
void RF_Init(void)
{
    uart_ln("[RF TX] Init...");
    uart_ln("[RF TX] DATA pin: PD1 (GPIOD GPIO_PIN_1 = RF_connector_Pin)");
    uart_ln("[RF TX] NOTE: PD01 remap must be active (see MX_GPIO_Init)");

    /* Reconfigure TIM3 to 1 MHz for OOK bit timing */
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, RF_TIM3_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFu);
    htim3.Instance->EGR |= TIM_EGR_UG;   /* force prescaler reload */
    HAL_TIM_Base_Start(&htim3);

    /* Idle: data line LOW (no carrier on FS1000A) */
    HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_RESET);

    s_hwOk         = true;
    s_txSeq        = 0u;
    s_lastDataTick = 0u;
    s_lastHiTick   = 0u;

    uart_ln("[RF TX] TIM3 → 1 MHz (prescaler=63, TIM3_CLK=64 MHz)");
    uart_ln("[RF TX] Init complete — 433 MHz OOK TX ready on PD1");
}

/* ── RF_Service  (call from main loop every ~200 ms) ────────────────── *
 *  Every 5s  → broadcast HELLO                                          *
 *  Every 12s → broadcast DATA × 3 repeats                              */
void RF_Service(uint8_t tank_level, uint8_t well_dry)
{
    if (!s_hwOk) return;

    uint32_t now = HAL_GetTick();

    /* ── HELLO ────────────────────────────────────────────────────── */
    if ((now - s_lastHiTick) >= RF_HELLO_INTERVAL_MS)
    {
        s_lastHiTick = now;
        char pkt[24];
        int  n = LoRa_BuildHello(pkt, sizeof(pkt), DeviceID_GetOwn());
        if (n > 0)
        {
            uart_ln("[RF TX] -> HELLO");
            RF_SendPacket(pkt, (uint8_t)n);
        }
    }

    /* ── DATA ─────────────────────────────────────────────────────── */
    if ((now - s_lastDataTick) >= RF_DATA_INTERVAL_MS)
    {
        s_lastDataTick = now;
        s_txSeq++;

        char pkt[RF_MAX_PAYLOAD];
        int  n = LoRa_BuildData(pkt, sizeof(pkt), DeviceID_GetOwn(),
                                s_txSeq, tank_level, well_dry);
        if (n > 0)
        {
            snprintf(dbg, sizeof(dbg),
                     "[RF TX] -> DATA  Level=%u%%  WD=%u  Seq=%08lX",
                     (unsigned)tank_level, (unsigned)well_dry,
                     (unsigned long)s_txSeq);
            uart_ln(dbg);
            RF_SendPacket(pkt, (uint8_t)n);
        }
    }
}
