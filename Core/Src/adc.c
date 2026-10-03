/* ====================================================================
 * adc.c  —  TRANSMITTER ADC driver
 *
 * Changes from previous version:
 *   • Added ADC_GetTankLevel()  — returns last-classified tank level
 *   • Added ADC_GetWellDry()    — returns last-classified well-dry flag
 *   • ADC_ClassifyPacket() still exists but is now OPTIONAL (kept for
 *     any code that still calls it).  main.c now calls
 *     ADC_GetTankLevel/WellDry directly and passes values to
 *     LoRa_Service() — no shared txPacket buffer required.
 *   • Removed the #include "lora.h" dependency.
 * ==================================================================== */

#include "adc.h"
#include "main.h"
#include "device_id.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern void UART_Print  (const char *s);
extern void UART_PrintLn(const char *s);
extern UART_HandleTypeDef huart1;

/* ── Thresholds ─────────────────────────────────────────────────────── *
 * The TX board has no resistors on L1-L5: an open input used to float
 * anywhere from 0 to ~3000 counts, so level and dry jumped by
 * themselves. The pins now use the MCU's internal pull-up (~40 kOhm):
 * open = ~4095, probe in water / on G = pulled low. Water is not a short,
 * so the threshold is half of 3.3 V: up to ~40 kOhm counts as water.   */
#define GND_ADC              2048u  /* probe in water / on G below this  */
#define WELL_SENSOR_GND_ADC  2048u  /* L5 dry sensor: same               */
/* Client GEN-01: 10 s end to end = 9 s stable + up to 1 s ADC + ~0.5 s air */
#define LEVEL_STABLE_MS       9000u

/* ── Module state ───────────────────────────────────────────────────── */
uint16_t g_adcRaw[ADC_CHANNEL_COUNT] = {0};

/* Last classification result — readable via getters */
static uint8_t s_tankLevel = 0;
static uint8_t s_wellDry   = 0;

/* ── Channel map ────────────────────────────────────────────────────── */
static const uint32_t adc_map[ADC_CHANNEL_COUNT] =
{
    /* Same probe order as the Main Controller (same PCB, same labels) */
    ADC_CHANNEL_0,   /* PA0 — 100 % (top probe)   */
    ADC_CHANNEL_1,   /* PA1 —  75 %               */
    ADC_CHANNEL_2,   /* PA2 —  50 %               */
    ADC_CHANNEL_3,   /* PA3 —  25 %               */
    ADC_CHANNEL_4,   /* PA4 — L5: dry-run / well sensor (TX connector L1-L5, G) */
    ADC_CHANNEL_5,   /* PA5 — G.W terminal, not on the TX connector: not used */
};

/* TX connector: L1 = 100 %, L2 = 75 %, L3 = 50 %, L4 = 25 %,
 * L5 = dry-run sensor (was the spare "empty" pin), G = GND (common). */
#define DRY_CH  4u

/* ── Getters (used by main.c to pass values to LoRa_Service) ────────── */
uint8_t ADC_GetTankLevel(void) { return s_tankLevel; }
uint8_t ADC_GetWellDry  (void) { return s_wellDry;   }

/* ── Low-level single-channel read ─────────────────────────────────── */
static uint16_t read_adc_raw(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    HAL_ADC_ConfigChannel(hadc, &sConfig);

    /* Two conversions, keep the second: the first one after a channel
     * switch (and the very first after power-up) carries charge from the
     * previous channel - it read an open PA0 as "grounded" = false 100 %. */
    uint16_t val = 0;
    for (int n = 0; n < 2; n++)
    {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10);
        val = (uint16_t)HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
    }
    return val;
}

/* ── Init ───────────────────────────────────────────────────────────── */
void ADC_Init(ADC_HandleTypeDef *hadc)
{
    /* L1-L5 (PA0-PA4) as input with the internal pull-up instead of plain
     * analog: the ADC still samples the pad in input mode on the F1, and
     * the pull-up holds an open probe high instead of letting it float. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_ADCEx_Calibration_Start(hadc);
    UART_PrintLn("[ADC] Calibration done");
}

/* ── Read all channels, classify, store into module state ───────────── */
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data)
{
    uint8_t i;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        uint16_t raw       = read_adc_raw(hadc, adc_map[i]);
        g_adcRaw[i]        = raw;
        data->rawValues[i] = raw;
    }

    /* ── Classify tank level ───────────────────────────────────────── */
    uint8_t level;
    if      (g_adcRaw[0] < GND_ADC) level = 100;
    else if (g_adcRaw[1] < GND_ADC) level =  75;
    else if (g_adcRaw[2] < GND_ADC) level =  50;
    else if (g_adcRaw[3] < GND_ADC) level =  25;
    else                             level =   0;

    /* ── Classify well dry from L5 (pull-up: HIGH = dry, LOW = water) ─ */
    uint8_t wellDry = (g_adcRaw[DRY_CH] >= WELL_SENSOR_GND_ADC) ? 1u : 0u;

    /* ── Client GEN-01: a new level counts only after it is stable for
     *    10 s. Done here on the TX, so the Main Controller shows the
     *    wireless level at once (update ~10 s end to end). The first
     *    reading after power-up is taken as is (fast reconnect). ───── */
    {
        static bool     s_first     = true;
        static uint8_t  s_candidate = 0;
        static uint32_t s_since     = 0;
        uint32_t now = HAL_GetTick();

        if (s_first)                  { s_first = false; s_tankLevel = level; s_candidate = level; }
        else if (level == s_tankLevel) { s_candidate = level; }
        else
        {
            if (level != s_candidate) { s_candidate = level; s_since = now; }
            if ((now - s_since) >= LEVEL_STABLE_MS) s_tankLevel = level;
        }
        level = s_tankLevel;
    }

    /* Store so getters can return them */
    s_tankLevel = level;
    s_wellDry   = wellDry;

    /* ── Diagnostics (kept from previous version) ──────────────────── */
    char buf[160];

    snprintf(buf, sizeof(buf),
        "[ADC] Raw  : CH0=%04u CH1=%04u CH2=%04u CH3=%04u CH4=%04u CH5=%04u",
        g_adcRaw[0], g_adcRaw[1], g_adcRaw[2],
        g_adcRaw[3], g_adcRaw[4], g_adcRaw[5]);
    UART_PrintLn(buf);

    snprintf(buf, sizeof(buf),
        "[ADC] Level: %u%%  WellDry: %u  L5(CH4)raw=%04u",
        level, wellDry, g_adcRaw[DRY_CH]);
    UART_PrintLn(buf);

    if (wellDry)
        UART_PrintLn("[ADC] *** WELL DRY-RUN ALARM — motor must be OFF ***");
    else
        UART_PrintLn("[ADC] WELL: water present — motor may run");
}

/* ── ADC_ClassifyPacket — kept for backward compatibility ───────────── *
 * Any code that still calls this can continue to do so.  It returns the
 * formatted packet string and also updates s_tankLevel / s_wellDry.
 * main.c no longer calls this — it uses the getters directly.
 * ──────────────────────────────────────────────────────────────────── */
void ADC_ClassifyPacket(char *out, uint8_t outSize, uint32_t seqNum)
{
    /* Use already-classified values from last ADC_ReadAllChannels() */
    uint32_t myDID = DeviceID_GetOwn();

    snprintf(out, outSize, "@TL:%03u,WD:%u,SQ:%08lX,DID:%08lX#",
             (unsigned)s_tankLevel, (unsigned)s_wellDry,
             (unsigned long)seqNum, (unsigned long)myDID);

    char buf[140];
    snprintf(buf, sizeof(buf),
             "[ADC] Packet: \"%s\"", out);
    UART_PrintLn(buf);
}
