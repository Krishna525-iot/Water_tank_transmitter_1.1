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

/* ── Thresholds ─────────────────────────────────────────────────────── */
#define GND_ADC              200u   /* probe-grounded threshold          */
#define WELL_SENSOR_GND_ADC  200u   /* well-sensor grounded threshold    */

/* ── Module state ───────────────────────────────────────────────────── */
uint16_t g_adcRaw[ADC_CHANNEL_COUNT] = {0};

/* Last classification result — readable via getters */
static uint8_t s_tankLevel = 0;
static uint8_t s_wellDry   = 0;

/* ── Channel map ────────────────────────────────────────────────────── */
static const uint32_t adc_map[ADC_CHANNEL_COUNT] =
{
    ADC_CHANNEL_0,   /* CH0 — 100 % (top probe)   */
    ADC_CHANNEL_1,   /* CH1 — spare               */
    ADC_CHANNEL_2,   /* CH2 —  25 %               */
    ADC_CHANNEL_3,   /* CH3 —  50 %               */
    ADC_CHANNEL_4,   /* CH4 —  75 %               */
    ADC_CHANNEL_5,   /* CH5 — well / dry-run       */
};

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
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

/* ── Init ───────────────────────────────────────────────────────────── */
void ADC_Init(ADC_HandleTypeDef *hadc)
{
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
    if      (g_adcRaw[4] < GND_ADC) level = 100;
    else if (g_adcRaw[3] < GND_ADC) level =  75;
    else if (g_adcRaw[2] < GND_ADC) level =  50;
    else if (g_adcRaw[1] < GND_ADC) level =  25;
    else                             level =   0;

    /* ── Classify well dry (pull-up: HIGH = dry, LOW = water) ──────── */
    uint8_t wellDry = (g_adcRaw[5] >= WELL_SENSOR_GND_ADC) ? 1u : 0u;

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
        "[ADC] Level: %u%%  WellDry: %u  CH5raw=%04u",
        level, wellDry, g_adcRaw[5]);
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
