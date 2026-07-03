/* ====================================================================
 * adc.h  —  TRANSMITTER ADC driver interface
 * ==================================================================== */

#ifndef ADC_H
#define ADC_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define ADC_CHANNEL_COUNT   6U

typedef struct
{
    uint16_t rawValues[ADC_CHANNEL_COUNT];
} ADC_Data;

extern uint16_t g_adcRaw[ADC_CHANNEL_COUNT];

/* Init + bulk read */
void ADC_Init         (ADC_HandleTypeDef *hadc);
void ADC_ReadAllChannels(ADC_HandleTypeDef *hadc, ADC_Data *data);

/* ── New: direct getters for main.c → LoRa_Service() path ──────────── *
 * Returns the classification from the last ADC_ReadAllChannels() call.
 * Call ADC_ReadAllChannels() first; these just return cached values.   */
uint8_t ADC_GetTankLevel(void);   /* 0, 25, 50, 75, or 100            */
uint8_t ADC_GetWellDry  (void);   /* 0 = water present, 1 = well dry  */

/* Legacy packet builder (still works, uses cached classify result)    */
void ADC_ClassifyPacket(char *out, uint8_t outSize, uint32_t seqNum);

#endif /* ADC_H */
