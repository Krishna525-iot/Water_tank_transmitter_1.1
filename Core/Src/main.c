/* ====================================================================
 * main.c  —  TRANSMITTER node  (water_level project)
 *
 * PCB: same hardware as RX board.
 * IOC / main.h updated 2026-06-09:
 *   RF_connector_Pin      = GPIO_PIN_1
 *   RF_connector_GPIO_Port = GPIOD  (PD1)
 *
 * Radio select via g_radio_mode:
 *   RADIO_MODE_LORA   (1) → LoRa Ra-02 SX1278, full ACK handshake
 *   RADIO_MODE_RF433  (0) → FS1000A OOK simplex, HELLO 5s / DATA 12s
 *
 * Key corrections vs previous TX main.c:
 *   FS1000A DATA → RF_connector_Pin is now PD1 (GPIOD), not PC13.
 *   __HAL_AFIO_REMAP_PD01_ENABLE() ADDED to MX_GPIO_Init().
 *   PD0/PD1 are OSC_IN/OSC_OUT by default; PD01 remap is mandatory
 *   to use PD1 as GPIO output for FS1000A DATA line.
 *   GPIOD clock enable added.
 *   GPIOC clock enable removed (PC13 no longer used).
 *   Radio_ConfigPin() updated to use RF_connector_GPIO_Port/Pin (PD1).
 * ==================================================================== */

#include "main.h"
#include "adc.h"
#include "lora.h"
#include "rf.h"
#include "device_id.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ── Radio mode constants ────────────────────────────────────────────── */
#define RADIO_MODE_RF433  0u
#define RADIO_MODE_LORA   1u

/* ── Cadences ────────────────────────────────────────────────────────── */
#define MAIN_LOOP_TICK_MS    200UL
#define ADC_REFRESH_MS      1000UL   /* probes every 1 s; a change is sent at once */
#define BUTTON_DEBOUNCE_MS   200UL
#define STATS_INTERVAL_MS  60000UL

/* ── Peripheral handles ─────────────────────────────────────────────── */
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c2;
RTC_HandleTypeDef  hrtc;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart1;

ADC_Data adcData;

/* ── Legacy storage required by lora.c extern ───────────────────────── */
volatile char txPacket[64];

/* ── Globals used by lora.c (extern there) ──────────────────────────── */
extern uint8_t  loraMode;
extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;
extern uint8_t  g_txConnected;

/* ── Private function prototypes ────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init       (void);
static void MX_ADC1_Init       (void);
static void MX_RTC_Init        (void);
static void MX_SPI1_Init       (void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init       (void);
static void MX_TIM3_Init       (void);
static void Radio_ConfigPin    (uint8_t mode);

/* ── UART helpers ────────────────────────────────────────────────────── */
void UART_Print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 1000u);
}

void UART_PrintLn(const char *s)
{
    UART_Print(s);
    UART_Print("\r\n");
}

/* ====================================================================
 *  Radio_ConfigPin
 *
 *  Configures RF_connector_Pin (PD1, GPIOD GPIO_PIN_1) for the radio.
 *  Called once in main() AFTER MX_GPIO_Init() (PD01 remap already active).
 *
 *  RADIO_MODE_RF433:
 *    PD1 → OUTPUT_PP HIGH-speed.
 *    FS1000A DATA line driven by MCU at ~833 bps OOK.
 *    GPIO_SPEED_FREQ_HIGH required for correct 300/900 µs edges.
 *
 *  RADIO_MODE_LORA:
 *    PD1 → INPUT floating.
 *    CN1 connector unused in LoRa mode.
 * ==================================================================== */
static void Radio_ConfigPin(uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (mode == RADIO_MODE_RF433)
    {
        /* Start idle LOW — no spurious carrier pulse on power-up */
        HAL_GPIO_WritePin(RF_connector_GPIO_Port, RF_connector_Pin, GPIO_PIN_RESET);

        GPIO_InitStruct.Pin   = RF_connector_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* required for OOK timing */
        HAL_GPIO_Init(RF_connector_GPIO_Port, &GPIO_InitStruct);  /* GPIOD */

        UART_PrintLn("[RADIO] PD1 → OUTPUT_PP HIGH-speed (FS1000A OOK TX)");
    }
    else
    {
        GPIO_InitStruct.Pin  = RF_connector_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(RF_connector_GPIO_Port, &GPIO_InitStruct);  /* GPIOD */

        UART_PrintLn("[RADIO] PD1 → INPUT floating (LoRa mode, CN1 unused)");
    }
}

/* ====================================================================
 *  main()
 * ==================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_RTC_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();

    ADC_Init(&hadc1);

    /* ================================================================
     *  ┌──────────────────────────────────────────────────────────┐
     *  │         RADIO SELECT  —  CHANGE THIS ONE LINE            │
     *  │   RADIO_MODE_LORA   (1)  LoRa Ra-02, ACK handshake      │
     *  │   RADIO_MODE_RF433  (0)  FS1000A OOK, simplex bcast     │
     *  └──────────────────────────────────────────────────────────┘
     * ================================================================ */
    uint8_t g_radio_mode = RADIO_MODE_LORA;   /* ← CHANGE HERE */

    /* ── Startup banner ───────────────────────────────────────────── */
    UART_PrintLn("\r\n");
    UART_PrintLn("=========================================");
    UART_PrintLn("  HELONIX  -  TANK TRANSMITTER NODE");
    UART_PrintLn("  Firmware : Single-Channel TX v6.4");
    UART_PrintLn("  PCB      : v1.0 — schematic verified");
    UART_PrintLn("  UART     : 115200 8N1");
    UART_PrintLn("  Loop     : 200 ms tick");
    UART_PrintLn("=========================================");
    UART_PrintLn("  Pin map:");
    UART_PrintLn("    FS1000A DATA  : PD1  (PD01 remap)");
    UART_PrintLn("    LoRa DIO0     : PB7  (RF_DATA_Pin, floating input)");
    UART_PrintLn("    LoRa NSS      : PA15 (SWJ_NOJTAG remap)");
    UART_PrintLn("    SPI1 CLK/MISO/MOSI : PB3/PB4/PB5 (SPI1 remap)");
    UART_PrintLn("    LoRa RST      : PB6");
    UART_PrintLn("    RELAY1/2/3    : PB0/PB1/PB2");
    UART_PrintLn("    SW1/2/3/4     : PB12/13/14/15");

    if (g_radio_mode == RADIO_MODE_LORA)
    {
        UART_PrintLn("  Radio    : LoRa Ra-02 SX1278 — bidirectional + ACK");
    }
    else
    {
        UART_PrintLn("  Radio    : RF433 FS1000A OOK — simplex broadcast");
        UART_PrintLn("             HELLO 5s / DATA 12s x3");
    }
    UART_PrintLn("=========================================");

    /* ── Raw silicon UID ──────────────────────────────────────────── */
    {
        volatile uint32_t *uid = (volatile uint32_t *)0x1FFFF7E8UL;
        char buf[64];
        snprintf(buf, sizeof(buf), "[UID] %08lX %08lX %08lX",
                 (unsigned long)uid[2],
                 (unsigned long)uid[1],
                 (unsigned long)uid[0]);
        UART_PrintLn(buf);
    }

    /* ── Device ID — init BEFORE any radio Init ───────────────────── */
    DeviceID_Init();
    {
        char myDID[9];
        DeviceID_GetHex(myDID);
        uint32_t didRaw = DeviceID_GetOwn();
        char buf[64];

        UART_PrintLn("-----------------------------------------");
        snprintf(buf, sizeof(buf), "[DID HEX] %s",       myDID);                  UART_PrintLn(buf);
        snprintf(buf, sizeof(buf), "[DID DEC] %lu",      (unsigned long)didRaw);   UART_PrintLn(buf);
        snprintf(buf, sizeof(buf), "[HELLO]   @HI:%s#",  myDID);                   UART_PrintLn(buf);
        UART_PrintLn("-----------------------------------------");
    }

    /* ── Configure RF_connector_Pin (PD1) for the chosen radio ──── *
     *  MX_GPIO_Init() already applied PD01 remap, so PD1 is a valid  *
     *  GPIO at this point. Radio_ConfigPin() sets direction only.     */
    Radio_ConfigPin(g_radio_mode);

    /* ── Initialise ONLY the active radio ───────────────────────── */
    if (g_radio_mode == RADIO_MODE_LORA)
    {
        LoRa_Init();
        loraMode = LORA_MODE_TRANSMITTER;
        UART_PrintLn("[INIT] LoRa: HELLO every 3s until connected");
        UART_PrintLn("[INIT] LoRa: DATA every 10s | PING every 30s");
    }
    else
    {
        /* RF_Init() reconfigures TIM3 → 1 MHz for OOK bit-bang.
         * Must be called AFTER MX_TIM3_Init().                        */
        RF_Init();
        UART_PrintLn("[INIT] RF433: HELLO broadcast every  5s");
        UART_PrintLn("[INIT] RF433: DATA  broadcast every 12s (3x repeated)");
        UART_PrintLn("[INFO] Simplex — no ACK, receiver deduplicates by seq");
    }

    UART_PrintLn("");

    /* ── Initial ADC read ──────────────────────────────────────────── */
    ADC_ReadAllChannels(&hadc1, &adcData);
    uint8_t currentLevel   = ADC_GetTankLevel();
    uint8_t currentWellDry = ADC_GetWellDry();

    /* ── Per-task timers ───────────────────────────────────────────── */
    uint32_t lastADCTick    = HAL_GetTick();
    uint32_t lastButtonTick = 0u;
    uint32_t lastStatsTick  = 0u;

    LoRa_TxResult lastLoraResult = LORA_TX_SKIPPED;
    uint8_t       prevConnected  = 0u;

    /* ================================================================
     *  Main loop — 200 ms nominal tick
     * ================================================================ */
    while (1)
    {
        uint32_t loopStart = HAL_GetTick();

        /* ── Step 1: button debounce (LoRa mode only) ──────────────── */
        if (g_radio_mode == RADIO_MODE_LORA)
        {
            if ((loopStart - lastButtonTick) >= BUTTON_DEBOUNCE_MS)
            {
                lastButtonTick = loopStart;
                if (HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin) == GPIO_PIN_RESET)
                {
                    UART_PrintLn("\r\n[BTN] Manual HELLO (LoRa) requested");
                    LoRa_ManualHello();
                }
            }
        }

        /* ── Step 2: ADC refresh every 1 s ─────────────────────────── */
        if ((loopStart - lastADCTick) >= ADC_REFRESH_MS)
        {
            lastADCTick = loopStart;
            ADC_ReadAllChannels(&hadc1, &adcData);
            currentLevel   = ADC_GetTankLevel();
            currentWellDry = ADC_GetWellDry();

            char buf[80];
            snprintf(buf, sizeof(buf),
                     "[ADC] Level:%u%%  WellDry:%u  Radio:%s",
                     currentLevel, currentWellDry,
                     (g_radio_mode == RADIO_MODE_LORA) ? "LoRa" : "RF433");
            UART_PrintLn(buf);
        }

        /* ── Step 3: Radio service ──────────────────────────────────── */
        if (g_radio_mode == RADIO_MODE_LORA)
        {
            LoRa_TxResult loraResult = LoRa_Service(currentLevel, currentWellDry);

            bool connChanged = (g_txConnected != prevConnected);
            if (connChanged)
            {
                prevConnected = g_txConnected;
                if (g_txConnected)
                    UART_PrintLn("\r\n[EVENT] LoRa *** CONNECTED — ACK'd data every 10s ***");
                else
                    UART_PrintLn("\r\n[EVENT] LoRa DISCONNECTED — retrying HELLO");
            }

            if (loraResult == LORA_TX_OK && LoRa_GetState() == LORA_STATE_CONNECTED)
            {
                char buf[100];
                snprintf(buf, sizeof(buf),
                         "[TX-OK] LoRa  Level:%u%%  WD:%u  OK:%lu  Fail:%lu",
                         currentLevel, currentWellDry,
                         (unsigned long)g_lora_tx_ok,
                         (unsigned long)g_lora_tx_fail);
                UART_PrintLn(buf);
            }
            else if (loraResult == LORA_TX_RETRY || loraResult == LORA_TX_FAIL)
            {
                char buf[80];
                snprintf(buf, sizeof(buf),
                         "[TX-FAIL] LoRa  State:%s  OK:%lu  Fail:%lu",
                         LoRa_GetStateString(),
                         (unsigned long)g_lora_tx_ok,
                         (unsigned long)g_lora_tx_fail);
                UART_PrintLn(buf);
            }

            (void)lastLoraResult;
            lastLoraResult = loraResult;

            if ((loopStart - lastStatsTick) >= STATS_INTERVAL_MS)
            {
                lastStatsTick = loopStart;
                uint32_t total = g_lora_tx_ok + g_lora_tx_retry + g_lora_tx_fail;
                uint32_t rate  = (total > 0u) ? ((g_lora_tx_ok * 100u) / total) : 0u;
                char buf[120];
                UART_PrintLn("-------- TX Statistics (LoRa) --------");
                snprintf(buf, sizeof(buf), "  State  : %s",    LoRa_GetStateString());         UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  OK     : %lu",   (unsigned long)g_lora_tx_ok);   UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  Retry  : %lu",   (unsigned long)g_lora_tx_retry);UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  Fail   : %lu",   (unsigned long)g_lora_tx_fail); UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  ACK%%   : %lu%%", (unsigned long)rate);           UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  Level  : %u%%  WD:%u", currentLevel, currentWellDry); UART_PrintLn(buf);
                UART_PrintLn("--------------------------------------");
            }
        }
        else  /* RADIO_MODE_RF433 */
        {
            RF_Service(currentLevel, currentWellDry);

            if ((loopStart - lastStatsTick) >= STATS_INTERVAL_MS)
            {
                lastStatsTick = loopStart;
                char myDID[9];
                DeviceID_GetHex(myDID);
                char buf[80];
                UART_PrintLn("-------- TX Statistics (RF433) -------");
                snprintf(buf, sizeof(buf), "  DID    : %s", myDID);                          UART_PrintLn(buf);
                snprintf(buf, sizeof(buf), "  Level  : %u%%  WD:%u  (last sent)",
                         currentLevel, currentWellDry);                                        UART_PrintLn(buf);
                UART_PrintLn("  Mode   : simplex OOK broadcast");
                UART_PrintLn("--------------------------------------");
            }
        }

        /* ── Step 6: pace loop to 200 ms tick ─────────────────────── */
        uint32_t elapsed = HAL_GetTick() - loopStart;
        if (elapsed < MAIN_LOOP_TICK_MS)
            HAL_Delay(MAIN_LOOP_TICK_MS - elapsed);
    }
}

/* ====================================================================
 *  SystemClock_Config
 *  HSI → PLL × 16 → 64 MHz SYSCLK
 *  APB1 = 32 MHz (÷2), APB2 = 64 MHz (÷1)
 *  ADC = PCLK2 / 6 ≈ 10.67 MHz, RTC = LSI
 * ==================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/* ====================================================================
 *  MX_GPIO_Init
 *
 *  AFIO remap sequence (ORDER IS CRITICAL):
 *    1. AFIO clock
 *    2. SWJ_NOJTAG  → frees PA15/PB3/PB4 from JTAG
 *    3. SPI1_ENABLE → moves SPI1 to PB3/PB4/PB5
 *    4. PD01_ENABLE → releases PD0/PD1 from OSC, PD1 usable as GPIO
 *    5. GPIO clocks (GPIOA, GPIOB, GPIOD — no GPIOC needed)
 *    6. Pin configurations
 *
 *  RF_connector_Pin (PD1): initialised as INPUT here.
 *  Radio_ConfigPin() reconfigures it to OUTPUT_PP (RF433) or
 *  leaves as INPUT (LoRa) after mode selection.
 * ==================================================================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ── Step 1: AFIO clock ────────────────────────────────────────── */
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* ── Step 2: SWJ → NOJTAG ─────────────────────────────────────── */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* ── Step 3: SPI1 remap → PB3/PB4/PB5 ────────────────────────── */
    __HAL_AFIO_REMAP_SPI1_ENABLE();

    /* ── Step 4: PD01 remap ────────────────────────────────────────── *
     *  Releases PD1 from OSC_OUT so it can be used as GPIO output     *
     *  for the FS1000A DATA line.  Without this, HAL_GPIO_WritePin()  *
     *  on PD1 has no effect — the oscillator function takes priority. */
    __HAL_AFIO_REMAP_PD01_ENABLE();

    /* ── Step 5: GPIO clocks ───────────────────────────────────────── *
     *  GPIOD for PD1 (RF_connector_Pin = FS1000A DATA)               */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ── Step 6: Safe output defaults ─────────────────────────────── */
    HAL_GPIO_WritePin(GPIOB,
        Relay1_Pin | Relay2_Pin | Relay3_Pin | LORA_STATUS_Pin | LED4_Pin | LED5_Pin,
        GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);

    /* ── Relays: PB0/PB1/PB2 ──────────────────────────────────────── */
    GPIO_InitStruct.Pin   = Relay1_Pin | Relay2_Pin | Relay3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── LoRa RST: PB6 ─────────────────────────────────────────────── */
    GPIO_InitStruct.Pin   = LORA_STATUS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_STATUS_GPIO_Port, &GPIO_InitStruct);

    /* ── LoRa DIO0: PB7 — INPUT floating (TX never reads DIO0) ────── */
    GPIO_InitStruct.Pin  = RF_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_DATA_GPIO_Port, &GPIO_InitStruct);

    /* ── LED4/LED5: PB8/PB9 ────────────────────────────────────────── */
    GPIO_InitStruct.Pin   = LED4_Pin | LED5_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── Switches: PB12-PB15, pull-up inputs ───────────────────────── */
    GPIO_InitStruct.Pin  = SWITCH1_Pin | SWITCH2_Pin | SWITCH3_Pin | SWITCH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── LED1: PA8 ─────────────────────────────────────────────────── */
    GPIO_InitStruct.Pin   = LED1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ── LED2/LED3: PA11/PA12 ──────────────────────────────────────── */
    GPIO_InitStruct.Pin   = LED2_Pin | LED3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ── LoRa NSS: PA15, high-speed ────────────────────────────────── */
    GPIO_InitStruct.Pin   = LORA_SELECT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_SELECT_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_SELECT_GPIO_Port, LORA_SELECT_Pin, GPIO_PIN_SET);

    /* ── RF_connector_Pin (PD1): start as INPUT floating ──────────── *
     *  Radio_ConfigPin() switches it to OUTPUT_PP HIGH (RF433) or    *
     *  leaves as INPUT (LoRa) after mode selection.                  */
    GPIO_InitStruct.Pin  = RF_connector_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RF_connector_GPIO_Port, &GPIO_InitStruct);  /* GPIOD */
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 8;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    uint32_t chList[] = {
        ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
        ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
    };
    for (uint8_t r = 0; r < 8; r++)
    {
        sConfig.Channel      = chList[r];
        sConfig.Rank         = r + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    }
}

static void MX_RTC_Init(void)
{
    RTC_TimeTypeDef  sTime        = {0};
    RTC_DateTypeDef  DateToUpdate = {0};
    RTC_AlarmTypeDef sAlarm       = {0};

    hrtc.Instance          = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut       = RTC_OUTPUTSOURCE_ALARM;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();

    sTime.Hours = 0x13; sTime.Minutes = 0x0; sTime.Seconds = 0x0;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();

    DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
    DateToUpdate.Month   = RTC_MONTH_JANUARY;
    DateToUpdate.Date    = 0x1;
    DateToUpdate.Year    = 0x0;
    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();

    sAlarm.AlarmTime.Hours = 0x13; sAlarm.AlarmTime.Minutes = 0x0; sAlarm.AlarmTime.Seconds = 0x0;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    UART_PrintLn("[ERROR] Error_Handler — system halted!");
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "[ASSERT] %s line %lu", (char *)file, line);
    UART_PrintLn(msg);
}
#endif
