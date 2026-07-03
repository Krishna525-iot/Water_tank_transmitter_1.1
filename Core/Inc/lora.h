/* ====================================================================
 * lora.h  —  TRANSMITTER public interface for SX1278 / Ra-02 433MHz
 *
 * v4.2 — register define corrections to match lora_TX.c fixes:
 *   LORA_REG_DETECT_OPT  (0x03) now written to reg 0x31 (RegDetectOptimize)
 *   LORA_REG_MODEM_CFG3  (0x04) now written to reg 0x26 (RegModemConfig3)
 *   LORA_REG_DETECTION_TH(0x0A) now written to reg 0x37 (RegDetectionThreshold)
 *   All OpMode values carry LowFrequencyModeOn (bit3=1) for 433 MHz.
 * ==================================================================== */

#ifndef LORA_TX_H
#define LORA_TX_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "lora_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* ── Pin definitions ───────────────────────────────────────────────── *
 *  NSS  : PA15 — SPI chip-select (freed by SWJ_NOJTAG remap)         *
 *  RST  : PB6  — hardware reset (active-LOW pulse)                    *
 *  DIO0 : PB7  — TX lora.c never reads DIO0 via GPIO (polls SPI IRQ) *
 *                Defined here for completeness; floating INPUT on TX.  */
#define LORA_NSS_PORT    LORA_SELECT_GPIO_Port   /* GPIOA, PA15 */
#define LORA_NSS_PIN     LORA_SELECT_Pin         /* GPIO_PIN_15 */

#define LORA_RESET_PORT  LORA_STATUS_GPIO_Port   /* GPIOB, PB6  */
#define LORA_RESET_PIN   LORA_STATUS_Pin         /* GPIO_PIN_6  */

#define LORA_DIO0_PORT   RF_DATA_GPIO_Port       /* GPIOB, PB7  */
#define LORA_DIO0_PIN    RF_DATA_Pin             /* GPIO_PIN_7  */

/* ── LoRa mode ─────────────────────────────────────────────────────── */
#define LORA_MODE_TRANSMITTER  0
#define LORA_MODE_RECEIVER     1

/* ── SX1278 / Ra-02 433MHz RF config ────────────────────────────────
 * Must match RX lora.h exactly.
 */
#define LORA_FREQ_HZ              433000000UL

/* RegModemConfig1 (0x1D): BW=125kHz, CR=4/5, Explicit Header */
#define LORA_REG_MODEM_CFG1       0x72

/* RegModemConfig2 (0x1E): SF7, CRC ON */
#define LORA_REG_MODEM_CFG2       0x74

/* RegModemConfig3 (0x26): LowDataRateOptimize=0, AgcAutoOn=1 */
#define LORA_REG_MODEM_CFG3       0x04

/* RegDetectOptimize (0x31): 0x03 for SF7-SF12 */
#define LORA_REG_DETECT_OPT       0x03

/* RegDetectOptimize2 — not used directly, kept for reference */
#define LORA_REG_DETECT_OPT2      0xC3

/* RegDetectionThreshold (0x37): 0x0A for SF7-SF12 */
#define LORA_REG_DETECTION_TH     0x0A

/* RegSyncWord (0x39): private network. Must match RX. */
#define LORA_SYNC_WORD            0x12

/* Preamble = 8 symbols */
#define LORA_PREAMBLE_MSB         0x00
#define LORA_PREAMBLE_LSB         0x08

/* PA_BOOST, Pout=14dBm */
#define LORA_REG_PA_CONFIG        0x8F

/* OCP ~100mA */
#define LORA_REG_OCP              0x2B

/* Normal PA DAC */
#define LORA_REG_PA_DAC           0x84

/* ── Timing ────────────────────────────────────────────────────────── */
#define LORA_TX_TIMEOUT_MS              3000UL
#define LORA_HELLO_ACK_WAIT_MS          2000UL
#define LORA_ACK_WAIT_MS                2000UL
#define LORA_TX_TO_RX_GAP_MS            10UL
#define LORA_RX_TO_TX_GAP_MS            10UL

#define TX_HELLO_RETRY_INTERVAL_MS      3000UL
#define TX_DATA_INTERVAL_MS             10000UL
#define TX_KEEPALIVE_INTERVAL_MS        30000UL
#define TX_DATA_RETRY_MAX               3
#define TX_NOACK_DISCONNECT_THRESH      3

/* ── Buffer sizes ──────────────────────────────────────────────────── */
#ifndef LORA_BUF_SIZE
#define LORA_BUF_SIZE                   128
#endif

#ifndef MAX_PACKET_LEN
#define MAX_PACKET_LEN                  96
#endif

#define TX_PACKET_SIZE                  MAX_PACKET_LEN

/* ── TX result ─────────────────────────────────────────────────────── */
typedef enum
{
    LORA_TX_OK = 0,
    LORA_TX_RETRY,
    LORA_TX_FAIL,
    LORA_TX_SKIPPED
} LoRa_TxResult;

/* ── Debug variable ────────────────────────────────────────────────── */
extern volatile char g_lastTxPacket[MAX_PACKET_LEN];

/* ── Public globals ────────────────────────────────────────────────── */
extern uint8_t  loraMode;
extern uint8_t  g_txConnected;
extern uint8_t  g_txDebugMode;
extern uint32_t g_lora_tx_ok;
extern uint32_t g_lora_tx_retry;
extern uint32_t g_lora_tx_fail;

/* ── Primary API ───────────────────────────────────────────────────── */
void              LoRa_Init           (void);
LoRa_ConnState_t  LoRa_GetState       (void);
const char *      LoRa_GetStateString (void);
uint32_t          LoRa_GetTxSequence  (void);

LoRa_TxResult     LoRa_Service        (uint8_t tank_level, uint8_t well_dry);

bool              LoRa_TriggerHandshake(void);
void              LoRa_ManualHello     (void);
void              LoRa_SendBye         (void);

void LoRa_BuildDataPacket(char *out, uint16_t out_size,
                          uint8_t tank_level, uint8_t well_dry);

/* ── Low-level register access ─────────────────────────────────────── */
void    LoRa_WriteReg(uint8_t addr, uint8_t data);
uint8_t LoRa_ReadReg (uint8_t addr);

/* ── Backward-compatible shim ──────────────────────────────────────── */
static inline LoRa_TxResult LoRa_Task(void)
{
    return LoRa_Service(0, 0);
}

#endif /* LORA_TX_H */
