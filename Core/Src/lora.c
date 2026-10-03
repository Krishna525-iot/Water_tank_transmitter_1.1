/* ====================================================================
 * lora.c  —  TRANSMITTER LoRa driver  (water_level project)
 *
 * v4.2 — 433 MHz register fixes + FIFO base-address fix
 *
 * Bugs fixed vs v4.1:
 *
 *  BUG 1 (CRITICAL) — LowFrequencyModeOn missing from all RegOpMode writes
 *    SX1278 RegOpMode bit3 = LowFrequencyModeOn, required for <525 MHz.
 *    Without it the PA/PLL use the high-band path → wrong frequency.
 *    All RegOpMode values updated:
 *      sleep    0x80 → 0x88
 *      standby  0x81 → 0x89
 *      TX       0x83 → 0x8B
 *      RX cont  0x85 → 0x8D
 *
 *  BUG 2 (MODERATE) — Wrong value/register for ModemConfig3
 *    Reg 0x26 = RegModemConfig3 (LowDataRateOpt/AgcAutoOn)
 *    Reg 0x31 = RegDetectOptimize (0x03 for SF7-12)
 *    v4.1 wrote DETECT_OPT(0x03) to reg 0x26 → AgcAutoOn=0.
 *    Fix: write MODEM_CFG3(0x04) to reg 0x26
 *         write DETECT_OPT(0x03) to reg 0x31
 *         write DETECTION_TH(0x0A) to reg 0x37 (explicit, default OK)
 *
 *  BUG 3 (COSMETIC) — LoRa_Reset() redundant initial HIGH pulse
 *    PB6 starts LOW (MX_GPIO_Init default). Corrected sequence:
 *    assert LOW → hold → release HIGH → settle.
 *
 *  v4.1 retained fix: FIFO base addresses set to 0 in init.
 * ==================================================================== */

#include "lora.h"
#include "lora_parser.h"
#include "device_id.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;
extern void UART_PrintLn(const char *s);

extern volatile char txPacket[TX_PACKET_SIZE];

volatile char g_lastTxPacket[MAX_PACKET_LEN] = {0};

/* ── Public globals ─────────────────────────────────────────────────── */
uint8_t  loraMode        = LORA_MODE_TRANSMITTER;
uint8_t  g_txConnected   = 0;
uint8_t  g_txDebugMode   = 0;
uint32_t g_lora_tx_ok    = 0;
uint32_t g_lora_tx_retry = 0;
uint32_t g_lora_tx_fail  = 0;

/* ── Private state ──────────────────────────────────────────────────── */
static LoRa_ConnState_t s_state            = LORA_STATE_DISCONNECTED;
static uint32_t         s_txSeq            = 0;
static uint32_t         s_lastDataTxTick   = 0;
static uint32_t         s_lastAnyTxTick    = 0;
static uint32_t         s_lastHelloTick    = 0;
static uint8_t          s_consecCycleFails = 0;
static uint8_t          s_rxBuf[LORA_BUF_SIZE];
static bool             s_loraHwOk         = false;

#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET)

/* ── SX1278 register addresses ──────────────────────────────────────── */
#define REG_OPMODE              0x01
#define REG_FIFO                0x00
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_MODEM_CONFIG3       0x26
#define REG_DETECT_OPTIMIZE     0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD           0x39
#define REG_VERSION             0x42
/* RegVersion reads at init (expect 0x12), kept for SWD diagnostics */
volatile uint8_t g_loraVer[3] = {0xEE, 0xEE, 0xEE};
/* Live SX1278 check for SWD, every 2 s: [0]=RegVersion [1]=OpMode [2]=IRQ */
volatile uint8_t g_loraLive[3] = {0};
volatile uint32_t g_loraReinits = 0;   /* radio re-inits after a lost setup */

/* ── RegOpMode for 433 MHz — LowFrequencyModeOn (bit3) = 1 ──────────── *
 *  0x88 = LoRa + LowFreq + Sleep                                        *
 *  0x89 = LoRa + LowFreq + Standby                                      *
 *  0x8B = LoRa + LowFreq + TX                                           *
 *  0x8D = LoRa + LowFreq + RX Continuous                                */
#define OPMODE_LORA_SLEEP    0x88u
#define OPMODE_LORA_STANDBY  0x89u
#define OPMODE_LORA_TX       0x8Bu
#define OPMODE_LORA_RXCONT   0x8Du

/* ── SPI primitives ─────────────────────────────────────────────────── */
void LoRa_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80u), data };
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    NSS_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7Fu), 0x00u };
    uint8_t rx[2] = { 0x00u, 0x00u };
    NSS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    NSS_HIGH();
    return rx[1];
}

static void LoRa_WriteBuffer(uint8_t addr, const uint8_t *data, uint8_t size)
{
    uint8_t a = addr | 0x80u;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

static void LoRa_ReadBuffer(uint8_t addr, uint8_t *data, uint8_t size)
{
    uint8_t a = addr & 0x7Fu;
    NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);
    NSS_HIGH();
}

/* ── Hardware reset ─────────────────────────────────────────────────── *
 *  PB6 is initialised LOW by MX_GPIO_Init — module already in reset.  *
 *  Sequence: assert LOW → hold → release HIGH → settle.               */
static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
}

/* ── Mode helpers ───────────────────────────────────────────────────── */
static bool LoRa_Standby(void)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_STANDBY);
        HAL_Delay(5);
        if (LoRa_ReadReg(REG_OPMODE) == OPMODE_LORA_STANDBY)
            return true;
    }
    UART_PrintLn("[LORA TX] ERROR: failed to enter LoRa Standby (0x89)");
    return false;
}

static bool LoRa_RxContinuous(void)
{
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    for (uint8_t i = 0; i < 5; i++)
    {
        LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_RXCONT);
        HAL_Delay(5);
        if (LoRa_ReadReg(REG_OPMODE) == OPMODE_LORA_RXCONT)
            return true;
    }
    UART_PrintLn("[LORA TX] ERROR: failed to enter RX continuous (0x8D)");
    return false;
}

/* ── Transmit one packet ────────────────────────────────────────────── */
static bool LoRa_DoTransmit(const char *pkt, uint8_t len)
{
    if (!s_loraHwOk || pkt == NULL || len == 0 || len >= LORA_BUF_SIZE)
        return false;

    if (len < MAX_PACKET_LEN)
    {
        memcpy((void *)g_lastTxPacket, pkt, len);
        g_lastTxPacket[len] = '\0';
    }

    if (!LoRa_Standby()) return false;

    HAL_Delay(LORA_TX_TO_RX_GAP_MS);

    LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
    LoRa_WriteBuffer(REG_FIFO, (const uint8_t *)pkt, len);
    LoRa_WriteReg(0x22, len);          /* RegPayloadLength */
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_TX);
    HAL_Delay(2);

    uint8_t op = LoRa_ReadReg(REG_OPMODE);
    if (op != OPMODE_LORA_TX)
    {
        char dbg[80];
        snprintf(dbg, sizeof(dbg),
                 "[LORA TX] ERROR: TX mode failed OP=0x%02X want 0x%02X",
                 op, OPMODE_LORA_TX);
        UART_PrintLn(dbg);
        return false;
    }

    uint32_t t0 = HAL_GetTick();
    while ((LoRa_ReadReg(REG_IRQ_FLAGS) & 0x08u) == 0u)
    {
        if ((HAL_GetTick() - t0) > LORA_TX_TIMEOUT_MS)
        {
            UART_PrintLn("[LORA TX] ERROR: TxDone timeout");
            LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
            (void)LoRa_Standby();
            return false;
        }
    }

    LoRa_WriteReg(REG_IRQ_FLAGS, 0x08u);
    (void)LoRa_Standby();
    s_lastAnyTxTick = HAL_GetTick();
    return true;
}

/* ── Receive with timeout ───────────────────────────────────────────── */
static uint8_t LoRa_DoReceive(uint32_t timeout_ms)
{
    if (!s_loraHwOk) return 0;

    HAL_Delay(LORA_RX_TO_TX_GAP_MS);

    if (!LoRa_RxContinuous()) return 0;

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);

        if (irq & 0x20u)
        {
            LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
            UART_PrintLn("[LORA TX] RX CRC error while waiting ACK");
            (void)LoRa_Standby();
            return 0;
        }

        if (irq & 0x40u)
        {
            uint8_t len = LoRa_ReadReg(REG_RX_NB_BYTES);
            if (len == 0u || len >= (LORA_BUF_SIZE - 1u))
            {
                LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
                (void)LoRa_Standby();
                return 0;
            }

            uint8_t fifoAddr = LoRa_ReadReg(REG_FIFO_RX_CURRENT);
            LoRa_WriteReg(REG_FIFO_ADDR_PTR, fifoAddr);
            LoRa_ReadBuffer(REG_FIFO, s_rxBuf, len);
            s_rxBuf[len] = '\0';
            LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
            (void)LoRa_Standby();

            char dbg[100];
            snprintf(dbg, sizeof(dbg), "[LORA TX] RX raw ACK: \"%s\"", s_rxBuf);
            UART_PrintLn(dbg);
            return len;
        }
    }

    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    (void)LoRa_Standby();
    return 0;
}

/* ── State helpers ──────────────────────────────────────────────────── */
static void set_state(LoRa_ConnState_t new_state)
{
    if (s_state == new_state) return;
    s_state = new_state;
    g_txConnected = (new_state == LORA_STATE_CONNECTED) ? 1 : 0;
    char dbg[80];
    snprintf(dbg, sizeof(dbg), "[LORA TX] STATE -> %s", LoRa_GetStateString());
    UART_PrintLn(dbg);
}

const char *LoRa_GetStateString(void)
{
    switch (s_state)
    {
        case LORA_STATE_DISCONNECTED: return "DISCONNECTED";
        case LORA_STATE_HANDSHAKING:  return "HANDSHAKING";
        case LORA_STATE_CONNECTED:    return "CONNECTED";
        case LORA_STATE_STALE:        return "STALE";
        default:                      return "?";
    }
}

LoRa_ConnState_t LoRa_GetState     (void) { return s_state; }
uint32_t         LoRa_GetTxSequence(void) { return s_txSeq; }

/* ── TX + wait for response ─────────────────────────────────────────── */
static bool send_and_wait_response(const char *pkt, uint8_t len,
                                   uint32_t wait_ms, ParsedPacket_t *out_resp)
{
    if (!LoRa_DoTransmit(pkt, len)) return false;
    uint8_t rlen = LoRa_DoReceive(wait_ms);
    if (rlen == 0) return false;
    return LoRa_ParsePacket((const char *)s_rxBuf, out_resp);
}

/* ── Handshake (one attempt) ────────────────────────────────────────── */
static bool do_handshake_once(void)
{
    char hello[32];
    int  n = LoRa_BuildHello(hello, sizeof(hello), DeviceID_GetOwn());
    if (n <= 0) return false;

    char dbg[80];
    snprintf(dbg, sizeof(dbg), "[LORA TX] -> %s  (waiting %lu ms for ACK)",
             hello, (unsigned long)LORA_HELLO_ACK_WAIT_MS);
    UART_PrintLn(dbg);

    ParsedPacket_t resp;
    if (!send_and_wait_response(hello, (uint8_t)n, LORA_HELLO_ACK_WAIT_MS, &resp))
        return false;

    if (resp.type == PKT_TYPE_ACK && resp.did == DeviceID_GetOwn())
        return true;

    if (resp.type == PKT_TYPE_REJECT)
        UART_PrintLn("[LORA TX] HELLO rejected — RX needs pairing mode");

    return false;
}

/* ── Public API ─────────────────────────────────────────────────────── */
bool LoRa_TriggerHandshake(void)
{
    UART_PrintLn("[LORA TX] Manual HELLO requested");
    set_state(LORA_STATE_HANDSHAKING);
    s_lastHelloTick = HAL_GetTick();

    if (do_handshake_once())
    {
        UART_PrintLn("[LORA TX] *** HANDSHAKE OK — CONNECTED ***");
        set_state(LORA_STATE_CONNECTED);
        s_consecCycleFails = 0;
        return true;
    }

    UART_PrintLn("[LORA TX] HELLO no response — back to DISCONNECTED");
    set_state(LORA_STATE_DISCONNECTED);
    return false;
}

void LoRa_ManualHello(void) { (void)LoRa_TriggerHandshake(); }

void LoRa_SendBye(void)
{
    char bye[24];
    int  n = LoRa_BuildBye(bye, sizeof(bye), DeviceID_GetOwn());
    if (n > 0) (void)LoRa_DoTransmit(bye, (uint8_t)n);
}

void LoRa_BuildDataPacket(char *out, uint16_t out_size,
                          uint8_t tank_level, uint8_t well_dry)
{
    (void)LoRa_BuildData(out, (int)out_size, DeviceID_GetOwn(),
                         s_txSeq, tank_level, well_dry);
}

/* ── DATA with retries ──────────────────────────────────────────────── */
static LoRa_TxResult send_data_with_retries(uint8_t level, uint8_t wd)
{
    uint32_t seq_for_this_attempt = s_txSeq + 1;

    for (uint8_t attempt = 0; attempt < TX_DATA_RETRY_MAX; attempt++)
    {
        char pkt[MAX_PACKET_LEN];
        int  n = LoRa_BuildData(pkt, sizeof(pkt), DeviceID_GetOwn(),
                                seq_for_this_attempt, level, wd);
        if (n <= 0) { g_lora_tx_fail++; return LORA_TX_FAIL; }

        ParsedPacket_t resp;
        bool got = send_and_wait_response(pkt, (uint8_t)n,
                                          LORA_ACK_WAIT_MS, &resp);

        if (got && resp.type == PKT_TYPE_ACK && resp.did == DeviceID_GetOwn())
        {
            g_lora_tx_ok++;
            s_txSeq = seq_for_this_attempt;
            s_lastDataTxTick = HAL_GetTick();
            return LORA_TX_OK;
        }
        if (got && resp.type == PKT_TYPE_REJECT)
        { g_lora_tx_fail++; return LORA_TX_FAIL; }
        if (got && resp.type == PKT_TYPE_SYNC_REQ)
        { UART_PrintLn("[LORA TX] RX sent SYNC_REQ — re-handshake"); g_lora_tx_retry++; return LORA_TX_FAIL; }

        g_lora_tx_retry++;
    }

    g_lora_tx_fail++;
    return LORA_TX_RETRY;
}

/* ── PING ───────────────────────────────────────────────────────────── */
static bool send_ping(void)
{
    char pkt[MAX_PACKET_LEN];
    int  n = LoRa_BuildPing(pkt, sizeof(pkt), DeviceID_GetOwn(), s_txSeq);
    if (n <= 0) return false;

    ParsedPacket_t resp;
    if (!send_and_wait_response(pkt, (uint8_t)n, LORA_ACK_WAIT_MS, &resp))
        return false;

    return (resp.type == PKT_TYPE_PONG || resp.type == PKT_TYPE_ACK)
           && resp.did == DeviceID_GetOwn();
}

/* ====================================================================
 *  LoRa_Init
 * ==================================================================== */
void LoRa_Init(void)
{
    char dbg[160];
    s_loraHwOk = false;

    UART_PrintLn("");
    UART_PrintLn("=========================================");
    UART_PrintLn("  HELONIX TX  —  LoRa 433MHz init v4.2");
    UART_PrintLn("=========================================");

    DeviceID_Init();

    NSS_HIGH();
    HAL_Delay(20);

    LoRa_Reset();
    HAL_Delay(20);

    /* ── SPI diagnostic ─────────────────────────────────────────────── */
    uint8_t ver1 = LoRa_ReadReg(REG_VERSION); HAL_Delay(2);
    uint8_t ver2 = LoRa_ReadReg(REG_VERSION); HAL_Delay(2);
    uint8_t ver3 = LoRa_ReadReg(REG_VERSION);
    uint8_t ver  = ver3;
    g_loraVer[0] = ver1; g_loraVer[1] = ver2; g_loraVer[2] = ver3;   /* read over SWD */

    snprintf(dbg, sizeof(dbg),
             "[LORA TX] RegVersion reads: 0x%02X 0x%02X 0x%02X", ver1, ver2, ver3);
    UART_PrintLn(dbg);
    snprintf(dbg, sizeof(dbg),
             "[LORA TX] RegVersion(0x42) = 0x%02X  (expected 0x12)", ver);
    UART_PrintLn(dbg);

    if (ver != 0x12)
    {
        UART_PrintLn("[LORA TX] *** SPI LINK BROKEN — radio unreachable ***");
        UART_PrintLn("[LORA TX]   0xFF = NSS/MISO floating");
        UART_PrintLn("[LORA TX]   0x00 = MISO pulled low / power issue");
        UART_PrintLn("[LORA TX] Check: NSS=PA15  SCK=PB3  MISO=PB4  MOSI=PB5  RST=PB6");
        UART_PrintLn("[LORA TX]        SPI1 remap active? Ra-02 3.3V power?");
        s_loraHwOk = false;
        set_state(LORA_STATE_DISCONNECTED);
        return;
    }

    s_loraHwOk = true;

    /* ── FSK sleep → LoRa sleep ─────────────────────────────────────── *
     *  Use LowFrequencyModeOn from the very first LoRa write.          */
    LoRa_WriteReg(REG_OPMODE, 0x00);               /* FSK sleep (clear all) */
    HAL_Delay(2);
    LoRa_WriteReg(REG_OPMODE, OPMODE_LORA_SLEEP);  /* LoRa + LowFreq + sleep */
    HAL_Delay(10);

    /* ── Frequency: 433.000 MHz ─────────────────────────────────────── */
    uint64_t frf = ((uint64_t)433000000ULL << 19) / 32000000ULL;
    LoRa_WriteReg(0x06, (uint8_t)(frf >> 16));
    LoRa_WriteReg(0x07, (uint8_t)(frf >>  8));
    LoRa_WriteReg(0x08, (uint8_t)(frf      ));

    /* ── Preamble ───────────────────────────────────────────────────── */
    LoRa_WriteReg(0x20, LORA_PREAMBLE_MSB);
    LoRa_WriteReg(0x21, LORA_PREAMBLE_LSB);

    /* ── PA / OCP / DAC ────────────────────────────────────────────── */
    LoRa_WriteReg(0x09, LORA_REG_PA_CONFIG);
    LoRa_WriteReg(0x0B, LORA_REG_OCP);
    LoRa_WriteReg(0x4D, LORA_REG_PA_DAC);

    /* ── Modem config ───────────────────────────────────────────────── */
    LoRa_WriteReg(0x1D, LORA_REG_MODEM_CFG1);
    LoRa_WriteReg(0x1E, LORA_REG_MODEM_CFG2);

    /* Reg 0x26 = RegModemConfig3: AgcAutoOn=1 (was written with wrong value) */
    LoRa_WriteReg(REG_MODEM_CONFIG3,       LORA_REG_MODEM_CFG3);   /* 0x04 */
    /* Reg 0x31 = RegDetectOptimize: 0x03 for SF7-12 */
    LoRa_WriteReg(REG_DETECT_OPTIMIZE,     LORA_REG_DETECT_OPT);   /* 0x03 */
    /* Reg 0x37 = RegDetectionThreshold: 0x0A for SF7-12 */
    LoRa_WriteReg(REG_DETECTION_THRESHOLD, LORA_REG_DETECTION_TH); /* 0x0A */

    LoRa_WriteReg(REG_SYNC_WORD, LORA_SYNC_WORD);

    /* ── FIFO base addresses (v4.1 fix retained) ───────────────────── */
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR,     0x00);
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    /* ── Verify ─────────────────────────────────────────────────────── */
    uint8_t v_tx  = LoRa_ReadReg(REG_FIFO_TX_BASE_ADDR);
    uint8_t v_rx  = LoRa_ReadReg(REG_FIFO_RX_BASE_ADDR);
    uint8_t v_op  = LoRa_ReadReg(REG_OPMODE);
    uint8_t v_mc3 = LoRa_ReadReg(REG_MODEM_CONFIG3);
    uint8_t v_dop = LoRa_ReadReg(REG_DETECT_OPTIMIZE);

    snprintf(dbg, sizeof(dbg),
             "[LORA TX] Verify: OpMode=0x%02X TxBase=0x%02X RxBase=0x%02X",
             v_op, v_tx, v_rx);
    UART_PrintLn(dbg);
    snprintf(dbg, sizeof(dbg),
             "[LORA TX] Verify: ModemCfg3=0x%02X(want 0x04) DetOpt=0x%02X(want 0x03)",
             v_mc3, v_dop);
    UART_PrintLn(dbg);

    if (!LoRa_Standby())
    {
        s_loraHwOk = false;
        UART_PrintLn("[LORA TX] Init stopped: cannot enter standby");
        return;
    }

    uint8_t v_op2 = LoRa_ReadReg(REG_OPMODE);
    snprintf(dbg, sizeof(dbg),
             "[LORA TX] OpMode after Standby: 0x%02X (want 0x%02X)",
             v_op2, OPMODE_LORA_STANDBY);
    UART_PrintLn(dbg);

    /* ── Reset software state ───────────────────────────────────────── */
    s_state            = LORA_STATE_DISCONNECTED;
    g_txConnected      = 0;
    s_txSeq            = 0;
    s_lastDataTxTick   = 0;
    s_lastAnyTxTick    = 0;
    s_lastHelloTick    = 0;
    s_consecCycleFails = 0;

    memset((void *)g_lastTxPacket, 0, sizeof(g_lastTxPacket));
    memset((void *)s_rxBuf, 0, sizeof(s_rxBuf));

    UART_PrintLn("[LORA TX] Init complete — long-range 433 MHz config active");
}

/* ====================================================================
 *  LoRa_Service  — main service routine, call from main loop
 * ==================================================================== */
LoRa_TxResult LoRa_Service(uint8_t tank_level, uint8_t well_dry)
{
    if (loraMode != LORA_MODE_TRANSMITTER) return LORA_TX_SKIPPED;

    {
        static uint32_t lastLiveTick = 0;
        if ((HAL_GetTick() - lastLiveTick) >= 2000UL)
        {
            lastLiveTick  = HAL_GetTick();
            g_loraLive[0] = LoRa_ReadReg(REG_VERSION);
            g_loraLive[1] = LoRa_ReadReg(REG_OPMODE);
            g_loraLive[2] = LoRa_ReadReg(REG_IRQ_FLAGS);

            /* The Ra-02 can reset on its own (supply glitch when the board
             * is moved): it comes back in FSK mode with all LoRa settings
             * lost and TX sends into the void. Seen on the bench 29-09.
             * Chip answers but is not in LoRa mode (or was missing at
             * boot and is there now) -> set it up again and re-handshake. */
            bool chipThere = (g_loraLive[0] == 0x12u);
            bool inLora    = (g_loraLive[1] & 0x80u) != 0u;
            if (chipThere && (!inLora || !s_loraHwOk))
            {
                g_loraReinits++;
                UART_PrintLn("[LORA TX] radio lost its LoRa setup - re-init");
                LoRa_Init();
            }
        }
    }

    if (!s_loraHwOk)
    {
        static uint32_t lastHwErrPrint = 0;
        uint32_t now = HAL_GetTick();
        if ((now - lastHwErrPrint) >= 5000UL)
        {
            lastHwErrPrint = now;
            UART_PrintLn("[LORA TX] SKIP: LoRa hardware not ready.");
        }
        return LORA_TX_SKIPPED;
    }

    uint32_t now = HAL_GetTick();

    if (s_state == LORA_STATE_DISCONNECTED)
    {
        if ((now - s_lastHelloTick) < TX_HELLO_RETRY_INTERVAL_MS)
            return LORA_TX_SKIPPED;

        s_lastHelloTick = now;
        UART_PrintLn("[LORA TX] DISCONNECTED — sending HELLO");
        set_state(LORA_STATE_HANDSHAKING);

        if (do_handshake_once())
        {
            UART_PrintLn("[LORA TX] *** HANDSHAKE OK — CONNECTED ***");
            g_lora_tx_ok++;
            s_lastDataTxTick   = HAL_GetTick();
            s_lastAnyTxTick    = HAL_GetTick();
            s_consecCycleFails = 0;
            set_state(LORA_STATE_CONNECTED);
            return LORA_TX_OK;
        }

        UART_PrintLn("[LORA TX] HELLO failed — no valid ACK");
        g_lora_tx_fail++;
        set_state(LORA_STATE_DISCONNECTED);
        return LORA_TX_RETRY;
    }

    if (s_state == LORA_STATE_HANDSHAKING)
    {
        UART_PrintLn("[LORA TX] HANDSHAKING — retry HELLO");
        if (do_handshake_once())
        {
            UART_PrintLn("[LORA TX] *** HANDSHAKE OK — CONNECTED ***");
            g_lora_tx_ok++;
            s_lastDataTxTick   = HAL_GetTick();
            s_lastAnyTxTick    = HAL_GetTick();
            s_consecCycleFails = 0;
            set_state(LORA_STATE_CONNECTED);
            return LORA_TX_OK;
        }
        g_lora_tx_fail++;
        set_state(LORA_STATE_DISCONNECTED);
        return LORA_TX_RETRY;
    }

    if (s_state != LORA_STATE_CONNECTED)
    {
        set_state(LORA_STATE_DISCONNECTED);
        return LORA_TX_SKIPPED;
    }

    /* A new level or well state is sent at once (not after the 10 s
     * cadence), so the Main Controller sees it within ~1 s. */
    static uint8_t s_sentLevel = 0xFF, s_sentWellDry = 0xFF;
    bool changed       = (tank_level != s_sentLevel) || (well_dry != s_sentWellDry);
    bool data_due      = changed || (now - s_lastDataTxTick) >= TX_DATA_INTERVAL_MS;
    bool keepalive_due = (now - s_lastAnyTxTick)  >= TX_KEEPALIVE_INTERVAL_MS;

    LoRa_TxResult result = LORA_TX_SKIPPED;

    if (data_due)
    {
        UART_PrintLn("[LORA TX] CONNECTED — sending DATA");
        result = send_data_with_retries(tank_level, well_dry);
        /* Remembered even on no-ACK: the 10 s cadence resends it, so a
         * failed change is not retried every loop. */
        s_sentLevel   = tank_level;
        s_sentWellDry = well_dry;
    }
    else if (keepalive_due)
    {
        UART_PrintLn("[LORA TX] CONNECTED — sending PING");
        result = send_ping() ? LORA_TX_OK : LORA_TX_RETRY;
    }
    else
    {
        return LORA_TX_SKIPPED;
    }

    if (result == LORA_TX_OK)
    {
        s_consecCycleFails = 0;
        s_lastAnyTxTick = HAL_GetTick();
    }
    else
    {
        s_consecCycleFails++;
        char dbg[80];
        snprintf(dbg, sizeof(dbg), "[LORA TX] No-ACK streak: %u/%u",
                 s_consecCycleFails, TX_NOACK_DISCONNECT_THRESH);
        UART_PrintLn(dbg);

        if (s_consecCycleFails >= TX_NOACK_DISCONNECT_THRESH)
        {
            UART_PrintLn("[LORA TX] !!! DISCONNECTED — will re-handshake !!!");
            set_state(LORA_STATE_DISCONNECTED);
            s_lastHelloTick = 0;
        }
    }

    return result;
}
