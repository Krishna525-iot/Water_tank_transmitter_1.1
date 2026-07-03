/* ====================================================================
 * device_id.c  —  TRANSMITTER version (NO EEPROM)
 *
 * The transmitter hardware has no EEPROM, so we derive the Device ID
 * entirely from the STM32 silicon UID at 0x1FFFF7E8.
 * The UID is factory-programmed and unique per die, so no persistence
 * is needed.  The same DID is reproduced on every boot automatically.
 *
 * PairedDev_* stubs are provided so lora.c compiles without changes;
 * on the TX side these functions are never called in normal operation.
 * ==================================================================== */

#include "device_id.h"
#include <string.h>
#include <stdio.h>

/* STM32F1 unique device ID base address (96-bit, three 32-bit words)  */
#define UID_BASE_ADDR   0x1FFFF7E8UL

/* ── Module state ───────────────────────────────────────────────────── */
static uint32_t s_ownDID    = 0;
static char     s_ownHex[9] = "00000000";

/* Simple but good enough mix to collapse 96 bits → 32 bits            */
static uint32_t mix32(uint32_t a, uint32_t b, uint32_t c)
{
    /* FNV-1a inspired fold */
    uint32_t h = 0x811C9DC5UL;
    h ^= a; h *= 0x01000193UL;
    h ^= b; h *= 0x01000193UL;
    h ^= c; h *= 0x01000193UL;
    return h;
}

/* ── DeviceID API ───────────────────────────────────────────────────── */

void DeviceID_Init(void)
{
    volatile uint32_t *uid = (volatile uint32_t *)UID_BASE_ADDR;
    uint32_t u0 = uid[0];
    uint32_t u1 = uid[1];
    uint32_t u2 = uid[2];

    uint32_t did = mix32(u0, u1, u2);

    /* Guarantee the two sentinel values are never produced             */
    if (did == 0U)              did = mix32(u0 ^ 0xAA, u1, u2);
    if (did == 0xFFFFFFFFUL)    did = mix32(u0, u1 ^ 0x55, u2);

    s_ownDID = did;
    snprintf(s_ownHex, sizeof(s_ownHex), "%08lX", (unsigned long)did);

    /* Caller (main.c) will print this — no UART call here             */
}

uint32_t DeviceID_GetOwn(void)
{
    return s_ownDID;
}

void DeviceID_GetHex(char out[9])
{
    if (out) memcpy(out, s_ownHex, 9);
}

/* ── PairedDev stubs (TX never needs to track paired receivers) ──────── *
 *
 * The TX just broadcasts to whoever is listening; it has no concept of
 * a paired-device list.  These stubs satisfy the linker if any module
 * happens to include device_id.h and call them.
 * ──────────────────────────────────────────────────────────────────── */

void     PairedDev_Init    (void)               { /* no-op on TX */ }
void     PairedDev_Clear   (void)               { /* no-op on TX */ }
bool     PairedDev_Add     (uint32_t did)       { (void)did; return true; }
bool     PairedDev_Remove  (uint32_t did)       { (void)did; return false; }
bool     PairedDev_IsAllowed(uint32_t did)      { (void)did; return true; }
uint8_t  PairedDev_Count   (void)               { return 0; }
uint32_t PairedDev_Get     (uint8_t index)      { (void)index; return 0; }
