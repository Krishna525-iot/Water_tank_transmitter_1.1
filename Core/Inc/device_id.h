/* ====================================================================
 * device_id.h  —  Device ID and paired-device list API
 *
 * Both TX and RX include this header.  The TX links device_id_tx.c
 * (no EEPROM).  The RX links device_id_rx.c (with EEPROM persistence).
 * ==================================================================== */

#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum number of paired transmitters stored on the RX side.        */
#define MAX_PAIRED   4U

/* ── Own device identity ────────────────────────────────────────────── */
void     DeviceID_Init   (void);
uint32_t DeviceID_GetOwn (void);
void     DeviceID_GetHex (char out[9]);   /* 8 hex chars + NUL */

/* ── Paired-device list (RX only; TX stubs return safe defaults) ─────── */
void     PairedDev_Init    (void);
void     PairedDev_Clear   (void);
bool     PairedDev_Add     (uint32_t did);
bool     PairedDev_Remove  (uint32_t did);
bool     PairedDev_IsAllowed(uint32_t did);
uint8_t  PairedDev_Count   (void);
uint32_t PairedDev_Get     (uint8_t index);

#endif /* DEVICE_ID_H */
