/**
 * @file    fault_logger.c
 * @brief   Implementação do log de falhas com buffer circular e CRC16-CCITT
 */

#include "fault_logger.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ── Buffer circular ─────────────────────────────────────────────────── */

static Fault_Entry_t s_log[FAULT_LOG_MAX_ENTRIES];
static uint8_t       s_head;    /**< Próxima posição de escrita */
static uint8_t       s_count;   /**< Entradas válidas (0..MAX)  */
static uint16_t      s_crc;     /**< CRC16 do buffer completo   */

/* ── CRC16-CCITT (polinômio 0x1021) ─────────────────────────────────── */

static uint16_t crc16_compute(void)
{
    uint16_t       crc  = 0xFFFFU;
    const uint8_t *data = (const uint8_t *)s_log;
    uint32_t       len  = sizeof(s_log);   /* sempre sobre o array inteiro */

    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8U; j++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ─────────────────────────────────────────────────────────────────────── */

void FaultLogger_Init(void)
{
    memset(s_log, 0, sizeof(s_log));
    s_head  = 0U;
    s_count = 0U;
    s_crc   = crc16_compute();
}

void FaultLogger_Log(Fault_Code_t code, uint32_t context)
{
    if (code == FAULT_NONE || code >= FAULT_MAX_CODE) {
        return;
    }

    s_log[s_head].code      = code;
    s_log[s_head].timestamp = HAL_GetTick();
    s_log[s_head].context   = context;

    s_head = (uint8_t)((s_head + 1U) % FAULT_LOG_MAX_ENTRIES);

    if (s_count < FAULT_LOG_MAX_ENTRIES) {
        s_count++;
    }
    /* Atualiza CRC após cada escrita */
    s_crc = crc16_compute();
}

uint8_t FaultLogger_GetCount(void)
{
    return s_count;
}

bool FaultLogger_GetEntry(uint8_t index, Fault_Entry_t *entry)
{
    if (entry == NULL || index >= s_count) {
        return false;
    }
    /* Índice do elemento mais antigo no buffer circular */
    uint8_t oldest = (s_count < FAULT_LOG_MAX_ENTRIES)
                     ? 0U
                     : s_head;  /* buffer cheio: s_head aponta para o mais antigo */
    uint8_t real = (uint8_t)((oldest + index) % FAULT_LOG_MAX_ENTRIES);
    *entry = s_log[real];
    return true;
}

bool FaultLogger_GetLatest(Fault_Entry_t *entry)
{
    if (entry == NULL || s_count == 0U) {
        return false;
    }
    uint8_t last = (uint8_t)((s_head + FAULT_LOG_MAX_ENTRIES - 1U)
                              % FAULT_LOG_MAX_ENTRIES);
    *entry = s_log[last];
    return true;
}

bool FaultLogger_CheckIntegrity(void)
{
    return (crc16_compute() == s_crc);
}

void FaultLogger_PrintAll(void *huart)
{
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)huart;
    if (h == NULL || s_count == 0U) {
        return;
    }

    char buf[80];
    int  len;

    len = snprintf(buf, sizeof(buf),
                   "=== FAULT LOG (%u entries, CRC:%s) ===\r\n",
                   (unsigned int)s_count,
                   FaultLogger_CheckIntegrity() ? "OK" : "FAIL");
    if (len > 0) {
        HAL_UART_Transmit(h, (uint8_t *)buf, (uint16_t)len, 30U);
    }

    for (uint8_t i = 0U; i < s_count; i++) {
        Fault_Entry_t e;
        if (FaultLogger_GetEntry(i, &e)) {
            len = snprintf(buf, sizeof(buf),
                           "[%02u] T=%7lu CODE=%02u CTX=%lu\r\n",
                           (unsigned int)i,
                           (unsigned long)e.timestamp,
                           (unsigned int)e.code,
                           (unsigned long)e.context);
            if (len > 0) {
                HAL_UART_Transmit(h, (uint8_t *)buf, (uint16_t)len, 30U);
            }
        }
    }
}
