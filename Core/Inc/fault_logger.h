/**
 * @file    fault_logger.h
 * @brief   Log de falhas em buffer circular RAM com verificação CRC16
 *
 * Armazena até FAULT_LOG_MAX_ENTRIES entradas em RAM.
 * Cada entrada: código de falha, timestamp (HAL_GetTick()) e contexto numérico.
 * Integridade verificada com CRC16-CCITT calculado sobre o buffer completo.
 *
 * @note Para persistência entre resets seria necessário Flash/SRAM de backup.
 *       Esta implementação usa apenas RAM — dados são perdidos no reset.
 *
 * @author  rrmatos
 */

#ifndef FAULT_LOGGER_H
#define FAULT_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* ── Capacidade do log ───────────────────────────────────────────────── */

#define FAULT_LOG_MAX_ENTRIES   16U   /**< Entradas no buffer circular */

/* ── Códigos de falha ────────────────────────────────────────────────── */

typedef enum {
    FAULT_NONE               = 0,
    FAULT_ESTOP_ACTIVE       = 1,   /**< E-Stop ativado                       */
    FAULT_ENCODER_RANGE      = 2,   /**< Encoder ADC fora do range plausível  */
    FAULT_VELOCITY_LIMIT     = 3,   /**< Velocidade excedeu limite seguro      */
    FAULT_POS_LIMIT_SW       = 4,   /**< Limite de posição por software        */
    FAULT_STALL_DETECTED     = 5,   /**< Stall do motor detectado              */
    FAULT_MOVEMENT_TIMEOUT   = 6,   /**< Timeout de movimento                  */
    FAULT_SELF_TEST_RAM      = 7,   /**< Falha no auto-teste de RAM            */
    FAULT_SAFE_STOP_ENTERED  = 8,   /**< Sistema entrou em Safe Stop           */
    FAULT_SAFE_STATE_ENTERED = 9,   /**< Estado seguro irreversível ativado    */
    FAULT_WATCHDOG_RESET     = 10,  /**< Reset causado por watchdog            */
    FAULT_MAX_CODE
} Fault_Code_t;

/* ── Estrutura de uma entrada de falha ───────────────────────────────── */

typedef struct {
    Fault_Code_t code;       /**< Código da falha              */
    uint32_t     timestamp;  /**< HAL_GetTick() no momento     */
    uint32_t     context;    /**< Dado de contexto (posição, ADC, etc.) */
} Fault_Entry_t;

/* ── Protótipos ──────────────────────────────────────────────────────── */

/** @brief Inicializa o log de falhas (limpa buffer, reseta CRC). */
void FaultLogger_Init(void);

/**
 * @brief Registra uma falha no buffer circular.
 * @param code     Código de falha (Fault_Code_t)
 * @param context  Dado de contexto dependente da falha
 */
void FaultLogger_Log(Fault_Code_t code, uint32_t context);

/**
 * @brief Retorna o número de entradas válidas no log (0..FAULT_LOG_MAX_ENTRIES).
 */
uint8_t FaultLogger_GetCount(void);

/**
 * @brief Lê uma entrada do log por índice (0 = mais antiga).
 * @param index  Índice desejado
 * @param entry  Ponteiro para receber os dados
 * @return true se índice válido e entry preenchida
 */
bool FaultLogger_GetEntry(uint8_t index, Fault_Entry_t *entry);

/**
 * @brief Retorna a entrada mais recente.
 * @param entry  Ponteiro para receber os dados
 * @return true se há pelo menos uma entrada
 */
bool FaultLogger_GetLatest(Fault_Entry_t *entry);

/**
 * @brief Verifica integridade do buffer via CRC16.
 * @return true se o CRC calculado coincide com o armazenado
 */
bool FaultLogger_CheckIntegrity(void);

/**
 * @brief Envia todas as entradas pela UART fornecida.
 * @param huart  Handle UART_HandleTypeDef (cast para void* para evitar dep. HAL)
 */
void FaultLogger_PrintAll(void *huart);

#endif /* FAULT_LOGGER_H */
