/**
 * @file    limits.h
 * @brief   Verificação de limites de segurança para o controlador de posição
 *
 * Verificações implementadas:
 *   1. Integridade do encoder — ADC fora da faixa plausível (sensor desconectado)
 *   2. Limites de posição por software (soft limits) — ±90% da faixa mecânica
 *   3. Limite de velocidade máxima segura
 *   4. Detecção de stall — motor comandado mas sem variação de posição
 *   5. Timeout de movimento — movimento excede tempo máximo esperado
 *
 * Os fault_flags desta estrutura usam os bitmasks SAFETY_FAULT_* definidos
 * em safety.h e são repassados diretamente para Safety_Update().
 *
 * @author  rrmatos
 */

#ifndef LIMITS_H
#define LIMITS_H

#include "safety.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Limites padrão ──────────────────────────────────────────────────── */

/** Soft limits em steps (±90% de ±32000 = ±28800 → ±9 voltas) */
#define SOFT_LIMIT_STEPS_MAX     28800
#define SOFT_LIMIT_STEPS_MIN    -28800

/** Velocidade máxima segura em steps/s (1,5 rev/s = 4800 steps/s c/ 1/16) */
#define VELOCITY_MAX_SAFE_SPS    4800U

/** Faixa válida do ADC — detecta sensor desconectado (margem de 5%, ADC 12-bit) */
#define ADC_VALID_MIN            205U   /**< ~5% de 4095  */
#define ADC_VALID_MAX            3890U  /**< ~95% de 4095 */

/** Parâmetros de detecção de stall */
#define STALL_CHECK_PERIOD_MS    100U   /**< Verificação a cada 100 ms                   */
#define STALL_MIN_DELTA_STEPS    1      /**< Mínimo de steps por período para não ser stall */
#define STALL_CONFIRM_COUNT      5U     /**< Períodos consecutivos sem movimento → stall  */

/** Timeout de movimento — se o motor não chega ao destino em 10 s → falha */
#define MOVEMENT_TIMEOUT_MS      10000U

/* ── Estrutura de limites ────────────────────────────────────────────── */

typedef struct {
    /* Limites de posição */
    int32_t  pos_limit_max;          /**< Limite superior de posição em steps */
    int32_t  pos_limit_min;          /**< Limite inferior de posição em steps */

    /* Limite de velocidade */
    uint32_t vel_max_sps;            /**< Velocidade máxima em steps/s        */

    /* Detecção de stall */
    int32_t  last_pos_for_stall;     /**< Posição na última verificação       */
    uint32_t stall_check_tick;       /**< Tick da última verificação de stall */
    uint8_t  stall_count;            /**< Contador de detecções consecutivas  */

    /* Timeout de movimento */
    uint32_t movement_start_tick;    /**< HAL_GetTick() ao iniciar movimento  */
    bool     movement_active;        /**< Verdadeiro enquanto há movimento    */

    /* Bitmask de falhas (SAFETY_FAULT_*) */
    uint32_t fault_flags;
} Limits_t;

/* ── Protótipos ──────────────────────────────────────────────────────── */

/**
 * @brief Inicializa a estrutura com valores padrão seguros.
 * @param lim  Ponteiro para Limits_t
 */
void Limits_Init(Limits_t *lim);

/**
 * @brief Executa todas as verificações de limite.
 *        Chamar no loop de controle (1 kHz).
 *
 * @param lim          Ponteiro para Limits_t
 * @param pos_steps    Posição atual em steps
 * @param vel_sps      Velocidade atual em steps/s (valor absoluto)
 * @param adc_raw      Leitura bruta do ADC (para checar sensor)
 * @param motor_moving Verdadeiro se o motor está sendo comandado
 */
void Limits_Update(Limits_t *lim,
                   int32_t  pos_steps,
                   uint32_t vel_sps,
                   uint16_t adc_raw,
                   bool     motor_moving);

/**
 * @brief Notifica início de movimento (reinicia timeout e stall counter).
 * @param lim       Ponteiro para Limits_t
 * @param pos_steps Posição inicial do movimento
 */
void Limits_NotifyMovementStart(Limits_t *lim, int32_t pos_steps);

/**
 * @brief Notifica conclusão de movimento (desativa timeout).
 * @param lim  Ponteiro para Limits_t
 */
void Limits_NotifyMovementEnd(Limits_t *lim);

/**
 * @brief Retorna os fault flags ativos (bitmask SAFETY_FAULT_*).
 * @param lim  Ponteiro para Limits_t
 * @return uint32_t bitmask de falhas
 */
uint32_t Limits_GetFaultFlags(const Limits_t *lim);

/**
 * @brief Limpa os flags especificados (após tratamento da falha).
 * @param lim   Ponteiro para Limits_t
 * @param flags Flags a limpar
 */
void Limits_ClearFaultFlags(Limits_t *lim, uint32_t flags);

#endif /* LIMITS_H */
