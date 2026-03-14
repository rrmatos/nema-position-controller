/**
 * @file    encoder.h
 * @brief   Encoder baseado em potenciômetro multivolta para STM32F103C8
 *
 * Converte leitura ADC (0–4095) em ângulo, voltas e steps.
 * Faixa: ±10 voltas (20 voltas no total, center = 2048 → 0°)
 * Filtro EMA passa-baixa: y[n] = ALPHA * x[n] + (1 - ALPHA) * y[n-1]
 *
 * @author  rrmatos
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ── Constantes do potenciômetro ─────────────────────────────────────── */
#define POT_ADC_MIN          0          /**< Valor mínimo do ADC (12-bit) */
#define POT_ADC_MAX          4095       /**< Valor máximo do ADC (12-bit) */
#define POT_ADC_CENTER       2048       /**< Centro do ADC → 0 graus / 0 voltas */
#define POT_MAX_TURNS        10         /**< Máximo de voltas em cada sentido (±10) */
#define POT_MAX_DEGREES      3600.0f    /**< Faixa total em graus (±3600°) */

/** Coeficiente do filtro EMA (0 < alpha ≤ 1). Menor = mais suave. */
#define ADC_FILTER_ALPHA     0.15f

/* ── Passos por revolução (compatível com stepper.h) ────────────────── */
#ifndef FULL_STEPS_PER_REV
#define FULL_STEPS_PER_REV   3200       /**< 200 passos × 16 microstepping */
#endif

/* ── Estrutura do encoder ────────────────────────────────────────────── */

/**
 * @brief Estrutura de controle do encoder por potenciômetro
 */
typedef struct {
    ADC_HandleTypeDef *hadc;      /**< Handle do periférico ADC */
    uint32_t           adc_channel; /**< Canal ADC (ex.: ADC_CHANNEL_0) */
    uint16_t           raw;         /**< Leitura bruta do ADC (0–4095) */
    float              filtered_raw; /**< Leitura após filtro EMA */
    float              angle_degrees; /**< Ângulo em graus (±3600°) */
    float              angle_turns;   /**< Ângulo em voltas (±10) */
    int32_t            position_steps; /**< Posição em steps (±32000) */
    float              position_normalized; /**< Posição normalizada (−1.0 a +1.0) */
} Encoder_Pot_t;

/* ── Protótipos de função ────────────────────────────────────────────── */

/**
 * @brief Inicializa o encoder por potenciômetro
 * @param enc      Ponteiro para a estrutura do encoder
 * @param hadc     Handle do ADC configurado no STM32CubeMX
 * @param channel  Canal ADC (ex.: ADC_CHANNEL_0 para PA0)
 */
void Encoder_Pot_Init(Encoder_Pot_t *enc, ADC_HandleTypeDef *hadc, uint32_t channel);

/**
 * @brief Lê o ADC e atualiza todos os campos da estrutura
 * @param enc  Ponteiro para a estrutura do encoder
 */
void Encoder_Pot_Update(Encoder_Pot_t *enc);

/**
 * @brief Retorna o ângulo atual em graus (±3600°)
 * @param enc  Ponteiro para a estrutura do encoder
 * @return float Ângulo em graus
 */
float Encoder_Pot_GetDegrees(const Encoder_Pot_t *enc);

/**
 * @brief Retorna o número de voltas atual (±10)
 * @param enc  Ponteiro para a estrutura do encoder
 * @return float Número de voltas
 */
float Encoder_Pot_GetTurns(const Encoder_Pot_t *enc);

/**
 * @brief Retorna a posição atual em steps (±32000)
 * @param enc  Ponteiro para a estrutura do encoder
 * @return int32_t Posição em steps
 */
int32_t Encoder_Pot_GetSteps(const Encoder_Pot_t *enc);

/**
 * @brief Retorna a posição normalizada no intervalo [−1.0, +1.0]
 * @param enc  Ponteiro para a estrutura do encoder
 * @return float Posição normalizada
 */
float Encoder_Pot_GetNormalized(const Encoder_Pot_t *enc);

#endif /* ENCODER_H */
