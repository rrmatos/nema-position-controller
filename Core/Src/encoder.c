/**
 * @file    encoder.c
 * @brief   Implementação do encoder por potenciômetro multivolta (STM32F411)
 *
 * Lê o ADC 10-bit (0–1023), aplica filtro EMA e converte em ângulo/voltas/steps.
 *
 * Fórmulas:
 *   filtered = ALPHA * raw + (1 - ALPHA) * filtered_prev
 *   angle_degrees = (filtered - 512) * (3600.0 / 512.0)
 *   angle_turns   = angle_degrees / 360.0
 *   position_steps = (int32_t)(angle_turns * FULL_STEPS_PER_REV)
 *   position_normalized = (filtered - 512) / 512.0
 *
 * @author  rrmatos
 */

#include "encoder.h"
#include <math.h>

/* Fator de conversão: (ADC_raw - 512) * escala → graus */
#define DEG_SCALE   (POT_MAX_DEGREES / (float)(POT_ADC_CENTER))  /* 3600 / 512 */

/* ── Implementações ──────────────────────────────────────────────────── */

void Encoder_Pot_Init(Encoder_Pot_t *enc, ADC_HandleTypeDef *hadc, uint32_t channel)
{
    /* Valida ponteiros */
    if (enc == NULL || hadc == NULL) {
        return;
    }

    enc->hadc              = hadc;
    enc->adc_channel       = channel;
    enc->raw               = POT_ADC_CENTER;   /* assume centro na inicialização */
    enc->filtered_raw      = (float)POT_ADC_CENTER;
    enc->angle_degrees     = 0.0f;
    enc->angle_turns       = 0.0f;
    enc->position_steps    = 0;
    enc->position_normalized = 0.0f;
}

void Encoder_Pot_Update(Encoder_Pot_t *enc)
{
    if (enc == NULL || enc->hadc == NULL) {
        return;
    }

    /* Inicia conversão por polling e aguarda resultado */
    HAL_ADC_Start(enc->hadc);
    if (HAL_ADC_PollForConversion(enc->hadc, 10) == HAL_OK) {
        enc->raw = (uint16_t)HAL_ADC_GetValue(enc->hadc);
    }
    HAL_ADC_Stop(enc->hadc);

    /* Satura o valor bruto nos limites do ADC */
    if (enc->raw > POT_ADC_MAX) {
        enc->raw = POT_ADC_MAX;
    }

    /* Filtro EMA passa-baixa: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
    enc->filtered_raw = ADC_FILTER_ALPHA * (float)enc->raw
                      + (1.0f - ADC_FILTER_ALPHA) * enc->filtered_raw;

    /* Converte para ângulo em graus (±3600°) */
    enc->angle_degrees = (enc->filtered_raw - (float)POT_ADC_CENTER) * DEG_SCALE;

    /* Converte para voltas (±10) */
    enc->angle_turns = enc->angle_degrees / 360.0f;

    /* Converte para steps (±32000) */
    enc->position_steps = (int32_t)(enc->angle_turns * (float)FULL_STEPS_PER_REV);

    /* Posição normalizada no intervalo [-1, +1] */
    enc->position_normalized = (enc->filtered_raw - (float)POT_ADC_CENTER)
                              / (float)POT_ADC_CENTER;
}

float Encoder_Pot_GetDegrees(const Encoder_Pot_t *enc)
{
    if (enc == NULL) {
        return 0.0f;
    }
    return enc->angle_degrees;
}

float Encoder_Pot_GetTurns(const Encoder_Pot_t *enc)
{
    if (enc == NULL) {
        return 0.0f;
    }
    return enc->angle_turns;
}

int32_t Encoder_Pot_GetSteps(const Encoder_Pot_t *enc)
{
    if (enc == NULL) {
        return 0;
    }
    return enc->position_steps;
}

float Encoder_Pot_GetNormalized(const Encoder_Pot_t *enc)
{
    if (enc == NULL) {
        return 0.0f;
    }
    return enc->position_normalized;
}
