/**
 * @file    stepper.c
 * @brief   Implementação do driver NEMA 17 com STEP/DIR via Timer PWM
 *
 * Geração do sinal STEP:
 *   - Timer configurado com prescaler 83 (clock 84 MHz → tick = 1 µs)
 *   - ARR = 1.000.000 / steps_per_sec  (período do STEP em µs)
 *   - Pulse (CCR) = ARR / 2            (duty cycle 50%)
 *
 * Pinos:
 *   - PA5 (TIM2_CH1) → STEP (PWM)
 *   - PA6             → DIR  (GPIO)
 *   - PA7             → EN   (GPIO, LOW = habilitado)
 *
 * @author  rrmatos
 */

#include "stepper.h"
#include <math.h>

/* Frequência de tick do timer em Hz (prescaler 83, clock 84 MHz) */
#define TIMER_TICK_HZ   1000000UL

/* ── Implementações ──────────────────────────────────────────────────── */

void Stepper_Init(Stepper_t *stp,
                  TIM_HandleTypeDef *htim, uint32_t channel,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin,
                  GPIO_TypeDef *en_port,  uint16_t en_pin)
{
    if (stp == NULL || htim == NULL) {
        return;
    }

    stp->current_pos   = 0;
    stp->target_pos    = 0;
    stp->current_speed = 0;
    stp->max_speed     = STEPPER_DEFAULT_MAX_SPEED;
    stp->acceleration  = STEPPER_DEFAULT_ACCEL;
    stp->is_moving     = false;
    stp->is_homed      = false;
    stp->direction     = STEPPER_DIR_CW;

    stp->htim_step  = htim;
    stp->tim_channel = channel;
    stp->dir_port   = dir_port;
    stp->dir_pin    = dir_pin;
    stp->en_port    = en_port;
    stp->en_pin     = en_pin;

    /* Garante que o motor inicia desabilitado */
    Stepper_Disable(stp);
}

void Stepper_Enable(Stepper_t *stp)
{
    if (stp == NULL) {
        return;
    }
    /* EN ativo em nível BAIXO para A4988/DRV8825 */
    HAL_GPIO_WritePin(stp->en_port, stp->en_pin, GPIO_PIN_RESET);
}

void Stepper_Disable(Stepper_t *stp)
{
    if (stp == NULL) {
        return;
    }
    /* EN inativo em nível ALTO → sem torque */
    HAL_GPIO_WritePin(stp->en_port, stp->en_pin, GPIO_PIN_SET);
    stp->is_moving = false;
}

void Stepper_SetDirection(Stepper_t *stp, Stepper_Dir_t dir)
{
    if (stp == NULL) {
        return;
    }
    stp->direction = dir;
    if (dir == STEPPER_DIR_CW) {
        HAL_GPIO_WritePin(stp->dir_port, stp->dir_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(stp->dir_port, stp->dir_pin, GPIO_PIN_SET);
    }
}

void Stepper_SetSpeedSteps(Stepper_t *stp, uint32_t steps_sec)
{
    if (stp == NULL) {
        return;
    }

    if (steps_sec == 0) {
        /* Para o motor: desabilita PWM */
        HAL_TIM_PWM_Stop(stp->htim_step, stp->tim_channel);
        stp->current_speed = 0;
        stp->is_moving     = false;
        return;
    }

    /* Limita à velocidade máxima */
    if (steps_sec > stp->max_speed) {
        steps_sec = stp->max_speed;
    }

    /* Calcula ARR: período do STEP em ticks de 1 µs */
    uint32_t arr = TIMER_TICK_HZ / steps_sec;
    if (arr < 2U) {
        arr = 2U;  /* valor mínimo para duty 50% */
    }

    /* Atualiza ARR e CCR (duty 50%) */
    __HAL_TIM_SET_AUTORELOAD(stp->htim_step, arr - 1U);
    __HAL_TIM_SET_COMPARE(stp->htim_step, stp->tim_channel, arr / 2U);

    /* Inicia PWM se ainda não estiver rodando */
    if (!stp->is_moving) {
        HAL_TIM_PWM_Start(stp->htim_step, stp->tim_channel);
    }

    stp->current_speed = steps_sec;
    stp->is_moving     = true;
}

void Stepper_SetSpeedRPM(Stepper_t *stp, float rpm)
{
    if (stp == NULL || rpm < 0.0f) {
        return;
    }
    /* Converte RPM para steps/s:  steps_s = rpm * FULL_STEPS_PER_REV / 60 */
    uint32_t steps_sec = (uint32_t)(rpm * (float)FULL_STEPS_PER_REV / 60.0f);
    Stepper_SetSpeedSteps(stp, steps_sec);
}

void Stepper_MoveTo(Stepper_t *stp, int32_t target, uint32_t steps_sec)
{
    if (stp == NULL) {
        return;
    }

    if (target == stp->current_pos) {
        return;
    }

    stp->target_pos = target;

    /* Define direção conforme o deslocamento */
    if (target > stp->current_pos) {
        Stepper_SetDirection(stp, STEPPER_DIR_CW);
    } else {
        Stepper_SetDirection(stp, STEPPER_DIR_CCW);
    }

    Stepper_SetSpeedSteps(stp, steps_sec);
}

void Stepper_MoveToAngle(Stepper_t *stp, float angle, float rpm)
{
    if (stp == NULL) {
        return;
    }
    int32_t target_steps = Stepper_AngleToSteps(angle);
    uint32_t steps_sec   = (uint32_t)(rpm * (float)FULL_STEPS_PER_REV / 60.0f);
    Stepper_MoveTo(stp, target_steps, steps_sec);
}

void Stepper_MoveRelative(Stepper_t *stp, int32_t delta, uint32_t steps_sec)
{
    if (stp == NULL) {
        return;
    }
    Stepper_MoveTo(stp, stp->current_pos + delta, steps_sec);
}

void Stepper_Stop(Stepper_t *stp)
{
    if (stp == NULL) {
        return;
    }
    HAL_TIM_PWM_Stop(stp->htim_step, stp->tim_channel);
    stp->current_speed = 0;
    stp->is_moving     = false;
}

float Stepper_StepsToAngle(int32_t steps)
{
    return (float)steps * DEGREES_PER_STEP;
}

int32_t Stepper_AngleToSteps(float angle)
{
    return (int32_t)(angle / DEGREES_PER_STEP);
}

bool Stepper_IsAtTarget(const Stepper_t *stp)
{
    if (stp == NULL) {
        return false;
    }
    return (stp->current_pos == stp->target_pos);
}
