/**
 * @file    pid_controller.c
 * @brief   Implementação do controlador PID com Deadzone e anti-windup
 *
 * Algoritmo de posição discreta (velocidade do erro):
 *   erro(k)    = setpoint - medido
 *   P          = kp * erro(k)
 *   I          = integral + ki * erro(k) * Ts
 *   D          = kd * (erro(k) - erro(k-1)) / Ts
 *   saída(k)   = P + I + D  (clamping para [output_min, output_max])
 *
 * Bloco Deadzone:
 *   Se |erro| < deadzone → saída = 0, integral = 0, in_deadzone = true
 *
 * @author  rrmatos
 */

#include "pid_controller.h"
#include <math.h>

/* ── Implementações ──────────────────────────────────────────────────── */

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float sample_time, float out_min, float out_max)
{
    if (pid == NULL) {
        return;
    }

    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->sample_time = sample_time;
    pid->output_min  = out_min;
    pid->output_max  = out_max;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->last_error  = 0.0f;
    pid->deadzone    = PID_DEADZONE_DEFAULT;
    pid->in_deadzone = false;
}

float PID_Compute(PID_t *pid, float setpoint, float measured)
{
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    /* Calcula o erro atual */
    float error = setpoint - measured;
    pid->last_error = error;

    /* ── Bloco Deadzone ─────────────────────────────────────────────── */
    if (fabsf(error) < pid->deadzone) {
        pid->in_deadzone = true;
        pid->integral    = 0.0f;   /* reseta integral para evitar windup */
        pid->prev_error  = 0.0f;   /* reseta derivativo */
        return 0.0f;
    }
    pid->in_deadzone = false;

    /* ── Termo Proporcional ─────────────────────────────────────────── */
    float p_term = pid->kp * error;

    /* ── Termo Integral com anti-windup por clamping ────────────────── */
    pid->integral += pid->ki * error * pid->sample_time;

    /* Limita a integral aos limites de saída para evitar windup */
    if (pid->integral > pid->output_max) {
        pid->integral = pid->output_max;
    } else if (pid->integral < pid->output_min) {
        pid->integral = pid->output_min;
    }

    /* ── Termo Derivativo ───────────────────────────────────────────── */
    float d_term = pid->kd * (error - pid->prev_error) / pid->sample_time;

    /* Armazena o erro para o próximo ciclo */
    pid->prev_error = error;

    /* ── Soma e clamping da saída ───────────────────────────────────── */
    output = p_term + pid->integral + d_term;

    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_error = 0.0f;
    pid->in_deadzone = false;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetDeadzone(PID_t *pid, float deadzone)
{
    if (pid == NULL) {
        return;
    }
    if (deadzone < 0.0f) {
        deadzone = 0.0f;
    }
    pid->deadzone = deadzone;
}

bool PID_IsInDeadzone(const PID_t *pid)
{
    if (pid == NULL) {
        return false;
    }
    return pid->in_deadzone;
}

float PID_GetLastError(const PID_t *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }
    return pid->last_error;
}
