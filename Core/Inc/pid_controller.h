/**
 * @file    pid_controller.h
 * @brief   Controlador PID com bloco de Deadzone para controle de posição
 *
 * Inclui:
 *  - Anti-windup por clamping da integral
 *  - Bloco de Deadzone: se |erro| < deadzone → saída = 0, integral resetada
 *
 * @author  rrmatos
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* ── Constante padrão de Deadzone ───────────────────────────────────── */

/** Deadzone padrão em steps. Erros menores que este valor são ignorados. */
#define PID_DEADZONE_DEFAULT  5.0f

/* ── Estrutura do controlador PID ───────────────────────────────────── */

/**
 * @brief Estrutura do controlador PID com deadzone
 */
typedef struct {
    /* Ganhos */
    float kp;           /**< Ganho proporcional */
    float ki;           /**< Ganho integral */
    float kd;           /**< Ganho derivativo */

    /* Estado interno */
    float integral;     /**< Acumulador da integral */
    float prev_error;   /**< Erro no instante anterior (para termo D) */
    float last_error;   /**< Último erro calculado (para leitura externa) */

    /* Limites de saída (anti-windup por clamping) */
    float output_min;   /**< Limite mínimo da saída */
    float output_max;   /**< Limite máximo da saída */

    /* Tempo de amostragem */
    float sample_time;  /**< Período de amostragem em segundos */

    /* Deadzone */
    float deadzone;     /**< Largura da zona morta (em steps ou unidade de erro) */
    bool  in_deadzone;  /**< Verdadeiro quando o erro está dentro da deadzone */
} PID_t;

/* ── Protótipos de função ───────────────────────────────────────────── */

/**
 * @brief Inicializa o controlador PID com os parâmetros fornecidos
 * @param pid          Ponteiro para a estrutura PID
 * @param kp           Ganho proporcional
 * @param ki           Ganho integral
 * @param kd           Ganho derivativo
 * @param sample_time  Período de amostragem em segundos (ex.: 0.001 para 1 kHz)
 * @param out_min      Limite mínimo da saída
 * @param out_max      Limite máximo da saída
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float sample_time, float out_min, float out_max);

/**
 * @brief Calcula a saída do PID para o erro fornecido
 *
 * Se |erro| < deadzone → retorna 0, reseta integral e marca in_deadzone.
 * Caso contrário, calcula P+I+D com clamping da saída e anti-windup.
 *
 * @param pid      Ponteiro para a estrutura PID
 * @param setpoint Valor desejado
 * @param measured Valor medido
 * @return float Saída do controlador (limitada entre output_min e output_max)
 */
float PID_Compute(PID_t *pid, float setpoint, float measured);

/**
 * @brief Reseta o estado interno do PID (integral e erro anterior)
 * @param pid  Ponteiro para a estrutura PID
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Atualiza os ganhos do PID em tempo de execução
 * @param pid  Ponteiro para a estrutura PID
 * @param kp   Novo ganho proporcional
 * @param ki   Novo ganho integral
 * @param kd   Novo ganho derivativo
 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief Define o valor da zona morta
 * @param pid       Ponteiro para a estrutura PID
 * @param deadzone  Novo valor de deadzone (≥ 0)
 */
void PID_SetDeadzone(PID_t *pid, float deadzone);

/**
 * @brief Informa se o erro atual está dentro da deadzone
 * @param pid  Ponteiro para a estrutura PID
 * @return true  Se |erro| < deadzone
 * @return false Caso contrário
 */
bool PID_IsInDeadzone(const PID_t *pid);

/**
 * @brief Retorna o último erro calculado por PID_Compute
 * @param pid  Ponteiro para a estrutura PID
 * @return float Último erro
 */
float PID_GetLastError(const PID_t *pid);

#endif /* PID_CONTROLLER_H */
