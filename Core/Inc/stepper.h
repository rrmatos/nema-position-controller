/**
 * @file    stepper.h
 * @brief   Driver para motor de passo NEMA 17 com geração de STEP/DIR via Timer PWM
 *
 * Configuração de hardware:
 *  - PA5 → STEP (TIM2_CH1, PWM)
 *  - PA6 → DIR  (GPIO saída)
 *  - PA7 → EN   (GPIO saída, ativo em LOW para A4988/DRV8825)
 *
 * Timer (TIM2):
 *  - Clock: 84 MHz (APB1 × 2 no STM32F411)
 *  - Prescaler: 83 → tick = 1 µs
 *  - ARR = 1.000.000 / steps_per_sec → período do STEP em µs
 *
 * @author  rrmatos
 */

#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Constantes do motor NEMA 17 ─────────────────────────────────────── */

#define STEPS_PER_REV       200         /**< Passos por revolução (full-step) */
#define MICROSTEPPING       16          /**< Fator de microstepping (1/16) */
#define FULL_STEPS_PER_REV  3200        /**< Passos efetivos por revolução */
#define DEGREES_PER_STEP    0.1125f     /**< Graus por passo (360 / 3200) */

/** Velocidade máxima padrão em steps/s */
#define STEPPER_DEFAULT_MAX_SPEED    6400U

/** Aceleração padrão em steps/s² */
#define STEPPER_DEFAULT_ACCEL        12800U

/* ── Enumeração de direção ───────────────────────────────────────────── */

/**
 * @brief Direção de rotação do motor
 */
typedef enum {
    STEPPER_DIR_CW  = 0,  /**< Sentido horário (DIR = LOW) */
    STEPPER_DIR_CCW = 1   /**< Sentido anti-horário (DIR = HIGH) */
} Stepper_Dir_t;

/* ── Estrutura do stepper ────────────────────────────────────────────── */

/**
 * @brief Estrutura de controle do motor de passo
 */
typedef struct {
    /* Posição e setpoint */
    int32_t  current_pos;     /**< Posição atual em steps */
    int32_t  target_pos;      /**< Posição alvo em steps */

    /* Velocidade e aceleração */
    uint32_t current_speed;   /**< Velocidade atual em steps/s */
    uint32_t max_speed;       /**< Velocidade máxima em steps/s */
    uint32_t acceleration;    /**< Aceleração em steps/s² */

    /* Flags de estado */
    bool is_moving;           /**< Verdadeiro se o motor está em movimento */
    bool is_homed;            /**< Verdadeiro se o homing foi realizado */

    /* Direção atual */
    Stepper_Dir_t direction;  /**< Direção de rotação atual */

    /* Periférico Timer para o sinal STEP */
    TIM_HandleTypeDef *htim_step; /**< Handle do TIM usado para gerar STEP (PWM) */
    uint32_t           tim_channel; /**< Canal do Timer (ex.: TIM_CHANNEL_1) */

    /* Pinos DIR e EN */
    GPIO_TypeDef *dir_port;   /**< Porta GPIO do pino DIR */
    uint16_t      dir_pin;    /**< Pino GPIO do DIR */
    GPIO_TypeDef *en_port;    /**< Porta GPIO do pino EN */
    uint16_t      en_pin;     /**< Pino GPIO do EN */
} Stepper_t;

/* ── Protótipos de função ────────────────────────────────────────────── */

/**
 * @brief Inicializa o driver do motor de passo
 * @param stp        Ponteiro para a estrutura do stepper
 * @param htim       Handle do Timer para STEP (TIM2)
 * @param channel    Canal PWM (ex.: TIM_CHANNEL_1)
 * @param dir_port   Porta GPIO do pino DIR
 * @param dir_pin    Pino GPIO do DIR
 * @param en_port    Porta GPIO do pino EN
 * @param en_pin     Pino GPIO do EN
 */
void Stepper_Init(Stepper_t *stp,
                  TIM_HandleTypeDef *htim, uint32_t channel,
                  GPIO_TypeDef *dir_port, uint16_t dir_pin,
                  GPIO_TypeDef *en_port,  uint16_t en_pin);

/**
 * @brief Habilita o driver (EN = LOW para A4988/DRV8825)
 * @param stp  Ponteiro para a estrutura do stepper
 */
void Stepper_Enable(Stepper_t *stp);

/**
 * @brief Desabilita o driver (EN = HIGH), motor sem torque
 * @param stp  Ponteiro para a estrutura do stepper
 */
void Stepper_Disable(Stepper_t *stp);

/**
 * @brief Define a direção de rotação
 * @param stp  Ponteiro para a estrutura do stepper
 * @param dir  Direção: STEPPER_DIR_CW ou STEPPER_DIR_CCW
 */
void Stepper_SetDirection(Stepper_t *stp, Stepper_Dir_t dir);

/**
 * @brief Define a velocidade em RPM e inicia o PWM
 * @param stp  Ponteiro para a estrutura do stepper
 * @param rpm  Velocidade em rotações por minuto
 */
void Stepper_SetSpeedRPM(Stepper_t *stp, float rpm);

/**
 * @brief Define a velocidade em steps por segundo e inicia o PWM
 * @param stp        Ponteiro para a estrutura do stepper
 * @param steps_sec  Velocidade em steps/s (0 para parar)
 */
void Stepper_SetSpeedSteps(Stepper_t *stp, uint32_t steps_sec);

/**
 * @brief Move o motor para uma posição absoluta em steps
 * @param stp        Ponteiro para a estrutura do stepper
 * @param target     Posição alvo em steps
 * @param steps_sec  Velocidade em steps/s
 */
void Stepper_MoveTo(Stepper_t *stp, int32_t target, uint32_t steps_sec);

/**
 * @brief Move o motor para um ângulo absoluto em graus
 * @param stp    Ponteiro para a estrutura do stepper
 * @param angle  Ângulo alvo em graus
 * @param rpm    Velocidade em RPM
 */
void Stepper_MoveToAngle(Stepper_t *stp, float angle, float rpm);

/**
 * @brief Move o motor um número relativo de steps a partir da posição atual
 * @param stp        Ponteiro para a estrutura do stepper
 * @param delta      Deslocamento em steps (positivo = CW, negativo = CCW)
 * @param steps_sec  Velocidade em steps/s
 */
void Stepper_MoveRelative(Stepper_t *stp, int32_t delta, uint32_t steps_sec);

/**
 * @brief Para o motor imediatamente (desabilita PWM)
 * @param stp  Ponteiro para a estrutura do stepper
 */
void Stepper_Stop(Stepper_t *stp);

/**
 * @brief Converte steps em graus
 * @param steps  Número de steps
 * @return float Ângulo equivalente em graus
 */
float Stepper_StepsToAngle(int32_t steps);

/**
 * @brief Converte graus em steps
 * @param angle  Ângulo em graus
 * @return int32_t Número equivalente de steps
 */
int32_t Stepper_AngleToSteps(float angle);

/**
 * @brief Verifica se o motor atingiu o alvo (current_pos == target_pos)
 * @param stp  Ponteiro para a estrutura do stepper
 * @return true  Se na posição alvo
 * @return false Caso contrário
 */
bool Stepper_IsAtTarget(const Stepper_t *stp);

#endif /* STEPPER_H */
