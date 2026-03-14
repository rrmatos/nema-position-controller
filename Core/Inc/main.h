/**
 * @file    main.h
 * @brief   Cabeçalho principal – Controlador de Posição NEMA 17 (STM32F103C8)
 *
 * Inclui os cabeçalhos HAL necessários e declara os handles de periférico
 * exportados por main.c para outros módulos.
 *
 * @author  rrmatos
 */

#ifndef MAIN_H
#define MAIN_H

#include "stm32f1xx_hal.h"

/* ── Handles de periférico (definidos em main.c) ─────────────────────── */

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim3;   /**< TIM3 — PWM STEP (PA6, canal 1) */
extern TIM_HandleTypeDef  htim4;   /**< TIM4 — loop de controle 1 kHz */
extern UART_HandleTypeDef huart1;

/* ── Definições de pinos (para referência rápida) ────────────────────── */

/** Pino do sinal STEP (TIM3_CH1, AF_PP) */
#define STEP_PIN      GPIO_PIN_6
#define STEP_PORT     GPIOA

/** Pino do sinal DIR */
#define DIR_PIN       GPIO_PIN_1
#define DIR_PORT      GPIOA

/** Pino do sinal EN (ativo LOW para A4988/DRV8825) */
#define EN_PIN        GPIO_PIN_2
#define EN_PORT       GPIOA

/** Pino do potenciômetro (ADC1_IN0) */
#define POT_PIN       GPIO_PIN_0
#define POT_PORT      GPIOA

/** Pino do botão USER / LED onboard */
#define BTN_PIN       GPIO_PIN_13
#define BTN_PORT      GPIOC

#endif /* MAIN_H */
