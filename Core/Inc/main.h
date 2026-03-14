/**
 * @file    main.h
 * @brief   Cabeçalho principal – Controlador de Posição NEMA 17 (STM32F411)
 *
 * Inclui os cabeçalhos HAL necessários e declara os handles de periférico
 * exportados por main.c para outros módulos.
 *
 * @author  rrmatos
 */

#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* ── Handles de periférico (definidos em main.c) ─────────────────────── */

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart2;

/* ── Definições de pinos (para referência rápida) ────────────────────── */

/** Pino do sinal STEP (TIM2_CH1, AF1) */
#define STEP_PIN      GPIO_PIN_5
#define STEP_PORT     GPIOA

/** Pino do sinal DIR */
#define DIR_PIN       GPIO_PIN_6
#define DIR_PORT      GPIOA

/** Pino do sinal EN (ativo LOW para A4988/DRV8825) */
#define EN_PIN        GPIO_PIN_7
#define EN_PORT       GPIOA

/** Pino do potenciômetro (ADC1_IN0) */
#define POT_PIN       GPIO_PIN_0
#define POT_PORT      GPIOA

/** Pino do botão USER */
#define BTN_PIN       GPIO_PIN_13
#define BTN_PORT      GPIOC

#endif /* MAIN_H */
