/**
 * @file    estop.c
 * @brief   Implementação do módulo E-Stop, freio mecânico e STO
 */

#include "estop.h"

void EStop_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ── PB0 ← E-Stop (entrada, pull-up, normalmente fechado) ────────── */
    GPIO_InitStruct.Pin  = ESTOP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ESTOP_PORT, &GPIO_InitStruct);

    /* ── PB1 → Freio mecânico (saída push-pull) ──────────────────────── */
    GPIO_InitStruct.Pin   = BRAKE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BRAKE_PORT, &GPIO_InitStruct);

    /* ── PB5 → STO (saída push-pull) ─────────────────────────────────── */
    GPIO_InitStruct.Pin   = STO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STO_PORT, &GPIO_InitStruct);

    /* ── PC13 → LED alarme (saída push-pull, sobrescreve config anterior) */
    GPIO_InitStruct.Pin   = ALARM_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ALARM_LED_PORT, &GPIO_InitStruct);

    /* ── Estado seguro imediato ───────────────────────────────────────── */
    EStop_BrakeEngage();    /* Freio ENGAJADO                */
    EStop_STO_Activate();   /* Torque REMOVIDO               */
    EStop_AlarmOff();       /* Alarme desligado (partida OK) */
}

bool EStop_IsActive(void)
{
    return (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == ESTOP_ACTIVE_LEVEL);
}

void EStop_BrakeEngage(void)
{
    HAL_GPIO_WritePin(BRAKE_PORT, BRAKE_PIN, GPIO_PIN_SET);
}

void EStop_BrakeRelease(void)
{
    HAL_GPIO_WritePin(BRAKE_PORT, BRAKE_PIN, GPIO_PIN_RESET);
}

void EStop_STO_Activate(void)
{
    /* LOW = STO ativo — driver desabilitado, sem torque */
    HAL_GPIO_WritePin(STO_PORT, STO_PIN, GPIO_PIN_RESET);
}

void EStop_STO_Deactivate(void)
{
    /* HIGH = STO inativo — driver habilitado, torque permitido */
    HAL_GPIO_WritePin(STO_PORT, STO_PIN, GPIO_PIN_SET);
}

void EStop_AlarmOn(void)
{
    HAL_GPIO_WritePin(ALARM_LED_PORT, ALARM_LED_PIN, GPIO_PIN_SET);
}

void EStop_AlarmOff(void)
{
    HAL_GPIO_WritePin(ALARM_LED_PORT, ALARM_LED_PIN, GPIO_PIN_RESET);
}

void EStop_TriggerEmergencySequence(void)
{
    EStop_STO_Activate();    /* (1) Remove torque — mais rápido que freio mecânico */
    EStop_BrakeEngage();     /* (2) Engaja freio mecânico                          */
    EStop_AlarmOn();         /* (3) Sinalização visual                              */
}
