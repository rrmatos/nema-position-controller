/**
 * @file    watchdog.c
 * @brief   Implementação do IWDG (Independent Watchdog) para STM32F103C8
 */

#include "watchdog.h"
#include "stm32f1xx_hal.h"

static IWDG_HandleTypeDef s_hiwdg;

void SafeWatchdog_Init(void)
{
    /*
     * LSI ≈ 40 000 Hz
     * Prescaler = 32  → f_tick = 40000 / 32 = 1250 Hz
     * Reload    = 624 → timeout = 625 / 1250 = 500 ms
     *
     * Após HAL_IWDG_Init() o watchdog está rodando — não pode ser parado.
     */
    s_hiwdg.Instance       = IWDG;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    s_hiwdg.Init.Reload    = 624U;
    HAL_IWDG_Init(&s_hiwdg);
}

void SafeWatchdog_Kick(void)
{
    HAL_IWDG_Refresh(&s_hiwdg);
}
