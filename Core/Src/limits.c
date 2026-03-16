/**
 * @file    limits.c
 * @brief   Implementação das verificações de limites de segurança
 */

#include "limits.h"
#include "fault_logger.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>

void Limits_Init(Limits_t *lim)
{
    lim->pos_limit_max        = SOFT_LIMIT_STEPS_MAX;
    lim->pos_limit_min        = SOFT_LIMIT_STEPS_MIN;
    lim->vel_max_sps          = VELOCITY_MAX_SAFE_SPS;
    lim->last_pos_for_stall   = 0;
    lim->stall_check_tick     = 0U;
    lim->stall_count          = 0U;
    lim->movement_start_tick  = 0U;
    lim->movement_active      = false;
    lim->fault_flags          = 0U;
}

void Limits_Update(Limits_t *lim,
                   int32_t  pos_steps,
                   uint32_t vel_sps,
                   uint16_t adc_raw,
                   bool     motor_moving)
{
    uint32_t now = HAL_GetTick();

    /* ── 1. Integridade do encoder ────────────────────────────────────── */
    if (adc_raw < (uint16_t)ADC_VALID_MIN || adc_raw > (uint16_t)ADC_VALID_MAX) {
        if (!(lim->fault_flags & SAFETY_FAULT_ENCODER_RANGE)) {
            lim->fault_flags |= SAFETY_FAULT_ENCODER_RANGE;
            FaultLogger_Log(FAULT_ENCODER_RANGE, (uint32_t)adc_raw);
        }
    } else {
        lim->fault_flags &= ~SAFETY_FAULT_ENCODER_RANGE;
    }

    /* ── 2. Limites de posição por software ───────────────────────────── */
    if (pos_steps > lim->pos_limit_max || pos_steps < lim->pos_limit_min) {
        if (!(lim->fault_flags & SAFETY_FAULT_POS_LIMIT)) {
            lim->fault_flags |= SAFETY_FAULT_POS_LIMIT;
            FaultLogger_Log(FAULT_POS_LIMIT_SW, (uint32_t)pos_steps);
        }
    } else {
        lim->fault_flags &= ~SAFETY_FAULT_POS_LIMIT;
    }

    /* ── 3. Limite de velocidade ─────────────────────────────────────── */
    if (vel_sps > lim->vel_max_sps) {
        if (!(lim->fault_flags & SAFETY_FAULT_VEL_LIMIT)) {
            lim->fault_flags |= SAFETY_FAULT_VEL_LIMIT;
            FaultLogger_Log(FAULT_VELOCITY_LIMIT, vel_sps);
        }
    } else {
        lim->fault_flags &= ~SAFETY_FAULT_VEL_LIMIT;
    }

    /* ── 4. Detecção de stall ─────────────────────────────────────────── */
    if (motor_moving) {
        if ((now - lim->stall_check_tick) >= STALL_CHECK_PERIOD_MS) {
            int32_t delta = pos_steps - lim->last_pos_for_stall;
            if (abs(delta) < STALL_MIN_DELTA_STEPS) {
                lim->stall_count++;
                if (lim->stall_count >= STALL_CONFIRM_COUNT) {
                    /* Stall confirmado após STALL_CONFIRM_COUNT períodos */
                    if (!(lim->fault_flags & SAFETY_FAULT_STALL)) {
                        lim->fault_flags |= SAFETY_FAULT_STALL;
                        FaultLogger_Log(FAULT_STALL_DETECTED, (uint32_t)pos_steps);
                    }
                }
            } else {
                lim->stall_count = 0U;
                lim->fault_flags &= ~SAFETY_FAULT_STALL;
            }
            lim->last_pos_for_stall = pos_steps;
            lim->stall_check_tick   = now;
        }
    } else {
        /* Motor parado: reseta contador de stall */
        lim->stall_count = 0U;
        lim->fault_flags &= ~SAFETY_FAULT_STALL;
    }

    /* ── 5. Timeout de movimento ──────────────────────────────────────── */
    if (lim->movement_active && motor_moving) {
        if ((now - lim->movement_start_tick) > MOVEMENT_TIMEOUT_MS) {
            if (!(lim->fault_flags & SAFETY_FAULT_TIMEOUT)) {
                lim->fault_flags |= SAFETY_FAULT_TIMEOUT;
                FaultLogger_Log(FAULT_MOVEMENT_TIMEOUT,
                                now - lim->movement_start_tick);
            }
        }
    }
}

void Limits_NotifyMovementStart(Limits_t *lim, int32_t pos_steps)
{
    lim->movement_active     = true;
    lim->movement_start_tick = HAL_GetTick();
    lim->last_pos_for_stall  = pos_steps;
    lim->stall_check_tick    = HAL_GetTick();
    lim->stall_count         = 0U;
    lim->fault_flags        &= ~SAFETY_FAULT_TIMEOUT;
}

void Limits_NotifyMovementEnd(Limits_t *lim)
{
    lim->movement_active  = false;
    lim->stall_count      = 0U;
    lim->fault_flags     &= ~(SAFETY_FAULT_TIMEOUT | SAFETY_FAULT_STALL);
}

uint32_t Limits_GetFaultFlags(const Limits_t *lim)
{
    return lim->fault_flags;
}

void Limits_ClearFaultFlags(Limits_t *lim, uint32_t flags)
{
    lim->fault_flags &= ~flags;
}
