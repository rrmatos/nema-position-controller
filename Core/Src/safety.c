/**
 * @file    safety.c
 * @brief   Implementação da máquina de estados de segurança funcional
 */

#include "safety.h"
#include "estop.h"
#include "fault_logger.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ─────────────────────────────────────────────────────────────────────── */

void Safety_Init(Safety_t *s)
{
    memset(s, 0, sizeof(Safety_t));
    s->state            = SAFETY_POWER_ON;
    s->prev_state       = SAFETY_POWER_ON;
    s->fault_flags      = 0U;
    s->state_entry_tick = HAL_GetTick();
    s->self_test_done   = 0U;
}

/* ── Auto-teste inicial ───────────────────────────────────────────────── */

bool Safety_RunSelfTest(Safety_t *s)
{
    s->prev_state       = s->state;
    s->state            = SAFETY_SELF_TEST;
    s->state_entry_tick = HAL_GetTick();

    bool ok = true;

    /* Teste 1: Integridade do log de falhas (CRC do buffer) */
    if (!FaultLogger_CheckIntegrity()) {
        ok = false;
        s->fault_flags |= SAFETY_FAULT_SELF_TEST;
        FaultLogger_Log(FAULT_SELF_TEST_RAM, 0U);
    }

    /* Teste 2: E-Stop deve estar em repouso (pino LOW = NC fechada) */
    if (EStop_IsActive()) {
        ok = false;
        s->fault_flags |= SAFETY_FAULT_ESTOP;
        FaultLogger_Log(FAULT_ESTOP_ACTIVE, 0U);
    }

    /* Resultado */
    if (ok) {
        s->self_test_done   = 1U;
        s->prev_state       = s->state;
        s->state            = SAFETY_OPERATIONAL;
    } else {
        s->prev_state       = s->state;
        s->state            = SAFETY_SAFE_STATE;
        FaultLogger_Log(FAULT_SAFE_STATE_ENTERED, s->fault_flags);
    }

    s->state_entry_tick = HAL_GetTick();
    return ok;
}

/* ── Atualização da máquina de estados ───────────────────────────────── */

void Safety_Update(Safety_t *s, bool estop_active, uint32_t fault_flags)
{
    /* Estado SAFE_STATE é irreversível — somente reset manual  */
    if (s->state == SAFETY_SAFE_STATE) {
        return;
    }

    /* E-Stop tem prioridade absoluta sobre qualquer outro estado */
    if (estop_active) {
        if (s->state != SAFETY_EMERGENCY_STOP) {
            s->prev_state       = s->state;
            s->state            = SAFETY_EMERGENCY_STOP;
            s->state_entry_tick = HAL_GetTick();
            s->fault_flags     |= SAFETY_FAULT_ESTOP;
            FaultLogger_Log(FAULT_ESTOP_ACTIVE, (uint32_t)s->prev_state);
        }
        return;
    }

    /* Falhas críticas → SAFE_STATE irreversível */
    if (fault_flags & SAFETY_CRITICAL_FAULT_MASK) {
        s->prev_state       = s->state;
        s->state            = SAFETY_SAFE_STATE;
        s->state_entry_tick = HAL_GetTick();
        s->fault_flags     |= fault_flags;
        FaultLogger_Log(FAULT_SAFE_STATE_ENTERED, fault_flags);
        return;
    }

    /* Atualiza bitmask */
    s->fault_flags = fault_flags;

    switch (s->state) {

        case SAFETY_OPERATIONAL:
            if (fault_flags & SAFETY_SERIOUS_FAULT_MASK) {
                /* Falha séria: vai direto para SAFE_STOP */
                s->prev_state       = s->state;
                s->state            = SAFETY_SAFE_STOP;
                s->state_entry_tick = HAL_GetTick();
                FaultLogger_Log(FAULT_SAFE_STOP_ENTERED, fault_flags);
            } else if (fault_flags != 0U) {
                /* Falha não-crítica: avança para WARNING */
                s->prev_state       = s->state;
                s->state            = SAFETY_WARNING;
                s->state_entry_tick = HAL_GetTick();
            }
            break;

        case SAFETY_WARNING:
            if (fault_flags == 0U) {
                /* Todas as falhas resolvidas: volta ao operacional */
                s->prev_state       = s->state;
                s->state            = SAFETY_OPERATIONAL;
                s->state_entry_tick = HAL_GetTick();
            } else if (fault_flags & SAFETY_SERIOUS_FAULT_MASK) {
                /* Escalou para falha séria */
                s->prev_state       = s->state;
                s->state            = SAFETY_SAFE_STOP;
                s->state_entry_tick = HAL_GetTick();
                FaultLogger_Log(FAULT_SAFE_STOP_ENTERED, fault_flags);
            }
            break;

        case SAFETY_SAFE_STOP:
            /* Permanece em SAFE_STOP pelo mínimo exigido, depois avalia */
            if ((HAL_GetTick() - s->state_entry_tick) >= SAFETY_SAFE_STOP_HOLD_MS) {
                if (fault_flags == 0U) {
                    /* Falhas limpas após hold: retorna ao operacional */
                    s->prev_state       = s->state;
                    s->state            = SAFETY_OPERATIONAL;
                    s->state_entry_tick = HAL_GetTick();
                }
                /* Se ainda há falhas: permanece em SAFE_STOP */
            }
            break;

        case SAFETY_EMERGENCY_STOP:
            /* E-Stop liberado: transita para SAFE_STOP (não volta direto a OPERATIONAL) */
            if (!estop_active) {
                s->prev_state       = s->state;
                s->state            = SAFETY_SAFE_STOP;
                s->state_entry_tick = HAL_GetTick();
            }
            break;

        default:
            break;
    }
}

/* ── Consultas ───────────────────────────────────────────────────────── */

bool Safety_IsSafeToOperate(const Safety_t *s)
{
    return (s->state == SAFETY_OPERATIONAL || s->state == SAFETY_WARNING);
}

void Safety_TriggerEmergency(Safety_t *s, uint32_t cause)
{
    if (s->state == SAFETY_SAFE_STATE) {
        return;  /* Já em estado final */
    }
    s->prev_state       = s->state;
    s->state            = SAFETY_EMERGENCY_STOP;
    s->state_entry_tick = HAL_GetTick();
    s->fault_flags     |= cause;
    FaultLogger_Log(FAULT_ESTOP_ACTIVE, cause);
}

const char *Safety_GetStateName(const Safety_t *s)
{
    switch (s->state) {
        case SAFETY_POWER_ON:        return "POWER_ON";
        case SAFETY_SELF_TEST:       return "SELF_TEST";
        case SAFETY_OPERATIONAL:     return "OPERATIONAL";
        case SAFETY_WARNING:         return "WARNING";
        case SAFETY_SAFE_STOP:       return "SAFE_STOP";
        case SAFETY_EMERGENCY_STOP:  return "EMERGENCY_STOP";
        case SAFETY_SAFE_STATE:      return "SAFE_STATE";
        default:                     return "UNKNOWN";
    }
}

uint32_t Safety_GetFaultFlags(const Safety_t *s)
{
    return s->fault_flags;
}

Safety_State_t Safety_GetState(const Safety_t *s)
{
    return s->state;
}
