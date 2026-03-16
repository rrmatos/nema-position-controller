/**
 * @file    trajectory.c
 * @brief   Perfil de velocidade trapezoidal para motor de passo
 *
 * Algoritmo:
 *
 *  1. Calcula os steps necessários para acelerar de 0 até v_max:
 *       accel_steps = v_max² / (2 × accel)
 *
 *  2. Se total_steps < 2 × accel_steps → perfil triangular (sem cruzeiro):
 *       v_peak = sqrt(accel × |total_steps|)
 *
 *  3. Fase de desaceleração inicia quando os steps restantes ≤ accel_steps.
 *
 *  A velocidade instantânea é sempre positiva. A direção de rotação é
 *  determinada pelo sinal de total_steps e configurada externamente no
 *  driver do stepper antes de chamar Trajectory_Start().
 */

#include "trajectory.h"
#include <math.h>
#include <stdlib.h>

void Trajectory_Init(Trajectory_t *traj, int32_t steps,
                     uint32_t v_max, uint32_t accel)
{
    traj->total_steps = steps;
    traj->v_max       = (v_max > 0U) ? v_max : 1U;
    traj->accel       = (accel > 0U) ? accel : 1U;
    traj->current_vel = 0.0f;
    traj->steps_done  = 0;
    traj->state       = TRAJ_IDLE;
}

void Trajectory_Start(Trajectory_t *traj)
{
    if (traj->total_steps == 0) {
        traj->state = TRAJ_COMPLETE;
        return;
    }
    traj->current_vel = 0.0f;
    traj->steps_done  = 0;
    traj->state       = TRAJ_ACCEL;
}

float Trajectory_Update(Trajectory_t *traj, float dt)
{
    if (traj->state == TRAJ_IDLE || traj->state == TRAJ_COMPLETE) {
        traj->current_vel = 0.0f;
        return 0.0f;
    }

    int32_t abs_total    = abs(traj->total_steps);
    int32_t abs_done     = abs(traj->steps_done);
    int32_t remaining    = abs_total - abs_done;
    float   accel_f      = (float)traj->accel;
    float   v_max_f      = (float)traj->v_max;

    /* Steps necessários para desacelerar da velocidade atual até parar */
    float decel_steps_needed = (traj->current_vel * traj->current_vel)
                               / (2.0f * accel_f);

    switch (traj->state) {

        case TRAJ_ACCEL:
            traj->current_vel += accel_f * dt;

            if (traj->current_vel >= v_max_f) {
                traj->current_vel = v_max_f;
                traj->state       = TRAJ_CRUISE;
            }
            /* Verifica se já deve desacelerar (perfil triangular) */
            if ((float)remaining <= decel_steps_needed) {
                traj->state = TRAJ_DECEL;
            }
            break;

        case TRAJ_CRUISE:
            /* Inicia desaceleração quando os steps restantes são críticos */
            decel_steps_needed = (v_max_f * v_max_f) / (2.0f * accel_f);
            if ((float)remaining <= decel_steps_needed) {
                traj->state = TRAJ_DECEL;
            }
            break;

        case TRAJ_DECEL:
            traj->current_vel -= accel_f * dt;
            if (traj->current_vel <= 0.0f || remaining <= 0) {
                traj->current_vel = 0.0f;
                traj->state       = TRAJ_COMPLETE;
            }
            break;

        default:
            traj->current_vel = 0.0f;
            break;
    }

    /* Integra steps executados neste ciclo */
    int32_t delta = (int32_t)(traj->current_vel * dt);
    if (traj->total_steps > 0) {
        traj->steps_done += delta;
    } else {
        traj->steps_done -= delta;
    }

    return traj->current_vel;
}

bool Trajectory_IsComplete(const Trajectory_t *traj)
{
    return (traj->state == TRAJ_COMPLETE);
}

void Trajectory_Abort(Trajectory_t *traj)
{
    traj->state       = TRAJ_IDLE;
    traj->current_vel = 0.0f;
}
