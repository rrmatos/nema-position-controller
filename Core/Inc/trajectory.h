/**
 * @file    trajectory.h
 * @brief   Perfil de velocidade trapezoidal para motor de passo (stub)
 *
 * Este módulo implementará geração de perfis de movimento trapezoidais:
 *
 *   Velocidade
 *       ^
 *  Vmax |    ___________
 *       |   /           \
 *       |  /             \
 *       | /  aceleração   \ desaceleração
 *       |/                 \
 *       +----+----+----+----+----> Tempo
 *            t1   t2   t3  t4
 *
 *  Fases:
 *    1. Aceleração  (t0 → t1): velocidade aumenta linearmente de 0 até Vmax
 *    2. Cruzeiro    (t1 → t2): velocidade constante em Vmax
 *    3. Desaceleração (t2 → t3): velocidade diminui linearmente de Vmax até 0
 *
 *  Parâmetros do perfil:
 *    - v_max       : velocidade de cruzeiro em steps/s
 *    - accel       : aceleração em steps/s²
 *    - total_steps : número total de steps para o movimento
 *
 *  Saída:
 *    - v(t)        : velocidade instantânea em cada instante de amostragem
 *    - Período do STEP: T_step = 1 / v(t)
 *    - ARR do Timer: ARR = 1.000.000 / v(t)  (com prescaler 83 → 1 µs/tick)
 *
 *  Nota: A malha de controle PID atualiza o setpoint de velocidade, e o
 *  gerador de perfil suaviza a transição entre velocidades para evitar
 *  perda de passos por variações bruscas.
 *
 * @todo Implementar:
 *       - Trajectory_Init()
 *       - Trajectory_Start()
 *       - Trajectory_Update() → chamado no callback do TIM3 (1 kHz)
 *       - Trajectory_IsComplete()
 *       - Trajectory_Abort()
 *
 * @author  rrmatos
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>

/* ── Estados do perfil trapezoidal ───────────────────────────────────── */

/**
 * @brief Estados do gerador de trajetória
 */
typedef enum {
    TRAJ_IDLE        = 0,  /**< Sem movimento ativo */
    TRAJ_ACCEL       = 1,  /**< Fase de aceleração */
    TRAJ_CRUISE      = 2,  /**< Fase de cruzeiro (velocidade constante) */
    TRAJ_DECEL       = 3,  /**< Fase de desaceleração */
    TRAJ_COMPLETE    = 4   /**< Movimento concluído */
} Traj_State_t;

/* ── Estrutura do perfil de trajetória ──────────────────────────────── */

/**
 * @brief Parâmetros e estado do gerador de perfil trapezoidal
 *
 * @note Stub — campos e funções serão expandidos na próxima versão.
 */
typedef struct {
    int32_t      total_steps;   /**< Total de steps do movimento */
    uint32_t     v_max;         /**< Velocidade de cruzeiro em steps/s */
    uint32_t     accel;         /**< Aceleração em steps/s² */
    float        current_vel;   /**< Velocidade instantânea em steps/s */
    int32_t      steps_done;    /**< Steps executados até o momento */
    Traj_State_t state;         /**< Estado atual do perfil */
} Trajectory_t;

/* ── Protótipos (a implementar) ──────────────────────────────────────── */

/* @todo void Trajectory_Init(Trajectory_t *traj, int32_t steps, uint32_t v_max, uint32_t accel); */
/* @todo void Trajectory_Start(Trajectory_t *traj); */
/* @todo float Trajectory_Update(Trajectory_t *traj, float dt); */
/* @todo bool Trajectory_IsComplete(const Trajectory_t *traj); */
/* @todo void Trajectory_Abort(Trajectory_t *traj); */

#endif /* TRAJECTORY_H */
