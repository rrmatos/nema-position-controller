/**
 * @file    safety.h
 * @brief   Máquina de estados de segurança funcional (ISO 13849 / IEC 62061)
 *          Nível de integridade alvo: PLd / SIL 2
 *
 * Diagrama de estados:
 *
 *   POWER_ON ──► SELF_TEST ──► OPERATIONAL ◄──► WARNING
 *                                   │                │
 *                                   │                │ (falha séria)
 *                                   ▼                ▼
 *                              SAFE_STOP  ◄──────────┘
 *                                   │
 *                                   │ (E-Stop / falha crítica)
 *                                   ▼
 *                          EMERGENCY_STOP
 *                                   │
 *                                   ▼
 *                            SAFE_STATE  ← irreversível sem reset manual
 *
 * Responsabilidades:
 *   - Agregar fault_flags dos módulos limits e estop
 *   - Expor Safety_IsSafeToOperate() para o loop de controle
 *   - Registrar transições de estado no FaultLogger
 *
 * @author  rrmatos
 */

#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>

/* ── Estados da máquina de segurança ─────────────────────────────────── */

typedef enum {
    SAFETY_POWER_ON        = 0,  /**< Estado inicial após power-on            */
    SAFETY_SELF_TEST       = 1,  /**< Executando auto-testes iniciais         */
    SAFETY_OPERATIONAL     = 2,  /**< Operação normal — motor pode se mover   */
    SAFETY_WARNING         = 3,  /**< Alerta: falha não-crítica detectada     */
    SAFETY_SAFE_STOP       = 4,  /**< Parada segura: motor parado, recuperável*/
    SAFETY_EMERGENCY_STOP  = 5,  /**< E-Stop ativo: sem torque, freio engajado*/
    SAFETY_SAFE_STATE      = 6   /**< Estado seguro irreversível — reset manual*/
} Safety_State_t;

/* ── Bitmask de falhas do sistema ────────────────────────────────────── */

#define SAFETY_FAULT_ESTOP          (1U << 0)  /**< E-Stop ativado              */
#define SAFETY_FAULT_ENCODER_RANGE  (1U << 1)  /**< Encoder fora do range válido*/
#define SAFETY_FAULT_VEL_LIMIT      (1U << 2)  /**< Velocidade acima do limite  */
#define SAFETY_FAULT_POS_LIMIT      (1U << 3)  /**< Posição fora dos soft limits*/
#define SAFETY_FAULT_STALL          (1U << 4)  /**< Stall do motor detectado    */
#define SAFETY_FAULT_TIMEOUT        (1U << 5)  /**< Timeout de movimento        */
#define SAFETY_FAULT_SELF_TEST      (1U << 6)  /**< Falha no auto-teste         */
#define SAFETY_FAULT_COMM_TIMEOUT   (1U << 7)  /**< Timeout de comunicação      */

/* Falhas críticas → transição direta para SAFE_STATE */
#define SAFETY_CRITICAL_FAULT_MASK  (SAFETY_FAULT_SELF_TEST | SAFETY_FAULT_STALL)

/* Falhas sérias → transição para SAFE_STOP */
#define SAFETY_SERIOUS_FAULT_MASK   (SAFETY_FAULT_POS_LIMIT  | \
                                     SAFETY_FAULT_VEL_LIMIT  | \
                                     SAFETY_FAULT_TIMEOUT    | \
                                     SAFETY_FAULT_ENCODER_RANGE)

/* ── Estrutura principal de segurança ────────────────────────────────── */

typedef struct {
    Safety_State_t state;            /**< Estado atual                         */
    Safety_State_t prev_state;       /**< Estado anterior (para logging)       */
    uint32_t       fault_flags;      /**< Bitmask de falhas ativas             */
    uint32_t       state_entry_tick; /**< HAL_GetTick() ao entrar no estado    */
    uint8_t        self_test_done;   /**< 1 após auto-teste bem-sucedido       */
} Safety_t;

/* ── Tempo mínimo em SAFE_STOP antes de permitir recuperação ─────────── */
#define SAFETY_SAFE_STOP_HOLD_MS    500U

/* ── Protótipos ──────────────────────────────────────────────────────── */

/**
 * @brief Inicializa a estrutura de segurança (estado POWER_ON).
 * @param s  Ponteiro para Safety_t
 */
void Safety_Init(Safety_t *s);

/**
 * @brief Executa auto-testes iniciais (RAM canary + E-Stop plausibility).
 *        Deve ser chamado UMA VEZ após inicialização de hardware.
 * @param s  Ponteiro para Safety_t
 * @return true se todos os testes passaram → estado OPERATIONAL
 *         false se algum teste falhou    → estado SAFE_STATE
 */
bool Safety_RunSelfTest(Safety_t *s);

/**
 * @brief Atualiza a máquina de estados com as condições atuais.
 *        Deve ser chamado no loop de controle (1 kHz).
 * @param s            Ponteiro para Safety_t
 * @param estop_active Verdadeiro se E-Stop está ativado
 * @param fault_flags  Bitmask de falhas (SAFETY_FAULT_*)
 */
void Safety_Update(Safety_t *s, bool estop_active, uint32_t fault_flags);

/**
 * @brief Retorna true se o sistema está em condição operacional segura.
 *        (estado == OPERATIONAL ou WARNING)
 */
bool Safety_IsSafeToOperate(const Safety_t *s);

/**
 * @brief Força transição imediata para EMERGENCY_STOP.
 *        Seguro para chamar de contexto de interrupção.
 * @param s     Ponteiro para Safety_t
 * @param cause Causa (bitmask SAFETY_FAULT_*)
 */
void Safety_TriggerEmergency(Safety_t *s, uint32_t cause);

/** @brief Retorna o estado atual como string (para telemetria/debug). */
const char *Safety_GetStateName(const Safety_t *s);

/** @brief Retorna os fault flags ativos. */
uint32_t Safety_GetFaultFlags(const Safety_t *s);

/** @brief Retorna o estado atual. */
Safety_State_t Safety_GetState(const Safety_t *s);

#endif /* SAFETY_H */
