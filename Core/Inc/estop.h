/**
 * @file    estop.h
 * @brief   Parada de Emergência (E-Stop), Safe Torque Off (STO) e freio mecânico
 *
 * Pinagem (STM32F103C8 Black Pill):
 *
 *   PB0 ← E-Stop  (NC — Normalmente Fechado, pull-up interno)
 *          Lógica: chave fechada  → PB0 = LOW  (normal)
 *                  chave aberta   → PB0 = HIGH (emergência ou fio cortado)
 *          Fail-safe: fio cortado = emergência detectada ✔
 *
 *   PB1 → Freio mecânico  (HIGH = freio ENGAJADO, LOW = freio livre)
 *          Deve ser freio de mola: ao perder sinal → freio engaja ✔
 *
 *   PB5 → STO — Safe Torque Off  (LOW = driver DESABILITADO sem torque)
 *          Pino de segurança dedicado no driver (separado do EN)
 *          Se o driver não possui STO: conectar ao EN (PA2)
 *
 *   PC13→ LED de alarme onboard (HIGH = LED aceso)
 *         NOTA: EStop_Init() reconfigura PC13 como saída OUTPUT_PP,
 *               sobrescrevendo eventual configuração anterior como input.
 *
 * @author  rrmatos
 */

#ifndef ESTOP_H
#define ESTOP_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ── Pinos de segurança ──────────────────────────────────────────────── */

#define ESTOP_PIN          GPIO_PIN_0    /**< PB0 ← E-Stop (NC, pull-up)      */
#define ESTOP_PORT         GPIOB

#define BRAKE_PIN          GPIO_PIN_1    /**< PB1 → Freio (HIGH = engajado)   */
#define BRAKE_PORT         GPIOB

#define STO_PIN            GPIO_PIN_5    /**< PB5 → STO (LOW = ativo)         */
#define STO_PORT           GPIOB

#define ALARM_LED_PIN      GPIO_PIN_13   /**< PC13 → LED alarme onboard       */
#define ALARM_LED_PORT     GPIOC

/* ── Nível lógico que indica E-Stop ativo ────────────────────────────── */
/** Pino PB0 = HIGH → emergência (NC aberta ou fio cortado) */
#define ESTOP_ACTIVE_LEVEL  GPIO_PIN_SET

/* ── Protótipos ──────────────────────────────────────────────────────── */

/**
 * @brief Inicializa os pinos de segurança.
 *        DEVE ser chamada o mais cedo possível na sequência de init.
 *        Estado seguro imediato: freio ENGAJADO, STO ATIVO, alarme DESLIGADO.
 */
void EStop_Init(void);

/**
 * @brief Verifica se o E-Stop está ativado (leitura direta do pino).
 * @return true se emergência ativa
 */
bool EStop_IsActive(void);

/** @brief Engaja o freio mecânico (PB1 = HIGH). */
void EStop_BrakeEngage(void);

/**
 * @brief Libera o freio mecânico (PB1 = LOW).
 *        SOMENTE chamar quando o sistema estiver em estado OPERATIONAL.
 */
void EStop_BrakeRelease(void);

/** @brief Ativa o STO — remove torque do driver (PB5 = LOW). */
void EStop_STO_Activate(void);

/**
 * @brief Desativa o STO — permite torque no motor (PB5 = HIGH).
 *        SOMENTE chamar quando o sistema estiver em estado OPERATIONAL.
 */
void EStop_STO_Deactivate(void);

/** @brief Liga o LED de alarme (PC13 = HIGH). */
void EStop_AlarmOn(void);

/** @brief Desliga o LED de alarme (PC13 = LOW). */
void EStop_AlarmOff(void);

/**
 * @brief Executa sequência completa de emergência:
 *        (1) STO ATIVO — remove torque imediatamente
 *        (2) Freio ENGAJADO — trava mecânica
 *        (3) Alarme LIGADO — sinalização visual
 *
 *        Seguro para chamar repetidamente (idempotente).
 */
void EStop_TriggerEmergencySequence(void);

#endif /* ESTOP_H */
