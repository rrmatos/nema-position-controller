/**
 * @file    watchdog.h
 * @brief   Configuração do IWDG (Independent Watchdog) para STM32F103C8
 *
 * O IWDG usa o oscilador LSI interno (~40 kHz), independente do clock principal.
 * Timeout configurado para ~500 ms.
 *
 * Cálculo:
 *   LSI ≈ 40 000 Hz
 *   Prescaler = 32  → f_tick = 40000 / 32 = 1250 Hz (0,8 ms/tick)
 *   Reload    = 624 → timeout = 625 / 1250 = 500 ms
 *
 * Comportamento:
 *   - SafeWatchdog_Init() inicia o IWDG — não pode ser parado.
 *   - SafeWatchdog_Kick() deve ser chamado dentro de cada janela de 500 ms.
 *   - Se o firmware travar, o IWDG faz reset completo do sistema.
 *   - Após reset, EStop_Init() recoloca o hardware em estado seguro.
 *
 * IMPORTANTE: Inicializar o watchdog APÓS toda a inicialização de
 *             hardware e auto-testes para evitar resets espúrios.
 *
 * @author  rrmatos
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

/** Timeout nominal do IWDG em milissegundos. */
#define IWDG_TIMEOUT_MS   500U

/**
 * @brief Inicializa e inicia o IWDG com timeout de ~500 ms.
 *        Chamar SOMENTE após toda a inicialização de hardware.
 */
void SafeWatchdog_Init(void);

/**
 * @brief Recarrega o contador do IWDG (kick/feed).
 *        Deve ser chamado com intervalo < IWDG_TIMEOUT_MS.
 *        Chamar mesmo em estado de emergência para evitar reset em loop.
 */
void SafeWatchdog_Kick(void);

#endif /* WATCHDOG_H */
