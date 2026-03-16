/**
 * @file    main.c
 * @brief   Loop principal – Controlador de Posição NEMA 17 com STM32F103C8 Black Pill
 *
 * Arquitetura:
 *   ┌─────────────┐     ┌───────────────┐     ┌──────────────┐
 *   │  Setpoint   │────►│  PID+Deadzone │────►│  Motor NEMA  │
 *   └─────────────┘     └───────────────┘     └──────────────┘
 *                                ▲                    │
 *                         ┌──────┴──────┐             │
 *                         │   Encoder   │◄────────────┘
 *                         │  (Pot ADC)  │
 *                         └─────────────┘
 *
 * Periféricos:
 *   TIM3  → PWM STEP (PA6, canal 1) — Prescaler 71, ARR variável
 *   TIM4  → Loop de controle 1 kHz  — Prescaler 7199, ARR 9
 *   ADC1  → Potenciômetro (PA0, canal 0) — 12-bit
 *   USART1→ Telemetria debug 115200 baud (PA9=TX, PA10=RX)
 *
 * Pinos:
 *   PA0  → Potenciômetro (ADC1_IN0)
 *   PA1  → DIR  (GPIO saída)
 *   PA2  → EN   (GPIO saída, LOW = habilitado)
 *   PA6  → STEP (TIM3_CH1 PWM)
 *   PA9  → USART1 TX
 *   PA10 → USART1 RX
 *   PC13 → LED onboard / botão homing (pull-up interno)
 *
 * Nota: PA0 é usado exclusivamente para ADC (potenciômetro).
 *       O sinal STEP foi movido para PA6 (TIM3_CH1) para evitar
 *       conflito com ADC1_IN0.
 *
 * Constantes PID:
 *   KP = 6.0,  KI = 0.3,  KD = 0.15
 *   Ts = 1 ms,  saída: ±3200 steps/s
 *   Deadzone = 4 steps
 *
 * @author  rrmatos
 */

#include "main.h"
#include "encoder.h"
#include "pid_controller.h"
#include "stepper.h"
#include "safety.h"
#include "estop.h"
#include "limits.h"
#include "watchdog.h"
#include "fault_logger.h"
#include <stdio.h>
#include <string.h>

/* ── Constantes do controlador ───────────────────────────────────────── */

#define PID_KP           6.0f      /**< Ganho proporcional */
#define PID_KI           0.3f      /**< Ganho integral */
#define PID_KD           0.15f     /**< Ganho derivativo */
#define PID_SAMPLE_TIME  0.001f    /**< Período de amostragem (1 ms) */
#define PID_OUT_MAX      3200.0f   /**< Saída máxima (steps/s) */
#define PID_DEADZONE     4.0f      /**< Zona morta em steps */

/* Divisor para telemetria a ~10 Hz (loop a 1 kHz → a cada 100 ciclos) */
#define TELEM_DIVIDER    100U

/* ── Handles de periférico (gerados pelo CubeMX / .ioc) ──────────────── */

ADC_HandleTypeDef  hadc1;   /**< ADC1 — potenciômetro (PA0, 12-bit) */
TIM_HandleTypeDef  htim3;   /**< TIM3 — PWM STEP (PA6, canal 1) */
TIM_HandleTypeDef  htim4;   /**< TIM4 — loop de controle 1 kHz */
UART_HandleTypeDef huart1;  /**< USART1 — debug/telemetria (PA9=TX, PA10=RX) */

/* ── Instâncias dos módulos de controle ──────────────────────────────── */

static Encoder_Pot_t encoder;    /**< Encoder por potenciômetro */
static PID_t         pid;        /**< Controlador PID com deadzone */
static Stepper_t     stepper;    /**< Driver do motor de passo */
static Safety_t      safety;     /**< Máquina de estados de segurança */
static Limits_t      limits;     /**< Verificações de limites de segurança */

/* ── Variáveis de controle ───────────────────────────────────────────── */

static volatile uint8_t control_flag = 0;  /**< Setada pelo callback TIM4 a 1 kHz */
static float            setpoint     = 0.0f; /**< Posição desejada em steps */
static uint32_t         telem_count  = 0;    /**< Contador para telemetria */

/* ── Protótipos de funções locais ────────────────────────────────────── */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void Send_Telemetry(float pid_output);
static void Error_Handler(void);

/* ═══════════════════════════════════════════════════════════════════════
 * main()
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    /* ── 1. Inicialização do HAL e clock ─────────────────────────────── */
    HAL_Init();
    SystemClock_Config();

    /* ── 2. Segurança: DEVE ser inicializada o mais cedo possível ───── */
    /* Coloca hardware em estado seguro (freio engajado, STO ativo)      */
    FaultLogger_Init();
    EStop_Init();          /* ← Freio ENGAJADO, STO ATIVO a partir daqui */
    Limits_Init(&limits);
    Safety_Init(&safety);

    /* ── 3. Inicialização dos periféricos ────────────────────────────── */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* ── 4. Inicialização dos módulos de controle ────────────────────── */

    /* Motor: TIM3 canal 1, DIR=PA1, EN=PA2 (ainda desabilitado) */
    Stepper_Init(&stepper, &htim3, TIM_CHANNEL_1,
                 GPIOA, GPIO_PIN_1,
                 GPIOA, GPIO_PIN_2);
    /* NÃO habilita o motor aqui — espera o auto-teste */

    /* Encoder: ADC1, canal 0 (PA0) */
    Encoder_Pot_Init(&encoder, &hadc1, ADC_CHANNEL_0);

    /* PID: KP, KI, KD, Ts, saída_min, saída_max */
    PID_Init(&pid, PID_KP, PID_KI, PID_KD,
             PID_SAMPLE_TIME, -PID_OUT_MAX, PID_OUT_MAX);
    PID_SetDeadzone(&pid, PID_DEADZONE);

    /* ── 5. Auto-teste de segurança ──────────────────────────────────── */
    if (!Safety_RunSelfTest(&safety)) {
        /* Auto-teste falhou: pisca alarme indefinidamente, watchdog vai resetar */
        EStop_TriggerEmergencySequence();
        while (1) {
            EStop_AlarmOn();
            HAL_Delay(150);
            EStop_AlarmOff();
            HAL_Delay(150);
            /* Não alimenta o watchdog → reset em 500 ms → recomeça do início */
        }
    }

    /* ── 6. Auto-teste OK: libera motor ──────────────────────────────── */
    EStop_BrakeRelease();     /* Libera freio mecânico  */
    EStop_STO_Deactivate();   /* Permite torque no driver */
    Stepper_Enable(&stepper); /* Habilita driver A4988/DRV8825 */

    /* ── 7. Lê posição inicial e usa como setpoint (não move ao ligar) ── */
    Encoder_Pot_Update(&encoder);
    setpoint = (float)Encoder_Pot_GetSteps(&encoder);

    /* ── 8. Inicia watchdog (ÚLTIMO passo — após tudo inicializado) ─── */
    SafeWatchdog_Init();

    /* ── 9. Inicia timer de controle (TIM4 → 1 kHz) ─────────────────── */
    HAL_TIM_Base_Start_IT(&htim4);

    /* ── 10. Loop principal ──────────────────────────────────────────── */
    while (1) {
        /* Aguarda a flag setada pelo callback do TIM4 */
        if (control_flag) {
            control_flag = 0;

            /* a) Alimenta o watchdog (sempre, para evitar reset em loop) */
            SafeWatchdog_Kick();

            /* b) Verifica E-Stop (prioridade máxima) */
            bool estop_active = EStop_IsActive();
            if (estop_active) {
                EStop_TriggerEmergencySequence();
                Stepper_Stop(&stepper);
                Stepper_Disable(&stepper);
            }

            /* c) Lê encoder */
            Encoder_Pot_Update(&encoder);
            float pos_atual = (float)Encoder_Pot_GetSteps(&encoder);

            /* d) Atualiza verificações de limite */
            Limits_Update(&limits,
                          (int32_t)encoder.position_steps,
                          stepper.current_speed,
                          encoder.raw,
                          stepper.is_moving);

            /* e) Atualiza máquina de estados de segurança */
            Safety_Update(&safety, estop_active, Limits_GetFaultFlags(&limits));

            /* f) Controla motor apenas se sistema seguro */
            float pid_output = 0.0f;
            if (Safety_IsSafeToOperate(&safety)) {
                /* f1) Calcula saída PID */
                pid_output = PID_Compute(&pid, setpoint, pos_atual);

                /* f2) Aplica ao motor */
                if (PID_IsInDeadzone(&pid)) {
                    Stepper_Stop(&stepper);
                    Limits_NotifyMovementEnd(&limits);
                } else {
                    if (!stepper.is_moving) {
                        Limits_NotifyMovementStart(&limits,
                                                   (int32_t)encoder.position_steps);
                    }
                    if (pid_output > 0.0f) {
                        Stepper_SetDirection(&stepper, STEPPER_DIR_CW);
                    } else {
                        Stepper_SetDirection(&stepper, STEPPER_DIR_CCW);
                    }
                    uint32_t speed = (uint32_t)fabsf(pid_output);
                    Stepper_SetSpeedSteps(&stepper, speed);
                }
            } else {
                /* Sistema não seguro: para motor e garante estado seguro */
                Stepper_Stop(&stepper);
                Limits_NotifyMovementEnd(&limits);
                if (!estop_active) {
                    /* Se não é E-Stop físico, ainda assim mantém STO ativo */
                    EStop_STO_Activate();
                    EStop_BrakeEngage();
                    EStop_AlarmOn();
                }
            }

            /* g) Atualiza posição estimada (integra steps) */
            if (stepper.is_moving) {
                if (stepper.direction == STEPPER_DIR_CW) {
                    stepper.current_pos += (int32_t)(stepper.current_speed
                                                     * PID_SAMPLE_TIME);
                } else {
                    stepper.current_pos -= (int32_t)(stepper.current_speed
                                                     * PID_SAMPLE_TIME);
                }
            }

            /* h) Telemetria a ~10 Hz */
            telem_count++;
            if (telem_count >= TELEM_DIVIDER) {
                telem_count = 0;
                Send_Telemetry(pid_output);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Callback do Timer TIM4 — seta a flag de controle a 1 kHz
 * ═══════════════════════════════════════════════════════════════════════ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        control_flag = 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Send_Telemetry — envia dados de estado pela UART a ~10 Hz
 * Formato:
 *   ADC:<raw> POS:<steps> VOLTAS:<turns> SP:<setpoint>
 *   ERR:<erro> PID:<saida> DZ:<0|1>\r\n
 * ═══════════════════════════════════════════════════════════════════════ */

static void Send_Telemetry(float pid_output)
{
    char buf[128];
    int len;

    float erro = PID_GetLastError(&pid);

    len = snprintf(buf, sizeof(buf),
                   "ADC:%4u POS:%6ld VOLTAS:%+6.2f SP:%6.0f "
                   "ERR:%+7.1f PID:%+7.1f DZ:%d SAF:%s\r\n",
                   (unsigned int)encoder.raw,
                   (long)encoder.position_steps,
                   (double)encoder.angle_turns,
                   (double)setpoint,
                   (double)erro,
                   (double)pid_output,
                   (int)PID_IsInDeadzone(&pid),
                   Safety_GetStateName(&safety));

    if (len > 0 && len < (int)sizeof(buf)) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Configuração do clock do sistema — 72 MHz (HSE 8 MHz × PLL × 9)
 * STM32F103C8: HSE=8MHz, PLLMUL=9 → SYSCLK=72MHz
 *              APB1=36MHz (÷2), APB2=72MHz (÷1)
 *              Timer clock = APB1 × 2 = 72 MHz
 * ═══════════════════════════════════════════════════════════════════════ */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Habilita HSE e configura PLL para 72 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;  /* 8 × 9 = 72 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 = 36 MHz (max) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* APB2 = 72 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_GPIO_Init — configura pinos DIR (PA1) e EN (PA2)
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PA1 = DIR, PA2 = EN — saídas digitais */
    GPIO_InitStruct.Pin   = GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC13: gerenciado por EStop_Init() como LED de alarme (OUTPUT_PP) */
    /* Não inicializar PC13 aqui para evitar conflito com estop.c        */

    /* Níveis iniciais: DIR=LOW (CW), EN=HIGH (desabilitado) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_ADC1_Init — ADC1 canal 0 (PA0), resolução 12-bit, polling
 *   STM32F103C8: ADC sempre 12-bit, sem campo Resolution/ClockPrescaler
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = ADC_DISCONTINUOUS_DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 0;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Configura PA0 como entrada analógica */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Canal 0 com tempo de amostragem de 7.5 ciclos */
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_TIM3_Init — TIM3 PWM para sinal STEP (PA6, canal 1)
 *   STM32F103C8: Clock TIM3 = 72 MHz, Prescaler = 71 → tick = 1 µs
 *   ARR inicial = 999 → 1 kHz (ajustado em tempo real pelo stepper)
 *   TIM3_CH1 → PA6 (AF_PP, sem remap)
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef   sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef  sMasterConfig      = {0};
    TIM_OC_InitTypeDef       sConfigOC          = {0};
    GPIO_InitTypeDef         GPIO_InitStruct     = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 → TIM3_CH1 (AF_PP, sem campo Alternate no F103) */
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Prescaler 71 → tick = 1 µs (72 MHz / 72 = 1 MHz) */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 71;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;          /* ARR inicial */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Canal 1: PWM modo 1, duty 50% */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 499;   /* duty 50% para ARR=999 */
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_TIM4_Init — TIM4 interrupção a 1 kHz (loop de controle)
 *   STM32F103C8: Clock TIM4 = 72 MHz, Prescaler = 7199, ARR = 9
 *   f = 72.000.000 / (7200 × 10) = 1000 Hz  →  Ts = 1 ms
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 7199;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 9;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Habilita interrupção TIM4 no NVIC */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_USART1_UART_Init — USART1 115200 baud (PA9=TX, PA10=RX)
 *   STM32F103C8: TX → GPIO_MODE_AF_PP, RX → GPIO_MODE_INPUT + PULLUP
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 = TX — saída em modo alternate function push-pull */
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA10 = RX — entrada com pull-up */
    GPIO_InitStruct.Pin  = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Error_Handler — loop infinito em caso de falha de inicialização
 * ═══════════════════════════════════════════════════════════════════════ */

static void Error_Handler(void)
{
    __disable_irq();
    /* Tenta colocar hardware em estado seguro — pode falhar se init não ocorreu */
    /* Reconfigura PC13 como saída para piscar alarme */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin  = GPIO_PIN_1; HAL_GPIO_Init(GPIOB, &g);   /* Freio: engaja */
    g.Pin  = GPIO_PIN_5; HAL_GPIO_Init(GPIOB, &g);   /* STO: ativo   */
    g.Pin  = GPIO_PIN_13; HAL_GPIO_Init(GPIOC, &g);  /* LED alarme   */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    /* Freio HIGH   */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  /* STO LOW      */
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}

/* ── Stub de main.h necessário para compilação autônoma ─────────────── */
/* (Normalmente gerado pelo CubeMX; incluído aqui apenas como referência) */
