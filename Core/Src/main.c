/**
 * @file    main.c
 * @brief   Loop principal – Controlador de Posição NEMA 17 com STM32F411 Nucleo
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
 *   TIM2  → PWM STEP (PA5, canal 1) — prescaler 83, ARR variável
 *   TIM3  → Loop de controle 1 kHz  — prescaler 8399, ARR 9
 *   ADC1  → Potenciômetro (PA0, canal 0)
 *   USART2→ Telemetria debug 115200 baud (PA9=TX, PA10=RX)
 *
 * Pinos:
 *   PA0  → Potenciômetro (ADC1_IN0)
 *   PA5  → STEP (TIM2_CH1 PWM)
 *   PA6  → DIR  (GPIO saída)
 *   PA7  → EN   (GPIO saída, LOW = habilitado)
 *   PA9  → UART TX
 *   PA10 → UART RX
 *   PC13 → Botão USER (pull-up interno)
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

ADC_HandleTypeDef  hadc1;   /**< ADC1 — potenciômetro (PA0) */
TIM_HandleTypeDef  htim2;   /**< TIM2 — PWM STEP (PA5) */
TIM_HandleTypeDef  htim3;   /**< TIM3 — loop de controle 1 kHz */
UART_HandleTypeDef huart2;  /**< USART2 — debug/telemetria */

/* ── Instâncias dos módulos de controle ──────────────────────────────── */

static Encoder_Pot_t encoder;    /**< Encoder por potenciômetro */
static PID_t         pid;        /**< Controlador PID com deadzone */
static Stepper_t     stepper;    /**< Driver do motor de passo */

/* ── Variáveis de controle ───────────────────────────────────────────── */

static volatile uint8_t control_flag = 0;  /**< Setada pelo callback TIM3 a 1 kHz */
static float            setpoint     = 0.0f; /**< Posição desejada em steps */
static uint32_t         telem_count  = 0;    /**< Contador para telemetria */

/* ── Protótipos de funções locais ────────────────────────────────────── */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
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

    /* ── 2. Inicialização dos periféricos ────────────────────────────── */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    /* ── 3. Inicialização dos módulos de controle ────────────────────── */

    /* Motor: TIM2 canal 1, DIR=PA6, EN=PA7 */
    Stepper_Init(&stepper, &htim2, TIM_CHANNEL_1,
                 GPIOA, GPIO_PIN_6,
                 GPIOA, GPIO_PIN_7);
    Stepper_Enable(&stepper);

    /* Encoder: ADC1, canal 0 (PA0) */
    Encoder_Pot_Init(&encoder, &hadc1, ADC_CHANNEL_0);

    /* PID: KP, KI, KD, Ts, saída_min, saída_max */
    PID_Init(&pid, PID_KP, PID_KI, PID_KD,
             PID_SAMPLE_TIME, -PID_OUT_MAX, PID_OUT_MAX);
    PID_SetDeadzone(&pid, PID_DEADZONE);

    /* ── 4. Lê posição inicial e usa como setpoint (não move ao ligar) ── */
    Encoder_Pot_Update(&encoder);
    setpoint = (float)Encoder_Pot_GetSteps(&encoder);

    /* ── 5. Inicia timer de controle (TIM3 → 1 kHz) ─────────────────── */
    HAL_TIM_Base_Start_IT(&htim3);

    /* ── 6. Loop principal ───────────────────────────────────────────── */
    while (1) {
        /* Aguarda a flag setada pelo callback do TIM3 */
        if (control_flag) {
            control_flag = 0;

            /* a) Lê encoder */
            Encoder_Pot_Update(&encoder);
            float pos_atual = (float)Encoder_Pot_GetSteps(&encoder);

            /* b) Calcula saída PID */
            float pid_output = PID_Compute(&pid, setpoint, pos_atual);

            /* c) Aplica ao motor */
            if (PID_IsInDeadzone(&pid)) {
                /* Dentro da deadzone: para o motor */
                Stepper_Stop(&stepper);
            } else {
                /* Define direção conforme sinal da saída PID */
                if (pid_output > 0.0f) {
                    Stepper_SetDirection(&stepper, STEPPER_DIR_CW);
                } else {
                    Stepper_SetDirection(&stepper, STEPPER_DIR_CCW);
                }

                /* Define velocidade proporcional ao módulo da saída PID */
                uint32_t speed = (uint32_t)fabsf(pid_output);
                Stepper_SetSpeedSteps(&stepper, speed);
            }

            /* d) Atualiza posição (integra steps a partir da velocidade) */
            if (stepper.is_moving) {
                if (stepper.direction == STEPPER_DIR_CW) {
                    stepper.current_pos += (int32_t)(stepper.current_speed
                                                     * PID_SAMPLE_TIME);
                } else {
                    stepper.current_pos -= (int32_t)(stepper.current_speed
                                                     * PID_SAMPLE_TIME);
                }
            }

            /* e) Telemetria a ~10 Hz */
            telem_count++;
            if (telem_count >= TELEM_DIVIDER) {
                telem_count = 0;
                Send_Telemetry(pid_output);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Callback do Timer TIM3 — seta a flag de controle a 1 kHz
 * ═══════════════════════════════════════════════════════════════════════ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
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
                   "ERR:%+7.1f PID:%+7.1f DZ:%d\r\n",
                   (unsigned int)encoder.raw,
                   (long)encoder.position_steps,
                   (double)encoder.angle_turns,
                   (double)setpoint,
                   (double)erro,
                   (double)pid_output,
                   (int)PID_IsInDeadzone(&pid));

    if (len > 0 && len < (int)sizeof(buf)) {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 10);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Configuração do clock do sistema — 84 MHz (HSI PLL)
 * Gerado pelo STM32CubeMX; ajuste conforme o seu .ioc
 * ═══════════════════════════════════════════════════════════════════════ */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Habilita HSI e configura PLL para 84 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 84;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_GPIO_Init — configura pinos DIR (PA6) e EN (PA7)
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PA6 = DIR, PA7 = EN — saídas digitais */
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC13 = botão USER (pull-up interno, sem interrupção) */
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Níveis iniciais: DIR=LOW (CW), EN=HIGH (desabilitado) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_ADC1_Init — ADC1 canal 0 (PA0), resolução 10-bit, polling
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_10B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Configura PA0 como entrada analógica */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Canal 0 com tempo de amostragem de 3 ciclos */
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_TIM2_Init — TIM2 PWM para sinal STEP (PA5, canal 1)
 *   Clock TIM2 = 84 MHz,  Prescaler = 83 → tick = 1 µs
 *   ARR inicial = 999 → 1 kHz (ajustado em tempo real pelo stepper)
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef   sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef  sMasterConfig      = {0};
    TIM_OC_InitTypeDef       sConfigOC          = {0};
    GPIO_InitTypeDef         GPIO_InitStruct     = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 → TIM2_CH1 (AF1) */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Prescaler 83 → tick = 1 µs (84 MHz / 84 = 1 MHz) */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;          /* ARR inicial */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Canal 1: PWM modo 1, duty 50% */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 499;   /* duty 50% para ARR=999 */
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_TIM3_Init — TIM3 interrupção a 1 kHz (loop de controle)
 *   Clock TIM3 = 84 MHz,  Prescaler = 8399, ARR = 9
 *   f = 84.000.000 / (8400 × 10) = 1000 Hz
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 8399;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 9;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Habilita interrupção TIM3 no NVIC */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════════
 * MX_USART2_UART_Init — USART2 115200 baud (PA9=TX, PA10=RX)
 * ═══════════════════════════════════════════════════════════════════════ */

static void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 = TX, PA10 = RX — função alternativa AF7 */
    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Error_Handler — loop infinito em caso de falha de inicialização
 * ═══════════════════════════════════════════════════════════════════════ */

static void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Pisca LED interno (PC13 invertido no Nucleo) para indicar erro */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}

/* ── Stub de main.h necessário para compilação autônoma ─────────────── */
/* (Normalmente gerado pelo CubeMX; incluído aqui apenas como referência) */
