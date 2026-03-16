# Documentação de Segurança Funcional
## Controlador de Posição NEMA 17 — STM32F103C8 Black Pill

**Nível de integridade alvo:** PLd / SIL 2  
**Normas de referência:** ISO 13849-1, IEC 62061  
**Aplicação:** Robô colaborativo / Máquina industrial com interação humana  
**Data:** 2026-03-14  
**Autor:** rrmatos

---

> ⚠️ **AVISO LEGAL**  
> Este documento e o firmware associado constituem um **protótipo de referência**.  
> O uso em produção com seres humanos **exige auditoria formal** por engenheiro  
> certificado (TÜV/BRTÜV) e certificado PLd/SIL2 por organismo notificado.

---

## 1. Arquitetura de Segurança

### 1.1 Camadas de Defesa (Defense in Depth)

```
┌─────────────────────────────────────────────────────────────────┐
│   CAMADA 1 — Hardware independente do software                  │
│     • Botão E-Stop NC (Normalmente Fechado) → falha-seguro       │
│     • Freio mecânico de mola (engaja ao perder sinal/energia)   │
│     • STO — Safe Torque Off no driver (pino dedicado)           │
│     • Fusível de sobrecorrente no barramento do motor           │
├─────────────────────────────────────────────────────────────────┤
│   CAMADA 2 — Firmware STM32 (este projeto)                      │
│     • IWDG — watchdog independente (~40 kHz LSI, 500 ms)        │
│     • safety.c — máquina de estados: 7 estados, transições      │
│     • estop.c  — leitura NC, STO, freio, alarme LED              │
│     • limits.c — soft limits, vel. máx., stall, timeout         │
│     • fault_logger.c — log circular CRC16 em RAM                 │
├─────────────────────────────────────────────────────────────────┤
│   CAMADA 3 — Comportamento Fail-Safe                             │
│     • Qualquer falha → motor parado, freio engajado, STO ativo  │
│     • Falha crítica (stall, self-test) → SAFE_STATE irreversível │
│     • Reset manual físico necessário para retomar operação       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Descrição dos Módulos de Segurança

### 2.1 `estop.c` — E-Stop e STO

| Pino  | Direção | Função                        | Lógica Fail-Safe              |
|-------|---------|-------------------------------|-------------------------------|
| PB0   | Entrada | Botão E-Stop (NC, pull-up)    | Fio cortado = HIGH = emergência |
| PB1   | Saída   | Freio mecânico (mola)         | HIGH = engajado (padrão)      |
| PB5   | Saída   | STO — Safe Torque Off         | LOW  = sem torque (padrão)    |
| PC13  | Saída   | LED de alarme onboard         | HIGH = alarme ativo           |

**Sequência de emergência:**
1. STO ativo (remove torque do motor — mais rápido que freio)
2. Freio engajado (trava mecânica)
3. Alarme visual ligado

### 2.2 `watchdog.c` — IWDG

```
LSI ≈ 40 000 Hz
Prescaler = 32  → f_tick = 1250 Hz (0,8 ms/tick)
Reload    = 624 → timeout = 625 / 1250 = 500 ms

Comportamento:
  - Firmware normal:    SafeWatchdog_Kick() a cada 1 ms → sem reset
  - Firmware travado:   sem kick → reset após 500 ms
  - Após reset:         EStop_Init() coloca hardware em estado seguro
```

### 2.3 `safety.c` — Máquina de Estados

```
POWER_ON ──► SELF_TEST ──► OPERATIONAL ◄──► WARNING
                                │                │
                                │ (falha séria)  │ (falha séria)
                                ▼                ▼
                           SAFE_STOP ◄───────────┘
                    (aguarda 500 ms + falhas limpas)
                                │
                          ◄─────┘ recupera para OPERATIONAL
                                │
                  E-Stop / falha crítica (stall, self-test)
                                ▼
                       EMERGENCY_STOP
                                │
                     E-Stop liberado ou falha crítica
                                ▼
                          SAFE_STATE  ← IRREVERSÍVEL
                        (reset manual necessário)
```

### 2.4 `limits.c` — Verificações de Limites

| Verificação       | Parâmetro                      | Ação ao Exceder          |
|-------------------|--------------------------------|--------------------------|
| Encoder válido    | ADC: 5%–95% (205–3890)         | SAFETY_FAULT_ENCODER_RANGE → SAFE_STOP |
| Soft limit pos.   | ±28800 steps (±9 voltas = 90%) | SAFETY_FAULT_POS_LIMIT   → SAFE_STOP |
| Velocidade máx.   | 4800 steps/s (1,5 rev/s)       | SAFETY_FAULT_VEL_LIMIT   → SAFE_STOP |
| Stall detection   | 5× sem movimento em 100 ms     | SAFETY_FAULT_STALL       → SAFE_STATE |
| Timeout movimento | 10 segundos                    | SAFETY_FAULT_TIMEOUT     → SAFE_STOP |

### 2.5 `fault_logger.c` — Log de Falhas

- Buffer circular de 16 entradas em RAM  
- Cada entrada: `Fault_Code_t`, `timestamp` (ms), `context` (valor numérico)  
- Integridade verificada por CRC16-CCITT sobre o array completo  
- Função `FaultLogger_PrintAll()` envia o log via UART para diagnóstico

---

## 3. Sequência de Inicialização

```
main()
  │
  ├─ HAL_Init() + SystemClock_Config()
  │
  ├─ FaultLogger_Init()          ← RAM
  ├─ EStop_Init()                ← FREIO ENGAJADO, STO ATIVO ← CRÍTICO
  ├─ Limits_Init()
  ├─ Safety_Init()
  │
  ├─ MX_GPIO_Init() + MX_ADC1_Init() + MX_TIM3_Init()
  ├─ MX_TIM4_Init() + MX_USART1_UART_Init()
  │
  ├─ Stepper_Init()    ← sem habilitar motor ainda
  ├─ Encoder_Pot_Init()
  ├─ PID_Init()
  │
  ├─ Safety_RunSelfTest()
  │     ├─ PASS → continua
  │     └─ FAIL → pisca alarme → watchdog reseta → repete
  │
  ├─ EStop_BrakeRelease()        ← apenas após self-test OK
  ├─ EStop_STO_Deactivate()      ← apenas após self-test OK
  ├─ Stepper_Enable()            ← apenas após self-test OK
  │
  ├─ Encoder_Pot_Update() → setpoint = posição_atual
  │
  ├─ SafeWatchdog_Init()         ← ÚLTIMO: watchdog começa aqui
  └─ HAL_TIM_Base_Start_IT(&htim4)
```

---

## 4. Loop de Controle (1 kHz)

```
TIM4 IRQ (1 kHz) → seta control_flag

while(1):
  if control_flag:
    (a) SafeWatchdog_Kick()          ← sempre (evita reset em loop)
    (b) EStop_IsActive() ?           ← prioridade máxima
         SIM → EStop_TriggerEmergencySequence()
               Stepper_Stop() + Stepper_Disable()
    (c) Encoder_Pot_Update()
    (d) Limits_Update()              ← enc range, pos, vel, stall, timeout
    (e) Safety_Update(estop, faults) ← atualiza FSM
    (f) Safety_IsSafeToOperate() ?
         SIM → PID_Compute() → Stepper_SetSpeedSteps()
         NÃO → Stepper_Stop() + STO + Freio + Alarme
    (g) Integra posição estimada
    (h) Telemetria UART @10 Hz      ← inclui estado de segurança
```

---

## 5. FMEA — Análise de Modos de Falha e Efeitos

| ID  | Componente        | Modo de Falha                | Efeito                        | Detecção                   | Mitigação                          |
|-----|-------------------|------------------------------|-------------------------------|----------------------------|------------------------------------|
| F01 | Botão E-Stop      | Fio cortado / aberto         | PB0 = HIGH → emergência ativa | Hardware NC                | EStop_TriggerEmergencySequence()   |
| F02 | Potenciômetro     | Sensor desconectado          | ADC < 5% ou > 95%             | ADC_VALID_MIN/MAX          | SAFETY_FAULT_ENCODER_RANGE → SAFE_STOP |
| F03 | Firmware          | Loop infinito / trava        | Watchdog não alimentado       | IWDG 500 ms                | Reset → EStop_Init() seguro        |
| F04 | Motor NEMA        | Travamento mecânico (stall)  | Motor não se move             | 5× sem delta posição       | SAFETY_FAULT_STALL → SAFE_STATE   |
| F05 | Driver A4988      | Supercorrente                | Fusível abre                  | Hardware externo           | Fusível + corrente de driver       |
| F06 | PID               | Saída divergente             | Velocidade acima de 4800 sps  | Limits_Update()            | SAFETY_FAULT_VEL_LIMIT → SAFE_STOP |
| F07 | Soft limits       | Encoder deriva               | Posição > ±28800 steps        | pos_steps comparação       | SAFETY_FAULT_POS_LIMIT → SAFE_STOP |
| F08 | Freio mecânico    | Perda de alimentação         | Freio engaja por mola         | N/A (fail-safe hardware)   | Design mecânico fail-safe          |
| F09 | RAM / Stack       | Corrupção de memória         | CRC16 do fault log fará match | FaultLogger_CheckIntegrity | SAFETY_FAULT_SELF_TEST → SAFE_STATE |
| F10 | Movimento         | Motor não chega ao alvo      | Movimento > 10 s              | MOVEMENT_TIMEOUT_MS        | SAFETY_FAULT_TIMEOUT → SAFE_STOP  |

---

## 6. Requisitos para Certificação PLd/SIL2

Para uso em produção com interação humana, os seguintes itens adicionais são necessários:

| Requisito                              | Status Atual     | Ação Necessária                                      |
|----------------------------------------|------------------|------------------------------------------------------|
| Auto-teste de CPU (CPU self-test)      | Parcial (RAM)    | Implementar teste de registradores e ALU             |
| Diversidade de hardware (dual-channel) | ❌ Não implementado | Adicionar MCU secundário ou lógica de votação       |
| Tempo de resposta E-Stop < 10 ms       | ~1 ms (polling)  | Usar EXTI interrupt para < 0,1 ms                    |
| Memória não-volátil para log de falhas | ❌ Somente RAM    | Gravar log em Flash (última página do STM32F103)     |
| Diagnóstico de cobertura > 99%         | ~60% estimado    | Auditoria formal com ferramenta FMEDA                |
| Certificado por organismo notificado   | ❌ Não realizado  | TÜV Rheinland / BRTÜV / DNV                          |

---

## 7. Pinagem Completa do Sistema

```
STM32F103C8 Black Pill
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  PA0  ◄──── Potenciômetro multivolta  (ADC1_IN0)          │
│  PA1  ────► DIR  → A4988/DRV8825                           │
│  PA2  ────► EN   → A4988/DRV8825 (LOW = habilitado)        │
│  PA6  ────► STEP → A4988/DRV8825 (TIM3_CH1 PWM)            │
│  PA9  ────► USART1 TX → Monitor/Debug (115200 baud)        │
│  PA10 ◄──── USART1 RX ← Comandos                           │
│                                                             │
│ [SEGURANÇA]                                                 │
│  PB0  ◄──── E-Stop NC (pull-up — fio cortado = emergência) │
│  PB1  ────► Freio mecânico (HIGH = engajado)               │
│  PB5  ────► STO — Safe Torque Off (LOW = sem torque)       │
│  PC13 ────► LED alarme onboard                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. Telemetria UART

A cada ~100 ms (10 Hz), o sistema envia pela USART1 (115200 baud):

```
ADC: 2048 POS: +0000 VOLTAS: +0.00 SP:    0 ERR:   +0.0 PID:   +0.0 DZ:1 SAF:OPERATIONAL
```

Campos:

| Campo    | Descrição                                |
|----------|------------------------------------------|
| `ADC`    | Leitura bruta ADC (0–4095)               |
| `POS`    | Posição em steps (±32000)                |
| `VOLTAS` | Posição em voltas (±10)                  |
| `SP`     | Setpoint atual em steps                  |
| `ERR`    | Erro de posição (setpoint − medido)      |
| `PID`    | Saída do controlador PID (steps/s)       |
| `DZ`     | 1 = dentro da deadzone, 0 = fora         |
| `SAF`    | Estado da máquina de segurança           |

---

*Documento gerado automaticamente. Revisão obrigatória antes de uso em produção.*
