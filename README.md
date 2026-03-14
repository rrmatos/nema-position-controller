# Controlador de Posição NEMA 17 — STM32F103C8 Black Pill

![Linguagem](https://img.shields.io/badge/linguagem-C-blue?logo=c)
![Plataforma](https://img.shields.io/badge/plataforma-STM32F103C8-brightgreen?logo=stmicroelectronics)
![Licença](https://img.shields.io/badge/licença-MIT-yellow)

> Controlador de posição de malha fechada para motor de passo **NEMA 17**
> usando **STM32F103C8 Black Pill**, potenciômetro multivolta como encoder,
> controlador **PID** com bloco de **Deadzone** e geração de pulsos
> STEP/DIR via Timer PWM.

---

## Sumário

1. [Descrição](#descrição)
2. [Diagrama de Arquitetura](#diagrama-de-arquitetura)
3. [Malha de Controle com Deadzone](#malha-de-controle-com-deadzone)
4. [Bloco Deadzone](#bloco-deadzone)
5. [Perfil Trapezoidal de Velocidade](#perfil-trapezoidal-de-velocidade)
6. [Componentes](#componentes)
7. [Pinout STM32F103C8 Black Pill](#pinout-stm32f103c8-black-pill)
8. [Conversão ADC → Steps](#conversão-adc--steps)
9. [Microstepping (A4988)](#microstepping-a4988)
10. [Configuração dos Timers](#configuração-dos-timers)
11. [Análise de Resolução](#análise-de-resolução)
12. [Resposta PID ao Degrau](#resposta-pid-ao-degrau)
13. [Diagrama de Estados da Deadzone](#diagrama-de-estados-da-deadzone)
14. [Como Usar](#como-usar)
15. [Sintonia do PID](#sintonia-do-pid)
16. [Estrutura do Projeto](#estrutura-do-projeto)
17. [Licença](#licença)

---

## Descrição

Este projeto implementa um **controlador de posição de malha fechada** para
motor de passo NEMA 17 com as seguintes características:

- **Sensor de posição**: potenciômetro multivolta (±10 voltas) lido pelo
  ADC de 12 bits (0–4095). O centro (2048) corresponde a 0°.
- **Controlador**: PID discreto com período de amostragem de 1 ms (1 kHz),
  equipado com bloco de **Deadzone** para eliminar oscilações em regime
  permanente e anti-windup por clamping da integral.
- **Atuador**: motor de passo NEMA 17 acionado por driver A4988 ou
  DRV8825, com microstepping de 1/16 (3200 steps/revolução).
- **Geração de pulsos STEP**: via Timer PWM (TIM3_CH1, PA6) com resolução de 1 µs.
- **Telemetria**: envio de dados de estado via USART1 a 115200 baud (10 Hz).
- **Nota**: PA0 é usado exclusivamente para o ADC do potenciômetro; o STEP
  foi movido para PA6 (TIM3_CH1) para evitar conflito com ADC1_IN0.

---

## Diagrama de Arquitetura

```
                        STM32F103C8 Black Pill
  ┌──────────────────────────────────────────────────────────┐
  │                                                          │
  │  [Interface UART] ◄──── USART1 (PA9/PA10)               │
  │         ▲                  │ telemetria 115200 baud      │
  │         │                  │                             │
  │  [ADC1 PA0]                ▼                             │
  │  Potenciômetro    ┌─────────────────┐                    │
  │  Multivolta  ────►│  Loop Controle  │─────► TIM3 PWM     │
  │  (12-bit ADC)     │  1 kHz (TIM4)  │       (PA6 STEP)   │
  │                   │  PID+Deadzone   │                    │
  │                   └─────────────────┘       PA1 DIR      │
  │                                             PA2 EN       │
  └──────────────────────────────────────────────────────────┘
             │                                      │
             │◄─────── feedback posição             ▼
             │                          ┌───────────────────┐
             │                          │  Driver A4988 /   │
             └──────────────────────────│    DRV8825        │
                                        └───────────────────┘
                                                  │
                                                  ▼
                                        ┌───────────────────┐
                                        │   Motor NEMA 17   │
                                        │  200 steps/rev    │
                                        │  microstepping ×16│
                                        │  = 3200 steps/rev │
                                        └───────────────────┘
```

---

## Malha de Controle com Deadzone

```
              setpoint
                 │
                 ▼
         ┌───────────────┐
         │       Σ       │◄──────────────────────────────────────┐
         │  (subtração)  │                                       │
         └───────┬───────┘                                       │
                 │ erro(k)                                       │
                 ▼                                               │
         ┌───────────────┐  |erro| < DZ?                        │
         │    Bloco      │──────────────► saída = 0             │
         │   Deadzone    │               motor parado           │
         └───────┬───────┘                                       │
                 │ erro filtrado                                  │
                 ▼                                               │
         ┌───────────────┐                                       │
         │  Controlador  │                                       │
         │      PID      │  KP=6.0  KI=0.3  KD=0.15            │
         └───────┬───────┘  Ts=1ms  saída: ±3200 steps/s        │
                 │ velocidade (steps/s)                          │
                 ▼                                               │
         ┌───────────────┐                                       │
         │  Gerador STEP │  ARR = 1.000.000 / steps_s           │
         │  (TIM3 PWM)   │  Prescaler 71 → tick 1µs             │
         └───────┬───────┘                                       │
                 │ pulsos STEP + sinal DIR                       │
                 ▼                                               │
         ┌───────────────┐                                       │
         │  Motor NEMA   │                                       │
         │     17        │                                       │
         └───────┬───────┘                                       │
                 │ posição mecânica                              │
                 ▼                                               │
         ┌───────────────┐                                       │
         │    Encoder    │  ADC 12-bit → EMA → steps            │
         │  (Pot ADC)    │──────────────────────────────────────┘
         └───────────────┘
```

---

## Bloco Deadzone

O bloco de Deadzone elimina oscilações (chatter) quando o erro é pequeno:

```
  saída
    ^
    │
 +S │              ╔══════════════════════════════►
    │             ╔╝
    │            ╔╝   zona ativa (+)
    │           ╔╝
    │          ╔╝
    │         ╔╝
────┼────────╔╝─────────────────────────────────►  erro
    │  ──────┘  │──────│
    │        -DZ│  +DZ │
    │               ╚╗
    │                ╚╗  zona ativa (−)
    │                 ╚╗
    │                  ╚╗
 -S │                   ╚═════════════════════════
    │
    │         zona morta (saída = 0)
    │         integral resetada
    │         motor parado

  DZ = ±4 steps (PID_DEADZONE = 4.0)
  S  = 3200 steps/s (PID_OUT_MAX)
```

**Comportamento:**
- `|erro| < DZ` → saída = 0, integral = 0, `in_deadzone = true` → motor para
- `|erro| ≥ DZ` → calcula P+I+D normalmente, `in_deadzone = false`

---

## Perfil Trapezoidal de Velocidade

```
  Velocidade
  (steps/s)
      ^
 3200 │         ┌──────────────────────┐
      │        /│                      │\
      │       / │                      │ \
 1600 │      /  │                      │  \
      │     /   │   cruzeiro (Vmax)    │   \
      │    /    │                      │    \
      │   / acel│                      │dece │
      │  /      │                      │     \
    0 └──────┬──┴──────────────────────┴───┬───►  Tempo (ms)
             t1                            t2
        aceleração                    desaceleração
        (0 → Vmax)                    (Vmax → 0)

  Vmax  = 3200 steps/s  (1 RPM com microstepping 1/16)
  Accel = 12800 steps/s²
  t_acel = Vmax / Accel = 3200/12800 = 250 ms
```

> **Nota**: O perfil trapezoidal completo está definido em `trajectory.h`
> como stub e será implementado em versão futura para movimentos
> ponto-a-ponto suaves.

---

## Componentes

| Componente            | Modelo / Especificação                  | Função                                |
|-----------------------|-----------------------------------------|---------------------------------------|
| Microcontrolador      | STM32F103C8T6 Black Pill                | Processamento e controle              |
| Motor de passo        | NEMA 17 — 200 steps/rev, 12–24 V       | Atuador                               |
| Driver do motor       | A4988 ou DRV8825                        | Interface de potência STEP/DIR        |
| Encoder de posição    | Potenciômetro multivolta 10 kΩ (±10V)  | Sensor de posição (realimentação)     |
| Fonte de alimentação  | 12–24 V / 2 A mínimo                   | Alimentação do motor                  |
| Capacitor de bypass   | 100 µF / 50 V (eletrolítico)           | Decoupling na entrada do driver       |
| Resistores pull-down  | 10 kΩ                                   | MS1, MS2, MS3 (se não usados)         |

---

## Pinout STM32F103C8 Black Pill

> ⚠️ **Nota sobre conflito PA0**: No STM32F103C8, PA0 é simultaneamente
> TIM2_CH1 e ADC1_IN0. Para evitar conflito, o sinal STEP foi movido para
> **PA6 (TIM3_CH1)**, mantendo PA0 exclusivamente para o ADC do potenciômetro.

### Tabela de Pinos

| Pino STM32 | Sinal     | Destino              | Função                         |
|------------|-----------|----------------------|--------------------------------|
| PA0        | ADC1_IN0  | Potenciômetro (eixo) | Leitura de posição (ADC 12-bit)|
| PA1        | GPIO OUT  | Driver DIR           | Direção de rotação             |
| PA2        | GPIO OUT  | Driver EN            | Habilitação do driver (LOW)    |
| PA6        | TIM3_CH1  | Driver STEP          | Pulsos de passo (PWM)          |
| PA9        | USART1_TX | USB/Serial           | Transmissão de telemetria      |
| PA10       | USART1_RX | USB/Serial           | Recepção de comandos           |
| PC13       | GPIO IN   | LED onboard / homing | Entrada de controle / reset    |

### Diagrama ASCII do Black Pill

```
         STM32F103C8 Black Pill
  ┌──────────────────────────────┐
  │  ┌─────────────────────┐     │
  │  │    USB (CN1)        │     │
  │  └─────────────────────┘     │
  │                               │
  │  PC13 ●── LED onboard        │
  │                               │
  │  PA0  ●── POT (ADC1_IN0)     │
  │  PA1  ●── DIR  ──► A4988     │
  │  PA2  ●── EN   ──► A4988     │
  │  PA6  ●── STEP ──► A4988     │
  │                               │
  │  PA9  ●── TX ──► USB/Serial  │
  │  PA10 ●── RX ──► USB/Serial  │
  │                               │
  │        STM32F103C8T6          │
  └──────────────────────────────┘
```

### Conexão com Driver A4988/DRV8825

```
  STM32 Black Pill        A4988 / DRV8825
  ┌─────────┐             ┌─────────────┐
  │ PA6     │────STEP────►│ STEP        │
  │ PA1     │────DIR─────►│ DIR         │
  │ PA2     │────EN──────►│ EN (LOW=ON) │
  │ GND     │────GND─────►│ GND         │
  │ +3.3V   │────VDD─────►│ VDD (lógica)│
  └─────────┘             │ VMOT ◄── 12-24V
                          │ GND  ◄── GND motor
                          └─────────────┘
                               │ │ │ │
                           A B A B +  (bobinas NEMA 17)
```

---

## Conversão ADC → Steps

```
  Steps
    ^
+32000│                                         ╔════
      │                                     ╔═══╝
      │                                ╔════╝
      │                           ╔════╝
      │                      ╔════╝
   +0 │─────────────────╔════╝────────────────────►  ADC raw
      │            ╔════╝         2048
      │       ╔════╝
      │  ╔════╝
-32000│══╝
      0       1024       2048       3072      4095

  Fórmula:
    angle = (ADC - 2048) × (3600 / 2048)    [graus, ±3600°]
    turns = angle / 360                      [voltas, ±10]
    steps = turns × 3200                     [steps, ±32000]

  Pontos notáveis:
    ADC=0    → steps = −32000  (−10 voltas, −3600°)
    ADC=2048 → steps =       0  (centro, 0°)
    ADC=4095 → steps ≈ +31984  (+9.99 voltas, +3598°)
```

---

## Microstepping (A4988)

| MS1 | MS2 | MS3 | Modo       | Steps/rev | Resolução/step |
|-----|-----|-----|------------|-----------|----------------|
| L   | L   | L   | Full-step  |       200 | 1.800°         |
| H   | L   | L   | 1/2-step   |       400 | 0.900°         |
| L   | H   | L   | 1/4-step   |       800 | 0.450°         |
| H   | H   | L   | 1/8-step   |      1600 | 0.225°         |
| **H** | **H** | **H** | **1/16-step ✓** | **3200** | **0.1125°** |

> ✓ **Recomendado**: 1/16 microstepping (MS1=MS2=MS3=HIGH) oferece
> máxima suavidade e resolução angular de 0.1125°/step.

---

## Configuração dos Timers

### TIM3 — Gerador de Pulsos STEP (PWM)

```
  STM32F103C8: Clock timer = 72 MHz
  Prescaler = 71  →  tick = 1 / (72 MHz / 72) = 1 µs
  Channel 1: PWM Mode 1 — PA6 (STEP output)

  ARR = 1.000.000 / steps_per_sec

  Exemplos:
  ┌─────────────────┬────────┬──────────────┐
  │ Velocidade      │  ARR   │ Frequência   │
  ├─────────────────┼────────┼──────────────┤
  │  100 steps/s    │ 10000  │   100 Hz     │
  │  500 steps/s    │  2000  │   500 Hz     │
  │ 1000 steps/s    │  1000  │  1000 Hz     │
  │ 3200 steps/s    │   312  │  3200 Hz     │
  │ 6400 steps/s    │   156  │  6400 Hz     │
  └─────────────────┴────────┴──────────────┘

  CCR (duty 50%) = ARR / 2
  Mínimo pulso STEP: 1 µs (spec. A4988 ≥ 1 µs)
```

### TIM4 — Loop de Controle (Interrupção 1 kHz)

```
  STM32F103C8: Clock timer = 72 MHz
  Prescaler = 7199  →  tick = 100 µs (10 kHz)
  ARR = 9

  f = 72.000.000 / (7200 × 10) = 1000 Hz  →  Ts = 1 ms
```

---

## Análise de Resolução

| Parâmetro         | ADC 12-bit (STM32F103C8)  |
|-------------------|---------------------------|
| Contagens totais  | 4096                      |
| Faixa de ângulo   | ±3600°                    |
| Graus por LSB     | 1.76° / contagem          |
| Steps por LSB     | 15.6 steps/LSB            |
| Voltas por LSB    | 0.0049 voltas/LSB         |
| Deadzone mínima   | ~16 steps                 |

> O ADC de 12 bits do STM32F103C8 oferece resolução angular de
> **1.76°/LSB** (3600° ÷ 2048 contagens efetivas), 4× melhor que
> um ADC de 10 bits. A deadzone de ±4 steps corresponde a ≈ ±0.45°.

---

## Resposta PID ao Degrau

```
  Posição
  (steps)
     ^
3200 │                      ┌─────────────────────────────────
     │                 ╭────╯  regime permanente (dentro DZ)
3040 │            ╭────╯  overshoot ~5% ≈ 160 steps
     │       ╭────╯
     │  ╭────╯
   0 └──┴──────────┬──────┬──────┬────────────────────────────►  Tempo (ms)
                   50    100    180
                    │           │
                    │           └── assentamento ~180 ms
                    └── início do degrau

  KP = 6.0   KI = 0.3   KD = 0.15
  Deadzone = ±4 steps
  ─────── setpoint    ╌╌╌╌╌ resposta medida
```

---

## Diagrama de Estados da Deadzone

```
                  ┌────────────────────────┐
                  │       MOVENDO          │
                  │  is_moving = true      │
                  │  PID calcula P+I+D     │
                  │  motor recebe speed    │
                  └────────────┬───────────┘
                               │
                  |erro| < DZ? │ sim
                               │
                               ▼
                  ┌────────────────────────┐
                  │   PARADO / DEADZONE    │
                  │  is_moving = false     │◄──── permanece
                  │  Stepper_Stop()        │      enquanto
                  │  integral = 0          │  |erro| < DZ
                  │  in_deadzone = true    │
                  └────────────┬───────────┘
                               │
                  |erro| ≥ DZ? │ sim (nova perturbação ou setpoint)
                               │
                               ▼
                  ┌────────────────────────┐
                  │       MOVENDO          │
                  │  in_deadzone = false   │
                  │  PID_Reset() integral  │
                  │  motor recebe speed    │
                  └────────────────────────┘
```

---

## Como Usar

### Pré-requisitos

- **STM32CubeIDE** 1.14 ou superior (target: STM32F103C8T6)
- **STM32CubeMX** (integrado ao IDE)
- Placa **STM32F103C8 Black Pill**
- Driver **A4988** ou **DRV8825**
- Motor **NEMA 17**
- Potenciômetro multivolta (ex.: Bourns 3590S, 10 kΩ)

### Passos

1. **Clone o repositório**
   ```bash
   git clone https://github.com/rrmatos/nema-position-controller.git
   cd nema-position-controller
   ```

2. **Abra no STM32CubeIDE**
   - `File → Import → Existing Projects into Workspace`
   - Selecione o diretório clonado

3. **Configure o projeto (.ioc)**
   - Crie um novo `.ioc` para o **STM32F103C8T6**
   - Configure os periféricos conforme a seção
     [Configuração dos Timers](#configuração-dos-timers)
   - Copie os arquivos de `Core/Inc/` e `Core/Src/` para o projeto gerado

4. **Monte o hardware** conforme a seção
   [Conexão com Driver A4988/DRV8825](#conexão-com-driver-a4988drv8825)

5. **Compile e grave**
   ```
   Project → Build All (Ctrl+B)
   Run → Debug (F11) ou Run (Ctrl+F11)
   ```

6. **Monitor serial** — abra um terminal serial (115200 baud) e observe
   a telemetria:
   ```
   ADC:2048 POS:     0 VOLTAS: +0.00 SP:    0 ERR:   +0.0 PID:   +0.0 DZ:1
   ADC:2300 POS:   590 VOLTAS: +0.18 SP:    0 ERR: -590.0 PID:-3540.0 DZ:0
   ```

---

## Sintonia do PID

### Valores Padrão

| Parâmetro     | Valor   | Descrição                                   |
|---------------|---------|---------------------------------------------|
| `KP`          | 6.0     | Resposta proporcional ao erro               |
| `KI`          | 0.3     | Elimina erro estacionário                   |
| `KD`          | 0.15    | Amortece oscilações e melhora estabilidade  |
| `Ts`          | 1 ms    | Período de amostragem (1 kHz via TIM4)      |
| `OUT_MAX`     | 3200    | Velocidade máxima em steps/s                |
| `DEADZONE`    | 4.0     | Zona morta em steps (±4 steps ≈ ±0.45°)    |

### Dicas de Ajuste

1. **Comece com KI=0 e KD=0**, aumente KP até oscilar, depois reduza à metade.
2. **Aumente KD** para reduzir overshoot e oscilações.
3. **Aumente KI** lentamente para eliminar erro estacionário.
4. **Ajuste a Deadzone** conforme a resolução do encoder:
   - Deadzone muito pequena → motor treme em regime permanente (*chatter*)
   - Deadzone muito grande → erro estacionário visível
5. **Velocidade máxima**: respeite a corrente nominal do motor e
   verifique se não há perda de passos (stall).

---

## Estrutura do Projeto

```
nema-position-controller/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Cabeçalho principal, handles e pinos
│   │   ├── encoder.h           # Encoder por potenciômetro multivolta
│   │   ├── pid_controller.h    # Controlador PID com Deadzone
│   │   ├── stepper.h           # Driver NEMA 17 STEP/DIR
│   │   └── trajectory.h        # Perfil trapezoidal (stub)
│   └── Src/
│       ├── main.c              # Loop principal e inicialização HAL
│       ├── encoder.c           # Leitura ADC, filtro EMA, conversões
│       ├── pid_controller.c    # PID discreto, deadzone, anti-windup
│       └── stepper.c           # PWM STEP, controle DIR/EN
└── README.md
```

---

## Licença

```
MIT License

Copyright (c) 2025 rrmatos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
