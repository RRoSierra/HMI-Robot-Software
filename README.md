# Firmware HMI Bare-Metal — Sysmic Robotics SSL

Este repositorio contiene la especificación y guía de implementación para el firmware de prueba y caracterización del robot omnidireccional (categoría Small Size League) de **Sysmic Robotics** (UTFSM).

El objetivo de este firmware es convertir al microcontrolador **STM32F767BI** en un actuador/sensor esclavo (Bare-Metal) que se comunique a alta velocidad con una **HMI en PC (escrita en Rust)**, permitiendo probar motores, sintonizar PIDs y validar modelos cinemáticos sin necesidad de recompilar el código.

---

## Arquitectura General del Sistema

A diferencia del firmware de competencia, esta versión **NO utiliza FreeRTOS**. Se basa en una arquitectura de interrupciones y DMA (Super-Loop) para minimizar la latencia.

* **Lazo de Control (1 kHz):** Se debe utilizar un Timer por hardware (ej. TIM6) para leer encoders
* **Comunicación (UART5 + DMA):** La PC envía y recibe datos constantemente. Para no bloquear la CPU, se exige el uso de **DMA en Recepción** combinado con la interrupción de **Línea Inactiva (IDLE Line Interrupt)**.
* **Hardware Actuador:** Los motores se controlan enviando un valor de voltaje de 12 bits (0-4095) al DAC externo **MAX5815** vía **I2C1**.
* **Hardware Sensor:** Los encoders incrementales se leen directamente desde los registros `CNT` de los Timers configurados en modo Encoder (`TIM3`, `TIM8`, `TIM2`, `TIM5`).

---

## Configuración de Periféricos (Pinout)

| Periférico | Instancia | Pines | Función |
| :--- | :--- | :--- | :--- |
| **UART (PC)** | **UART5** | PD8 (TX), PD9 (RX) | Comunicación con HMI (115200 baudios, 8N1). **Usar DMA en RX.** |
| **I2C (DAC)** | **I2C1** | PB6 (SCL), PB7 (SDA) | Control de voltaje de motores (MAX5815). |
| **Encoder M1** | **TIM3** | PC6, PC7 | Lectura de velocidad Rueda 1. |
| **Encoder M2** | **TIM8** | PI5, PI6 | Lectura de velocidad Rueda 2. |
| **Encoder M3** | **TIM2** | PA0, PA1 | Lectura de velocidad Rueda 3. |
| **Encoder M4** | **TIM5** | PH10, PH11 | Lectura de velocidad Rueda 4. |

>  **Atención:** No usar los pines PC10/PC11 para UART, ya que PC11 está asignado físicamente al `Enable` del Motor 2.

---

##  Protocolo de Comunicación (Binario Estricto)

La comunicación entre el PC (Rust) y el STM32 se realiza mediante tramas binarias de tamaño fijo (**12 bytes**). El formato numérico es **Little Endian** (Byte bajo primero).

### 1. Recepción (PC -> STM32) : Comandos a 50 Hz

La HMI envía comandos de actuación periódicamente.

| Byte | Campo | Tipo | Descripción |
| :--- | :--- | :--- | :--- |
| 0 | `START` | `uint8_t` | Fijo: `0xAA` |
| 1 | `MODE` | `uint8_t` | `0x00` (Manual / DAC) o `0x01` (Cinemático / rad/s) |
| 2-3 | `VAL_M1` | `int16_t` | Setpoint M1 (Rango: -4095 a +4095 ó rad/s * 100) |
| 4-5 | `VAL_M2` | `int16_t` | Setpoint M2 |
| 6-7 | `VAL_M3` | `int16_t` | Setpoint M3 |
| 8-9 | `VAL_M4` | `int16_t` | Setpoint M4 |
| 10 | `CHK` | `uint8_t` | Checksum (Suma con desbordamiento desde byte 1 al 9) |
| 11 | `END` | `uint8_t` | Fijo: `0x0A` (`\n`) |

*Nota sobre `MODE`: Si el modo es 0x00, aplicar los valores absolutos al I2C y el signo al pin de dirección. Si es 0x01, los valores vienen multiplicados por 100 (ej. `3250` = `32.5 rad/s`).*

### 2. Transmisión (STM32 -> PC) : Telemetría a 50-100 Hz

El STM32 debe reportar el estado de los 4 contadores de encoder de manera periódica usando un Timer o chequeo de `HAL_GetTick()`.

| Byte | Campo | Tipo | Descripción |
| :--- | :--- | :--- | :--- |
| 0 | `START_1` | `uint8_t` | Fijo: `0xAA` |
| 1 | `START_2` | `uint8_t` | Fijo: `0xBB` |
| 2-3 | `ENC_M1` | `uint16_t` | Valor crudo de `TIM3->CNT` |
| 4-5 | `ENC_M2` | `uint16_t` | Valor crudo de `TIM8->CNT` |
| 6-7 | `ENC_M3` | `uint16_t` | Valor crudo de `TIM2->CNT` |
| 8-9 | `ENC_M4` | `uint16_t` | Valor crudo de `TIM5->CNT` |
| 10 | `CHK` | `uint8_t` | Checksum (Suma desde byte 2 al 9) |
| 11 | `END` | `uint8_t` | Fijo: `0x0A` |

---

## Guía de Implementación para el Desarrollador

Para asegurar que el microcontrolador no pierda paquetes, sigue esta estrategia:

### Paso 1: Configurar DMA en RX e Interrupción IDLE
En tu `main.c`, después de iniciar los periféricos, arranca el DMA de recepción en modo circular sobre un buffer global amplio (ej. 64 bytes) y habilita la interrupción de línea inactiva.

```c
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE];

// Iniciar recepción DMA circular
HAL_UART_Receive_DMA(&huart5, rx_buffer, RX_BUFFER_SIZE);
// Habilitar interrupción de línea inactiva
__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);