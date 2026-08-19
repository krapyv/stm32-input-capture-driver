# Input Capture Peripheral Driver Module

This directory implements a reusable, register-level, bare-metal Input Capture peripheral driver for the STM32F411 microcontroller. Operating entirely at the register boundaries without relying on bloated abstraction layers, the driver enables any general-purpose timer (`TIM2` to `TIM5`) to capture fine-grained timestamps of edge transitions (Rising, Falling, or Both) across any compatible General Purpose Input/Output (`GPIO`) pin. 
A core feature of this module is its dual-edge handling mechanism, which dynamically toggles capture polarity at runtime to calculate precise elapsed intervals - such as physical button press durations - while handling 16-bit timer overflows asynchronously via interrupt-driven counters.

## 📂 Project Structure

```text
├── core/               # Centralized hardware configuration definitions
    ├── core_cm4.h      # ARM Cortex-M4 core peripheral access layer (NVIC interfaces)
│   └── stm32f411.h     # Core memory map and explicit register structs
└── periph/             # Reusable peripheral driver modules
    ├── input_capture_driver.c      # Low-level pin routing, timer clock gating, CCMR filtering, and ISR handlers
    ├── input_capture_driver.h      # Handle abstractions, structure definitions, and public API signatures
    ├── uart.c      # Non-blocking RingBuffer-backed UART driverr and printf redirection
    └── uart.h      # UART macros, base register boundary maps, and function declarations APIs
```

## 🧠 Core Low-Level Concepts
### Dynamic Polarity Toggling & Edge Tracking

To measure elapsed periods (like how long a button is held down) using a single timer channel, the driver intercepts the physical signal dynamically using a state-flipping interrupt architecture:
* **1. First Edge Catch:** The channel is initialized to watch for the initial active transition (e.g., Falling Edge when using active-low pull-up buttons). When the event fires, `TIM3_FirstEdgeHandler()` saves the raw counter value to `first_timestamp` and flips the polarity bits inside the Capture/Compare Enable Register (`CCER`) to look for the opposite edge.

* **Second Edge Catch:** When the opposite edge arrives, `TIM3_SecondEdgeHandler()` latches the ending counter value into `second_timestamp`. It automatically checks the timer type to calculate overflow additions before storing the complete duration delta, and then resets the channel to its initial baseline state.

### Alternate Function & Bitwise Index Math

To route the raw external silicon pin to the timer's digital capture matrix, the pin must be switched out of standard input mode and mapped into its corresponding Alternate Function:

* **Mode Configuration:** The GPIO Mode Register (`MODER`) must map the pin to `0x2` (Alternate Function Mode). Every pin uses exactly **2 bits**, meaning the shift offset evaluates to **$\text{pin} \times 2$**.
* **Low vs. High Register Split:** Alternate function mapping indices use **4 bits per pin**, distributed across two separate registers: `AFRL` (Pins 0-7) and `AFRH` (Pins 8-15).

$$\text{Bit Shift Offset (Pins 0--7)} = \text{pin} \times 4$$
$$\text{Bit Shift Offset (Pins 8--15)} = (\text{pin} - 8) \times 4$$

### Capture/Compare Register Mapping Abstraction (`CCMR1`/`CCMR2`)

Timer channel configuration registers are paired symmetrically. Channel 1 and 2 map directly to `CCMR1`, whereas Channels 3 and 4 map to `CCMR2`. The driver computes the register choice and internal bit positioning dynamically based on the channel index:

```c
uint32_t ccs_pos = ((ic->channel - 1) * 8); // for Channels 1 & 2
uint32_t ccs_pos_high = ((ic->channel - 3) * 8); // for Channels 3 & 4
```

The driver forces the `CCxS` bitfields to `0b01`, mapping the physical channel pin to its internal timer input line (`TIx`), and loads the `ICxF` field with `0xF` ($f_{DTS} / 32, N=8$) to act as a hardware low-pass debouncing filter.

## 📐 Mathematical Modeling & Timing Budget

The physical execution granularity is derived directly from the clock domain feeding the target timer instance. Driven by the internal High-Speed Interval (`HSI`) oscillator running at a native **16 MHz** with an undivided prescaler ($PSC = 0$), the timer counter increments exactly sixteen million times per second.

$$\text{Tick Frequency} = \frac{f_{HSI}}{PSC + 1} = \frac{16{,}000{,}000\text{ Hz}}{0 + 1} = 16\text{ MHz}$$

$$\text{Time Resolution per Tick} = \frac{1}{16\text{ MHz}} = 62.5\text{ ns}$$

### Asynchronous Overflow Compensation

Because the microcontroller mixes 32-bit timers (`TIM2` and `TIM5`) with 16-bit timers (`TIM3` and `TIM4`), the driver uses an intelligent scale adjustment loop:

* **32-bit Timers:** The counter overruns approximately every 4.47 minutes ($\frac{2^{32}}{16\text{ MHz}} \approx 268.4\text{ s}$). No software tracking is required for standard human interactions.

* **16-bit Timers:** The counter overruns every 4.096 milliseconds ($\frac{2^{16}}{16\text{ MHz}} = 4.096\text{ ms}$). To capture extended actions seamlessly, the driver catches the Update Interrupt Flag (UIF) in the ISR and logs it to a private `overflow_count` variable.

The absolute time interval is computed using the following mathematical model:

$$\Delta_{\text{Ticks}} = (\text{Overflow Count} \times 65{,}536) + (\text{Timestamp}_{\text{Second}} - \text{Timestamp}_{\text{First}})$$

$$\text{Total Duration (ms)} = \frac{\Delta_{\text{Ticks}}}{16{,}000}$$ 

## 🛠️  Unified Register Mapping Architecture

This driver uses standardized layout overlay structures to read and write directly to memory-mapped registers. These definitions cleanly align with the underlying geometry of the silicon layout.

### Configuration Structure Handle

The driver abstracts channel allocations by requiring users to populate a configuration structure instance (`IC_HandleTypeDef`). This contains both the static initialization properties and an isolated, volatile tracking block:

```c
typedef enum
{
    IC_RE = 0,   // 00 - Active Edge Capture: Rising Edge
    IC_FE,       // 01 - Active Edge Capture: Falling Edge
    IC_RESERVED, // 10 - Hardware Reserved State Alignment
    IC_BE        // 11 - Active Edge Capture: Both Edges
} IC_Polarity_t;

typedef struct
{
    uint32_t overflow_count; // Track counter roll-overs for 16-bit timers
    uint32_t first_timestamp; // Captured tick metrics on initial edge trigger
    uint32_t second_timestamp; // Captured tick metrics on terminal edge trigger
    uint32_t delta; // Absolute execution interval computed in hardware ticks
} IC_Data_t;

typedef struct
{
    TIM_RegDef_t *Instance; // Memory pointer to timer base block
    GPIO_RegDef_t *Port; // Memory pointer to GPIO base block
    uint32_t pin; // Pin allocation
    GPIO_AF_t af; // Alternate function
    uint32_t channel; // Timer channel mapping index
    IC_Polarity_t polarity; // Baseline active capture edge
    volatile IC_Data_t data; // Live operational capture variables
} IC_HandleTypeDef;
```

## 🔌 Hardware Circuit & Connection Guide

Because bare-metal drivers interact directly with physical silicon states, the input stage requires a predictable hardware topology to prevent floating pins from triggering spurious capture interrupts.

### 1. Schematic Topology (Active-Low Configuration)

The driver is optimized for an **Active-Low** input circuit utilizing the internal microcontroller pull-up resistor or an external equivalent.

```text
       VDD (3.3V)
          │
          ▼
        ┌───┐
        │   │ Internal / External Pull-Up Resistor
        └───┘
          │
          ├───────────────● Microcontroller Input Pin (PB0)
          │
          ▼  Push Button (Normally Open)
         ───
        │   │
         ───
          │
          ▼
         GND
```

### 2. Physical Pin Interface Mapping

Connect your target development board to the external circuitry using the following hardware mapping boundaries:

| Microcontroller Pin | Function | Hardware Peripheral Mapping | Target Device Component |
|---------------------|----------|-----------------------------|-------------------------|
| **PB0** | Input Capture Channel | `TIM3_CH3` (Alternate Function 2) | Physical Tactile Button / Signal Generator |
| **GND** | System Ground Reference | Common Ground Plane | External Circuit Ground Bus |
| **PA2** | Console Transmit (TX) | `USART2_TX` (Alternate Function 7) | PC Serial Terminal Receiver (RX) |
| **PA3** | Console Receive (RX) | `USART2_RX` (Alternate Function 7) | PC Serial Terminal Transmitter (TX) |

⚠️  **Hardware Safety Warning:** The STM32F411 general-purpose pins operate on a **3.3V logic level**. Ensure that any external signal generator or external pull-up network does not exceed 3.3V to prevent permanent over-voltage damage to the silicon input structures.

### 3. Hardware Debouncing Layer (104 Ceramic Capacitor)

Mechanical tactile switches do not transition between open and closed states cleanly. On a microscopic scale, the internal metal contacts bounce against each other for several milliseconds when pressed, generating a rapid sequence of spurious high-frequency glitches. Because the STM32 timer operates at 16 MHz, it is fast enough to capture these microsecond-long bounces as multiple legitimate button presses, corrupting your telemetry.

```text
                  VDD (3.3V)
                     │
                     ▼
                   ┌───┐
                   │   │ Pull-Up Resistor
                   └───┘
                     │
                     ├──────────────┬──────────────● Microcontroller Input Pin (PB0)
                     │              │
                     ▼ Push Button  ┴  104 Capacitor
                    ───             ┬  (0.1 µF)
                   │   │            │
                    ───             │
                     │              │
                     └──────────────┴──────────────● GND
```

To suppress this behavior before it reaches the microcontroller, a **104 ceramic capacitor (0.1 µF)** is placed in parallel directly across the switch contacts to form a low-pass Hardware RC Filter:

* **Signal Smoothing:** The capacitor acts as a tiny energy reservoir. When the button is unpressed, it charges up to 3.3V. When the button is pressed and the contacts bounce open and closed rapidly, the capacitor cannot discharge instantly. It absorbs and smooths out the high-frequency voltage spikes, presenting a clean, monotonic falling edge to the `PB0` input pin.

* **Complementary Debouncing:** This hardware capacitor works hand-in-hand with a software input filter configuration (`ICxF = 0xF`) initialized inside your `input_capture_init` routine, ensuring maximum tracking stability under real-world electromagnetic interference (EMI).

## 🚀 Driver Integration & Usage Guide

### 1. Application Implementation Example (main.c)

To deploy the driver, instantiate the target handle globally and implement a polling verification sequence inside your application's execution loop:

```c
#include <stdio.h>
#include "uart.h"
#include "core_cm4.h"
#include "stm32f411.h"
#include "input_capture_driver.h"

extern IC_HandleTypeDef input_capture;

int main(void)
{
    // initializing the system clock and console debugging
    timer_init();
    usart2_init();

    printf("--- System Boot: Drivers Initialized ---\n");

    // configuring the Input Capture structure
    input_capture.Instance = TIM3;
    input_capture.Port = GPIOB;
    input_capture.pin = 0;
    input_capture.af = GPIO_AF2;
    input_capture.channel = 3;
    input_capture.polarity = IC_FE;

    input_capture.data.overflow_count = 0;
    input_capture.data.first_timestamp = 0;
    input_capture.data.second_timestamp = 0;
    input_capture.data.delta = 0;

    // initializing the hardware channels and enabling interrupts
    nvic_interrupts_init(&input_capture);
    input_capture_init(&input_capture);

    while (1)
    {
        // if the first_timestamp has reset to 0, but the delta contains values, a full timing cycle finished and had been captured by the ISR
        if (input_capture.data.first_timestamp == 0 && input_capture.data.delta != 0)
        {
            uint32_t raw_ticks = input_capture.data.delta;
            input_capture.data.delta = 0;

            // we are using the Processor Clock with 16 MHz with prescaler = 0
            uint32_t total_ms = raw_ticks / 16000;
            uint32_t passed_seconds = total_ms / 1000;
            uint32_t remainder_ms = total_ms % 1000;

            printf("Button Hold Duration: %lu s %lu ms (Raw Ticks: %lu)\n\r", passed_seconds, remainder_ms, raw_ticks);
            fflush(stdout);
        }
    }

    return 0;
}
```

### 2. Verified Console Telemetry

When communicating with a desktop PC terminal application (via a USB-to-UART bridge mapped to `USART2`), successful signal transitions generate clean runtime data updates:

```text
--- System Boot: Drivers Initialized ---
Button Hold Duration: 0 s 363 ms (Raw Ticks: 5815840)
Button Hold Duration: 1 s 514 ms (Raw Ticks: 24228480)
Button Hold Duration: 1 s 384 ms (Raw Ticks: 22150688)
Button Hold Duration: 1 s 983 ms (Raw Ticks: 31733408)
Button Hold Duration: 5 s 873 ms (Raw Ticks: 93971584)
Button Hold Duration: 1 s 342 ms (Raw Ticks: 21485248)
Button Hold Duration: 0 s 727 ms (Raw Ticks: 11636096)
Button Hold Duration: 3 s 469 ms (Raw Ticks: 55518624)
```

## 3. Build System Path Configuration (Makefile)

To resolve includes cleanly when compilation blocks cross sibling folder boundaries, you must explicitly expose the directory paths to the preprocessor using -I flag.

Assuming your library architecture places core files and peripheral files in neighboring directories:

```text
├── core/
    ├── core_cm4.h
│   └── stm32f411.h
└── periph/
    ├── uart.c
    ├── uart.h
    ├── ring_buffer.c
    ├── ring_buffer.h
    ├── input_capture_driver.c
    └── input_capture_driver.h
```

Append these search directories directly to your global compilation flags variable inside your `Makefile`:

```makefile
# Define include search paths for neighboring module structures
INC_DIRS += -I../core
INC_DIRS += -I../periph

# Append include directories to the GNU C Compiler configuration flags
CFLAGS += $(INC_DIRS)
CFLAGS += -mcpu=cortex-m4 -mthumb -Wall -O2
```

## 🗺️   Next Steps & Learning Roadmap

To transition this bare-metal input capture module into a robust, automotive-grade signal processing subsystem, upcoming development phases will introduce:

1. **Hardware-Based Pulse-Width and Period Measurement (PWM Input Mode):** Configure dual internal channel routing (`TI1FP1` and `TI1FP2`) to latch both edges simultaneously using a single input pin. This allows the hardware to calculate both the absolute frequency and the duty cycle of an incoming signal in real time without CPU overhead.

2. **Advanced Software & Hardware Filter Debouncing:** Implement progressive time-threshold validation inside the Input Capture ISR or tune the timer's internal clock division (`fDTS`) sampling ratios to eliminate mechanical switch bounce and high-frequency EMI noise common in automotive chassis environments.

3. **Automotive Sensor Protocol Decoding (Wheel Speed & IC Sensors):** Adapt the capture subsystem to decode incoming data streams from digital variable reluctance (VR) or Hall-effect speed sensors, establishing the foundation for baseline anti-lock braking (ABS) and traction tracking emulation loops.

## 🪪 Author & License
* **Author:** Dmytro Krapyvianskyi
* **License:** This project is open-source software licensed under the MIT License.

