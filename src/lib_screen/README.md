# Raspberry Pi Pico - GC9A01A High-Perf Library

A double-buffered, non-blocking DMA driver for the GC9A01A Round LCD (240x240), including a specific "Minion Eye" animation module.

## Features
* **Double Buffering:** Prevents screen tearing and flickering (Requires ~230kB RAM).
* **Async DMA:** Transmission happens entirely in the background. 0% CPU usage during screen refresh.
* **Decoupled Design:** `GC9A01A` is the generic hardware driver; `minion_eye` is a physics-based application.
* **Pico 2 W Ready:** Optimized for RP2040 and RP2350 architectures.

## Wiring (Pico 2 W)

| Display Pin | Pico Pin | Function |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |
| **SCL** | GP18 | SPI0 SCK |
| **SDA** | GP19 | SPI0 TX (MOSI) |
| **RES** | Any / NC | Reset (Can use Software Reset -1) |
| **DC** | GP16 | Data/Command |
| **CS** | GP17 | Chip Select |

## API Reference

### 1. Core Driver Primitives (`GC9A01A.h`)
These functions handle the low-level SPI/DMA operations and basic drawing.

| Function | Usage / Description |
| :--- | :--- |
| **`gc9a01a_init`** | **Setup.** Initializes the driver struct and configures GPIO pins for CS, DC, and Reset. |
| **`gc9a01a_begin`** | **Startup.** Begins SPI communication and sends the initial "wake up" command sequence to the display. |
| **`gc9a01a_is_busy`** | **Flow Control.** Returns `true` if the DMA is currently sending a frame. Used to prevent writing to the buffer while it is being transmitted. |
| **`gc9a01a_update_async`** | **Render (Non-Blocking).** Swaps the front/back buffers and triggers a DMA transfer in the background. Returns immediately so the CPU can do other work. |
| **`gc9a01a_fill_screen`** | **Graphics.** Fills the *entire* current drawing buffer with a single 16-bit color. |
| **`gc9a01a_draw_pixel`** | **Graphics.** Writes a single pixel to the current buffer. |
| **`gc9a01a_fill_circle`** | **Graphics.** Draws a filled circle. Optimized for the round display form factor. |

### 2. Minion Eye Application Primitives (`minion_eye.h`)
These functions sit on top of the driver to handle the physics and "personality" of the eye.

| Function | Usage / Description |
| :--- | :--- |
| **`minion_eye_init`** | **Setup.** Connects the eye logic to the initialized `gc9a01a` driver instance. |
| **`minion_eye_update_non_blocking`** | **Main Loop Task.** Handles physics, blinking, and rendering. <br>• **0µs cost** if DMA is busy (returns immediately).<br>• **~20ms cost** if a new frame needs to be drawn. |
| **`minion_eye_set_emotion`** | **Control.** Forces the eye into a specific state (e.g., `EMOTION_HAPPY`, `EMOTION_SUSPICIOUS`). |

## Usage Example

### 1. CMakeLists.txt
Ensure you link the required hardware libraries.

```cmake
target_link_libraries(my_robot 
    pico_stdlib 
    hardware_spi 
    hardware_dma
    hardware_interp # Optional, good for graphics math
)
```


### 2. main.c Implementation
Here is how to combine the primitives in your main loop.

```c
#include "pico/stdlib.h"
#include "GC9A01A.h"
#include "minion_eye.h"

int main() {
    stdio_init_all();

    // 1. Initialize Driver
    gc9a01a_t tft;
    // tft instance, CS=17, DC=16, RST=-1 (Soft Reset)
    gc9a01a_init(&tft, 17, 16, -1); 
    gc9a01a_begin(&tft);

    // 2. Initialize Eye Application
    minion_eye_init(&tft);

    // 3. Set initial mood
    minion_eye_set_emotion(EMOTION_NORMAL);

    while (true) {
        // Run the non-blocking update
        // This will only render if the previous frame is finished sending
        minion_eye_update_non_blocking();

        // You can run other robot logic here freely!
        // The display driver will not pause your code.
        robot_balance_logic(); 
    }
}
```
### 3. Emotion States
You can pass these enums to minion_eye_set_emotion():
- EMOTION_NORMAL: Standard blinking and looking around.
- EMOTION_HAPPY: Squints slightly, often used when an interaction occurs.
- EMOTION_TIRED: Droopy eyelids, slower movement.
- EMOTION_SUSPICIOUS: Narrowed eyes, side-glance.
- EMOTION_AUTO: The system picks random emotions automatically.