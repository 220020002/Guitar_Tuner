# Real-Time FFT Guitar Tuner on Tiva C Series (TM4C123G)

## Project Overview
This project implements a digital guitar tuner using the **Tiva C Series TM4C123G LaunchPad**. It utilizes the ARM **CMSIS-DSP** library to perform real-time Fast Fourier Transforms (FFT) on audio signals captured via a **MAX4466** microphone amplifier.

The system samples audio at **5 kHz**, processes the signal to detect the fundamental frequency, and visualizes the tuning status (Flat, Sharp, or In-Tune) using an LED array.

## Features
* **Real-Time Signal Processing:** Utilizes `arm_rfft_fast_f32` for efficient spectral analysis.
* **Precise Sampling:** Timer-triggered ADC interrupts ensure a stable sampling rate of 5000 Hz.
* **Noise Reduction:** Implements a Hanning Window to reduce spectral leakage.
* **Frequency Stabilization:** Uses a "Mode Filter" (History Buffer) to reject transient noise and stabilize frequency readings.
* **Visual Interface:** 5-LED display indicating how far the string is from the target frequency.

## Hardware Requirements
1.  **Microcontroller:** Tiva C Series TM4C123G LaunchPad.
2.  **Audio Input:** MAX4466 Electret Microphone Amplifier (with adjustable gain).
3.  **Display:** 5 LEDs (2x Red, 1x Green, 2x Yellow) + 220 Ohm Resistors.
4.  **Connecting Wires:** Breadboard and jumpers.

## Build Artifacts & Configuration
To ensure the project compiles correctly within the specific IDE environment, we created custom build artifacts.

### 1. Custom Static Library (`dsplib-cm4f.lib`)
Instead of including all raw CMSIS source files in the project (which increases build time and complexity), we compiled a custom static library.
* **Purpose:** Contains pre-compiled object code for the Complex Math, Transform Functions (FFT), and Statistics functions.
* **Configuration:** Built specifically for **ARM Cortex-M4** with **Little Endian** format and **Floating Point Unit (FPU)** enabled (`-mfloat-abi=hard`).
* **Usage:** This library is linked during the build process to provide the DSP functionality.

### 2. DSP Configuration Patch File
A patch file was included to resolve compatibility issues between the TivaWare drivers and the CMSIS-DSP library.
* **Function:** Modifies project preprocessor definitions and header inclusions to ensure the `ARM_MATH_CM4` and FPU settings are correctly recognized by the compiler.
* **Requirement:** Without this patch, the compiler fails to link the DSP functions or generates type conflict errors.

## Pin Configuration

### Audio Input (Microphone)
| Module Pin | Tiva Pin | Function |
| :--- | :--- | :--- |
| **VCC** | VBUS (3.3V) | Power |
| **GND** | GND | Ground |
| **OUT** | **PE2** | ADC Input (AIN1) |

### LED Output (Port B)
The LEDs function as a visual needle.

| Tiva Pin | Color | Meaning | Condition |
| :--- | :--- | :--- | :--- |
| **PB0** | Yellow | Very Flat | < -10 Hz |
| **PB1** | Yellow | Flat | -3 Hz to -10 Hz |
| **PB2** | **Green** | **In Tune** | +/- 3 Hz |
| **PB3** | Red | Sharp | +3 Hz to +10 Hz |
| **PB4** | Red | Very Sharp | > +10 Hz |

## Installation & Setup

1.  **Hardware Setup:**
    * Connect the MAX4466 output to pin **PE2**.
    * Connect LEDs to **PB0-PB4** according to the pinout above.
2.  **Software Configuration:**
    * **Startup Code:** Manually update the startup file (e.g., `startup_tm4c123.c`) to register `ADC1SS3_Handler` in the interrupt vector table.
    * **Apply Patch:** Apply the included DSP patch file to your project settings/headers.
    * **Link Library:** Add `dsplib-cm4f.lib` to your project's linker settings.
3.  **Build & Flash:**
    * Compile `main.c` using CCS.
    * Flash the code to the board.
4.  **Usage:**
    * Pluck the guitar string.
    * Adjust the tuning peg until the **Green LED (PB2)** lights up.

## Author
[Your Name]
B.Tech Student, 4th Year
