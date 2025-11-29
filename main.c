/*
 * ==============================================================================
 * Tiva C Tuner - FINAL LED VERSION (Noise Filtered)
 * ==============================================================================
 * Features:
 * 1. Frequency Stability (Hanning Window + Mode Filter)
 * 2. LED Indicators (PB0-PB4) for tuning.
 * 3. FIFO Bug Fix included.
 * 4. Stricter Noise Gating to prevent background "Yellow" lights.
 * ==============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/tm4c123gh6pm.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "arm_math.h"

// --- Configuration ---
#define FFT_SIZE            1024
#define SAMPLE_RATE_HZ      5000.0f
#define SYSTEM_CLOCK_HZ     16000000
#define PI                  3.14159265359f

// --- Noise Gating ---
// Adjust this! If LEDs won't turn on when playing, LOWER this (e.g., to 200).
// If LEDs turn on with background noise, RAISE this (e.g., to 800).
#define NOISE_THRESHOLD     500.0f

// --- Tuning Configuration ---
// Change this to the note you are tuning!
volatile float32_t g_targetFrequency = 261.63f; // Default: Middle C

// --- Buffers ---
volatile uint32_t g_ADCSamples[FFT_SIZE];
volatile uint32_t g_ui32SampleCount = 0;
volatile bool g_bBufferFull = false;

// --- FFT Variables ---
static arm_rfft_fast_instance_f32 g_fftInstance;
static float32_t g_fftInput[FFT_SIZE];
static float32_t g_fftOutput[FFT_SIZE];
static float32_t g_fftMagnitude[FFT_SIZE / 2];
static float32_t g_hanningWindow[FFT_SIZE];

// --- Stability Variables ---
#define HISTORY_SIZE 10
float32_t g_freqHistory[HISTORY_SIZE] = {0.0f};
uint32_t  g_historyIndex = 0;
volatile float32_t g_stableFrequency = 0.0f;

// --- Raw Results ---
volatile float32_t g_detectedFrequency = 0.0f;
volatile float32_t g_peakMagnitude = 0.0f;
static uint32_t g_peakIndex = 0;

// --- Helper Functions ---

void InitHanningWindow(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        g_hanningWindow[i] = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(FFT_SIZE - 1)));
    }
}

// --- Interrupt Handler ---
void ADC1SS3_Handler(void) {
    ADC1_ISC_R = 0x08;

    if (g_ui32SampleCount < FFT_SIZE) {
        g_ADCSamples[g_ui32SampleCount] = ADC1_SSFIFO3_R;
        g_ui32SampleCount++;
    }

    if (g_ui32SampleCount >= FFT_SIZE) {
        g_bBufferFull = true;
        TIMER0_CTL_R &= ~0x01; // Stop Timer
        ADC1_IM_R &= ~0x08;    // Mask Int
    }
}

// --- Stability Logic ---
float32_t GetStableFrequency(float32_t newFreq) {
    g_freqHistory[g_historyIndex] = newFreq;
    g_historyIndex = (g_historyIndex + 1) % HISTORY_SIZE;

    float32_t bestVal = 0.0f;
    int maxCount = 0;

    for (int i = 0; i < HISTORY_SIZE; i++) {
        float32_t currentVal = g_freqHistory[i];
        int currentCount = 0;
        for (int j = 0; j < HISTORY_SIZE; j++) {
            if (g_freqHistory[j] == currentVal) currentCount++;
        }
        if (currentCount > maxCount) {
            maxCount = currentCount;
            bestVal = currentVal;
        }
    }
    return bestVal;
}

// --- FFT Logic ---
void RunFFT(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        g_fftInput[i] = (float32_t)g_ADCSamples[i];
    }

    float32_t dcOffset;
    arm_mean_f32(g_fftInput, FFT_SIZE, &dcOffset);
    arm_offset_f32(g_fftInput, -dcOffset, g_fftInput, FFT_SIZE);
    arm_mult_f32(g_fftInput, g_hanningWindow, g_fftInput, FFT_SIZE);
    arm_rfft_fast_f32(&g_fftInstance, g_fftInput, g_fftOutput, 0);
    arm_cmplx_mag_f32(g_fftOutput, g_fftMagnitude, FFT_SIZE / 2);

    // Peak Detect
    arm_max_f32(&g_fftMagnitude[1], (FFT_SIZE / 2) - 1, &g_peakMagnitude, &g_peakIndex);
    g_peakIndex = g_peakIndex + 1;

    g_detectedFrequency = (float32_t)g_peakIndex * (SAMPLE_RATE_HZ / (float32_t)FFT_SIZE);
    g_stableFrequency = GetStableFrequency(g_detectedFrequency);
}

// --- Hardware Setup ---
void SetupADC(void) {
    SYSCTL_RCGCADC_R |= 0x02;   // ADC1
    SYSCTL_RCGCGPIO_R |= 0x10;  // Port E
    volatile uint32_t d = SYSCTL_RCGCGPIO_R;

    GPIO_PORTE_DIR_R &= ~0x04;  // PE2 Input
    GPIO_PORTE_AFSEL_R |= 0x04;
    GPIO_PORTE_DEN_R &= ~0x04;
    GPIO_PORTE_AMSEL_R |= 0x04;

    ADC1_ACTSS_R &= ~0x08;
    ADC1_EMUX_R = (ADC1_EMUX_R & ~0xF000) | 0x5000;
    ADC1_SSMUX3_R = 1;
    ADC1_SSCTL3_R = 0x06;
    ADC1_SAC_R = 0x02;
    ADC1_IM_R |= 0x08;
    NVIC_EN1_R |= 0x00080000;
    ADC1_ACTSS_R |= 0x08;
}

void SetupTimer(void) {
    SYSCTL_RCGCTIMER_R |= 0x01;
    volatile uint32_t d = SYSCTL_RCGCTIMER_R;
    TIMER0_CTL_R = 0;
    TIMER0_CFG_R = 0x00;
    TIMER0_TAMR_R = 0x02;
    TIMER0_TAILR_R = (SYSTEM_CLOCK_HZ / (uint32_t)SAMPLE_RATE_HZ) - 1;
    TIMER0_CTL_R |= 0x20;
    TIMER0_CTL_R |= 0x01;
}

void SetupLEDs(void) {
    SYSCTL_RCGCGPIO_R |= 0x02; // Port B
    volatile uint32_t d = SYSCTL_RCGCGPIO_R;

    GPIO_PORTB_DIR_R |= 0x1F;  // PB0-PB4 Output
    GPIO_PORTB_DEN_R |= 0x1F;  // Enable Digital
    GPIO_PORTB_DATA_R &= ~0x1F; // Clear all
}

// --- LED Tuning Logic ---
void UpdateTuningLEDs(void) {
    // 1. Clear previous state
    GPIO_PORTB_DATA_R &= ~0x1F;

    // 2. Volume Gate
    // Increased to 500.0f to filter out background noise
    if (g_peakMagnitude < NOISE_THRESHOLD) {
        return;
    }

    // 3. Frequency Gate
    // Ignore hum below 75Hz (Electrical hum is usually 50Hz/60Hz)
    // If we detect 60Hz, diff would be -200, which lights up Yellows. We want to avoid that.
    if (g_stableFrequency < 75.0f) {
        return;
    }

    float32_t diff = g_stableFrequency - g_targetFrequency;

    // --- TUNING LOGIC ---

    // GREEN (IN TUNE): +/- 3 Hz
    if (diff >= -3.0f && diff <= 3.0f) {
        GPIO_PORTB_DATA_R |= 0x04; // PB2 (Green)
    }
    // YELLOW (SLIGHTLY FLAT): -3 to -10 Hz
    else if (diff >= -10.0f && diff < -3.0f) {
        GPIO_PORTB_DATA_R |= 0x02; // PB1 (Yellow)
    }
    // YELLOW (VERY FLAT): Worse than -10 Hz -> GLOW BOTH PB0 and PB1
    else if (diff < -10.0f) {
        GPIO_PORTB_DATA_R |= 0x03; // PB0 | PB1 (0x01 | 0x02 = 0x03)
    }
    // RED (SLIGHTLY SHARP): +3 to +10 Hz
    else if (diff > 3.0f && diff <= 10.0f) {
        GPIO_PORTB_DATA_R |= 0x08; // PB3 (Red)
    }
    // RED (VERY SHARP): Worse than +10 Hz -> GLOW BOTH PB3 and PB4
    else if (diff > 10.0f) {
        GPIO_PORTB_DATA_R |= 0x18; // PB3 | PB4 (0x08 | 0x10 = 0x18)
    }
}

// --- Main ---
int main(void) {
    FPUEnable();
    FPULazyStackingEnable();

    // 1. Init
    arm_rfft_fast_init_f32(&g_fftInstance, FFT_SIZE);
    InitHanningWindow();
    SetupADC();
    SetupTimer();
    SetupLEDs();

    // 2. Startup Flash (Verifies LEDs are working)
    GPIO_PORTB_DATA_R = 0x1F; // All ON
    SysCtlDelay(SYSTEM_CLOCK_HZ / 6); // Delay ~0.5s
    GPIO_PORTB_DATA_R = 0x00; // All OFF

    // 3. Enable Interrupts
    __asm("    cpsie    i\n");

    while(1) {
        // A. Wait for Buffer
        while (g_bBufferFull == false) {};

        // B. Process
        RunFFT();

        // C. Update LEDs
        UpdateTuningLEDs();

        // D. Delay
        SysCtlDelay(SYSTEM_CLOCK_HZ / 30);

        // E. Restart
        g_ui32SampleCount = 0;
        g_bBufferFull = false;

        // Drain FIFO (Fixed)
        while((ADC1_SSFSTAT3_R & 0x100) == 0) {
            volatile uint32_t trash = ADC1_SSFIFO3_R;
        }

        // Re-arm
        ADC1_ISC_R = 0x08;
        ADC1_IM_R |= 0x08;
        TIMER0_CTL_R |= 0x01;
    }
}
