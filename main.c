#include <stdbool.h>
#include <stdint.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "arm_math.h"       // Main CMSIS-DSP header

// Define PI for our sine wave generator
#ifndef PI_F
#define PI_F 3.1415926535897932384626433832795f
#endif

// --- FFT Configuration ---
#define FFT_SIZE 1024
#define SAMPLE_RATE 1024.0f
#define TEST_FREQ 45.0f     // Test frequency: 82.0 Hz (a slightly flat low-E string)

// --- CMSIS-DSP Buffers & Instance ---

// The "instance" structure holds configuration data for the FFT
static arm_rfft_fast_instance_f32 g_fftInstance;

// This is our 1024-point input buffer
static float32_t g_fftInput[FFT_SIZE];

// This buffer will hold the 1024-point "packed" output of the FFT
static float32_t g_fftOutput[FFT_SIZE];

// *** NEW ***
// This buffer will hold the "magnitude" (strength) of each frequency bin
// It has N/2 entries (512)
static float32_t g_fftMagnitude[FFT_SIZE / 2];

// *** NEW ***
// This will hold our final answer
static float32_t g_detectedFrequency = 0.0f;
static float32_t g_peakMagnitude = 0.0f;
static uint32_t g_peakIndex = 0;


/**
 * main.c
 * This code runs a 1024-point FFT on a simulated 82.0 Hz sine wave
 * and finds the peak frequency.
 */
int main(void)
{
    // --- 1. System Setup ---
    FPUEnable();
    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


    // --- 2. Initialize the FFT ---
    arm_rfft_fast_init_f32(&g_fftInstance, FFT_SIZE);


    // --- 3. Simulate the Input Signal ---
    for (uint32_t i = 0; i < FFT_SIZE; i++)
    {
        float32_t sampleTime = (float32_t)i / SAMPLE_RATE;
        g_fftInput[i] = arm_sin_f32(2.0f * PI_F * TEST_FREQ * sampleTime);
    }


    // --- 4. Run the FFT ---
    arm_rfft_fast_f32(&g_fftInstance, g_fftInput, g_fftOutput, 0);


    // --- 5. Calculate Magnitude ---
    // The FFT gives us complex numbers. We need the "magnitude" (strength)
    // of each frequency bin. This converts the 512 complex pairs
    // from g_fftOutput into 512 real magnitude values in g_fftMagnitude.
    arm_cmplx_mag_f32(g_fftOutput, g_fftMagnitude, FFT_SIZE / 2);


    // --- 6. Find the Strongest Frequency Peak ---
    //
    // We use arm_max_f32 to find the bin with the highest magnitude.
    // CRITICAL: We start searching from index 1, not 0.
    // Index 0 is the "DC component" (the average value), not a musical note.
    //
    arm_max_f32(
        &g_fftMagnitude[1],       // Start searching from the 2nd bin
        (FFT_SIZE / 2) - 1,   // Number of bins to search
        &g_peakMagnitude,         // Store the strength of the peak here
        &g_peakIndex              // Store the *index* of the peak here
    );

    // We must add 1 back to the index to correct for starting at [1]
    g_peakIndex = g_peakIndex + 1;


    // --- 7. Convert Bin Index to Frequency (Hz) ---
    // Freq = BinIndex * (SampleRate / FftSize)
    // Freq = g_peakIndex * (1024.0 / 1024.0)
    // Freq = g_peakIndex * 1.0
    //
    g_detectedFrequency = (float32_t)g_peakIndex * (SAMPLE_RATE / (float32_t)FFT_SIZE);


    // --- 8. Stop and Check Results ---

    // Put a breakpoint on the while(1) line.
    //
    // In the "Expressions" window, look at these new variables:
    //
    // g_peakIndex         == 82         (This is Bin 82)
    // g_peakMagnitude     == 512.0      (This is N/2)
    // g_detectedFrequency == 82.0       <-- THIS IS YOUR ANSWER

    while(1){
      // Stop here
    }

    return 0; // Never reached
}
