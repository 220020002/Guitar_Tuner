#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"


#include "driverlib/fpu.h"
#include "driverlib/sysctl.h" // Added for SysCtlDelay
#include "arm_math.h"

#define FFT_SIZE            1024
#define SAMPLE_RATE_HZ      10000.0f
#define SYSTEM_CLOCK_HZ     16000000

//ADC Buffers
volatile uint32_t g_ADCSamples[FFT_SIZE];
volatile uint32_t g_ui32SampleCount = 0;
volatile bool g_bBufferFull = false;

//FFT Buffers
static arm_rfft_fast_instance_f32 g_fftInstance; //strucy holding look up tables
static float32_t g_fftInput[FFT_SIZE];          //copy of ADC data stored
static float32_t g_fftOutput[FFT_SIZE];
static float32_t g_fftMagnitude[FFT_SIZE / 2];


volatile float32_t g_detectedFrequency = 0.0f;
static float32_t g_peakMagnitude = 0.0f;
static uint32_t g_peakIndex = 0;


void ADC1SS3_Handler(void) {

    ADC1_ISC_R = ADC_ISC_IN3;      // Clear the interrupt flag

    if (g_ui32SampleCount < FFT_SIZE) {
        g_ADCSamples[g_ui32SampleCount] = ADC1_SSFIFO3_R;   //read data from FIFO and store in SRAM
        g_ui32SampleCount++;
    }

    // If buffer is now full
    if (g_ui32SampleCount >= FFT_SIZE) {
        g_bBufferFull = true;           //set flag
        TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // Disable Timer 0A
        ADC1_IM_R &= ~ADC_IM_MASK3;      // Disable ADC Sequencer 3 interrupt
    }
}

void SetupADC(void) {
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;   //Turns on power to ADC
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // Port E

    // Wait for clocks
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {};
    while((SYSCTL_PRADC_R & SYSCTL_PRADC_R1) == 0) {};

    // Configure PE2 (AIN1)
    GPIO_PORTE_DIR_R &= ~0x04;
    GPIO_PORTE_AFSEL_R |= 0x04;
    GPIO_PORTE_DEN_R &= ~0x04;
    GPIO_PORTE_AMSEL_R |= 0x04;

    // Configure ADC1 SS3
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;      //Disable SS3
    ADC1_EMUX_R = (ADC1_EMUX_R & ~ADC_EMUX_EM3_M) | ADC_EMUX_EM3_TIMER;   //Clearing the previous setting and selecting the Timer0 as the trigger source
    ADC1_SSMUX3_R = 1;                                                     // Select AIN1 ie PE2
    ADC1_SSCTL3_R = ADC_SSCTL3_IE0 | ADC_SSCTL3_END0;                    //Triggers the interrupt after reading 1 sample
    ADC1_SAC_R = ADC_SAC_AVG_4X; // 4x Oversampling

    ADC1_IM_R |= ADC_IM_MASK3; // Enable Interrupt
    NVIC_EN1_R |= 0x80000;     // Enable IRQ 51
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;     //Enable SS3
}

void SetupTimer(void) {
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    while((SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R0) == 0) {};

    TIMER0_CTL_R = 0;
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;          //32 bit timer
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER0_TAILR_R = (SYSTEM_CLOCK_HZ / (uint32_t)SAMPLE_RATE_HZ) - 1;   //Load value
    TIMER0_CTL_R |= TIMER_CTL_TAOTE; // physically connecting Timer A A to ADC
    TIMER0_CTL_R |= TIMER_CTL_TAEN;  // Enable Timer
}


void RunFFT(void)
{
    // Casting
    for (int i = 0; i < FFT_SIZE; i++) {
        g_fftInput[i] = (float32_t)g_ADCSamples[i];
    }

    //Zero-Centering
    float32_t dcOffset;
    arm_mean_f32(g_fftInput, FFT_SIZE, &dcOffset);
    arm_offset_f32(g_fftInput, -dcOffset, g_fftInput, FFT_SIZE);


    arm_rfft_fast_f32(&g_fftInstance, g_fftInput, g_fftOutput, 0);      //FFT
    arm_cmplx_mag_f32(g_fftOutput, g_fftMagnitude, FFT_SIZE / 2);       //Magnitude

    //Peak
    arm_max_f32(&g_fftMagnitude[1], (FFT_SIZE / 2) - 1, &g_peakMagnitude, &g_peakIndex);
    g_peakIndex = g_peakIndex + 1;

    // 6. Calculate Frequency
    g_detectedFrequency = (float32_t)g_peakIndex * (SAMPLE_RATE_HZ / (float32_t)FFT_SIZE);
}


int main(void) {

    FPUEnable();
    FPULazyStackingEnable();

    arm_rfft_fast_init_f32(&g_fftInstance, FFT_SIZE);      //Initialize FFT

    SetupADC();
    SetupTimer();

    // Enable Global Interrupts
    __asm("    cpsie    i\n");

    while(1) {
        while (g_bBufferFull == false) {
           //do nothing
        }

        RunFFT();

        SysCtlDelay(SYSTEM_CLOCK_HZ / 3/5 );     //Because we are watching the variables so our output should not change instantaneously

        // RESTART ACQUISITION
        g_ui32SampleCount = 0;
        g_bBufferFull = false;

        ADC1_ISC_R = ADC_ISC_IN3;          // Clear any pending interrupt flags that might have occurred
        ADC1_IM_R |= ADC_IM_MASK3;          // Re-enable the ADC Interrupt
        TIMER0_CTL_R |= TIMER_CTL_TAEN;      // Re-enable the Timer
    }
}
