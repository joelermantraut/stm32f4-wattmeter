/*
******************************************************************************
File:     	 main.c
Date:     	 2021-03-07
Authors: 	 Abdala Julian, Joel Ermantraut (finally improved by last one)
Description: Measurement of active, reactive and apparent power.
Connection:	 PA1: ACD1, canal 1, voltage transducer 
 	 	  	 PA2: ACD1, canal 2, current transducer 

******************************************************************************
*/

// INCLUDES
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_dma.h>
#include "arm_math.h"
#include "misc.h"
// STM libraries
#include "tm_stm32f4_hd44780.h"
// Third part libraries
#include "stdio_ext.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
// Standard C libraries

// DEFINES
#define	SAMPLES	    200						    // Samples taken each cycle
#define CYCLES		4						    // Number of cycles 
#define N			(SAMPLES * CYCLES) 	        // Number of each measurement 
#define VREF		2.94					    // Max voltage in ADC 
#define MAX_ADC		4095.0
#define V_SCALE		(282 * sqrt(2)) / (VREF / 2)// Max voltage level provided by transducer 
#define I_SCALE		17.51					    // Max current level provided by transducer

// ESTRUCTURES
typedef struct {
	uint32_t peripheric;
	DMA_Stream_TypeDef *stream;
	uint32_t channel;
	uint32_t buffer_size;
	uint32_t peripheric_data_size;
	uint32_t memory_data_size;
	uint32_t mode;
	IRQn_Type IRQ;
} DMA_PIN;

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t peripheric_port;
	uint32_t peripheric_adc;
	ADC_TypeDef *adc;
	uint8_t injected_channel;
	uint8_t channel;
} ADC_PIN;

// VARIABLES
float32_t P = 0, Pq = 0, Ps = 0, FP = 0;	// Power and power factor
float32_t Vef = 0, Ief = 0;                 // Effective value of signal
// DMA buffer
uint32_t buffer[2 * N] = {0};
// Sample arrays
float32_t V[N];		    // Voltage vector
float32_t I[N];		    // Current vector
float32_t VAC[N];	    // Voltage vector without input offset
float32_t IAC[N];	    // Current vector without input offset
float32_t Vinst[N];	    // Voltage vector scaled to real values
float32_t Iinst[N];	    // Current vector scaled to real values 
float32_t Vmed = 0.0;	// Mean voltage value
float32_t Imed = 0.0;	// Mean current value
uint8_t DMA_ready = 0;
// General use
uint16_t PrescalerValue = 0;

DMA_PIN DMA_measure = {
	RCC_AHB1Periph_DMA2,
	DMA2_Stream0,
	DMA_Channel_0,
	2 * N,
	DMA_PeripheralDataSize_HalfWord,
	DMA_MemoryDataSize_HalfWord,
	DMA_Mode_Circular,
	DMA2_Stream0_IRQn
};

ADC_PIN current_pin = {
	GPIOC,
	GPIO_Pin_1,
	RCC_AHB1Periph_GPIOC,
	RCC_APB2Periph_ADC1,
	ADC1,
	ADC_InjectedChannel_1,
	ADC_Channel_11
};

ADC_PIN voltage_pin = {
	GPIOC,
	GPIO_Pin_2,
	RCC_AHB1Periph_GPIOC,
	RCC_APB2Periph_ADC1,
	ADC1,
	ADC_InjectedChannel_1,
	ADC_Channel_12
};

/* Private function prototypes */
void TIMER_Config(void);
void ADC1_Configuration(ADC_PIN adc_pin1, ADC_PIN adc_pin2, DMA_PIN dma_pin, uint32_t buffer[]);
void NVIC_Config();
void process_data();
void LCD();

/**
**===========================================================================
**  Abstract: main program
**===========================================================================
*/
int main(void) {
	SystemInit();
    // DISPLAY --------------------------------------------------------
	TM_HD44780_Init(20, 4);
    TM_HD44780_Puts(0, 3, "Wattmeter");
    // ADC ------------------------------------------------------------
	ADC1_Configuration(voltage_pin, current_pin, DMA_measure, buffer);
    // DMA Interrupt -------------------------------------------
	NVIC_Config(dma_pin);
    // timer config ------------------------------------------------
	TIMER_Config();
    // Program -------------------------------------------------------
    while (1)
    {
        if (DMA_ready) {
            TIM_Cmd(TIM3, DISABLE);
            process_data();
            LCD();
            TIM_Cmd(TIM3, ENABLE);

            DMA_ready = 0;
        }
    }
}

void process_data () {
	int i;

    // First, divide ADC buffer to take voltage and current arrays
	for(i = 0; i < N; i++)
	{
        // Scale values within to defined ADC limits
		V[i] = buffer[2 * i] * VREF / MAX_ADC;
		I[i] = buffer[2 * i + 1] * VREF / MAX_ADC;
	}
    // Calcule mean values
	arm_mean_f32(V, N, &Vmed);
	arm_mean_f32(I, N, &Imed);

    // Deletes offset
	arm_offset_f32(V, -Vmed, VAC, N);
	arm_offset_f32(I, -Imed, IAC, N);

    // Scale array to real values
	arm_scale_f32(VAC, V_SCALE, Vinst, N);
	arm_scale_f32(IAC, I_SCALE, Iinst, N);

	// Active power
	arm_dot_prod_f32(Vinst, Iinst, N, &P);
	P = P / N;
	// Effective voltage and current 
	arm_rms_f32(Vinst, N, &Vef);
	arm_rms_f32(Iinst, N, &Ief);
	// Apparente power
	Ps = Vef * Ief;
	// Reactive power
	Pq = sqrt(Ps * Ps - P * P);
	// Power factor
	FP = P / Ps;
}

void LCD () {
    char string[21];
    // LCD size + 1 (\0)
	sprintf(string, "%4.2lf V    %4.2lf A ", Vef, Ief);
	TM_HD44780_Puts(1, 0, string);
	sprintf(string, "P:%4.2lf W FP:%4.2lf ", P, FP);
	TM_HD44780_Puts(0, 1, string);
	sprintf(string, "Ps:%4.2lf VA ", Ps);
	TM_HD44780_Puts(0, 2, string);
	sprintf(string, "Pq:%4.2lf VAr   ", Pq);
	TM_HD44780_Puts(0, 3, string);
}

void DMA2_Stream0_IRQHandler(void) {
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET) {
        DMA_ready = 1;
	}

	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
}

void TIMER_Config(void) { // Timer3 config
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	PrescalerValue = (uint16_t)((SystemCoreClock / 2) / 200000) - 1;
    // TIM3=SystemCoreClock/2=84MHz -> Prescaler=419 -> 200kHz

	TIM_TimeBaseStructure.TIM_Period = (200000 / (50 * SAMPLES)) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    // This options sends an UPDATE that the ADC uses to trigger measure
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void NVIC_Config(DMA_PIN dma_pin) {
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Enable and set DMA2 Stream0 Interrupt to the lower priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = dma_pin.IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// Interrupts when array is full
	DMA_ITConfig(dma_pin.stream, DMA_IT_TC, ENABLE);
    // TC: Transfer Complete
}

void ADC1_Configuration(ADC_PIN adc_pin1, ADC_PIN adc_pin2, DMA_PIN dma_pin, uint32_t buffer[]) {
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd(dma_pin.peripheric, ENABLE);
	RCC_APB2PeriphClockCmd(adc_pin1.peripheric_adc, ENABLE);
	// DMA2_Stream0 channel0 configuration
	DMA_DeInit(dma_pin.stream);
	DMA_InitStructure.DMA_Channel = dma_pin.channel;                                // DMA, ADC and the pin are associated with stream and channel
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(adc_pin1.adc)->DR;       // Where to store values
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         // DMA transmits from peripheral to memory (from PIN to array)
	DMA_InitStructure.DMA_BufferSize = dma_pin.buffer_size;                         // Number of values to convert 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                // Disable it when an array address is defined
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         // Change to next cell when a value is stored
	DMA_InitStructure.DMA_PeripheralDataSize = dma_pin.peripheric_data_size;        // Size of bit in peripheric 
	DMA_InitStructure.DMA_MemoryDataSize = dma_pin.memory_data_size;                // Size of bit in memory
	DMA_InitStructure.DMA_Mode = dma_pin.mode;                                      // In this mode, after last value in array, restarts in initial position
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(dma_pin.stream, &DMA_InitStructure);
	// DMA2_Stream0 enable
	DMA_Cmd(dma_pin.stream, ENABLE);
	// ADC Common Init
	RCC_AHB1PeriphClockCmd(adc_pin1.peripheric_port, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = adc_pin1.pin | adc_pin2.pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(adc_pin1.port, &GPIO_InitStructure);
	// Config ADC
	RCC_APB2PeriphClockCmd(adc_pin1.peripheric_port, ENABLE);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	// ADC1 Init 
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                            // Scan mode means multiple channels will be converted simultaneously
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                                     // Continuous mode is to uncontrolled triggering, and here we are interrupting by clock
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;                  // What triggers ADC, in this case, timer 3
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;                                              // Number of ADC used in each convertion
	ADC_Init(adc_pin1.adc, &ADC_InitStructure);
	// ADC1 regular channel configuration
	ADC_RegularChannelConfig(adc_pin1.adc, adc_pin1.channel, 1, ADC_SampleTime_480Cycles);  // Third parameter defines order of convertion
	ADC_RegularChannelConfig(adc_pin1.adc, adc_pin2.channel, 2, ADC_SampleTime_480Cycles);
	// Enable DMA request after last transfer (Single-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(adc_pin1.adc, ENABLE);
	// Enable ADC1 DMA

	ADC_DMACmd(adc_pin1.adc, ENABLE);
	// Enable ADC1
	ADC_Cmd(adc_pin1.adc, ENABLE);
}
