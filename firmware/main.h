#include "stdint.h"

/* -- Base addresses --------------------------------------- */
#define RCC_BASE        0x40021000UL
#define SYSCFG				0x40010000UL
#define ADC					0x50040000UL
#define USART2				0x40004400UL


/* -- GPIO port base addresses ----------------------------- */
#define GPIOA  0x48000000UL
#define GPIOB  0x48000400UL
#define GPIOC  0x48000800UL
#define GPIOD  0x48000C00UL
#define GPIOE  0x48001000UL
#define GPIOF  0x48001400UL
#define GPIOG  0x48001800UL
#define GPIOH  0x48001C00UL

/* -- TIM port base addresses ----------------------------- */
#define TIM3   0x40000400UL	
#define TIM1   0x40012C00UL
#define TIM15 	0x40014000UL
#define TIM16      0x40014400UL
#define TIM2   0x40000000UL   // General-purpose 32-bit timer
#define TIM4   0x40000800UL   // General-purpose 16-bit timer


/* -- PWR port base addresses ----------------------------- */
#define PWR_BASE  0x40007000UL

/* -- GPIO register offsets -------------------------------- */
#define GPIO_MODER(base)   (*(volatile uint32_t *)((base) + 0x00))
#define GPIO_OTYPER(base)  (*(volatile uint32_t *)((base) + 0x04))
#define GPIO_OSPEEDR(base) (*(volatile uint32_t *)((base) + 0x08))
#define GPIO_PUPDR(base)   (*(volatile uint32_t *)((base) + 0x0C))
#define GPIO_BSRR(base)    (*(volatile uint32_t *)((base) + 0x18)) 
#define GPIO_AFRL(base)		(*(volatile uint32_t *)((base) + 0x20)) 
#define GPIO_AFRH(base)		(*(volatile uint32_t *)((base) + 0x24)) 
#define GPIO_ODR(base)		(*(volatile uint32_t *)((base) + 0x14)) 
#define GPIO_ASCR(base)		(*(volatile uint32_t *)((base) + 0x2C)) 
	
/* -- RCC register offsets -------------------------------- */
#define RCC_CR        (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR   (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1		(*(volatile uint32_t *)(RCC_BASE + 0x58))
#define RCC_APB2ENR		(*(volatile uint32_t *)(RCC_BASE + 0x60))
#define RCC_AHB1ENR		(*(volatile uint32_t *)(RCC_BASE + 0x48))
#define RCC_CCIPR			(*(volatile uint32_t *)(RCC_BASE + 0x88))
#define RCC_PLLSAI1CFGR (*(volatile uint32_t *)(RCC_BASE + 0x10))

/* -- FLASH register offsets -------------------------------- */
#define FLASH_ACR     (*(volatile uint32_t *)(0x40022000UL + 0x00))
	
/* -- TIM register offsets -------------------------------- */
#define TIM_CR1(base)    (*(volatile uint32_t *)((base) + 0x00))
#define TIM_CR2(base)    (*(volatile uint32_t *)((base) + 0x04))
#define TIM_SMCR(base)   (*(volatile uint32_t *)((base) + 0x08))
#define TIM_DIER(base)   (*(volatile uint32_t *)((base) + 0x0C))
#define TIM_SR(base)     (*(volatile uint32_t *)((base) + 0x10))
#define TIM_EGR(base)    (*(volatile uint32_t *)((base) + 0x14))
#define TIM_CCMR1(base)  (*(volatile uint32_t *)((base) + 0x18))
#define TIM_CCMR2(base)  (*(volatile uint32_t *)((base) + 0x1C))
#define TIM_CCER(base)   (*(volatile uint32_t *)((base) + 0x20))
#define TIM_CNT(base)    (*(volatile uint32_t *)((base) + 0x24))
#define TIM_PSC(base)    (*(volatile uint32_t *)((base) + 0x28))
#define TIM_RCR(base)	 (*(volatile uint32_t *)((base) + 0x30))
#define TIM_ARR(base)    (*(volatile uint32_t *)((base) + 0x2C))
#define TIM_CCR1(base) 	 (*(volatile uint32_t *)((base) + 0x34))
#define TIM_CCR2(base) 	 (*(volatile uint32_t *)((base) + 0x38))
#define TIM_CCR3(base) 	 (*(volatile uint32_t *)((base) + 0x3C))
#define TIM_CCR4(base) 	 (*(volatile uint32_t *)((base) + 0x40))
#define TIM_BDTR(base)   (*(volatile uint32_t *)((base) + 0x44))
	
#define PWR_CR1   (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CR2   (*(volatile uint32_t *)(PWR_BASE + 0x04))
#define PWR_SR1   (*(volatile uint32_t *)(PWR_BASE + 0x10))
#define PWR_SR2   (*(volatile uint32_t *)(PWR_BASE + 0x14))
	
#define ADC1		        0x50040000UL
// below are only for master ADC1 -> for adc2/3 look at the register map
#define ADC_ISR(base)         (*(volatile uint32_t *)((base) + 0x00))
#define ADC_IER(base)         (*(volatile uint32_t *)((base) + 0x04))
#define ADC_CR(base)         (*(volatile uint32_t *)((base) + 0x08))
#define ADC_CFGR(base)      (*(volatile uint32_t *)((base) + 0x0C))
#define ADC_CFGR2(base)      (*(volatile uint32_t *)((base) + 0x10))
#define ADC_SMPR1(base)      (*(volatile uint32_t *)((base) + 0x14))
#define ADC_SMPR2(base)      (*(volatile uint32_t *)((base) + 0x18))
#define ADC_TR1(base)      (*(volatile uint32_t *)((base) + 0x20))
#define ADC_TR2(base)      (*(volatile uint32_t *)((base) + 0x24))
#define ADC_TR3(base)      (*(volatile uint32_t *)((base) + 0x28))
#define ADC_SQR1(base)      (*(volatile uint32_t *)((base) + 0x30))
#define ADC_SQR2(base)      (*(volatile uint32_t *)((base) + 0x34))
#define ADC_SQR3(base)      (*(volatile uint32_t *)((base) + 0x38))
#define ADC_SQR4(base)      (*(volatile uint32_t *)((base) + 0x3C))
#define ADC_DR(base)      	 (*(volatile uint32_t *)((base) + 0x40))
#define ADC_JSQR(base)      (*(volatile uint32_t *)((base) + 0x4C))
#define ADC_OFR1(base)      (*(volatile uint32_t *)((base) + 0x60))
#define ADC_OFR2(base)      (*(volatile uint32_t *)((base) + 0x64))
#define ADC_OFR3(base)      (*(volatile uint32_t *)((base) + 0x68))
#define ADC_OFR4(base)      (*(volatile uint32_t *)((base) + 0x6C))
#define ADC_JDR1(base)      (*(volatile uint32_t *)((base) + 0x80))
#define ADC_JDR2(base)      (*(volatile uint32_t *)((base) + 0x84))
#define ADC_JDR3(base)      (*(volatile uint32_t *)((base) + 0x88))
#define ADC_JDR4(base)      (*(volatile uint32_t *)((base) + 0x8C))
#define ADC_AWD2CR(base)      (*(volatile uint32_t *)((base) + 0xA0))
#define ADC_AWD3CR(base)      (*(volatile uint32_t *)((base) + 0xA4))
#define ADC_DIFSEL(base)      (*(volatile uint32_t *)((base) + 0xB0))
#define ADC_CALFACT(base)     (*(volatile uint32_t *)((base) + 0xB4))
#define ADC_CCR(base)     (*(volatile uint32_t *)((base) + 0x308))
	
#define DMA1      0x40020000UL

// ==========================
// GLOBAL REGISTERS
// ==========================
#define DMA_ISR(base)      (*(volatile uint32_t *)((base) + 0x00))
#define DMA_IFCR(base)     (*(volatile uint32_t *)((base) + 0x04))

// ==========================
// CHANNEL 1
// ==========================
#define DMA_CCR1(base)     (*(volatile uint32_t *)((base) + 0x08))
#define DMA_CNDTR1(base)   (*(volatile uint32_t *)((base) + 0x0C))
#define DMA_CPAR1(base)    (*(volatile uint32_t *)((base) + 0x10))
#define DMA_CMAR1(base)    (*(volatile uint32_t *)((base) + 0x14))

// ==========================
// CHANNEL 2
// ==========================
#define DMA_CCR2(base)     (*(volatile uint32_t *)((base) + 0x1C))
#define DMA_CNDTR2(base)   (*(volatile uint32_t *)((base) + 0x20))
#define DMA_CPAR2(base)     (*(volatile uint32_t *)((base) + 0x24))
#define DMA_CMAR2(base)    (*(volatile uint32_t *)((base) + 0x28))

// ==========================
// CHANNEL 3
// ==========================
#define DMA_CCR3(base)     (*(volatile uint32_t *)((base) + 0x30))
#define DMA_CNDTR3(base)   (*(volatile uint32_t *)((base) + 0x34))
#define DMA_CPAR3(base)    (*(volatile uint32_t *)((base) + 0x38))
#define DMA_CMAR3(base)    (*(volatile uint32_t *)((base) + 0x3C))

// ==========================
// CHANNEL 4
// ==========================
#define DMA_CCR4(base)     (*(volatile uint32_t *)((base) + 0x44))
#define DMA_CNDTR4(base)   (*(volatile uint32_t *)((base) + 0x48))
#define DMA_CPAR4(base)    (*(volatile uint32_t *)((base) + 0x4C))
#define DMA_CMAR4(base)    (*(volatile uint32_t *)((base) + 0x50))

// ==========================
// CHANNEL 5
// ==========================
#define DMA_CCR5(base)     (*(volatile uint32_t *)((base) + 0x58))
#define DMA_CNDTR5(base)   (*(volatile uint32_t *)((base) + 0x5C))
#define DMA_CPAR5(base)    (*(volatile uint32_t *)((base) + 0x60))
#define DMA_CMAR5(base)    (*(volatile uint32_t *)((base) + 0x64))

// ==========================
// CHANNEL 6
// ==========================
#define DMA_CCR6(base)     (*(volatile uint32_t *)((base) + 0x6C))
#define DMA_CNDTR6(base)   (*(volatile uint32_t *)((base) + 0x70))
#define DMA_CPAR6(base)    (*(volatile uint32_t *)((base) + 0x74))
#define DMA_CMAR6(base)    (*(volatile uint32_t *)((base) + 0x78))

// ==========================
// CHANNEL 7
// ==========================
#define DMA_CCR7(base)     (*(volatile uint32_t *)((base) + 0x80))
#define DMA_CNDTR7(base)   (*(volatile uint32_t *)((base) + 0x84))
#define DMA_CPAR7(base)    (*(volatile uint32_t *)((base) + 0x88))
#define DMA_CMAR7(base)    (*(volatile uint32_t *)((base) + 0x8C))

#define SYSCFG_CFGR1		(*(volatile uint32_t *)(SYSCFG + 0x04))
	
#define NVIC_ISER0       (*(volatile uint32_t *)0xE000E100UL)
#define TIM16_IRQn       25

#define NVIC_EnableIRQ(x) (NVIC_ISER0 |= (1 << (x)))

#define LED1_Pin 				2
#define LED2_Pin 				3
#define ECHO_OUT_Pin 		0
#define TP2_Pin 				4
#define PWR_DRV0_Pin 		5
#define SWITCH_B_Pin 		7
#define SWITCH_A_Pin 		0
#define PWR_ON_Pin 			1
#define PWR_DRV1_Pin 		2
#define TP1_Pin 				12
#define LED3_Pin 				6

#define PWR_DRV0_GPIO_Port 	GPIOA
#define TP2_GPIO_Port 			GPIOA
#define ECHO_OUT_GPIO_Port 	GPIOA
#define LED2_GPIO_Port 			GPIOC
#define LED1_GPIO_Port 			GPIOC
#define LED3_GPIO_Port 			GPIOC
#define TP1_GPIO_Port 			GPIOB
#define PWR_DRV1_GPIO_Port 	GPIOB
#define SWITCH_A_GPIO_Port		GPIOB
#define PWR_ON_GPIO_Port 		GPIOB
#define SWITCH_B_GPIO_Port 	GPIOA

#define DMA1_CCR1   (*(volatile uint32_t *)0x40020008)
#define DMA1_CNDTR1 (*(volatile uint32_t *)0x4002000C)
#define DMA1_CPAR1  (*(volatile uint32_t *)0x40020010)
#define DMA1_CMAR1  (*(volatile uint32_t *)0x40020014)
#define DMA1_CSELR  (*(volatile uint32_t*)0x400200A8)

#define DMA1_EN     (1 << 0)
#define DMA1_CIRC   (1 << 7)
#define DMA1_MINC   (1 << 7)
#define DMA1_PL     (1 << 5)

#define ADC_DMAEN   (1 << 0)
#define ADC_DMACFG  (1 << 1)

#define DMA1_BASE        0x40020000UL

#define USART_SR(USARTx)   (*(volatile uint32_t *)(USARTx + 0x00))
#define USART_TDR(USARTx)  (*(volatile uint32_t *)(USARTx + 0x28))
#define USART_BRR(USARTx)  (*(volatile uint32_t *)(USARTx + 0x0C))
#define USART_CR1(USARTx)  (*(volatile uint32_t *)(USARTx + 0x00))
#define USART_CR2(USARTx)  (*(volatile uint32_t *)(USARTx + 0x10))
#define USART_CR3(USARTx)  (*(volatile uint32_t *)(USARTx + 0x08))
#define USART_GTPR(USARTx) (*(volatile uint32_t *)(USARTx + 0x18))
	
#define BUFFER_SIZE 				250
#define PULSE_COUNT 				8
#define DELAY_SILENT_ZONE 		70
#define BAUDRATE 					921600
#define DELAY_BUFFER_TRANSMIT 15 //(((((BUFFER_SIZE + 1) * 2) * 10) / BAUDRATE) + 5) //10 to conside stop start bits and 3 for added safety
#define DELAY_RINGING			20 //should consider time required for sampling too (1ms for sampling RN)

void USART2_DMA_TX_Init(void);

void TIM3_RateCheckADC_EVTCheckTIM2();

void ADC1_DMA_TIM2_Config(void);
void ADC1_RateCheck(void);

void TIM2_SlaveGateMode_TIM1(void);

void TIM4_TriggerInit();

void TIM1_GateForADCTimer(uint16_t samples);

void TIM16_PWM_BurstInit();
inline static void TX_pulses(uint16_t pulses);

void TIM15_DelayInit();
inline static void delay_us(uint16_t us);
inline static void delay_ms(uint16_t ms);
	
void GPIO_Config(uint32_t PORT, uint8_t PIN);
void GPIO_Init();
static inline void GPIO_Set(uint32_t PORT, uint8_t PIN, uint8_t SET1_RESET0);
static inline void GPIO_Toggle(uint32_t PORT, uint8_t PIN);

void RCC_Init();

void SomethingsWrong();