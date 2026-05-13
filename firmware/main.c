#include "main.h"

// Buffers
uint16_t adc_buffer1[BUFFER_SIZE + 1] = {[0] = 0xAA55};
uint16_t adc_buffer2[BUFFER_SIZE + 1] = {[0] = 0xBB55};
uint16_t adc_buffer3[BUFFER_SIZE + 1] = {[0] = 0xCC55};
uint16_t adc_buffer4[BUFFER_SIZE + 1] = {[0] = 0xDD55};

uint32_t cnt = 0;

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ MAIN                                                                       │
//   └────────────────────────────────────────────────────────────────────────────┘
int main(void)
{
    RCC_Init();
    GPIO_Init();
    TIM15_DelayInit();
    TIM16_PWM_BurstInit();
    TIM4_TriggerInit();
    TIM1_GateForADCTimer(BUFFER_SIZE);
    TIM2_SlaveGateMode_TIM1();
    TIM3_RateCheckADC_EVTCheckTIM2();
    ADC1_DMA_TIM2_Config();
    USART2_DMA_TX_Init();

    delay_ms(10);

    GPIO_Set(PWR_ON_GPIO_Port, PWR_ON_Pin, 0);
    GPIO_Set(PWR_DRV0_GPIO_Port, PWR_DRV0_Pin, 1);
    GPIO_Set(PWR_DRV1_GPIO_Port, PWR_DRV1_Pin, 1);

    delay_ms(1);

    while (1)
    {
        uint16_t *buffers[4] = {
            adc_buffer1,
            adc_buffer2,
            adc_buffer3,
            adc_buffer4};


        for (int i = 0; i < 4; i++)
        {
            if (i == 0)
                south_out();
            else if (i == 1)
                north_out();
            else if (i == 2)
                west_out();
            else
                east_out();

            TIM_CNT(TIM2) = 0;
            TIM_CNT(TIM3) = 0;

            // Stop DMA
            DMA_CCR(DMA1, DMA_CHANNEL_1) &= ~(1 << 0);
            while (DMA_CCR(DMA1, DMA_CHANNEL_1) & 1);

            // Configure DMA
            DMA_CMAR(DMA1, DMA_CHANNEL_1) = (uint32_t)(buffers[i] + 1);
            DMA_IFCR(DMA1) = 0xF;
            DMA_CNDTR(DMA1, DMA_CHANNEL_1) = BUFFER_SIZE;

            // Start DMA
            DMA_CCR(DMA1, DMA_CHANNEL_1) |= 1;

            // Start ADC
            ADC_CR(ADC1) |= (1 << 2);

            TX_pulses(PULSE_COUNT);
            delay_ms(DELAY_RINGING);

            cnt = TIM_CNT(TIM3);
        }

        // -------- SEND ALL BUFFERS --------
        for (int i = 0; i < 4; i++)
        {
            DMA_CCR(DMA1, DMA_CHANNEL_7) &= ~(1 << 0);
            while (DMA_CCR(DMA1, DMA_CHANNEL_7) & 1);

            DMA_CMAR(DMA1, DMA_CHANNEL_7) = (uint32_t)buffers[i];
            DMA_CNDTR(DMA1, DMA_CHANNEL_7) = (BUFFER_SIZE + 1) * 2;

            DMA_CCR(DMA1, DMA_CHANNEL_7) |= 1;

            delay_ms(DELAY_BUFFER_TRANSMIT);
        }

        GPIO_Toggle(TP1_GPIO_Port, TP1_Pin);
        GPIO_Toggle(LED1_GPIO_Port, LED1_Pin);
        GPIO_Toggle(LED2_GPIO_Port, LED2_Pin);
        GPIO_Toggle(LED3_GPIO_Port, LED3_Pin);
    }
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ USART2                                                                     │
//   │ config      -> 921600 baud, no oversampling, tx DMA on                     │
//   │ function    -> transfer buffer over UART using DMA                         │
//   └────────────────────────────────────────────────────────────────────────────┘
void USART2_DMA_TX_Init(void)
{
    RCC_AHB2ENR |= (1 << 0);
    RCC_AHB1ENR |= (1 << 0);
    RCC_APB1ENR1 |= (1 << 17);

    // PA2 TX
    GPIO_MODER(GPIOA) &= ~(3 << (2 * 2));
    GPIO_MODER(GPIOA) |= (2 << (2 * 2));
    GPIO_OSPEEDR(GPIOA) |= (3 << (2 * 2));
    GPIO_AFRL(GPIOA) &= ~(0xF << (4 * 2));
    GPIO_AFRL(GPIOA) |= (7 << (4 * 2));

    USART_CR1(USART2) = 0;
    USART_BRR(USART2) = 87; // 921600 @ 80MHz
    USART_CR3(USART2) |= (1 << 7);
    USART_CR1(USART2) |= (1 << 3);
    USART_CR1(USART2) |= (1 << 0);

    DMA_CCR(DMA1, DMA_CHANNEL_7) &= ~1;
    while (DMA_CCR(DMA1, DMA_CHANNEL_7) & 1);

    DMA_CPAR(DMA1, DMA_CHANNEL_7) = (uint32_t)&USART_TDR(USART2);
    DMA_CMAR(DMA1, DMA_CHANNEL_7) = (uint32_t)adc_buffer1;
    DMA_CNDTR(DMA1, DMA_CHANNEL_7) = 2002;

    DMA_CSELR(DMA1) &= ~(0xF << (6 * 4));
    DMA_CSELR(DMA1) |= (0x2 << (6 * 4));

    DMA_CCR(DMA1, DMA_CHANNEL_7) =
        (1 << 7) | // MINC
        (1 << 4);  // DIR

    DMA_CCR(DMA1, DMA_CHANNEL_7) |= 1;
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ ADC1                                                                       │
//   │ config      -> DMA enabled, min sampling cycle, slave for TIM2             │
//   │ function    -> triggerred by TIM2 (1MSPS) and transfer to buffer over DMA  │
//   └────────────────────────────────────────────────────────────────────────────┘
void ADC1_DMA_TIM2_Config(void)
{
    RCC_AHB2ENR |= (1 << 13);
    RCC_AHB2ENR |= (1 << 0);
    RCC_AHB1ENR |= (1 << 0);

    GPIO_MODER(GPIOA) |= (3 << 0);
    GPIO_ASCR(GPIOA) |= (1 << 0);

    ADC_CR(ADC1) = 0;
    ADC_CR(ADC1) |= (1 << 28);
    delay_ms(1);

    ADC_CR(ADC1) |= (1 << 31);
    while (ADC_CR(ADC1) & (1 << 31))
        ;

    ADC_CFGR(ADC1) |= (1 << 10);
    ADC_CFGR(ADC1) |= (11 << 6);
    ADC_CFGR(ADC1) |= (1 << 0);

    ADC_SQR1(ADC1) |= (5 << 6);

    DMA_CCR(DMA1, DMA_CHANNEL_1) &= ~1;

    DMA_CPAR(DMA1, DMA_CHANNEL_1) = (uint32_t)&ADC_DR(ADC1);
    DMA_CMAR(DMA1, DMA_CHANNEL_1) = (uint32_t)(adc_buffer1 + 1);
    DMA_CNDTR(DMA1, DMA_CHANNEL_1) = BUFFER_SIZE;

    DMA_CCR(DMA1, DMA_CHANNEL_1) |= (1 << 7);  // MINC
    DMA_CCR(DMA1, DMA_CHANNEL_1) |= (1 << 8);  // PSIZE 16
    DMA_CCR(DMA1, DMA_CHANNEL_1) |= (1 << 10); // MSIZE 16

    ADC_CR(ADC1) |= 1;
    while (!(ADC_ISR(ADC1) & 1))
        ;

    ADC_CR(ADC1) |= (1 << 2);
}
void ADC1_RateCheck(void)
{
    TX_pulses(10);
    for (int i = 0; i < 100000; i++)
    {
        if (ADC_ISR(ADC1) & (1 << 2))
        {
            cnt++;
            ADC_ISR(ADC1) |= (1 << 2);
        }
    }
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM3                                                                       │
//   │ config      -> us tick, external clock mode                                │
//   │ function    -> uses adc event to calculate rate (precautions)              │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM3_RateCheckADC_EVTCheckTIM2()
{
    RCC_APB1ENR1 |= (1 << 1);

    TIM_CR1(TIM3) = 0;
    TIM_SMCR(TIM3) = 0;
    TIM_PSC(TIM3) = 0;      // no prescaler
    TIM_ARR(TIM3) = 0xFFFF; // max range
    TIM_CNT(TIM3) = 0;
    TIM_SMCR(TIM3) |= (7 << 0); // SMS = 111 ? External clock mode 1
    TIM_SMCR(TIM3) |= (1 << 4); // TS = 001 ? ITR1 (TIM2)
    TIM_CR1(TIM3) |= (1 << 0);  // CEN

    TIM_CNT(TIM2) = 0;
    TIM_CNT(TIM3) = 0;

    TX_pulses(10);
    delay_ms(5);
    cnt = TIM_CNT(TIM3);
    if (cnt < 995)
        SomethingsWrong();
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM1                                                                       │
//   │ config      -> us tick, trg0 goes to adc1, pwm 1 mode, slave for tim1      │
//   │ function    -> generates triggers for ADC (1MSPS) till tim1 is active      │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM2_SlaveGateMode_TIM1(void)
{
    // --- Enable clocks ---
    RCC_AHB2ENR |= (1 << 0);  // GPIOAEN
    RCC_APB1ENR1 |= (1 << 0); // TIM2EN

    // --- PA15 -> TIM2_CH1 (AF1) ---
    GPIO_MODER(GPIOA) &= ~(3 << (15 * 2));
    GPIO_MODER(GPIOA) |= (2 << (15 * 2));

    GPIO_AFRH(GPIOA) &= ~(0xF << ((15 - 8) * 4));
    GPIO_AFRH(GPIOA) |= (1 << ((15 - 8) * 4));

    GPIO_OSPEEDR(GPIOA) |= (3 << (15 * 2));

    // --- Timer base ---
    TIM_CR1(TIM2) = 0;

    TIM_PSC(TIM2) = 0;
    TIM_ARR(TIM2) = 79;
    TIM_CCR1(TIM2) = 40;

    // --- PWM Mode ---
    TIM_CCMR1(TIM2) = 0;
    TIM_CCMR1(TIM2) |= (6 << 4); // PWM1
    TIM_CCMR1(TIM2) |= (1 << 3); // preload

    TIM_CCER(TIM2) = 0;
    TIM_CCER(TIM2) |= (1 << 0);

    // --- Slave gated mode ---
    TIM_SMCR(TIM2) = 0;
    TIM_SMCR(TIM2) |= (5 << 0); // Gated
    TIM_SMCR(TIM2) |= (0 << 4); // ITR0 (TIM1)

    TIM_CR2(TIM2) &= ~(7 << 4);
    TIM_CR2(TIM2) |= (2 << 4); // MMS = 010 ? Update event as TRGO

    TIM_EGR(TIM2) |= 1;
    TIM_CR1(TIM2) |= (1 << 7); // ARPE
    TIM_CR1(TIM2) |= (1 << 0); // CEN
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM1                                                                       │
//   │ config      -> us tick, gated mode for tim2, pwm mode 1                    │
//   │ function    -> provides gate to make tim2 works for n us                   │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM1_GateForADCTimer(uint16_t pulse_us)
{
    RCC_AHB2ENR |= (1 << 0);  // GPIOAEN (correct)
    RCC_APB2ENR |= (1 << 11); // TIM1EN (CORRECTED - was (1<<0))

    GPIO_MODER(GPIOA) &= ~(3 << 16);
    GPIO_MODER(GPIOA) |= (2 << 16);
    GPIO_AFRH(GPIOA) &= ~(0xF << 0);
    GPIO_AFRH(GPIOA) |= (1 << 0);

    TIM_CR1(TIM1) = 0;
    TIM_PSC(TIM1) = 79;
    TIM_ARR(TIM1) = pulse_us + 1;
    TIM_CCR1(TIM1) = 1;
    TIM_EGR(TIM1) |= 1;
    TIM_SR(TIM1) &= ~1;

    TIM_SMCR(TIM1) = 0;
    TIM_SMCR(TIM1) |= (6 << 0);
    TIM_SMCR(TIM1) |= (3 << 4);

    TIM_CCMR1(TIM1) = 0;
    TIM_CCMR1(TIM1) |= (7 << 4);
    TIM_CCMR1(TIM1) |= (1 << 3);

    TIM_CR2(TIM1) &= ~(7 << 4);
    TIM_CR2(TIM1) |= (4 << 4);

    TIM_CCER(TIM1) = 0;
    TIM_CCER(TIM1) |= (1 << 0);

    TIM_CR2(TIM1) &= ~(1 << 8); // Idle LOW after pulse

    TIM_CR1(TIM1) |= (1 << 3); // OPM = 1
    TIM_CR1(TIM1) |= 1;        // CEN = 1

    (*(volatile uint32_t *)(TIM1 + 0x44)) |= (1 << 15); // MOE = 1
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM4                                                                       │
//   │ config -> us tick and generates trg0 update event for tim1                 │
//   │ function -> avoid silent zone and then trigger next timer                  │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM4_TriggerInit(void)
{
    RCC_APB1ENR1 |= (1 << 2); // TIM4 clock enable

    TIM_CR1(TIM4) = 0;                 // disable timer
    TIM_PSC(TIM4) = 79;                // 1 MHz tick (80MHz / 80)
    TIM_ARR(TIM4) = DELAY_SILENT_ZONE; // 200 us
    TIM_EGR(TIM4) |= 1;                // UG = latch ARR
    TIM_SR(TIM4) &= ~1;                // needed as writting egr makes update event to rewrite shadow register and thus interrupts will start firring as soon as initialized
    TIM_CR1(TIM4) |= (1 << 3);         // OPM = 1 (one pulse mode)

    TIM_CR2(TIM4) &= ~(0x7 << 4); // clear MMS
    TIM_CR2(TIM4) |= (2 << 4);    // MMS = 010 -> update event as TRGO
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM16                                                                      │
//   │ config      ->  40Khz frequency and 50% duty cycle                         │
//   │ TX_pulses   -> uses RCR to generate specific amount of pulses              │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM16_PWM_BurstInit()
{
    RCC_APB2ENR |= (1 << 17); // TIM16 clock enable
    RCC_AHB2ENR |= (1 << 0);  // GPIOA clock

    GPIO_MODER(GPIOA) &= ~(3 << (6 * 2));
    GPIO_MODER(GPIOA) |= (2 << (6 * 2)); // AF mode
    GPIO_AFRL(GPIOA) &= ~(0xF << (6 * 4));
    GPIO_AFRL(GPIOA) |= (14 << (6 * 4)); // AF14 (TIM16_CH1)

    TIM_CR1(TIM16) = 0;    // disable timer
    TIM_PSC(TIM16) = 1;    // 1 MHz tick
    TIM_ARR(TIM16) = 999;  // period
    TIM_CCR1(TIM16) = 500; // duty
    TIM_RCR(TIM16) = 9;    // 10 pulse burst
    TIM_CCMR1(TIM16) &= ~(0xFF);
    TIM_CCMR1(TIM16) |= (7 << 4); // PWM mode 2
    TIM_CCMR1(TIM16) |= (1 << 3); // preload enable
    TIM_CCER(TIM16) |= 1;         // enable CH1
    TIM_BDTR(TIM16) |= (1 << 15); // MOE
    TIM_CR1(TIM16) |= (1 << 3);
    TIM_DIER(TIM16) |= 1; // UIE
    TIM_EGR(TIM16) |= 1;  // latch registers
    TIM_SR(TIM16) &= ~1;  // ? clear the spurious UIF BEFORE enabling NVIC
    NVIC_EnableIRQ(TIM16_IRQn);
}
inline static void TX_pulses(uint16_t pulses)
{
    TIM_CR1(TIM16) &= ~1; // stop timer first
    TIM_RCR(TIM16) = pulses - 1;
    TIM_CNT(TIM16) = 0;
    TIM_SR(TIM16) &= ~1; // clear any pending UIF
    TIM_EGR(TIM16) |= 1; // latch RCR value
    TIM_SR(TIM16) &= ~1; // clear UIF caused by EGR
    TIM_CR1(TIM16) |= 1; // start
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM15                                                                      │
//   │ config      ->  for normal counting at 1us per tick                        │
//   │ delay_us    -> uses the config tick for delay in USART2                    │
//   │ delay_ms    -> uses the same delay_us to generate ms delay                 │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM15_DelayInit()
{
    RCC_APB2ENR |= (1 << 16); // TIM15 clock enable (APB2ENR, bit 16)

    TIM_CR1(TIM15) = 0;      // disable timer during setup
    TIM_PSC(TIM15) = 79;     // prescaler: 80MHz / (79+1) = 1MHz ? 1us per tick
    TIM_ARR(TIM15) = 0xFFFF; // max 16-bit auto-reload
    TIM_EGR(TIM15) |= 1;     // UG = 1, generate update event
    TIM_CR1(TIM15) |= 1;     // CEN = 1, start TIM15
}
void delay_us(uint16_t us)
{
    TIM_CNT(TIM15) = 0;
    while (TIM_CNT(TIM15) < us)
        ;
}
void delay_ms(uint16_t ms)
{
    for (int i = 0; i < ms; i++)
    {
        delay_us(1000);
    }
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ RCC                                                                        │
//   │ system clock    -> 80Mhz from 24Mhz via PLL                                │
//   │ ADC clock       -> enable PLLSAI1 to drive it on 80Mh                      │
//   └────────────────────────────────────────────────────────────────────────────┘
void RCC_Init(void)
{
    RCC_APB1ENR1 |= (1 << 28);

    PWR_CR1 &= ~(3 << 9); // clear VOS[1:0]
    PWR_CR1 |= (1 << 9);  // VOS = 0b01 = Range 1
    while (PWR_SR2 & (1 << 10));

    RCC_CR |= (1 << 16); // HSEON
    while (!(RCC_CR & (1 << 17)));

    FLASH_ACR = (4 << 0);   // LATENCY = 4 WS
    RCC_CR &= ~(1 << 24);   // PLLON = 0
    while (RCC_CR & (1 << 25));

    RCC_PLLCFGR =
        (3 << 0) |  // PLLSRC = HSE
        (5 << 4) |  // PLLM   = 5 ? divider of /6 (PLLM+1 on STM32L4)
        (40 << 8) | // PLLN   = 40
        (0 << 25) | // PLLR   = 0 ? divider of /2
        (1 << 24);  // PLLREN = enable PLLR output

    RCC_CR |= (1 << 24); // PLLON
    while (!(RCC_CR & (1 << 25)))
        ; // wait for PLLRDY

    RCC_CFGR &= ~(0xF << 4);  // HPRE  = /1  (AHB)
    RCC_CFGR &= ~(0x7 << 8);  // PPRE1 = /1  (APB1)
    RCC_CFGR &= ~(0x7 << 11); // PPRE2 = /1  (APB2)

    RCC_CFGR &= ~(3 << 0); // clear SW
    RCC_CFGR |= (3 << 0);  // SW = 0b11 = PLL as SYSCLK
    while (((RCC_CFGR >> 2) & 3) != 3);

    RCC_CR &= ~(1 << 26); // PLLSAI1ON = 0
    while (RCC_CR & (1 << 27));

    RCC_PLLSAI1CFGR =
        (3 << 0) |  // PLLSRC = HSE
        (5 << 4) |  // PLLM = /6  ? 24/6 = 4 MHz input
        (40 << 8) | // PLLN = 24  ? 4*40 = 160 MHz VCO
        (0 << 25) | // PLLR = /2  ? 160/2 = 80 MHz ADC clock
        (1 << 24);  // PLLREN enable

    RCC_CR |= (1 << 26); // PLLSAI1ON
    while (!(RCC_CR & (1 << 27)));

    RCC_CCIPR &= ~(3 << 28);
    RCC_CCIPR |= (1 << 28); // ADCSEL = PLLSAI1
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ GPIO                                                                       │
//   │ config      -> no alternate functionality (normal output mode)             │
//   │ init        -> initialize gpio as per macros and pcb                       │
//   │ set/toggle  -> modify gpio pins                                            │
//   └────────────────────────────────────────────────────────────────────────────┘
void GPIO_Config(uint32_t PORT, uint8_t PIN) //	only to set GPIOs output with max speed and pull down with push pull configuration
{
    switch (PORT)
    {
    case GPIOA:
        RCC_AHB2ENR |= 1 << 0;
        break;
    case GPIOB:
        RCC_AHB2ENR |= 1 << 1;
        break;
    case GPIOC:
        RCC_AHB2ENR |= 1 << 2;
        break;
    case GPIOD:
        RCC_AHB2ENR |= 1 << 3;
        break;
    case GPIOE:
        RCC_AHB2ENR |= 1 << 4;
        break;
    case GPIOF:
        RCC_AHB2ENR |= 1 << 5;
        break;
    case GPIOG:
        RCC_AHB2ENR |= 1 << 6;
        break;
    case GPIOH:
        RCC_AHB2ENR |= 1 << 7;
        break;
    }
    GPIO_MODER(PORT)    &= ~(3 << (PIN * 2));   // clear bits
    GPIO_MODER(PORT)    |= (1 << (PIN * 2));    // set output mode
    GPIO_OTYPER(PORT)   &= ~(1 << PIN);         // 0 = push-pull
    GPIO_OSPEEDR(PORT)  &= ~(3 << (PIN * 2));   // high speed
    GPIO_OSPEEDR(PORT)  |= (3 << (PIN * 2));
    GPIO_PUPDR(PORT)    &= ~(3 << (PIN * 2));   // pull down
    GPIO_PUPDR(PORT)    |= (2 << (PIN * 2));
}
void GPIO_Init()
{
    GPIO_Config(PWR_ON_GPIO_Port, PWR_ON_Pin);
    GPIO_Config(PWR_DRV0_GPIO_Port, PWR_DRV0_Pin);
    GPIO_Config(PWR_DRV1_GPIO_Port, PWR_DRV1_Pin);
    GPIO_Config(SWITCH_A_GPIO_Port, SWITCH_A_Pin);
    GPIO_Config(SWITCH_B_GPIO_Port, SWITCH_B_Pin);
    GPIO_Config(LED1_GPIO_Port, LED1_Pin);
    GPIO_Config(LED2_GPIO_Port, LED2_Pin);
    GPIO_Config(LED3_GPIO_Port, LED3_Pin);
    GPIO_Config(TP1_GPIO_Port, TP1_Pin);
    GPIO_Config(TP2_GPIO_Port, TP2_Pin);
}
static inline void GPIO_Set(uint32_t PORT, uint8_t PIN, uint8_t SET1_RESET0)
{
    GPIO_BSRR(PORT) = 1 << (PIN + (SET1_RESET0 ? 0 : 16));
}
static inline void GPIO_Toggle(uint32_t PORT, uint8_t PIN)
{
    GPIO_ODR(PORT) ^= 1 << PIN;
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ DEBUG                                                                      │
//   │ add function where the error may occur                                     │
//   └────────────────────────────────────────────────────────────────────────────┘
void SomethingsWrong()
{
    while (1) //change this as per the required check functionality
    {
        GPIO_Toggle(LED1_GPIO_Port, LED1_Pin);
        GPIO_Toggle(LED2_GPIO_Port, LED2_Pin);
        GPIO_Toggle(LED3_GPIO_Port, LED3_Pin);
        delay_ms(30);
        break;
    }
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ ISR                                                                        │
//   │ TIM1_16 -> start tim4 to complete the trigger chain (tim16 dont have trg0) │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM1_UP_TIM16_IRQHandler()
{
    if (TIM_SR(TIM16) & 1)
    {
        TIM_SR(TIM16) &= ~1; // clear TIM16 UIF first
        TIM_CR1(TIM4) &= ~1; // ensure timer stopped
        TIM_CNT(TIM4) = 0;   // reset counter
        TIM_SR(TIM4) &= ~1;  // clear any pending update flag
        TIM_CR1(TIM4) |= 1;  // start TIM4
    }
}