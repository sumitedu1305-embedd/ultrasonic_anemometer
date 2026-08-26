#include "main.h"

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ VARIABLE DEFINITION                                                        │
//   └────────────────────────────────────────────────────────────────────────────┘

// buffers
uint16_t adc_buffer1[BUFFER_SIZE + 1] = {[0] = 0xAA55};
uint16_t adc_buffer2[BUFFER_SIZE + 1] = {[0] = 0xBB55};
uint16_t adc_buffer3[BUFFER_SIZE + 1] = {[0] = 0xCC55};
uint16_t adc_buffer4[BUFFER_SIZE + 1] = {[0] = 0xDD55};
uint16_t temp_buffer[10] = {0};

// debug variable for frequency estimation
uint32_t cnt = 0;
uint8_t temp_buff_index = 0;

// temperature definition
float current_temp_c = 25.0;
volatile uint16_t raw_temp_adc = 0;
float sound_speed_wrt_temp = 0;
float calib_bias_offset[CALIB_SAMPLES] = {0};
int count_calib_offset = 0;
float calib_bias = 0.0f;

void SystemInit(void) 
{
    SCB_CPACR |= (0xF << 20);  // enable full access to FPU coprocessors CP10/CP11
    SCB_VTOR = 0x08004000;      // tell CPU vector table moved here
    // intentionally added — clock/RCC setup already handled inside main() via RCC_Init()
    // otherwise there would be error when running startup as it contains SystemInit() in ResetHandler()
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ MAIN                                                                       │
//   └────────────────────────────────────────────────────────────────────────────┘

//issue in south out and west out

int main(void)
{
    RCC_Init();
    GPIO_Init();

    TIM16_DelayInit();
    TIM1_PWM_BurstInit();
    TIM4_TriggerInit();
    TIM3_GateForADCTimer(BUFFER_SIZE);
    TIM2_SlaveGateMode_TIM3();
    ADC1_DMA_TIM2_Config();
    USART2_DMA_TX_Init();
    ADC2_TemperatureInit();

    delay_ms(10);

    GPIO_Set(PWR_ON_GPIO_Port, PWR_ON_Pin, 0);

    delay_ms(10);

    int cnt_led = 0;
		
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
            
            delay_ms(1);

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

        Process_Temperature_Math(0,24.73);

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

        // send the temperature readings over the uart
        static uint16_t temp_packet[3];
        temp_packet[0] = 0xEE55;                          // header marker (consistent with ADC buffers)
        memcpy(&temp_packet[1], &sound_speed_wrt_temp, sizeof(float));  // 4 bytes = 2 x uint16_t

        DMA_CCR(DMA1, DMA_CHANNEL_7) &= ~(1 << 0);
        while (DMA_CCR(DMA1, DMA_CHANNEL_7) & 1);

        DMA_CMAR(DMA1, DMA_CHANNEL_7) = (uint32_t)temp_packet;
        DMA_CNDTR(DMA1, DMA_CHANNEL_7) = sizeof(temp_packet);  // 6 bytes

        DMA_CCR(DMA1, DMA_CHANNEL_7) |= 1;
        

        if(cnt_led == 0)
        {
            GPIO_Toggle(LED1_GPIO_Port, LED1_Pin);
            cnt_led++;
        }
        else if(cnt_led == 1)
        {
				GPIO_Toggle(LED2_GPIO_Port, LED2_Pin);
            cnt_led = 0;
        }
    }
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ ADC2                                                                       │
//   │ config -> slower 640.5 cycles sampling rate, circular mode along with dma  │
//   └────────────────────────────────────────────────────────────────────────────┘

void ADC2_TemperatureInit(void)
{
    // enable Clock for GPIOA and ADC
    RCC_AHB2ENR |= (1 << 0);       
    RCC_AHB2ENR |= (1 << 13);     

    // PA1 as Analog Mode
    GPIO_MODER(GPIOA) |= (3 << (1 * 2));    // Set PA1 to 11 (Analog)
    GPIO_PUPDR(GPIOA) &= ~(3 << (1 * 2));   // No pull-up/pull-down
    GPIO_ASCR(GPIOA)  |= (1 << 1);

    // dma config to auto store adc value into variable
    DMA_CCR(DMA1, DMA_CHANNEL_2)    = 0;
    DMA_CPAR(DMA1, DMA_CHANNEL_2)   = (uint32_t)&ADC_DR(ADC2);
    DMA_CMAR(DMA1, DMA_CHANNEL_2)   = (uint32_t)&raw_temp_adc;
    DMA_CNDTR(DMA1, DMA_CHANNEL_2)  = 1;
    DMA_CSELR(DMA1)                 &= ~(0xF << 4);
    DMA_CCR(DMA1, DMA_CHANNEL_2)    |= (1 << 10) | (1 << 8) | (1 << 5);
    DMA_CCR(DMA1, DMA_CHANNEL_2)    |= (1 << 0); 

    // adc config and calibration
    ADC_CR(ADC2) &= ~(1 << 29);             // DEEPPWD = 0
    ADC_CR(ADC2) |= (1 << 28);              // ADVREGEN = 1
    delay_ms(1);                            // Wait for regulator to stabilize
    ADC_CR(ADC2) |= (1 << 31);              // ADCAL = 1
    while (ADC_CR(ADC2) & (1 << 31));       // Wait for calibration to finish
    ADC_SQR1(ADC2) &= ~(0xF << 0);          // L = 0 (1 conversion per sequence)
    ADC_SQR1(ADC2) &= ~(0x1F << 6);         // Clear SQ1
    ADC_SQR1(ADC2) |= (6 << 6);             // SQ1 = Channel 6 (PA1)
    ADC_SMPR1(ADC2) |= (7 << (3 * 6));   
    ADC_CFGR(ADC2) |= (1 << 13);            // CONT = 1 (Continuous Conversion Mode)
    ADC_CFGR(ADC2) |= (1 << 1);             // DMACFG = 1 (DMA Circular Mode)
    ADC_CFGR(ADC2) |= (1 << 0);             // DMAEN = 1 (Enable DMA request generation)
    ADC_ISR(ADC2) |= (1 << 0);              // Clear ADRDY
    ADC_CR(ADC2) |= (1 << 0);               // ADEN = 1
    while (!(ADC_ISR(ADC2) & (1 << 0)));    // Wait until ADC is ready
    ADC_CR(ADC2) |= (1 << 2);               // ADSTART = 1
}
void Process_Temperature_Math(bool calib, float ref) 
{
	// pt100 is non linear actually
	// but non linear terms start affecting after high temps
	// thus they can be ignored
	// resolution required would not be greater than 0.1 degrees (I am targetting 1 degree right now)
	// R(t) = R_0(1 + At + Bt^2)a
	// R0 = 100 | A = 0.0039083 | B = -5.775 * 10^{-7}
    // current_temp_c = ((float)raw_temp_adc * 0.00540265) + 22.02146f;//27.1f + ((float)(raw_temp_adc - 662) * 0.06000f);
    // sound_speed_wrt_temp = 331.3f + (0.606f * current_temp_c);

    // 1. Voltage conversion (assuming 12-bit ADC and 3.3V reference)
    float vout = ((float)raw_temp_adc / 4095.0f) * 3.3f;

    // 2. Bridge / amplifier reverse transfer function
    float vp = (vout + 33.0f) / 111.0f;

    // Guard against division by zero
    if (fabsf(vp - 3.3f) < 1e-4f) return;
    float rt = (2970.0f - 10900.0f * vp) / (vp - 3.3f);

    // 3. Temperature calculation
    // Linear approximation is accurate to <0.5 deg C between -20C and 100C
    float current_temp_c = (rt - 100.0f) / 0.385055f;

    // 4. Calibration & Bias handling
    if (calib)
    {
        current_temp_c = current_temp_c * GAIN_OFFSET;
        
        if (count_calib_offset < CALIB_SAMPLES)
        {
            calib_bias_offset[count_calib_offset++] = current_temp_c - ref;
        }

        if (count_calib_offset >= CALIB_SAMPLES)
        {
            float sum = 0.0f;
            for (int i = 0; i < CALIB_SAMPLES; i++)
            {
                sum += calib_bias_offset[i];
            }
            calib_bias = sum / (float)CALIB_SAMPLES;
            count_calib_offset = 0;
        }
        current_temp_c -= calib_bias; // Subtract offset to match reference
    }
    else
    {
        current_temp_c = (current_temp_c * GAIN_OFFSET) - BIAS_OFFSET;
    }

    // 5. Sound speed calculation wrt temperature
    sound_speed_wrt_temp = 331.3f + (0.606f * current_temp_c);
    temp_buffer[temp_buff_index] = sound_speed_wrt_temp;
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
//   │ TIM2                                                                       │
//   │ config      -> us tick, trg0 goes to adc1, pwm 1 mode, slave for tim1      │
//   │ function    -> generates triggers for ADC (1MSPS) till tim1 is active      │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM2_SlaveGateMode_TIM3(void)
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
    TIM_SMCR(TIM2) |= (2 << 4); // ITR2 (TIM1)

    TIM_CR2(TIM2) &= ~(7 << 4);
    TIM_CR2(TIM2) |= (2 << 4); // MMS = 010 ? Update event as TRGO

    TIM_EGR(TIM2) |= 1;
    TIM_CR1(TIM2) |= (1 << 7); // ARPE
    TIM_CR1(TIM2) |= (1 << 0); // CEN
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM3                                                                       │
//   │ config      -> us tick, gated mode for tim2, pwm mode 1                    │
//   │ function    -> provides gate to make tim2 works for n us                   │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM3_GateForADCTimer(uint16_t pulse_us)
{
    RCC_AHB2ENR |= (1 << 1);   // GPIOBEN (bit 1, not bit 0 — that's GPIOA)
    RCC_APB1ENR1 |= (1 << 1);  // TIM3EN

    GPIO_MODER(GPIOB) &= ~(3 << (4 * 2));
    GPIO_MODER(GPIOB) |= (2 << (4 * 2)); // AF mode, PB4
    GPIO_AFRL(GPIOB) &= ~(0xF << (4 * 4));
    GPIO_AFRL(GPIOB) |= (2 << (4 * 4));  // AF2 (TIM3_CH1)

    TIM_CR1(TIM3) = 0;
    TIM_PSC(TIM3) = 79;
    TIM_ARR(TIM3) = pulse_us + 1;
    TIM_CCR1(TIM3) = 1;
    TIM_EGR(TIM3) |= 1;
    TIM_SR(TIM3) &= ~1;

    TIM_SMCR(TIM3) = 0;
    TIM_SMCR(TIM3) |= (6 << 0); // SMS = 110, trigger mode
    TIM_SMCR(TIM3) |= (3 << 4); // TS = 011 -> ITR3 = TIM4

    TIM_CCMR1(TIM3) = 0;
    TIM_CCMR1(TIM3) |= (7 << 4);
    TIM_CCMR1(TIM3) |= (1 << 3);

    TIM_CR2(TIM3) &= ~(7 << 4);
    TIM_CR2(TIM3) |= (4 << 4); // OC1REF as TRGO -> gate for TIM2

    TIM_CCER(TIM3) = 0;
    TIM_CCER(TIM3) |= (1 << 0);

    TIM_CR1(TIM3) |= (1 << 3); // OPM = 1
    TIM_CR1(TIM3) |= 1;        // CEN = 1
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM4                                                                       │
//   │ config -> us tick and generates trg0 update event for tim1                 │
//   │ function -> avoid silent zone and then trigger next timer                  │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM4_TriggerInit(void)
{
    RCC_APB1ENR1 |= (1 << 2); // TIM4 clock enable

    TIM_CR1(TIM4) = 0;
    TIM_PSC(TIM4) = 79;
    TIM_ARR(TIM4) = DELAY_SILENT_ZONE; // 200 us
    TIM_EGR(TIM4) |= 1;
    TIM_SR(TIM4) &= ~1;
    TIM_CR1(TIM4) |= (1 << 3); // OPM = 1

    TIM_SMCR(TIM4) = 0;
    TIM_SMCR(TIM4) |= (6 << 0); // SMS = 110, trigger mode -> auto-starts CEN on trigger
    TIM_SMCR(TIM4) |= (0 << 4); // TS = 000 -> ITR0 = TIM1

    TIM_CR2(TIM4) &= ~(0x7 << 4);
    TIM_CR2(TIM4) |= (2 << 4); // MMS = 010, TRGO on update -> feeds TIM3
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM1                                                                       │
//   │ config      ->  40Khz frequency and 50% duty cycle                         │
//   │ TX_pulses   -> uses RCR to generate specific amount of pulses              │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM1_PWM_BurstInit()
{
    RCC_APB2ENR |= (1 << 11); // TIM1 clock enable
    RCC_AHB2ENR |= (1 << 0);  // GPIOA clock

    GPIO_MODER(GPIOA) &= ~(3 << (8 * 2));
    GPIO_MODER(GPIOA) |= (2 << (8 * 2)); // AF mode, PA8
    GPIO_AFRH(GPIOA) &= ~(0xF << ((8 - 8) * 4));
    GPIO_AFRH(GPIOA) |= (1 << ((8 - 8) * 4)); // AF1 (TIM1_CH1)
    GPIO_OTYPER(GPIOA) &= ~(1 << 8);
    GPIO_OSPEEDR(GPIOA) &= ~(3 << (8 * 2));
	 GPIO_PUPDR(GPIOA) &= ~(3 << (8 * 2)); 

    TIM_CR1(TIM1) = 0;
    TIM_PSC(TIM1) = 1;
    TIM_ARR(TIM1) = 999;
    TIM_CCR1(TIM1) = 500;
    TIM_RCR(TIM1) = PULSE_COUNT;
    TIM_CCMR1(TIM1) &= ~(0xFF);
    TIM_CCMR1(TIM1) |= (7 << 4);
    TIM_CCMR1(TIM1) |= (1 << 3);
    TIM_CCER(TIM1) |= 1;
    TIM_BDTR(TIM1) |= (1 << 15); // MOE
    TIM_CR2(TIM1) &= ~(0x7 << 4);
    TIM_CR2(TIM1) |= (2 << 4); // MMS = 010, TRGO on update event -> fires when RCR hits 0, i.e. burst complete
    TIM_CR1(TIM1) |= (1 << 3);
    TIM_DIER(TIM1) |= 1;
    TIM_EGR(TIM1) |= 1;
    TIM_SR(TIM1) &= ~1;
}
inline static void TX_pulses(uint16_t pulses)
{
    /*
    Earlier egr was used
    This made ISR work as soon as i write it
    Thus interrupts fire two times one when EGR is written and other after ARR is hit

    Now this works fine for delay less than 200
    As TIM1 gets activate before other time interrupt fires
    Thus the second activate basically dont have any impact on TIM1

    The 8-pulse burst takes exactly 200µs to complete (8 * 25µs). 
    If DELAY_SILENT_ZONE was >= 200, TIM4 would hit 200µs at the *exact same moment* * the physical burst finished. 
    The real interrupt would fire, jump into the ISR, and completely reset TIM4 back to zero just as it was about to trigger the ADC. 
    This caused TIM4 to count all over again, resulting in a massive time shift.

    We now temporarily mask the interrupt before triggering the EGR update.
    1. Disable TIM16 interrupts (TIM_DIER &= ~1).
    2. Trigger the EGR update to latch the Repetition Counter (RCR).
    3. Clear the spurious UIF flag caused by EGR (TIM_SR &= ~1).
    4. Re-enable interrupts (TIM_DIER |= 1) and start the timer.

    Silly lol !!!!!!!!!!!!!!!!!!!!!!!!!!
    */
    TIM_CR1(TIM1) &= ~1;   // stop timer first
    TIM_DIER(TIM1) &= ~1;  // disable interrupt to prevent phantom ISR jump
    TIM_RCR(TIM1) = pulses - 1;
    TIM_CNT(TIM1) = 0;
    TIM_SR(TIM1) &= ~1;    // may fire isr and thus clear pending UIF
    TIM_EGR(TIM1) |= 1;    // latch RCR value -> the culprit
    TIM_SR(TIM1) &= ~1;    // clear the UIF flag caused by EGR safely while interrupts are off
    TIM_DIER(TIM1) |= 1;   // re-enable interrupt and start
    TIM_CR1(TIM1) |= 1;  
}

//   ┌────────────────────────────────────────────────────────────────────────────┐
//   │ TIM16                                                                      │
//   │ config      ->  for normal counting at 1us per tick                        │
//   │ delay_us    -> uses the config tick for delay in USART2                    │
//   │ delay_ms    -> uses the same delay_us to generate ms delay                 │
//   └────────────────────────────────────────────────────────────────────────────┘
void TIM16_DelayInit()
{
    RCC_APB2ENR |= (1 << 17); // TIM16 clock enable (APB2ENR, bit 17)

    TIM_CR1(TIM16) = 0;      // disable timer during setup
    TIM_PSC(TIM16) = 79;     // prescaler: 80MHz / (79+1) = 1MHz -> 1us per tick
    TIM_ARR(TIM16) = 0xFFFF; // max 16-bit auto-reload
    TIM_EGR(TIM16) |= 1;     // UG = 1, generate update event
    TIM_CR1(TIM16) |= 1;     // CEN = 1, start TIM16
}
void delay_us(uint16_t us)
{
    TIM_CNT(TIM16) = 0;
    while (TIM_CNT(TIM16) < us);
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
    (4 << 4) |  // PLLM = 5 (field value 4, divider = value+1)
    (32 << 8) | // PLLN = 32 -> 5MHz * 32 = 160MHz VCO
    (0 << 25) | // PLLR = /2 -> 80MHz SYSCLK
    (1 << 24);

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
    (32 << 8) | // PLLN = 32 -> 5MHz * 32 = 160MHz VCO (matches main PLL VCO)
    (0 << 25) |
    (1 << 24);

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
