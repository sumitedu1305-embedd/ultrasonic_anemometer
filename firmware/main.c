#include "main.h"

uint16_t adc_buffer1[351] = {[0]=0xAA55};
uint16_t adc_buffer2[351] = {[0]=0xBB55};
uint16_t adc_buffer3[351] = {[0]=0xCC55};
uint16_t adc_buffer4[351] = {[0]=0xDD55};
uint32_t hello;
uint32_t cnt = 0;

int main(void)
{
	RCC_Init(); 	// 80MHz clk set from external 24mhz crystal	
	GPIO_Init();	// LED + PWM_DRV + MUX 
	TIM15_DelayInit();
	TIM16_PWM_BurstInit();
	TIM4_TriggerInit();
	TIM1_GateForADCTimer(350);
	TIM2_SlaveGateMode_TIM1();
	TIM3_RateCheckADC_EVTCheckTIM2();
	ADC1_DMA_TIM2_Config();
	USART2_DMA_TX_Init();

	delay_ms(10);
	
	GPIO_Set(PWR_ON_GPIO_Port,PWR_ON_Pin,0);
	
	GPIO_Set(PWR_DRV0_GPIO_Port,PWR_DRV0_Pin,1);
	GPIO_Set(PWR_DRV1_GPIO_Port,PWR_DRV1_Pin,1);
	
	delay_ms(1);
		
	while(1)
	{
		
		  //----------------------------------------------- SOUTH OUT -----------------------------------------------------------------
		
		  GPIO_Set(SWITCH_A_GPIO_Port,SWITCH_A_Pin,0);
		  GPIO_Set(SWITCH_B_GPIO_Port,SWITCH_B_Pin,0);
		
        TIM_CNT(TIM2) = 0;             // reset TIM2 counter
        TIM_CNT(TIM3) = 0;             // reset TIM3 event counter
			
		  DMA1_CCR1 &= ~(1<<0);
		  while(DMA1_CCR1 & (1<<0)){}
		  DMA_CMAR1(DMA1) = (uint32_t)(adc_buffer1+1);      // memory = buffer
		  DMA_IFCR(DMA1) = 0xF;
		  DMA1_CNDTR1 = 350;
		  DMA1_CCR1 |= 1<<0;
		  ADC_CR(ADC1) |= 1<<2;
        
        TX_pulses(8);  // your pulse function
        delay_ms(5);    // enough time for 1000 events at 1 MHz (~1 ms needed, extra safe)
			  
        cnt = TIM_CNT(TIM3);  // should be ~1000
		  
		  //----------------------------------------------- NORTH OUT -----------------------------------------------------------------
			  
		  GPIO_Set(SWITCH_A_GPIO_Port,SWITCH_A_Pin,0);
		  GPIO_Set(SWITCH_B_GPIO_Port,SWITCH_B_Pin,1);
		  
		  TIM_CNT(TIM2) = 0;             // reset TIM2 counter
        TIM_CNT(TIM3) = 0;             // reset TIM3 event counter
			
		  DMA1_CCR1 &= ~(1<<0);
		  while(DMA1_CCR1 & (1<<0)){}
		  DMA_CMAR1(DMA1) = (uint32_t)(adc_buffer2+1);      // memory = buffer
		  DMA_IFCR(DMA1) = 0xF;
		  DMA1_CNDTR1 = 350;
		  DMA1_CCR1 |= 1<<0;
		  ADC_CR(ADC1) |= 1<<2;
        
        TX_pulses(8);  // your pulse function
        delay_ms(5);    // enough time for 1000 events at 1 MHz (~1 ms needed, extra safe)
			  
        cnt = TIM_CNT(TIM3);  // should be ~1000
			  
		  //----------------------------------------------- WEST OUT -----------------------------------------------------------------
			 
		  GPIO_Set(SWITCH_A_GPIO_Port,SWITCH_A_Pin,1);
		  GPIO_Set(SWITCH_B_GPIO_Port,SWITCH_B_Pin,0);
		  
		  TIM_CNT(TIM2) = 0;             // reset TIM2 counter
        TIM_CNT(TIM3) = 0;             // reset TIM3 event counter
			
		  DMA1_CCR1 &= ~(1<<0);
		  while(DMA1_CCR1 & (1<<0)){}
		  DMA_CMAR1(DMA1) = (uint32_t)(adc_buffer3+1);      // memory = buffer
		  DMA_IFCR(DMA1) = 0xF;
		  DMA1_CNDTR1 = 350;
		  DMA1_CCR1 |= 1<<0;
		  ADC_CR(ADC1) |= 1<<2;
        
        TX_pulses(8);  // your pulse function
        delay_ms(5);    // enough time for 1000 events at 1 MHz (~1 ms needed, extra safe)
			  
        cnt = TIM_CNT(TIM3);  // should be ~1000
			  
		  //----------------------------------------------- EAST OUT -----------------------------------------------------------------
		  
		  GPIO_Set(SWITCH_A_GPIO_Port,SWITCH_A_Pin,1);
		  GPIO_Set(SWITCH_B_GPIO_Port,SWITCH_B_Pin,1);
		  
		  TIM_CNT(TIM2) = 0;             // reset TIM2 counter
        TIM_CNT(TIM3) = 0;             // reset TIM3 event counter
			
		  DMA1_CCR1 &= ~(1<<0);
		  while(DMA1_CCR1 & (1<<0)){}
		  DMA_CMAR1(DMA1) = (uint32_t)(adc_buffer4+1);      // memory = buffer
		  DMA_IFCR(DMA1) = 0xF;
		  DMA1_CNDTR1 = 350;
		  DMA1_CCR1 |= 1<<0;
		  ADC_CR(ADC1) |= 1<<2;
        
        TX_pulses(8);  // your pulse function
        delay_ms(5);    // enough time for 1000 events at 1 MHz (~1 ms needed, extra safe)
			  
        cnt = TIM_CNT(TIM3);  // should be ~1000
			  
		  //----------------------------------------------- SEND BUFFERS -----------------------------------------------------------------
			  
		  DMA_CCR7(DMA1) &= ~(1 << 0);
		  while (DMA_CCR7(DMA1) & (1 << 0));
		  DMA_CMAR7(DMA1)  = (uint32_t)adc_buffer1;
		  DMA_CNDTR7(DMA1) = 702;
		  DMA_CCR7(DMA1) |= (1 << 0);
		  delay_ms(15);
			  
		  DMA_CCR7(DMA1) &= ~(1 << 0);
		  while (DMA_CCR7(DMA1) & (1 << 0));
		  DMA_CMAR7(DMA1)  = (uint32_t)adc_buffer2;
		  DMA_CNDTR7(DMA1) = 702;
		  DMA_CCR7(DMA1) |= (1 << 0);
		  delay_ms(15);
		  
		  DMA_CCR7(DMA1) &= ~(1 << 0);
		  while (DMA_CCR7(DMA1) & (1 << 0));
		  DMA_CMAR7(DMA1)  = (uint32_t)adc_buffer3;
		  DMA_CNDTR7(DMA1) = 702;
		  DMA_CCR7(DMA1) |= (1 << 0);
		  delay_ms(15);
		  
		  DMA_CCR7(DMA1) &= ~(1 << 0);
		  while (DMA_CCR7(DMA1) & (1 << 0));
		  DMA_CMAR7(DMA1)  = (uint32_t)adc_buffer4;
		  DMA_CNDTR7(DMA1) = 702;
		  DMA_CCR7(DMA1) |= (1 << 0);
		  delay_ms(15);
			  
        GPIO_Toggle(TP1_GPIO_Port, TP1_Pin);
        GPIO_Toggle(LED1_GPIO_Port, LED1_Pin);
        GPIO_Toggle(LED2_GPIO_Port, LED2_Pin);
        GPIO_Toggle(LED3_GPIO_Port, LED3_Pin);

	}
	
}
void USART2_DMA_TX_Init(void)
{
    RCC_AHB2ENR |= 1<<0;    // GPIOA clock
    RCC_AHB1ENR |= 1<<0;    // DMA1 clock
    RCC_APB1ENR1 |= 1<<17;  // USART2 clock

    GPIO_MODER(GPIOA) &= ~(3 << (2 * 2));
    GPIO_MODER(GPIOA) |=  (2 << (2 * 2));      // Alternate function
    GPIO_OSPEEDR(GPIOA) |= (3 << (2 * 2));     // High speed
    GPIO_AFRL(GPIOA) &= ~(0xF << (4 * 2));
    GPIO_AFRL(GPIOA) |=  (7 << (4 * 2));     // AF7 = USART2

    USART_CR1(USART2) = 0;                      // Reset
    USART_BRR(USART2) = 87;                     // Baud rate (e.g., 9600 @ 16 MHz)
    USART_CR3(USART2) |= 1<<7;        				// Enable DMA for TX
    USART_CR1(USART2) |= 1<<3;			         // Enable transmitter
    USART_CR1(USART2) |= 1<<0;		  				// Enable USART

	 DMA_CCR7(DMA1) &= ~(1 << 0);
    while (DMA_CCR7(DMA1) & 1);  // Wait until disabled
    DMA_CPAR7(DMA1)  = (uint32_t)&USART_TDR(USART2); // Peripheral (TX reg)
    DMA_CMAR7(DMA1)  = (uint32_t)adc_buffer1;       // Memory
    DMA_CNDTR7(DMA1) = 2002;                 		// Size
    DMA1_CSELR &= ~(0xF << (6 * 4));    				// Clear CH7 bits
    DMA1_CSELR |=  (0x2 << (6 * 4));    				// Set to 0010
    DMA_CCR7(DMA1) =
          (1 << 7) |   // MINC (memory increment)
          (1 << 4) |   // DIR (memory -> peripheral)
          (0 << 5) |   // PINC disabled
          (0 << 8) |   // PSIZE = 8-bit
          (0 << 10);   // MSIZE = 8-bit
    DMA_CCR7(DMA1) |= 1;
}
void TIM3_RateCheckADC_EVTCheckTIM2()
{
    RCC_APB1ENR1 |= (1 << 1); 
	
    TIM_CR1(TIM3) = 0;
    TIM_SMCR(TIM3) = 0;
    TIM_PSC(TIM3) = 0;          // no prescaler
    TIM_ARR(TIM3) = 0xFFFF;     // max range
    TIM_CNT(TIM3) = 0;
    TIM_SMCR(TIM3) |= (7 << 0);   // SMS = 111 ? External clock mode 1
    TIM_SMCR(TIM3) |= (1 << 4);   // TS = 001 ? ITR1 (TIM2)
    TIM_CR1(TIM3) |= (1 << 0);    // CEN
	 
	 TIM_CNT(TIM2) = 0;
	 TIM_CNT(TIM3) = 0;
	
	 TX_pulses(10);
	 delay_ms(5);
	 cnt = TIM_CNT(TIM3);
	 if(cnt < 995)
	 	SomethingsWrong();	
}
void ADC1_DMA_TIM2_Config(void)
{
	
   RCC_AHB2ENR |= 1<<13;// ADCEN clk
	RCC_AHB2ENR |= 1<<0;	// GPIOA clk
	RCC_AHB1ENR |= 1<<0; // DMA1 clk
	
	//configure GPIO
	GPIO_MODER(GPIOA) &= ~(3<<0);
	GPIO_MODER(GPIOA) |= (3<<0); // analog mode
	GPIO_ASCR(GPIOA) |= 1<<0;
	
	//configure ADC
	ADC_CR(ADC1) = 0; 				// reset the control register
	ADC_CR(ADC1) &= ~(1<<29);		// deepdown mode disabled
	ADC_CR(ADC1) |= 1<<28;			// ADC regen volatge regulator enabled
	delay_ms(1);						// short delay to let voltage regulator stabilize
	ADC_CR(ADC1) &= ~(1<<30);		// ADCCAL DIF 0 for single ended calibration
	ADC_CR(ADC1) &= ~(1<<0);		// ensure the adc is disabled before calibration
	ADC_CR(ADC1) |= 1<<31;			// calibration start
	while(ADC_CR(ADC1) & 1<<31);	// wait till the calibration is complete 
	ADC_CFGR(ADC1) &= ~(1<<13);	// single conversion mode
	ADC_CFGR(ADC1) |= (1<<12);		// overrun data overwritten
	ADC_CFGR(ADC1) |= 1<<10;		// hardware trigger on rising edge
	ADC_CFGR(ADC1) &= ~(0xF<<6);  // clear the bits first
	ADC_CFGR(ADC1) |= 11<<6;		// TIM2 TRG0 is on event 11
	ADC_CFGR(ADC1) &= ~(3<<3);		// 12 bit resolution
	ADC_CFGR(ADC1) &= ~(1<<1);		// DMA ADC one shot (stops after dma is stopped much safer with timer trigerring)
	ADC_CFGR(ADC1) |= 1<<0;			// DMA enabled on adc
	ADC_SMPR1(ADC1) = 0;				// all channel sampling cycles to 2.5 cycles
	ADC_SMPR1(ADC1) &= ~(7 << 15);   // clear CH5
	ADC_SQR1(ADC1) = 0;				// length is one conversion
	ADC_SQR1(ADC1) |= 5<<6;			// 5th channel as the first conversion
	
	
	//configure DMA
	DMA_CCR1(DMA1) &= ~(1<<0);
	DMA_CPAR1(DMA1) = (uint32_t)&ADC_DR(ADC1);   // peripheral = ADC data register
   DMA_CMAR1(DMA1) = (uint32_t)(adc_buffer1+1);      // memory = buffer
   DMA_CNDTR1(DMA1) = 1000;             			// number of samples
   DMA_CCR1(DMA1) |= 1<<7;             			// MINC = memory increment
   DMA_CCR1(DMA1) |= 1<<8;              			// PSIZE = 16-bit
   DMA_CCR1(DMA1) |= 1<<10;             			// MSIZE = 16-bit
   DMA_CCR1(DMA1) &= ~(1<<4);           			// DIR = 0 (peripheral ? memory)
	
	//start ADC and wait for trigger as soon as it is ready
	ADC_CR(ADC1) |= 1<<0;			// enable adc to start convertin
	while(!(ADC_ISR(ADC1) & 1));	// wait till adc is ready
	ADC_CR(ADC1) |= 1<<2;			// start ADC
	
}
void ADC1_RateCheck(void)
{
	TX_pulses(10);
	for(int i=0; i<100000; i++)
	{
		if (ADC_ISR(ADC1) & (1 << 2))
		{
			cnt++;
			ADC_ISR(ADC1) |= (1 << 2);
		}
	}
}
void TIM2_SlaveGateMode_TIM1(void)
{
    // --- Enable clocks ---
    RCC_AHB2ENR  |= (1 << 0);    // GPIOAEN
    RCC_APB1ENR1 |= (1 << 0);    // TIM2EN

    // --- PA15 -> TIM2_CH1 (AF1) ---
    GPIO_MODER(GPIOA) &= ~(3 << (15 * 2));
    GPIO_MODER(GPIOA) |=  (2 << (15 * 2));

    GPIO_AFRH(GPIOA) &= ~(0xF << ((15 - 8) * 4));
    GPIO_AFRH(GPIOA) |=  (1 << ((15 - 8) * 4));

    GPIO_OSPEEDR(GPIOA) |= (3 << (15 * 2));

    // --- Timer base ---
    TIM_CR1(TIM2) = 0;

    TIM_PSC(TIM2)  = 0;
    TIM_ARR(TIM2)  = 79;
    TIM_CCR1(TIM2) = 40;

    // --- PWM Mode ---
    TIM_CCMR1(TIM2) = 0;
    TIM_CCMR1(TIM2) |= (6 << 4);   // PWM1
    TIM_CCMR1(TIM2) |= (1 << 3);   // preload

    TIM_CCER(TIM2) = 0;
    TIM_CCER(TIM2) |= (1 << 0);

    // --- Slave gated mode ---
    TIM_SMCR(TIM2) = 0;
    TIM_SMCR(TIM2) |= (5 << 0);    // Gated
    TIM_SMCR(TIM2) |= (0 << 4);    // ITR0 (TIM1)
	 
	 TIM_CR2(TIM2) &= ~(7 << 4);
	 TIM_CR2(TIM2) |=  (2 << 4);   // MMS = 010 ? Update event as TRGO

    TIM_EGR(TIM2) |= 1;
	 TIM_CR1(TIM2) |= (1 << 7);   // ARPE
	 TIM_CR1(TIM2) |= (1 << 0);   // CEN
	 
}
void TIM1_GateForADCTimer(uint16_t pulse_us)
{
    RCC_AHB2ENR |= (1 << 0);   // GPIOAEN (correct)
    RCC_APB2ENR |= (1 << 11);  // TIM1EN (CORRECTED - was (1<<0))
    
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
	 TIM_CR2(TIM1) |=  (4 << 4);
    
    TIM_CCER(TIM1) = 0;
    TIM_CCER(TIM1) |= (1 << 0);
	 
	 TIM_CR2(TIM1) &= ~(1 << 8);   // Idle LOW after pulse
    
    TIM_CR1(TIM1) |= (1 << 3);      // OPM = 1
    TIM_CR1(TIM1) |= 1;             // CEN = 1
    
    (*(volatile uint32_t*)(TIM1 + 0x44)) |= (1 << 15);  // MOE = 1
}

void TIM4_TriggerInit(void)
{
    RCC_APB1ENR1 |= (1 << 2);          // TIM4 clock enable

    TIM_CR1(TIM4) = 0;                  // disable timer
    TIM_PSC(TIM4) = 79;                 // 1 MHz tick (80MHz / 80)
    TIM_ARR(TIM4) = 400;                // 200 us
    TIM_EGR(TIM4) |= 1;                 // UG = latch ARR
	 TIM_SR(TIM4) &= ~1; 					 //needed as writting egr makes update event to rewrite shadow register and thus interrupts will start firring as soon as initialized
    TIM_CR1(TIM4) |= (1 << 3);          // OPM = 1 (one pulse mode)

    TIM_CR2(TIM4) &= ~(0x7 << 4);       // clear MMS
    TIM_CR2(TIM4) |= (2 << 4);          // MMS = 010 -> update event as TRGO
	
	 //TIM_DIER(TIM4) |= 1;                // enable update interrupt (UIE)
    //NVIC_EnableIRQ(30);                  // TIM4_IRQn = 30 on STM32L476RE
}

void TIM16_PWM_BurstInit()
{
    RCC_APB2ENR |= (1 << 17);       // TIM16 clock enable
	 RCC_AHB2ENR |= (1 << 0);                    // GPIOA clock

    GPIO_MODER(GPIOA) &= ~(3 << (6 * 2));
    GPIO_MODER(GPIOA) |=  (2 << (6 * 2));       // AF mode
    GPIO_AFRL(GPIOA) &= ~(0xF << (6 * 4));
    GPIO_AFRL(GPIOA) |=  (14 << (6 * 4));       // AF14 (TIM16_CH1)

    TIM_CR1(TIM16) = 0;             // disable timer
    TIM_PSC(TIM16) = 1;             // 1 MHz tick
    TIM_ARR(TIM16) = 999;           // period
    TIM_CCR1(TIM16) = 500;          // duty
    TIM_RCR(TIM16) = 9;             // 10 pulse burst
    TIM_CCMR1(TIM16) &= ~(0xFF);
    TIM_CCMR1(TIM16) |= (7 << 4);   // PWM mode 2
    TIM_CCMR1(TIM16) |= (1 << 3);   // preload enable
    TIM_CCER(TIM16) |= 1;           // enable CH1
    TIM_BDTR(TIM16) |= (1 << 15);   // MOE
	 TIM_CR1(TIM16) |= (1 << 3);
    TIM_DIER(TIM16) |= 1;        // UIE
	 TIM_EGR(TIM16)  |= 1;        // latch registers
	 TIM_SR(TIM16)   &= ~1;       // ? clear the spurious UIF BEFORE enabling NVIC
	 NVIC_EnableIRQ(TIM16_IRQn);
}
inline static void TX_pulses(uint16_t pulses)
{
    TIM_CR1(TIM16) &= ~1;       // stop timer first
    TIM_RCR(TIM16) = pulses - 1; 
    TIM_CNT(TIM16) = 0;
    TIM_SR(TIM16)  &= ~1;       // clear any pending UIF
    TIM_EGR(TIM16) |= 1;        // latch RCR value
    TIM_SR(TIM16)  &= ~1;       // clear UIF caused by EGR
    TIM_CR1(TIM16) |= 1;        // start
}
void TIM15_DelayInit()
{
	 RCC_APB2ENR |= (1 << 16);       // TIM15 clock enable (APB2ENR, bit 16)

    TIM_CR1(TIM15) = 0;             // disable timer during setup
    TIM_PSC(TIM15) = 79;            // prescaler: 80MHz / (79+1) = 1MHz ? 1us per tick
    TIM_ARR(TIM15) = 0xFFFF;        // max 16-bit auto-reload
    TIM_EGR(TIM15) |= 1;            // UG = 1, generate update event
    TIM_CR1(TIM15) |= 1;            // CEN = 1, start TIM15
}
void delay_us(uint16_t us)
{
	TIM_CNT(TIM15) = 0;
    while(TIM_CNT(TIM15) < us);
}
void delay_ms(uint16_t ms)
{
	for(int i=0; i<ms; i++)
	{
		delay_us(1000);
	}
}
void RCC_Init(void)
{
    /* 1. Enable PWR clock */
    RCC_APB1ENR1 |= (1 << 28);

    /* 2. Boost voltage scaling to Range 1 BEFORE touching PLL
          Default on reset is Range 2 (1.0V) which caps SYSCLK at 26MHz
          Range 1 (1.2V) is required for 80MHz                          */
    PWR_CR1 &= ~(3 << 9);        // clear VOS[1:0]
    PWR_CR1 |=  (1 << 9);        // VOS = 0b01 = Range 1
    while(PWR_SR2 & (1 << 10));  // wait for VOSF to clear (voltage stable)

    /* 3. Enable HSE and wait for it to stabilize */
    RCC_CR |= (1 << 16);         // HSEON
    while(!(RCC_CR & (1 << 17))); // wait for HSERDY

    /* 4. Set Flash latency BEFORE increasing clock speed
          80MHz @ Range 1 ? 4 wait states + prefetch + icache + dcache  */
    FLASH_ACR  =  (4 << 0)   // LATENCY = 4 WS
               |  (1 << 8)   // PRFTEN  = prefetch enable
               |  (1 << 9)   // ICEN    = instruction cache enable
               |  (1 << 10); // DCEN    = data cache enable

    /* 5. Disable PLL before configuring it */
    RCC_CR &= ~(1 << 24);        // PLLON = 0
    while(RCC_CR & (1 << 25));   // wait for PLLRDY to clear

    /* 6. Configure PLL
          HSE = 24 MHz
          PLLM = /6  ?  24/6     =  4 MHz  (VCO input, must be 2.66-8MHz)
          PLLN = 40  ?  4*40     = 160 MHz (VCO output, must be 64-344MHz)
          PLLR = /2  ?  160/2    =  80 MHz (SYSCLK)                       */
    RCC_PLLCFGR =
        (3  <<  0) |   // PLLSRC = HSE
        (5  <<  4) |   // PLLM   = 5 ? divider of /6 (PLLM+1 on STM32L4)
        (40 <<  8) |   // PLLN   = 40
        (0  << 25) |   // PLLR   = 0 ? divider of /2
        (1  << 24);    // PLLREN = enable PLLR output

    /* 7. Enable PLL and wait for lock */
    RCC_CR |= (1 << 24);         // PLLON
    while(!(RCC_CR & (1 << 25))); // wait for PLLRDY

    /* 8. Set all bus prescalers to /1 */
    RCC_CFGR &= ~(0xF << 4);    // HPRE  = /1  (AHB)
    RCC_CFGR &= ~(0x7 << 8);    // PPRE1 = /1  (APB1)
    RCC_CFGR &= ~(0x7 << 11);   // PPRE2 = /1  (APB2)

    /* 9. Switch SYSCLK to PLL */
    RCC_CFGR &= ~(3 << 0);      // clear SW
    RCC_CFGR |=  (3 << 0);      // SW = 0b11 = PLL as SYSCLK
    while(((RCC_CFGR >> 2) & 3) != 3); // wait for SWS confirmation
	 
	 // --- 10. Configure PLLSAI1 for ADC ---

		RCC_CR &= ~(1 << 26);              // PLLSAI1ON = 0
		while (RCC_CR & (1 << 27));        // wait PLLSAI1RDY = 0

		RCC_PLLSAI1CFGR =
			 (3  << 0)  |   // PLLSRC = HSE
			 (5  << 4)  |   // PLLM = /6  ? 24/6 = 4 MHz input
			 (40 << 8)  |   // PLLN = 24  ? 4*40 = 160 MHz VCO
			 (0  << 25) |   // PLLR = /2  ? 160/2 = 80 MHz ADC clock
			 (1  << 24);    // PLLREN enable

		RCC_CR |= (1 << 26);               // PLLSAI1ON
		while (!(RCC_CR & (1 << 27)));     // wait ready

		// --- Select ADC clock = PLLSAI1 ---
		RCC_CCIPR &= ~(3 << 28);
		RCC_CCIPR |=  (1 << 28);           // ADCSEL = PLLSAI1
}
void GPIO_Config(uint32_t PORT, uint8_t PIN) //	only to set GPIOs output with max speed and pull down with push pull configuration
{
	 switch(PORT)
	 {
		 case GPIOA : RCC_AHB2ENR |= 1<<0; break;
		 case GPIOB : RCC_AHB2ENR |= 1<<1; break;
		 case GPIOC : RCC_AHB2ENR |= 1<<2; break;
		 case GPIOD : RCC_AHB2ENR |= 1<<3; break;
		 case GPIOE : RCC_AHB2ENR |= 1<<4; break;
		 case GPIOF : RCC_AHB2ENR |= 1<<5; break;
		 case GPIOG : RCC_AHB2ENR |= 1<<6; break;
		 case GPIOH : RCC_AHB2ENR |= 1<<7; break;
	 }
    GPIO_MODER(PORT) &= ~(3 << (PIN * 2));   // clear bits
    GPIO_MODER(PORT) |=  (1 << (PIN * 2));   // set output mode
    GPIO_OTYPER(PORT) &= ~(1 << PIN);        // 0 = push-pull
    GPIO_OSPEEDR(PORT) &= ~(3 << (PIN * 2));	// high speed
    GPIO_OSPEEDR(PORT) |=  (3 << (PIN * 2));
    GPIO_PUPDR(PORT) &= ~(3 << (PIN * 2));	//pull down
    GPIO_PUPDR(PORT) |=  (2 << (PIN * 2));
}
void GPIO_Init()
{
	GPIO_Config(PWR_ON_GPIO_Port,PWR_ON_Pin);
	GPIO_Config(PWR_DRV0_GPIO_Port,PWR_DRV0_Pin);
	GPIO_Config(PWR_DRV1_GPIO_Port,PWR_DRV1_Pin);
	GPIO_Config(SWITCH_A_GPIO_Port,SWITCH_A_Pin);
	GPIO_Config(SWITCH_B_GPIO_Port,SWITCH_B_Pin);
	GPIO_Config(LED1_GPIO_Port,LED1_Pin);
	GPIO_Config(LED2_GPIO_Port,LED2_Pin);
	GPIO_Config(LED3_GPIO_Port,LED3_Pin);
	GPIO_Config(TP1_GPIO_Port,TP1_Pin);
	GPIO_Config(TP2_GPIO_Port,TP2_Pin);
}
static inline void GPIO_Set(uint32_t PORT, uint8_t PIN, uint8_t SET1_RESET0) 
{
    GPIO_BSRR(PORT) = 1 << (PIN + (SET1_RESET0 ? 0 : 16));
}

static inline void GPIO_Toggle(uint32_t PORT, uint8_t PIN) 
{
    GPIO_ODR(PORT) ^= 1 << PIN;
}
void SomethingsWrong()
{
	while(1)
	{
		GPIO_Toggle(LED1_GPIO_Port,LED1_Pin);
		GPIO_Toggle(LED2_GPIO_Port,LED2_Pin);
		GPIO_Toggle(LED3_GPIO_Port,LED3_Pin);
		delay_ms(30);
		
		break;
	}
}
/***************************************************************************************************************************************/
void TIM1_UP_TIM16_IRQHandler()
{
    if (TIM_SR(TIM16) & 1) 
    {
        TIM_SR(TIM16) &= ~1;        // clear TIM16 UIF first

        TIM_CR1(TIM4) &= ~1;        // ensure timer stopped
        TIM_CNT(TIM4) = 0;          // reset counter
        TIM_SR(TIM4) &= ~1;         // clear any pending update flag
        TIM_CR1(TIM4) |= 1;         // start TIM4
    }
}
void TIM4_IRQHandler()
{
	TIM_SR(TIM4) &= ~1;
}









