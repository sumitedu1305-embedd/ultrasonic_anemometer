#include <stdint.h>

#define SCB_VTOR (*(volatile uint32_t *)0xE000ED08)
#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88)

#define APP_SLOT_A_ADDR 0x08004000UL

typedef void (*app_entry_t)(void);

void SystemInit(void) {
    SCB_CPACR |= (0xF << 20);   // enable FPU access
}

static void jump_to_app(uint32_t app_addr)
{
    uint32_t app_stack = *(volatile uint32_t *)(app_addr);
    uint32_t app_reset  = *(volatile uint32_t *)(app_addr + 4);

    // relocate vector table to the app's address before jumping
    SCB_VTOR = app_addr;

    // set main stack pointer to app's initial SP
    __asm volatile ("MSR msp, %0" : : "r" (app_stack));

    app_entry_t app_entry = (app_entry_t)app_reset;
    app_entry();
}

int main(void)
{

    jump_to_app(APP_SLOT_A_ADDR);

    // should never reach here
    while (1);
}