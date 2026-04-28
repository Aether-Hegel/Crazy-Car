/*
 * Override FreeRTOS weak timer setup to ensure SysTick is configured correctly
 * for the MIMXRT1064 Cortex-M7 core.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "core_cm7.h"

/*
 * FreeRTOS port.c defines vPortSetupTimerInterrupt() as a weak symbol.
 * Providing our own implementation here ensures SysTick is configured with
 * the correct clock source and reload value for this platform.
 */
void vPortSetupTimerInterrupt( void )
{
    /* Stop SysTick while we configure it. */
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;

    /*
     * Set reload to generate an interrupt at configTICK_RATE_HZ.
     * The SysTick clock source is the core clock.
     */
    SysTick->LOAD = ( SystemCoreClock / configTICK_RATE_HZ ) - 1UL;

    /* Set SysTick priority to lowest. */
    NVIC_SetPriority( SysTick_IRQn, ( 1UL << __NVIC_PRIO_BITS ) - 1UL );

    /* Enable SysTick, its interrupt, and select core clock. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
