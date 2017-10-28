

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

//#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/* Common function descriptions */
#include "clock.h"

/* milliseconds since boot */
static volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void)
{
	system_millis++;
}

/* simple sleep for delay milliseconds */
void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

/* Getter function for the current time */
uint32_t mtime(void)
{
	return system_millis;
}

/*
  clock_setup(void)
 
  This function sets up both the base board clock rate
  and a 1khz "system tick" count. The SYSTICK counter is
  a standard feature of the Cortex-M series.

  extern const struct rcc_clock_scale rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_END];
  extern const struct rcc_clock_scale rcc_hse_12mhz_3v3[RCC_CLOCK_3V3_END];
  extern const struct rcc_clock_scale rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_END];
  extern const struct rcc_clock_scale rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_END];
  
*/

void clock_setup(void)
{
	/* Base board frequency, set to 168Mhz */

	//rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]); //NOPE NOPE NOPE 
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	/* clock rate / 168000 to get 1mS interrupt rate */
	systick_set_reload(168000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();

	/* this done last */
	systick_interrupt_enable();
}