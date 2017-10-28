
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


static void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);
	//rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART2);
}


static void usart_setup(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 38400);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);

  	usart_set_mode(USART2, USART_MODE_TX_RX );  

	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);//enable the USART.
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO7 on GPIO port B for Green LED. */
	//gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

	/* Setup GPIO pins for USART2 TX/RX. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2); //TX
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3); //RX

	/* Setup USART2 TX/RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2); //TX
	gpio_set_af(GPIOA, GPIO_AF7, GPIO3); //RX

}

int main(void)
{
	int i, j = 0, c = 0;

	clock_setup();
	gpio_setup();
	usart_setup();

    int x = 0;

	/* Blink the LED (PD12) on the board with every byte. */
	while (1) {
		//gpio_toggle(GPIOB, GPIO7); /* LED on/off */
		char ch = 0;

		//ch = usart_recv_blocking(USART2);  
    	//usart_send_blocking(USART2, ch) ;

        //__builtin_trap();
        //__asm__("BKPT");
 
        for(x=0;x<10;x++){
            
            //we set breakpoint here 
			__asm__("NOP");
			
        }

		for (i = 0; i < 100000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}

	return 0;
}
