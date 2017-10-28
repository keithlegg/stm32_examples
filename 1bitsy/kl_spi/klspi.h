
#ifndef KLSPI_H
#define KLSPI_H

#include <stddef.h>

#include <libopencm3/stm32/gpio.h>


#define SPI_DATA_PORT  GPIOB
//#define SPI_DATA_PINS (GPIO15 << 1 - GPIO8)
#define SPI_DATA_PINS (GPIO15 | GPIO14 | GPIO13 | GPIO12 |      \
                       GPIO11 | GPIO10 | GPIO9  | GPIO8)

#define SPI_CSX_PORT   GPIOC
#define SPI_CSX_PIN    GPIO3

#define SPI_RESX_PORT  GPIOC
#define SPI_RESX_PIN   GPIO2

//#define SPI_DCX_PORT   GPIOC
//#define SPI_DCX_PIN    GPIO6

#define SPI_WRX_PORT   GPIOB
#define SPI_WRX_PIN    GPIO1

#define SPI_RDX_PORT   GPIOB
#define SPI_RDX_PIN    GPIO0

#define SPI_DATA_PORT  GPIOB

/*
typedef struct gpio_pin {
    uint32_t gp_port;           // GPIOA .. GPIOF
    uint16_t gp_pin;            // GPIO0 .. GPIO15
    uint8_t  gp_mode   : 2;     // GPIO_MODE_INPUT/OUTPUT/AF/ANALOG
    uint8_t  gp_pupd   : 2;     // GPIO_PUPD_NONE/PULLUP/PULLDOWN
    uint8_t  gp_af     : 4;     // GPIO_AF0 .. GPIO_AF15
    uint8_t  gp_ospeed : 2;     // GPIO_OSPEED_2/25/60/100MHZ
    uint8_t  gp_otype  : 1;     // GPIO_OTYPE_PP/OD (push-pull, open drain)
    uint8_t  gp_level  : 1;     // 0 or 1
} gpio_pin;
extern void gpio_init_pin(const gpio_pin *);
extern void gpio_init_pins(const gpio_pin *, size_t count);
*/


#endif 