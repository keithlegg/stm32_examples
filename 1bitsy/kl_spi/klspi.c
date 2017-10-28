#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "../common/button_boot.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/spi.h>


#include "klspi.h"

/*

   inspired by  :
   https://github.com/ericherman/stm32f4-discovery-example

*/


static void setup_spi(void)
{
    /* chip select */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    
    /* set to high which is not-selected */
    gpio_set(GPIOE, GPIO3);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
            /* serial clock */
            GPIO5 |
            /* master in/slave out */
            GPIO6 |
            /* master out/slave in */
            GPIO7);

    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

    spi_disable_crc(SPI1);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32,
            /* high or low for the peripheral device */
            SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
            /* CPHA: Clock phase: read on rising edge of clock */
            SPI_CR1_CPHA_CLK_TRANSITION_2,
            /* DFF: Date frame format (8 or 16 bit) */
            SPI_CR1_DFF_8BIT,
            /* Most or Least Sig Bit First */
            SPI_CR1_MSBFIRST);

    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    //spi_clear_mode_fault(SPI1);

    spi_enable(SPI1);
}


static void setup_main_clock(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void setup_peripheral_clocks(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_OTGFS);
    rcc_periph_clock_enable(RCC_SPI1);
}

/*
static uint8_t send_command(uint16_t command, uint8_t data)
{
    uint16_t return_value;
    uint16_t ignore;

    gpio_clear(GPIOE, GPIO3);
    spi_send(SPI1, command);
    ignore = spi_read(SPI1);
    (void)ignore;
    spi_send(SPI1, data);
    return_value = spi_read(SPI1);
    gpio_set(GPIOE, GPIO3);
    return (uint8_t)return_value;
}

static uint8_t keith_test(uint8_t byte)
{
    uint16_t command;
    uint8_t data;

    data = 0;

    command =
        // READ bit
        (0x1 << 7) |
        // MS bit:  When 0 do not increment address 
        (0x0 << 6) |
        // bits 2-7 are address 
        (byte << 0);

    return send_command(command, data);
}
*/




float deg_to_rad (float deg ){
    return deg * 0.0174532925 ;//# degree = radian * (180 / PI) # PI = 3.14159265
}


float rad_to_deg (float rad ){
    return rad * 57.29577951; // # radian = degree * (PI/180)
}


static uint8_t kgl_send_command(uint8_t data)
{
    uint16_t return_value;
    uint16_t ignore;

    gpio_clear(GPIOA, GPIO4); //CS line bitbang 

    spi_send(SPI1, data);
    return_value = spi_read(SPI1);
    gpio_set(GPIOA, GPIO4); //CS line bitbang 

    return (uint8_t)return_value;
}

/*
    cd examples/1bitsy/pwmledfade
    arm-none-eabi-gdb pwmledfade.elf
    target extended-remote /dev/ttyACM0
    monitor version
    monitor jtag_scan
    attach 1
    load
    run
*/

void init_sprite_cpu(){
    //SEND SYNC HEADER/FUCKING HACK 
    gpio_set(GPIOA, GPIO4); //CS line high/OFF in start 
    kgl_send_command(0);
    kgl_send_command(0);
    kgl_send_command(0);
    kgl_send_command(0);
}


void set_sprite_XY(int x, int y){

    //command frame is 4 bytes         
    kgl_send_command(0xF0);
    kgl_send_command(0x01);
    kgl_send_command(x);            
    kgl_send_command(0);

    //////////////////////
    //command frame is 4 bytes 
    kgl_send_command(0x0F);
    kgl_send_command(0x01);
    kgl_send_command(y); 
    kgl_send_command(0);

}


void set_sprite_scale(uint8_t scalex, uint8_t scaley){
    //command frame is 4 bytes 
    kgl_send_command(0x32);
    kgl_send_command(scalex);
    kgl_send_command(scaley); 
    kgl_send_command(0);

}


void set_stardepth(uint8_t msb, uint8_t lsb){
    //command frame is 4 bytes 

    kgl_send_command(0x96);
    kgl_send_command(msb);  //16 down to 10 bit value
    kgl_send_command(lsb);  //16 down to 10 bit value
    kgl_send_command(0);

}

void set_starfield(uint8_t posx, uint8_t posy){
    //command frame is 4 bytes 
    kgl_send_command(0x23);
    kgl_send_command(posx);
    kgl_send_command(posy); 
    kgl_send_command(0);

}


void set_reset(){
    //command frame is 4 bytes 
    kgl_send_command(0x89);
    kgl_send_command(0);    
    kgl_send_command(0);
    kgl_send_command(0);        
}


void circle_dance(){
    uint8_t j = 0;
    int i = 0;
    float x = 0;
    float y = 0;
    float scale = 150;


    for (j = 0; j < 360; j++) {  
        set_sprite_scale(x,20);

        x = sin( deg_to_rad(j) );
        y = cos( deg_to_rad(j) );
        set_sprite_XY( x*scale, y*scale );

        // poor mans delay function 
        for (i = 0; i < 100000; i++) {  
            __asm__("nop");
        }

    }

}


void star_dance(){
    uint8_t x = 0;
    int i;

    for (x = 0; x < 20; x++) {  
        set_stardepth(x,x/2);
        set_reset();
        // poor mans delay function 

        for (i = 0; i < 1000000; i++) {  
            __asm__("nop");
        }

    }
}



//extern void initialise_monitor_handles(void);
//extern "C" void initialise_monitor_handles(void); 


int main(void)
{

   // initialise_monitor_handles();
	
    button_boot(); // does not appear to work 
    
    setup_main_clock();
    setup_peripheral_clocks();

    setup_spi();
 
    init_sprite_cpu();//send four zeros to start
    
   
    
    
    printf("hello world!\n");

    
	while (1) {

        //uint8_t my_data =  kgl_send_command(x);

        //circle_dance();
        
        //star_dance();


    }




	return 0;
}
