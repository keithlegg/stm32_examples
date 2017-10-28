

#include <stdint.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


#include <libopencm3/stm32/i2c.h>

//#include "clock.h" //things got stuck when i moved this 
#include "i2c.h"


/*************************************/

/* 
#include <libopencm3/cm3/systick.h>
static volatile uint32_t system_millis;
void sys_tick_handler(void){system_millis++;}
void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
} */


/*************************************/
void delay2(){
	uint32_t a = 0;

    for(a=0;a<1000000;a++){
        __asm__("nop");
    } 
}
 


/*************************************/

int main(void)
{

	i2c_init(I2C3);
    delay2(); //this was important 



    uint16_t eep_addr    = 0x0000; //address of memory IN eeprom to SEND over i2c
    uint8_t dev_i2c_addr = 0x05;   //address of 12C HARDWARE itself (eeprom)

    /***************/    
    /***************/
    // single byte read/write 
    
    //write a single byte 
    //write_24LC256(dev_i2c_addr, eep_addr, 0x39);

    //read a single byte 
    //uint8_t buffer = 0;
    //buffer = read_24LC256(dev_i2c_addr, eep_addr ) ;
    

    /***************/
    /***************/

    //mutli byte read/write 

    //setup buffer to use for RX/TX- (can only write up to 64 bytes!)
    uint8_t eeprom_buffer[64] = {0};
    uint8_t *pData = 0;
    pData = &eeprom_buffer;
   
    /***************/
    //setup a mutli byte write   - only worksup to 64 bytes!(?)
 
    /* 
        //fill array with data to send 
        uint8_t i = 0;
        for(i=0;i<64;i++){
            eeprom_buffer[i] = i; 
        }
        //send it 
        write_multi_24LC256(dev_i2c_addr, eep_addr, pData, 63 );
    */

    /***************/
    //setup a mutli byte read - no limit on size except memory, of course 
    /*    
    read_multi_24LC256( dev_i2c_addr, eep_addr, pData, 20  );

    int i = 0;
    uint8_t rx_byte = 0;

    for(i=0;i<15;i++){
        rx_byte = eeprom_buffer[i];//<-- set breakpoiont here for debugging
    }*/
   
    /***************/

    //we are done, this just keeps main from exiting 
    while(1){
    
    }
   


    return 0;
}
