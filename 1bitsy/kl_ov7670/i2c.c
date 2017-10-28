#include <stdint.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"
#include "i2c.h"



#define eeprom_24lc256 0x50  //Address of 24LC256 eeprom chip (3 hardware ADDR all low)  

/*****************************************/

//couldnt figure out SysTick so this is a hack for now. 
void delay(uint32_t ticks){
    uint32_t t = 0;
    for(t=0;t<1000000;t++){
       // _asm_('nop');
    }

}

/*****************************************/

void i2c_init(uint32_t i2c)
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_I2C3);
    
    
    //GPIO A8 is SCL 
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);

    
    //GPIO C9 SDA 
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);


  
	i2c_peripheral_disable(i2c); // disable i2c during setup 
	i2c_reset(i2c);

	i2c_set_fast_mode(i2c);
	i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_42MHZ);
	i2c_set_ccr(i2c, 35);   // I2C Set Bus Clock Frequency.
	i2c_set_trise(i2c, 43); // I2C Set the Rise Time.

	//i2c_set_speed(i2c, 0); //this was disabled 
	i2c_peripheral_enable(i2c); // finally enable i2c  
	i2c_set_own_7bit_slave_address(i2c, 0x00);   
  


}

/*****************************************/

void i2c_deinit(void)
{

	i2c_send_stop(I2C3);
	i2c_reset(I2C3);
	i2c_peripheral_disable(I2C3); /* disable i2c during setup */

}

/*****************************************/

uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode)
{
	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, address, mode);

	int timeout = 20000;
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
		if (timeout > 0) {
			timeout--;
		} else {
			return 1;
		}
	}

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	return 0;
}


/*****************************************/

//generic i2C write - modify as needed 
uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg,uint8_t data)
{
	i2c_start(i2c, address, I2C_WRITE); //keith disabled this 

	i2c_send_data(i2c, reg);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
	i2c_send_data(i2c, data);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_send_stop(i2c);

	return 0;
}

/*****************************************/
//generic i2c read - modify as needed 
int i2c_read(uint32_t i2c, uint8_t address, uint8_t reg)
{
	uint32_t timeout = 20000;

	while ((I2C_SR2(i2c) & I2C_SR2_BUSY)); 
	
	if (i2c_start(i2c, address, I2C_WRITE)) {
		return 0;
	}
	i2c_send_data(i2c, reg);

	timeout = 20000;
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF))) {
		if (timeout > 0) {
			timeout--;
		} else {
			return -1;
		}
	}

	i2c_start(i2c, address, I2C_READ);

	i2c_send_stop(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));

	int result = (int)i2c_get_data(i2c);

	I2C_SR1(i2c) &= ~I2C_SR1_AF;
	msleep(50);
	i2c_send_stop(i2c);

	return result;
}


/**********************************************/

void write_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t data ) 
{

    /*
       write a single byte to 24LC256 eeprom 

       device address is the EEPROM itself (lower 3 bits hardwired)
       eeaddress is the memory location (16bit) of the data IN the eeprom 
       

       //notes on logic analyzer protocol:
       If you read the packet as "7 bits, address only" it will be 0x50 +(R/W)
       If you read it as 8 bits including (R/W) it will look like - 
       -> write : 1010 (control code) + 000 (A2, A1, A0 value) + 0 (write) –> 0xA0
       -> read  : 1010 (control code) + 000 (A2, A1, A0 value) + 1 (read)  –> 0xA1

       http://www.jechavarria.com/2013/06/28/components-i-usually-use-ii-24lc256-i2c-eeprom-memory/

       Generate the Start condition in the I2C bus (SDA low, then SCL low).
       Send the device address (0xA0 or 0xA1 if continuing with the example above) 
       Send the high byte of the address to access
       Send the low byte of the address to access.

    */

    uint32_t base = I2C3;

 	i2c_start(base, eeprom_24lc256, I2C_WRITE);

    // //send the high byte of address
	i2c_send_data(base, (int)(eeaddress >> 8) );
	while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    // //send the low byte of address
	i2c_send_data(base, (int)(eeaddress & 0xFF) );
	while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    // //send the data to be written
	i2c_send_data(base, data );
	while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
	
    i2c_send_stop(base);
    
    //delay(100);
 	//msleep(50);

}


/**********************************************/

/*
   read a single byte from a 24LC256 eeprom. 
   
   Device address is the EEPROM itself (lower 3 bits hardwired)
   eeaddress is the memory location (16bit) of the data IN the eeprom 

   http://www.jechavarria.com/2013/06/28/components-i-usually-use-ii-24lc256-i2c-eeprom-memory/

*/

uint8_t read_24LC256(int deviceaddress, uint16_t eeaddress ) 
{

    uint8_t rdata = 0xFF;
    
    uint32_t base = I2C3;

    //uint32_t timeout = 20000;

    i2c_start(base, eeprom_24lc256, I2C_WRITE);
    
    // //send the high byte of address
    i2c_send_data(base, (int)(eeaddress >> 8) );
    while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    // //send the low byte of address
    i2c_send_data(base, (int)(eeaddress & 0xFF) );
    while (!(I2C_SR1(base) & (I2C_SR1_BTF))); //Byte transfer finished
    
    //seems to be a proper i2c restart ?
    i2c_start(base, eeprom_24lc256, I2C_READ);
    i2c_send_stop(base);//WHY DO WE STOP HERE???

    while (!(I2C_SR1(base) & I2C_SR1_RxNE)); //Data register not empty
    rdata = (int)i2c_get_data(base); //WHY CAST TO INT?

    I2C_SR1(base) &= ~I2C_SR1_AF; // Clear ack failure bit - NACK? 
    i2c_send_stop(base);

    return rdata;

}


/**********************************************/
/*
    write up to 64 bytes (1 page) of data at a time to an address in EEPROM 
    EEPROM internal address increments automatically from the start address
    read datasheet for page size and more info
*/

void write_multi_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t *data, size_t count ) 
{
    uint32_t base = I2C3;

    //i2c start condition
 	i2c_start(I2C3, eeprom_24lc256, I2C_WRITE);

    //send the high byte of address
	i2c_send_data(base, (int)(eeaddress >> 8) );
	while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    //send the low byte of address
	i2c_send_data(base, (int)(eeaddress & 0xFF) );
	while (!(I2C_SR1(base) & (I2C_SR1_BTF)));

    // write of bytes out  
    // EEPROM internal address increments automatically.  
    uint16_t i = 0;
    for (i = 0; i < count; i++) {
        I2C_DR(base) = data[i];
        while (!(I2C_SR1(base) & I2C_SR1_BTF));
    }
    //IS THIS NACK?? seems to work, so ... yes?	
    I2C_SR1(base) &= ~I2C_SR1_AF; 

    //i2c stop condition 
    i2c_send_stop(base);

}


/**********************************************/

void read_multi_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t *data, size_t count ) 
{
    //STM32 I2C peripheral address
    uint32_t base = I2C3;

    //i2c start condition
    i2c_start(base, eeprom_24lc256, I2C_WRITE);
    
    // send the high byte of address
    i2c_send_data(base, (int)(eeaddress >> 8) );
    while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    // send the low byte of address
    i2c_send_data(base, (int)(eeaddress & 0xFF) );
    while (!(I2C_SR1(base) & (I2C_SR1_BTF)));
    
    /***************************/
    //i2c restart
    i2c_start(base, eeprom_24lc256, I2C_READ); 

    // read a stream of bytes - limit is hardware dependent 
    // consult the datasheet 
    //set CR to ACK and read (count) bytes
    uint8_t i = 0;
    for (i = 0; i < count; i++) {
        I2C_CR1(base) |= I2C_CR1_ACK; //THIS WAS THE SECRET !!! - SET ACK!
        while (!(I2C_SR1(base) & I2C_SR1_RxNE));
        data[i] = (int)i2c_get_data(base); //WHY CAST TO INT?

    }

    /***************************/
    //IS THIS NACK?? seems to work, so ... yes?
    I2C_SR1(base) &= ~I2C_SR1_AF; 

    //i2c stop condition 
    i2c_send_stop(base);

}
