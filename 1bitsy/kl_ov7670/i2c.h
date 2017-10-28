#ifndef __I2C_H
#define __I2C_H

#define I2C_CR2_FREQ_MASK	0x3ff
#define I2C_CCR_CCRMASK	0xfff
#define I2C_TRISE_MASK	0x3f

void i2c_init(uint32_t i2c);
void i2c_deinit(void);


uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg,
	uint8_t data);

uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode);

int i2c_read(uint32_t i2c, uint8_t address, uint8_t reg);

void write_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t data ) ;

uint8_t read_24LC256(int deviceaddress, uint16_t eeaddress ) ;


void read_multi_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t *data, size_t count ); 
void write_multi_24LC256(int deviceaddress, uint16_t eeaddress, uint8_t *data, size_t count );


#endif