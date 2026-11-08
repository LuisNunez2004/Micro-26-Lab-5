/*
 * i2c.h
 *
 * Created: 23/10/2026 17:30:54
 *  Author: UTEC-5695
 */ 


#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L 

void I2C_Init(void) {
	TWSR = 0; 
	TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
	TWCR = (1 << TWEN);
}

void I2C_Start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

void I2C_Write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

#endif
