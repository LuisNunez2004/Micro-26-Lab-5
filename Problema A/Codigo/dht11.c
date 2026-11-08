/*
 * dht11.c
 *
 * Created: 8/11/2026 2:45:01
 *  Author: UTEC-5695
 */ 
#include "dht11.h"

void DHT_Init(void) {
	DHT_DDR |= (1 << DHT_PIN);
	DHT_PORT |= (1 << DHT_PIN);
}

static void DHT_start(void) {
	DHT_DDR |= (1 << DHT_PIN);    
	DHT_PORT &= ~(1 << DHT_PIN);  
	_delay_ms(18);                
	DHT_PORT |= (1 << DHT_PIN);   
	_delay_us(40);                
}

static uint8_t DHT_response(void) {
	DHT_DDR &= ~(1 << DHT_PIN);   
	_delay_us(40);
	if (!(DHT_PIN_IN & (1 << DHT_PIN))) { 
		_delay_us(80);
		if ((DHT_PIN_IN & (1 << DHT_PIN))) {  
			_delay_us(80);
			return 1; 
		}
	}
	return 0; 
}

static uint8_t DHT_read_byte(void) {
	uint8_t result = 0;
	for (int i = 0; i < 8; i++) {
		while (!(DHT_PIN_IN & (1 << DHT_PIN))); 
		_delay_us(30);                           
		if (DHT_PIN_IN & (1 << DHT_PIN))         
		result |= (1 << (7 - i));
		while (DHT_PIN_IN & (1 << DHT_PIN));     
	}
	return result;
}

uint8_t DHT_GetData(uint8_t *temperature, uint8_t *humidity) {
	uint8_t hum_hi, hum_lo, temp_hi, temp_lo, checksum;

	DHT_start();
	if (!DHT_response()) {
		return 0; 
	}

	hum_hi = DHT_read_byte();
	hum_lo = DHT_read_byte();
	temp_hi = DHT_read_byte();
	temp_lo = DHT_read_byte();
	checksum = DHT_read_byte();

	
	if (checksum == (hum_hi + hum_lo + temp_hi + temp_lo)) {
		*humidity = hum_hi;
		*temperature = temp_hi;
		return 1; 
	}

	return 0; 
}
