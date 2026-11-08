#ifndef DHT11_H
#define DHT11_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define DHT_PIN     PD4
#define DHT_DDR     DDRD
#define DHT_PORT    PORTD
#define DHT_PIN_IN  PIND

void DHT_Init(void);
uint8_t DHT_GetData(uint8_t *temperature, uint8_t *humidity);

#endif
