/* Lectura DHT11 con timeout para no bloquear el programa. */
#include "dht11.h"
#include <avr/interrupt.h>

void DHT_Init(void)
{
	DHT_DDR |= (1U << DHT_PIN);
	DHT_PORT |= (1U << DHT_PIN);
}

/* Esperar nivel logico hasta un limite de tiempo (aproximado). */
static uint8_t esperar_nivel(uint8_t alto)
{
	uint16_t limite = 250;
	while ((((DHT_PIN_IN & (1U << DHT_PIN)) != 0U) ? 1U : 0U) != alto) {
		if (limite-- == 0U)
		return 0;
		_delay_us(1);
	}
	return 1;
}

static uint8_t leer_byte(uint8_t *dato)
{
	uint8_t valor = 0;
	for (uint8_t i = 0; i < 8; i++) {
		/* Cada bit: 50 us a nivel bajo y luego pulso alto. */
		if (!esperar_nivel(1))
		return 0;
		_delay_us(35);
		valor <<= 1;
		if (DHT_PIN_IN & (1U << DHT_PIN)) {
			valor |= 1U;
			if (!esperar_nivel(0))
			return 0;
		}
	}
	*dato = valor;
	return 1;
}

uint8_t DHT_GetData(uint8_t *temperature, uint8_t *humidity)
{
	uint8_t datos[5] = {0};
	uint8_t valido = 1;

	/* Inicio: MCU en bajo >18 ms, luego libera DATA. */
	DHT_DDR |= (1U << DHT_PIN);
	DHT_PORT &= (uint8_t)~(1U << DHT_PIN);
	_delay_ms(20);

	uint8_t sreg = SREG;
	cli();
	DHT_PORT |= (1U << DHT_PIN);
	_delay_us(30);
	DHT_DDR &= (uint8_t)~(1U << DHT_PIN);

	/* Handshake: 80 us bajo, 80 us alto y regreso a bajo. */
	if (!esperar_nivel(0) || !esperar_nivel(1) || !esperar_nivel(0))
	valido = 0;

	if (valido) {
		for (uint8_t i = 0; i < 5; i++) {
			if (!leer_byte(&datos[i])) {
				valido = 0;
				break;
			}
		}
	}

	SREG = sreg;

	if (!valido)
	return 0;

	if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4])
	return 0;

	*humidity = datos[0];
	*temperature = datos[2];
	return 1;
}
