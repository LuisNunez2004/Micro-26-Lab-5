// ETAPA 1: Inicializacion de USART0, Buferes Circulares e Init0 para Reset r2.

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// --- Definición Exhaustiva de Estados (FSM) ---
#define ESTADO_ACTIVO  0
#define ESTADO_STANDBY 1
#define ESTADO_APAGADO 2

// --- Asignación Estricta de Hardware ---
#define LED_ACTIVO    PB0   // LED Verde (Indica Estado Activo)
#define LED_BLOQUEO   PB1   // LED Amarillo (Indica Standby o Falla de Software)
#define BOTON_ACT     PD2   // Pulsador Físico de Actividad / Despertar (INT0)
#define CANAL_ADC     0     // PC0 / Entrada del Potenciómetro

// --- Configuración del Protocolo de Tramas Bluetooth (HC-05) ---
#define TRAMA_START '['
#define TRAMA_STOP  ']'
#define BUFFER_SIZE 32

// --- Variables Globales de Sistema ---
uint8_t causa_reset_real __attribute__((section(".noinit")));
volatile uint8_t estado_actual = ESTADO_ACTIVO;
volatile uint32_t contador_ms = 0;
volatile uint32_t ultimo_evento = 0;
char buffer_diagnostico[64];

// Búferes Circulares para UART
volatile char rx_ring[BUFFER_SIZE];
volatile uint8_t rx_head = 0; volatile uint8_t rx_tail = 0;

volatile char tx_ring[BUFFER_SIZE];
volatile uint8_t tx_head = 0; volatile uint8_t tx_tail = 0;

volatile uint8_t despertado_por_wdt_8s = 0;

// =========================================================================
// 1. CAPTURA TEMPRANA DE LA CAUSA DEL RESET (.init0)
// =========================================================================
void capturar_causa_real(void) __attribute__((naked, section(".init0")));
void capturar_causa_real(void) {
	if (MCUSR == 0) {
		__asm__ __volatile__("sts causa_reset_real, r2 \n\t");
		} else {
		causa_reset_real = MCUSR;
	}
	MCUSR = 0;
	wdt_disable();
}

// =========================================================================
// 2. CONTROLADORES DE INTERRUPCIÓN (ISR)
// =========================================================================
ISR(USART_RX_vect) {
	char dato = UDR0;
	uint8_t sig = (rx_head + 1) % BUFFER_SIZE;
	if (sig != rx_tail) {
		rx_ring[rx_head] = dato;
		rx_head = sig;
	}
	ultimo_evento = contador_ms;
}

ISR(USART_UDRE_vect) {
	if (tx_tail != tx_head) {
		UDR0 = tx_ring[tx_tail];
		tx_tail = (tx_tail + 1) % BUFFER_SIZE;
		} else {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

ISR(TIMER1_COMPA_vect) {
	contador_ms++;
}

ISR(INT0_vect) {
	ultimo_evento = contador_ms;
}

ISR(WDT_vect) {
	despertado_por_wdt_8s = 1;
}

// =========================================================================
// 3. MÓDULOS DE CONFIGURACIÓN Y REDUCCIÓN DE POTENCIA (PRR)
// =========================================================================
void uart_configurar(void) {
	PRR &= ~(1 << PRUSART0);
	UBRR0H = 0; UBRR0L = 103; // 9600 bps a 16MHz
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
}

void uart_enviar_char(char c) {
	uint8_t sig = (tx_head + 1) % BUFFER_SIZE;
	while (sig == tx_tail);
	tx_ring[tx_head] = c;
	tx_head = sig;
	UCSR0B |= (1 << UDRIE0);
}

void uart_enviar(const char *texto) {
	while (*texto) { uart_enviar_char(*texto++); }
}

void adc_configurar(void) {
	PRR &= ~(1 << PRADC);
	ADMUX = (1 << REFS0) | (CANAL_ADC & 0x0F);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_leer(void) {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void timer1_configurar(void) {
	PRR &= ~(1 << PRTIM1);
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS11); // CTC con prescaler 8
	OCR1A = 1999; // Interrupción cada exactamente 1ms
	TIMSK1 |= (1 << OCIE1A);
}

// =========================================================================
// 4. PROCESAMIENTO ASÍNCRONO DE TRAMAS BLUETOOTH
// =========================================================================
void procesar_tramas_bluetooth(void) {
	uint8_t disp = (rx_head >= rx_tail) ? (rx_head - rx_tail) : (BUFFER_SIZE - rx_tail + rx_head);
	if (disp < 5) return;

	if (rx_ring[rx_tail] != TRAMA_START) {
		rx_tail = (rx_tail + 1) % BUFFER_SIZE;
		return;
	}

	uint8_t i_cmd = (rx_tail + 1) % BUFFER_SIZE;
	uint8_t i_dat = (rx_tail + 2) % BUFFER_SIZE;
	uint8_t i_chk = (rx_tail + 3) % BUFFER_SIZE;
	uint8_t i_stp = (rx_tail + 4) % BUFFER_SIZE;

	if (rx_ring[i_stp] != TRAMA_STOP) {
		rx_tail = (rx_tail + 1) % BUFFER_SIZE;
		return;
	}

	char cmd  = rx_ring[i_cmd];
	char dato = rx_ring[i_dat];
	char chk  = rx_ring[i_chk];
	rx_tail = (rx_tail + 5) % BUFFER_SIZE;

	if ((char)(cmd + dato) != chk) {
		uart_enviar("[BT_ERROR] Checksum invalido.\r\n");
		return;
	}

	if (cmd == 'S') {
		snprintf(buffer_diagnostico, sizeof(buffer_diagnostico), "[APP] Estado: %d\r\n", estado_actual);
		uart_enviar(buffer_diagnostico);
	}
	else if (cmd == 'R') {
		uint16_t pot = adc_leer();
		snprintf(buffer_diagnostico, sizeof(buffer_diagnostico), "[APP] Pot: %d\r\n", pot);
		uart_enviar(buffer_diagnostico);
	}
	else if (cmd == 'E') {
		if (dato == 'A') { estado_actual = ESTADO_ACTIVO; ultimo_evento = contador_ms; uart_enviar("[BT] FSM -> ACTIVO\r\n"); }
		if (dato == 'S') { estado_actual = ESTADO_STANDBY; uart_enviar("[BT] FSM -> STANDBY\r\n"); }
	}
}

// =========================================================================
// 5. PROGRAMA PRINCIPAL
// =========================================================================
// ETAPA 2: Lógica de la Máquina de Estados (FSM Cooperativa) y control energetico con el registro PRR.

int main(void)
{
	// Captura inicial de la causa en la simulación
	causa_reset_real = MCUSR;
	MCUSR = 0;
	wdt_disable();

	cli();
	DDRB |= (1 << LED_ACTIVO) | (1 << LED_BLOQUEO);
	PORTB &= ~((1 << LED_ACTIVO) | (1 << LED_BLOQUEO));

	DDRD &= ~(1 << BOTON_ACT); PORTD |= (1 << BOTON_ACT);

	EICRA = 0; EIMSK |= (1 << INT0);

	uart_configurar();
	adc_configurar();
	timer1_configurar();
	sei();

	if (causa_reset_real & (1 << WDRF)) {
		uart_enviar("\r\n[SISTEMA RECUPERADO] Reinicio provocado por Watchdog Timer.\r\n");
		} else {
		uart_enviar("\r\n[ARRANQUE NORMAL] Inicializando sistema cooperativo en C...\r\n");
	}

	uint32_t ultimo_muestreo_normal = 0;

	while (1)
	{
		procesar_tramas_bluetooth();
		
		if (estado_actual == ESTADO_ACTIVO)
		{
			PRR = 0x00;
			PORTB |= (1 << LED_ACTIVO); PORTB &= ~(1 << LED_BLOQUEO);

			if (contador_ms - ultimo_muestreo_normal >= 500) {
				ultimo_muestreo_normal = contador_ms;
				uint16_t pot_val = adc_leer();
				snprintf(buffer_diagnostico, sizeof(buffer_diagnostico), "Ejecutando normal | ADC: %d\r\n", pot_val);
				uart_enviar(buffer_diagnostico);
			}

			if (!(PIND & (1 << BOTON_ACT))) { ultimo_evento = contador_ms; }

			if (contador_ms - ultimo_evento >= 10000) {
				estado_actual = ESTADO_STANDBY;
				uart_enviar("10 segundos sin actividad. Transicion a STANDBY.\r\n");
			}

			wdt_enable(WDTO_2S);
			wdt_reset();
		}
		else if (estado_actual == ESTADO_STANDBY)
		{
			PRR = (1 << PRSPI) | (1 << PRTWI);
			PORTB &= ~(1 << LED_ACTIVO); PORTB |= (1 << LED_BLOQUEO);

			if (!(PIND & (1 << BOTON_ACT))) {
				ultimo_evento = contador_ms;
				estado_actual = ESTADO_ACTIVO;
				uart_enviar("[EVENTO VÁLIDO] Volviendo a Estado ACTIVO.\r\n");
			}

			if (contador_ms - ultimo_evento >= 30000) {
				estado_actual = ESTADO_APAGADO;
			}

			wdt_enable(WDTO_2S);
			wdt_reset();

			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
		else if (estado_actual == ESTADO_APAGADO)
		{
			uart_enviar("30 segundos cumplidos. Entrando en Power-Down...\r\n");
			while (tx_tail != tx_head);
			_delay_ms(20);

			PORTB &= ~((1 << LED_ACTIVO) | (1 << LED_BLOQUEO));
			PRR = (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRTWI);

			cli();
			MCUSR &= ~(1 << WDRF);
			WDTCSR |= (1 << WDCE) | (1 << WDE);
			WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
			sei();

			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_mode();

			PRR = 0x00;
			uart_configurar();
			adc_configurar();
			timer1_configurar();

			if (despertado_por_wdt_8s) {
				despertado_por_wdt_8s = 0;
				uart_enviar("[DESPERTAR WDT] Transmitiendo lectura en reposo de 8s.\r\n");
				uint16_t lectura_rapida = adc_leer();
				snprintf(buffer_diagnostico, sizeof(buffer_diagnostico), "[PRUEBA A DATA] ADC post-sueño: %d\r\n", lectura_rapida);
				uart_enviar(buffer_diagnostico);
			}

			contador_ms = 0;
			ultimo_evento = contador_ms;
			estado_actual = ESTADO_ACTIVO;
		}

		// --- SIMULACIÓN DE BLOQUEO CRÍTICO (PRUEBA B RESET) ---
		if (estado_actual == ESTADO_ACTIVO && !(PIND & (1 << BOTON_ACT)) && (contador_ms - ultimo_evento > 3000)) {
			PORTB &= ~(1 << LED_ACTIVO);
			PORTB |= (1 << LED_BLOQUEO);
			uart_enviar("!!! SIMULACIÓN DE BLOQUEO CRÍTICO !!! Entrando en bucle infinito...\r\n");
			while (tx_tail != tx_head); _delay_ms(50);

			cli();
			while (1) {
				// Bucle infinito deliberado. El WDT morderá a los 2 segundos exactos.
			}
		}
	}
	return 0;
}
