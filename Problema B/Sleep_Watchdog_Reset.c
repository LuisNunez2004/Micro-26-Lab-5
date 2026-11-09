/*
 * B_03_WDT_Reset_Fisico.c
 *
 * Created: 8/11/2026 21:57:42
 * Author : rosel
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// --- Configuración de Pines Estrictos ---
#define LED_INTEGRADO PB5   // Pin 13 de Arduino Uno (LED 'L')
#define BOTON         PD2   // Pin 2 de Arduino Uno (Pulsador a GND)

// --- Variables Globales ---
// .noinit asegura que el compilador no limpie esta variable durante el arranque de C
uint8_t causa_reset __attribute__((section(".noinit")));

volatile uint8_t envio_listo = 1;
volatile uint8_t posicion_tx = 0;
const char * volatile mensaje_tx;
char buffer_diagnostico[80];

// =========================================================================
// 1. CAPTURA TEMPRANA DE LA CAUSA DEL RESET (Antes de entrar a main)
// =========================================================================
void capturar_causa_real(void) __attribute__((naked, section(".init0")));

void capturar_causa_real(void)
{
	// El bootloader Optiboot copia el MCUSR original en el registro r2 de la CPU.
	// Lo movemos a nuestra variable global mediante ensamblador en línea.
	__asm__ __volatile__("sts causa_reset, r2 \n\t");
	
	// Desactivamos inmediatamente el Watchdog para evitar bucles de reinicio.
	MCUSR = 0;
	wdt_disable();
}

// =========================================================================
// 2. CONTROLADOR DE LA UART POR INTERRUPCIONES
// =========================================================================
ISR(USART_UDRE_vect)
{
	char caracter = mensaje_tx[posicion_tx];
	if (caracter != '\0')
	{
		UDR0 = caracter;
		posicion_tx++;
	}
	else
	{
		UCSR0B &= ~(1 << UDRIE0);
		UCSR0B |= (1 << TXCIE0);
	}
}

ISR(USART_TX_vect)
{
	UCSR0B &= ~(1 << TXCIE0);
	envio_listo = 1;
}

void uart_configurar(void)
{
	PRR &= ~(1 << PRUSART0);
	UCSR0B = 0;
	UCSR0A = (1 << TXC0);
	
	// Configurar a 9600 baudios con un reloj de 16 MHz
	UBRR0H = 0;
	UBRR0L = 103;
	
	// Frame: 8 bits de datos, sin paridad, 1 bit de parada
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << TXEN0);
}

void uart_enviar(const char *texto)
{
	while (!envio_listo);
	
	uint8_t sreg_anterior = SREG;
	cli();
	
	mensaje_tx = texto;
	posicion_tx = 0;
	envio_listo = 0;
	
	UCSR0A = (1 << TXC0);
	UCSR0B = (1 << TXEN0) | (1 << UDRIE0);
	
	SREG = sreg_anterior;
}

// =========================================================================
// 3. PROGRAMA PRINCIPAL (LÓGICA DE CONTROL DE RESET)
// =========================================================================
int main(void)
{
	cli(); // Deshabilitar interrupciones para inicializar periféricos
	
	// Configurar pin del LED Integrado como salida
	DDRB |= (1 << LED_INTEGRADO);
	PORTB |= (1 << LED_INTEGRADO); // Encender el LED al arrancar
	
	// Configurar pulsador en D2 con su resistencia pull-up interna activa
	DDRD &= ~(1 << BOTON);
	PORTD |= (1 << BOTON);
	
	// Inicializar módulo de hardware UART
	uart_configurar();
	
	sei(); // Habilitar las interrupciones globales
	
	// --- EVALUACIÓN DE LA CAUSA DEL RESET ---
	// Analizamos si el bit WDRF (Watchdog Reset Flag - Bit 3) estaba activo en r2
	if (causa_reset & (1 << WDRF))
	{
		uart_enviar("\r\n[SISTEMA RECUPERADO] Reinicio provocado de forma exitosa por el Watchdog Timer.\r\n");
	}
	else
	{
		snprintf(buffer_diagnostico, sizeof(buffer_diagnostico),
		"\r\n[ARRANQUE NORMAL] Causa MCUSR capturada desde r2: 0x%02X\r\n", causa_reset);
		uart_enviar(buffer_diagnostico);
		uart_enviar("Presione el pulsador en D2 para simular una falla en el codigo...\r\n");
	}
	
	// Habilitar el Watchdog en modo RESET estricto (tiempo nominal de 2 segundos)
	wdt_enable(WDTO_2S);
	
	while (1)
	{
		// Verificar el estado del pulsador (Alto = No presionado debido a la Pull-Up)
		if (PIND & (1 << BOTON))
		{
			// El programa opera de forma saludable: Atendemos al WDT continuamente
			wdt_reset();
			_delay_ms(10); // Evitar ciclos infinitos a máxima velocidad de reloj
		}
		else
		{
			// --- SIMULACIÓN DE BLOQUEO CRÍTICO ---
			PORTB &= ~(1 << LED_INTEGRADO); // Apagar el LED L para dar feedback visual de congelamiento
			uart_enviar("!!! BLOQUEO DETECTADO !!! Entrando en bucle infinito deliberado...\r\n");
			
			// Esperamos un instante corto para asegurar que la UART vacíe el buffer de salida
			while (!envio_listo);
			_delay_ms(50);
			
			cli(); // Deshabilitamos interrupciones de forma absoluta para congelar el microcontrolador
			while (1)
			{
				// Bucle infinito vacío. No se ejecuta wdt_reset().
				// El Watchdog morderá de forma inevitable a los 2 segundos exactos.
			}
		}
	}
}

