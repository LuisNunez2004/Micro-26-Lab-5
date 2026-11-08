/*
 * Lab 5 A.c con LCD I2C integrada
 *
 * Created: 6/11/2026 1:07:59
 * Author : UTEC-5695
 */

#define F_CPU 16000000UL
#define PCF8574 0x27                            

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <util/atomic.h>

// Librerías de I2C, LCD y DHT11
#include "i2c.h"
#include "lcd_i2c.h"
#include "dht11.h"

volatile uint8_t flag_100ms = 0;
volatile uint8_t flag_pulsador = 0;
volatile uint8_t buzzer_activo = 0;

// Variables globales para almacenar las lecturas del DHT11
uint8_t temperatura_actual = 0;
uint8_t humedad_actual = 0;

#define LED_PIN     PB5
#define LED_DDR     DDRB
#define LED_PORT    PORTB

#define BUZZER_PIN  PD3
#define BUZZER_DDR  DDRD
#define BUZZER_PORT PORTD

void estado_activo(void);
void estado_standby(void);
void estado_apagado(void);

typedef void (*fsm_state_t)(void);
fsm_state_t current_state = estado_activo;

volatile uint32_t ms_ticks = 0; 
uint32_t last_activity_tick = 0;
uint32_t buzzer_off_tick = 0; 

uint8_t lcd_estado_actual_previo = 0xFF;

void estado_activo(void) {
    PRR = 0x00; 
    if ((ms_ticks - last_activity_tick) >= 10000) {
        current_state = estado_standby;
        lcd_estado_actual_previo = 0xFF; 
    }
}

void estado_standby(void) {
    if (lcd_estado_actual_previo != 1) {
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_String("Esperando...");
        LCD_SetCursor(1, 0);
        LCD_String("Modo Standby");
        lcd_estado_actual_previo = 1;
    }

    set_sleep_mode(SLEEP_MODE_IDLE);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sleep_enable();
    }
    sleep_cpu();
    sleep_disable();

    if ((ms_ticks - last_activity_tick) >= 30000) {
        current_state = estado_apagado;
        lcd_estado_actual_previo = 0xFF;
    }
}

void estado_apagado(void) {
    if (lcd_estado_actual_previo != 2) {
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_String("Sistema Apagado");
        lcd_estado_actual_previo = 2;
    }
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    ADCSRA &= ~(1 << ADEN); 
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sleep_enable();
        PRR |= (1 << PRADC); 
    }
    sleep_cpu();
    sleep_disable();
}

void init_timers (void) {
    TCCR0A = (1 << WGM01); 
    TCCR0B = (1 << CS01) | (1 << CS00); 
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A); 
    
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12); 
    OCR1A = 6249; 
    TIMSK1 = (1 << OCIE1A);
}

void init_peripherals(void) {
    LED_DDR |= (1 << LED_PIN);
    
    ADMUX = (1 << REFS0); 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EIMSK |= (1 << INT0);
    PORTD |= (1 << PD2);
    
    I2C_Init();
    LCD_Init();
    DHT_Init(); // Inicializar sensor DHT11
    
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_String("Sistema Iniciado");
    _delay_ms(1000);
}

ISR(TIMER0_COMPA_vect) {
    ms_ticks++;
}

ISR(TIMER1_COMPA_vect) {
    flag_100ms = 1;
}

ISR(INT0_vect) {
    flag_pulsador = 1;
}

int main(void) {
    init_timers(); 
    init_peripherals();

    BUZZER_DDR |= (1 << BUZZER_PIN);
    sei();
    
    uint32_t last_led = 0;
    uint32_t last_lcd = 0;
    uint32_t last_dht = 0;
    
    while (1) {
        uint32_t current_ms;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            current_ms = ms_ticks;
        }
        
        current_state();
        
        // Parpadeo del LED cada 200 ms[cite: 4]
        if (current_ms - last_led >= 200) {
            last_led = current_ms;
            LED_PORT ^= (1 << LED_PIN);
        }
        
        if (flag_100ms) {
            flag_100ms = 0;
            ADCSRA |= (1 << ADSC); 
        }
        
        // Actualización de la LCD cada 500 ms (Cooperativa)[cite: 4, 51]
        if (current_state == estado_activo && (current_ms - last_lcd >= 500)) {
            last_lcd = current_ms;
            
            uint16_t valor_adc = 0;
            if (!(ADCSRA & (1 << ADSC))) {
                valor_adc = ADC;
            }
            
            char lcd_buffer[17];
            LCD_Clear();
            LCD_SetCursor(0, 0);
            sprintf(lcd_buffer, "ADC:%u T:%uC", valor_adc, temperatura_actual);
            LCD_String(lcd_buffer);
            
            LCD_SetCursor(1, 0);
            sprintf(lcd_buffer, "Humedad: %u %%", humedad_actual);
            LCD_String(lcd_buffer);
        }
        
        // Lectura del sensor DHT11 cada 2000 ms (2 s como exige la norma del sensor)[cite: 3, 4, 43]
        if (current_ms - last_dht >= 2000) {
            last_dht = current_ms;
            uint8_t t, h;
            if (DHT_GetData(&t, &h)) {
                temperatura_actual = t;
                humedad_actual = h;
            }
        }
        
        // Control del buzzer
        if (buzzer_activo && (current_ms >= buzzer_off_tick)) {
            buzzer_activo = 0;
        }
        
        if (buzzer_activo) {
            BUZZER_PORT |= (1 << BUZZER_PIN);
        } else {
            BUZZER_PORT &= ~(1 << BUZZER_PIN);
        }
        
        // Atención del pulsador (evento externo inmediato)[cite: 4]
        if (flag_pulsador) {
            flag_pulsador = 0;
            last_activity_tick = current_ms; 
            
            if (current_state == estado_apagado || current_state == estado_standby) {
                current_state = estado_activo;
                lcd_estado_actual_previo = 0xFF;
            }
            
            buzzer_activo = 1;
            buzzer_off_tick = current_ms + 500; 
        }
    }
}
