/*
 * LABORATORIO 5 - PROBLEMA A - ATmega328P / Arduino Uno (16 MHz)
 * LED PB5 (D13) | Buzzer PD3 (D3) | Pulsador INT0 PD2 (D2)
 * DHT11 PD4 (D4) | ADC PC0 (A0) | LCD I2C PC4/PC5 (A4/A5)
 * LCD PCF8574 = 0x27 (lcd_i2c.h)
 * ACTIVO 0-10 s, STANDBY 10-30 s, POWER-DOWN a los 30 s.
 * El pulsador despierta desde STANDBY y POWER-DOWN.
 */
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c.h"
#include "lcd_i2c.h"
#include "dht11.h"

#define LED_PIN       PB5
#define BUZZER_PIN    PD3
#define BOTON_PIN     PD2

static volatile uint32_t ms_ticks = 0;
static volatile uint8_t flag_100ms = 0;
static volatile uint8_t flag_pulsador = 0;

static uint32_t ultima_actividad = 0;
static uint32_t ultimo_led = 0;
static uint32_t ultima_lcd = 0;
static uint32_t ultimo_dht = 0;
static uint32_t fin_buzzer = 0;

static uint16_t adc_actual = 0;
static uint8_t temperatura = 0;
static uint8_t humedad = 0;
static uint8_t dht_valido = 0;
static uint8_t buzzer_activo = 0;
static uint8_t pantalla_estado = 0xFF;

static void estado_activo(void);
static void estado_standby(void);
static void estado_apagado(void);
typedef void (*fsm_state_t)(void);
static fsm_state_t estado_actual = estado_activo;

ISR(TIMER0_COMPA_vect)
{
    ms_ticks++;  /* Un milisegundo por interrupcion */
}

ISR(TIMER1_COMPA_vect)
{
    flag_100ms = 1;
}

/* Desarmar INT0 impide una tormenta de interrupciones por nivel bajo. */
ISR(INT0_vect)
{
    EIMSK &= (uint8_t)~(1U << INT0);
    flag_pulsador = 1;
}

static uint32_t ahora_ms(void)
{
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = ms_ticks;
    }
    return t;
}

static void iniciar_timers(void)
{
    /* TIMER0: 1 ms; 16 MHz / 64 / (249+1) */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    TCNT0 = 0;
    OCR0A = 249;
    TIMSK0 |= (1U << OCIE0A);

    /* TIMER1: 100 ms; 16 MHz / 256 / (6249+1) */
    TCCR1A = 0;
    TCCR1B = (1U << WGM12) | (1U << CS12);
    TCNT1 = 0;
    OCR1A = 6249;
    TIMSK1 |= (1U << OCIE1A);
}

static void iniciar_componentes(void)
{
    DDRB |= (1U << LED_PIN);
    PORTB &= (uint8_t)~(1U << LED_PIN);

    DDRD |= (1U << BUZZER_PIN);
    PORTD &= (uint8_t)~(1U << BUZZER_PIN);

    /* D2: pulsador contra GND y pull-up interno. */
    DDRD &= (uint8_t)~(1U << BOTON_PIN);
    PORTD |= (1U << BOTON_PIN);

    /* INT0 con disparo por nivel bajo para despertar de POWER-DOWN. */
    EICRA &= (uint8_t)~((1U << ISC01) | (1U << ISC00));
    EIFR = (1U << INTF0);
    EIMSK |= (1U << INT0);

    PRR &= (uint8_t)~(1U << PRADC);
    ADMUX = (1U << REFS0); /* Referencia AVcc, canal ADC0. */
    ADCSRA = (1U << ADEN) | (1U << ADPS2) |
             (1U << ADPS1) | (1U << ADPS0);

    I2C_Init();
    LCD_Init();
    DHT_Init();
}

static uint16_t leer_adc(void)
{
    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) { }
    return ADC;
}

/* Rellenar los 16 caracteres: sin Clear cada medio segundo. */
static void escribir_linea(uint8_t fila, const char *texto)
{
    uint8_t terminado = 0;
    LCD_SetCursor(fila, 0);
    for (uint8_t i = 0; i < 16; i++) {
        if (!terminado && texto[i] != '\0') {
            LCD_Char((uint8_t)texto[i]);
        } else {
            terminado = 1;
            LCD_Char(' ');
        }
    }
}

static void mostrar_medidas(void)
{
    char linea[17];
    snprintf(linea, sizeof(linea), "ADC:%4u T:%2uC",
             (unsigned)adc_actual, (unsigned)temperatura);
    escribir_linea(0, linea);

    if (dht_valido) {
        snprintf(linea, sizeof(linea), "Humedad:%3u %%", (unsigned)humedad);
        escribir_linea(1, linea);
    } else {
        escribir_linea(1, "DHT SIN LECTURA");
    }
}

static void activar_modo_activo(uint32_t t)
{
    PRR &= (uint8_t)~(1U << PRADC);
    ADCSRA |= (1U << ADEN);
    estado_actual = estado_activo;
    ultima_actividad = t;
    pantalla_estado = 0xFF;
    ultima_lcd = t - 500UL; /* Mostrar al siguiente ciclo. */
    ultimo_dht = t - 2000UL;
}

static void procesar_pulsador(uint32_t t)
{
    if (flag_pulsador) {
        flag_pulsador = 0;
        activar_modo_activo(t);
        buzzer_activo = 1;
        fin_buzzer = t + 500UL;
    }

    /* Volver a habilitar INT0 solo al soltar D2. */
    if ((PIND & (1U << BOTON_PIN)) && !(EIMSK & (1U << INT0))) {
        EIFR = (1U << INTF0);
        EIMSK |= (1U << INT0);
    }
}

static void estado_activo(void)
{
    uint32_t t = ahora_ms();

    if (pantalla_estado != 0) {
        LCD_Clear();
        pantalla_estado = 0;
        ultima_lcd = t - 500UL;
    }

    if ((uint32_t)(t - ultimo_led) >= 200UL) {
        ultimo_led = t;
        PORTB ^= (1U << LED_PIN);
    }

    if (flag_100ms) {
        flag_100ms = 0;
        /* Lectura COMPLETA, no mostrar cero si esta convirtiendo. */
        adc_actual = leer_adc();
    }

    if ((uint32_t)(t - ultimo_dht) >= 2000UL) {
        uint8_t temp, hum;
        ultimo_dht = t;
        if (DHT_GetData(&temp, &hum)) {
            temperatura = temp;
            humedad = hum;
            dht_valido = 1;
        } else {
            dht_valido = 0;
        }
    }

    if ((uint32_t)(t - ultima_lcd) >= 500UL) {
        ultima_lcd = t;
        mostrar_medidas();
    }

    if ((uint32_t)(t - ultima_actividad) >= 10000UL) {
        estado_actual = estado_standby;
        pantalla_estado = 0xFF;
    }
}

static void estado_standby(void)
{
    uint32_t t = ahora_ms();

    if (pantalla_estado != 1) {
        PORTB &= (uint8_t)~(1U << LED_PIN);
        LCD_Clear();
        escribir_linea(0, "Esperando...");
        escribir_linea(1, "Modo Standby");
        pantalla_estado = 1;
    }

    if ((uint32_t)(t - ultima_actividad) >= 30000UL) {
        estado_actual = estado_apagado;
        pantalla_estado = 0xFF;
        return;
    }

    /* El TIMER0 sigue activo y permite contar hasta los 30 s. */
    set_sleep_mode(SLEEP_MODE_IDLE);
    cli();
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}

static void estado_apagado(void)
{
    if (pantalla_estado != 2) {
        PORTB &= (uint8_t)~(1U << LED_PIN);
        PORTD &= (uint8_t)~(1U << BUZZER_PIN);
        buzzer_activo = 0;

        LCD_Clear();
        escribir_linea(0, "Sistema Apagado");
        escribir_linea(1, "Pulse D2");
        pantalla_estado = 2;
        _delay_ms(500);

        /* PCF8574: bit BL = 0. Escribir 0 apaga backlight. */
        LCD_Write_I2C(0);
    }

    ADCSRA &= (uint8_t)~(1U << ADEN);
    PRR |= (1U << PRADC);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();

    /* Si el pulsador ya fue detectado, no dormir de nuevo. */
    if (!flag_pulsador && (PIND & (1U << BOTON_PIN))) {
        EIFR = (1U << INTF0);
        EIMSK |= (1U << INT0);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    } else {
        sei();
    }

    /* Reanudar perifericos y restaurar pantalla tras INT0. */
    PRR &= (uint8_t)~(1U << PRADC);
    ADCSRA |= (1U << ADEN);
    I2C_Init();
    LCD_Init();

    activar_modo_activo(ahora_ms());
    /* El main procesara flag_pulsador y activara el buzzer. */
}

int main(void)
{
    cli();
    iniciar_timers();
    iniciar_componentes();

    LCD_Clear();
    escribir_linea(0, "Sistema Iniciado");
    escribir_linea(1, "Problema A");
    _delay_ms(800);

    ultimo_dht = ahora_ms() - 2000UL;
    ultima_lcd = ahora_ms() - 500UL;
    sei();

    for (;;) {
        uint32_t t = ahora_ms();

        procesar_pulsador(t);

        if (buzzer_activo && (int32_t)(t - fin_buzzer) >= 0)
            buzzer_activo = 0;

        if (buzzer_activo)
            PORTD |= (1U << BUZZER_PIN);
        else
            PORTD &= (uint8_t)~(1U << BUZZER_PIN);

        estado_actual();
    }
}
