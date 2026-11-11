
/*
 * LABORATORIO 5 - PROBLEMA B
 * ATmega328P - 16 MHz - PROTEUS
 *
 * PB0: LED verde
 * PB1: LED amarillo
 * PB2: Actuador opcional
 * PD2: Pulsador DESPERTAR
 * PC0: Potenciometro ADC
 * PC4: SDA
 * PC5: SCL
 *
 * PCF8574: direccion 0x20
 *
 * ACTIVO: lectura ADC y LCD
 * 10 segundos: STANDBY / IDLE
 * 30 segundos: POWER-DOWN
 * WDT interrupcion cada 8 segundos
 * Pulsador: despertar
 * Pulsacion larga: bloqueo y reset WDT
 *
 * UART: 9600 bps
 * Tramas [CDHH]
 * HH = XOR ASCII de C y D
 *
 * Sin switch-case.
 * Sin bibliotecas TWI externas.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

/* DEFINICIONES */

#define VERDE       PB0
#define AMARILLO    PB1
#define ACTUADOR    PB2
#define BOTON       PD2

#define LCD_ADDR    0x27U
#define RS          0x01U
#define EN          0x04U
#define BL          0x08U

#define ACTIVO      0U
#define STANDBY     1U
#define APAGADO     2U
#define SEGURO      3U

#define RX_N        32U

/* CAUSA DE REINICIO */

uint8_t causa_reset
__attribute__((section(".noinit")));

void capturar_reset(void)
__attribute__((naked, used, section(".init3")));

void capturar_reset(void)
{
    causa_reset = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

/* VARIABLES GLOBALES */

static volatile uint32_t ms = 0;

static volatile uint8_t evento_boton = 0;
static volatile uint8_t evento_wdt = 0;
static volatile uint8_t despertadores_wdt = 0;

static volatile uint8_t rx[RX_N];
static volatile uint8_t rx_in = 0;
static volatile uint8_t rx_out = 0;

static uint8_t estado = ACTIVO;
static uint8_t lcd_ok = 1;
static uint8_t luz = BL;

static uint32_t ultima_actividad = 0;
static uint16_t ultimo_adc = 0;

/* INTERRUPCIONES */

ISR(TIMER1_COMPA_vect)
{
    ms += 100UL;
}

ISR(INT0_vect)
{
    evento_boton = 1;
    EIMSK &= (uint8_t)~(1U << INT0);
}

ISR(WDT_vect)
{
    evento_wdt = 1;

    if (despertadores_wdt != 255U)
        despertadores_wdt++;
}

ISR(USART_RX_vect)
{
    uint8_t c = UDR0;

    uint8_t siguiente =
        (uint8_t)((rx_in + 1U) & (RX_N - 1U));

    if (siguiente != rx_out)
    {
        rx[rx_in] = c;
        rx_in = siguiente;
    }
}

/* TIMER1 - BASE DE TIEMPO */

static uint32_t ahora_ms(void)
{
    uint32_t t;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        t = ms;
    }

    return t;
}

static void iniciar_timer1(void)
{
    PRR &= (uint8_t)~(1U << PRTIM1);

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = 6249U;

    TIFR1 = (1U << OCF1A);
    TIMSK1 = (1U << OCIE1A);

    TCCR1B =
        (1U << WGM12) |
        (1U << CS12);
}

/* ADC */

static void iniciar_adc(void)
{
    PRR &= (uint8_t)~(1U << PRADC);

    ADMUX = (1U << REFS0);

    ADCSRA =
        (1U << ADEN) |
        (1U << ADPS2) |
        (1U << ADPS1) |
        (1U << ADPS0);

    DIDR0 |= (1U << ADC0D);
}

static uint16_t leer_adc(void)
{
    ADCSRA |= (1U << ADSC);

    while (ADCSRA & (1U << ADSC))
    {
    }

    return ADC;
}

/* UART 9600 BAUDIOS */

static void iniciar_uart(void)
{
    PRR &= (uint8_t)~(1U << PRUSART0);

    UBRR0H = 0;
    UBRR0L = 103;

    UCSR0A = 0;

    UCSR0C =
        (1U << UCSZ01) |
        (1U << UCSZ00);

    UCSR0B =
        (1U << RXEN0) |
        (1U << TXEN0) |
        (1U << RXCIE0);
}

static void uart_caracter(char c)
{
    uint16_t espera = 60000U;

    while (!(UCSR0A & (1U << UDRE0)) && espera)
    {
        espera--;
    }

    if (espera)
        UDR0 = (uint8_t)c;
}

static void uart_texto(const char *p)
{
    while (*p)
        uart_caracter(*p++);
}

/* I2C */

static uint8_t twi_esperar(void)
{
    uint16_t espera = 30000U;

    while (!(TWCR & (1U << TWINT)) && espera)
    {
        espera--;
    }

    return espera != 0U;
}

static void iniciar_i2c(void)
{
    PRR &= (uint8_t)~(1U << PRTWI);

    TWSR = 0;
    TWBR = 72U;
    TWCR = (1U << TWEN);
}

static void twi_stop(void)
{
    TWCR =
        (1U << TWINT) |
        (1U << TWSTO) |
        (1U << TWEN);
}

static uint8_t twi_start(void)
{
    TWCR =
        (1U << TWINT) |
        (1U << TWSTA) |
        (1U << TWEN);

    if (!twi_esperar())
        return 0U;

    return
        ((TWSR & 0xF8U) == 0x08U ||
         (TWSR & 0xF8U) == 0x10U);
}

static uint8_t twi_dato(uint8_t dato)
{
    TWDR = dato;

    TWCR =
        (1U << TWINT) |
        (1U << TWEN);

    if (!twi_esperar())
        return 0U;

    return
        ((TWSR & 0xF8U) == 0x18U ||
         (TWSR & 0xF8U) == 0x28U);
}

/* LCD POR PCF8574 */

static void pcf(uint8_t dato)
{
    uint8_t bien;

    if (!lcd_ok)
        return;

    if (!twi_start())
    {
        twi_stop();
        lcd_ok = 0;
        return;
    }

    bien = twi_dato(
        (uint8_t)(LCD_ADDR << 1));

    if (bien)
        bien = twi_dato(dato);

    twi_stop();

    if (!bien)
        lcd_ok = 0;
}

static void lcd_nibble(uint8_t dato, uint8_t rs)
{
    uint8_t p =
        (uint8_t)((dato & 0xF0U) | luz | rs);

    pcf((uint8_t)(p | EN));
    _delay_us(2);

    pcf(p);
    _delay_us(50);
}

static void lcd_byte(uint8_t dato, uint8_t rs)
{
    lcd_nibble(dato, rs);

    lcd_nibble((uint8_t)(dato << 4), rs);
}

static void lcd_cmd(uint8_t dato)
{
    lcd_byte(dato, 0);

    if (dato == 1U || dato == 2U)
        _delay_ms(2);
}

static void iniciar_lcd(void)
{
    lcd_ok = 1;
    luz = BL;

    _delay_ms(40);

    lcd_nibble(0x30, 0);
    _delay_ms(5);

    lcd_nibble(0x30, 0);
    _delay_us(200);

    lcd_nibble(0x30, 0);
    lcd_nibble(0x20, 0);

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
}

static void lcd_linea(
    uint8_t direccion,
    const char *texto)
{
    uint8_t n = 0;

    if (!lcd_ok)
        return;

    lcd_cmd(direccion);

    while (*texto && n < 16U)
    {
        lcd_byte((uint8_t)*texto++, RS);
        n++;
    }

    while (n < 16U)
    {
        lcd_byte(' ', RS);
        n++;
    }
}

static void lcd_mostrar(
    const char *a,
    const char *b)
{
    if (!lcd_ok)
    {
        iniciar_i2c();
        iniciar_lcd();
    }

    lcd_linea(0x80, a);
    lcd_linea(0xC0, b);
}

/* CONFIGURACION DEL WATCHDOG */

static void wdt_config(uint8_t valor)
{
    uint8_t previo = SREG;

    uint8_t apertura =
        (1U << WDCE) |
        (1U << WDE);

    cli();

    wdt_reset();

    MCUSR &= (uint8_t)~(1U << WDRF);

    __asm__ __volatile__(
        "sts %[reg], %[a]\n\t"
        "sts %[reg], %[b]\n\t"
        :
        : [reg] "n" (_SFR_MEM_ADDR(WDTCSR)),
          [a] "r" (apertura),
          [b] "r" (valor)
        : "memory"
    );

    SREG = previo;
}

/* WDT MODO RESET: 2 SEGUNDOS */

static void wdt_reset_2s(void)
{
    wdt_config(
        (1U << WDE) |
        (1U << WDP2) |
        (1U << WDP1) |
        (1U << WDP0)
    );
}

/* WDT MODO INTERRUPCION: 8 SEGUNDOS */

static void wdt_interrupt_8s(void)
{
    wdt_config(
        (1U << WDIE) |
        (1U << WDP3) |
        (1U << WDP0)
    );
}

/* CONFIGURACION INT0 */

static void int0_standby(void)
{
    EIMSK &= (uint8_t)~(1U << INT0);

    EICRA =
        (uint8_t)(
            (EICRA &
             (uint8_t)~(
                (1U << ISC01) |
                (1U << ISC00)
             )) |
            (1U << ISC01)
        );

    EIFR = (1U << INTF0);
    evento_boton = 0;

    EIMSK |= (1U << INT0);
}

static void int0_powerdown(void)
{
    EIMSK &= (uint8_t)~(1U << INT0);

    EICRA &=
        (uint8_t)~(
            (1U << ISC01) |
            (1U << ISC00)
        );

    EIFR = (1U << INTF0);
    evento_boton = 0;

    EIMSK |= (1U << INT0);
}

/* PRUEBA DE BLOQUEO POR WDT */

static void bloquear_por_wdt(void)
{
    lcd_mostrar(
        "BLOQUEO SIMULADO",
        "RESET VIA WDT"
    );

    PORTB &= (uint8_t)~(1U << VERDE);
    PORTB |= (1U << AMARILLO);

    wdt_reset_2s();

    cli();

    for (;;)
    {
        /* No alimentar al Watchdog */
        __asm__ __volatile__("nop");
    }
}

/* PROTOCOLO UART */

static uint8_t hex_valor(uint8_t c)
{
    if (c >= '0' && c <= '9')
        return (uint8_t)(c - '0');

    if (c >= 'A' && c <= 'F')
        return (uint8_t)(c - 'A' + 10);

    if (c >= 'a' && c <= 'f')
        return (uint8_t)(c - 'a' + 10);

    return 255U;
}

static void comando(uint8_t c, uint8_t d)
{
    char linea[40];

    if (c == '!' && d == '1')
    {
        estado = SEGURO;

        PORTB &= (uint8_t)~(1U << ACTUADOR);

        uart_texto("[EMERGENCIA]\r\n");
    }

    else if (c == 'S' && d == '0')
    {
        const char *e =
            (estado == ACTIVO) ? "ACTIVO" :
            (estado == STANDBY) ? "STANDBY" :
            (estado == APAGADO) ? "APAGADO" :
            "SEGURO";

        snprintf(
            linea,
            sizeof(linea),
            "[ESTADO:%s]\r\n",
            e
        );

        uart_texto(linea);
    }

    else if (c == 'R' && d == '0' &&
             estado != SEGURO)
    {
        if (!(ADCSRA & (1U << ADEN)))
            iniciar_adc();

        ultimo_adc = leer_adc();

        snprintf(
            linea,
            sizeof(linea),
            "[ADC:%u]\r\n",
            (unsigned)ultimo_adc
        );

        uart_texto(linea);
    }

    else if (c == 'A' && d == '0')
    {
        snprintf(
            linea,
            sizeof(linea),
            "[WDT:%u RESET:%u]\r\n",
            (unsigned)despertadores_wdt,
            (unsigned)(
                (causa_reset & (1U << WDRF)) != 0U
            )
        );

        uart_texto(linea);
    }

    else if (c == 'E' &&
             estado != SEGURO &&
             (d == 'A' || d == 'S' || d == 'P'))
    {
        estado =
            (d == 'A') ? ACTIVO :
            (d == 'S') ? STANDBY :
            APAGADO;

        uart_texto("[ESTADO OK]\r\n");
    }

    else if (c == 'L' &&
             estado != SEGURO &&
             (d == '0' || d == '1'))
    {
        if (d == '1')
            PORTB |= (1U << ACTUADOR);
        else
            PORTB &= (uint8_t)~(1U << ACTUADOR);

        uart_texto("[ACTUADOR OK]\r\n");
    }

    else
    {
        uart_texto("[COMANDO ERROR]\r\n");
        return;
    }

    ultima_actividad = ahora_ms();

    if (estado == STANDBY &&
        !(c == 'E' && d == 'S'))
    {
        estado = ACTIVO;
    }
}

static void procesar_uart(void)
{
    static uint8_t trama[6];
    static uint8_t indice = 0;

    uint8_t maximo = RX_N;

    while (rx_out != rx_in && maximo--)
    {
        uint8_t c = rx[rx_out];
        uint8_t h, l;

        rx_out =
            (uint8_t)((rx_out + 1U) &
                      (RX_N - 1U));

        if (c == '[')
        {
            trama[0] = '[';
            indice = 1;
            continue;
        }

        if (!indice)
            continue;

        trama[indice++] = c;

        if (indice < 6U)
            continue;

        indice = 0;

        h = hex_valor(trama[3]);
        l = hex_valor(trama[4]);

        if (trama[5] != ']' ||
            h > 15U ||
            l > 15U ||
            (uint8_t)(trama[1] ^ trama[2]) !=
            (uint8_t)((h << 4) | l))
        {
            uart_texto("[TRAMA ERROR]\r\n");
        }
        else
        {
            comando(trama[1], trama[2]);
        }
    }
}

/* MODO POWER-DOWN */

static void entrar_powerdown(void)
{
    lcd_mostrar(
        "MODO POWER-DOWN",
        "WDT CADA 8 SEG"
    );

    uart_texto("[POWER-DOWN]\r\n");

    luz = 0;
    pcf(0);

    PORTB &=
        (uint8_t)~(
            (1U << VERDE) |
            (1U << AMARILLO) |
            (1U << ACTUADOR)
        );

    /* Deshabilitar perifericos */

    UCSR0B = 0;
    ADCSRA &= (uint8_t)~(1U << ADEN);
    TWCR = 0;

    ACSR |= (1U << ACD);

    PRR |=
        (1U << PRADC) |
        (1U << PRUSART0) |
        (1U << PRSPI) |
        (1U << PRTIM1) |
        (1U << PRTIM0) |
        (1U << PRTIM2) |
        (1U << PRTWI);

    int0_powerdown();

    evento_wdt = 0;

    /* Watchdog modo interrupcion */

    wdt_interrupt_8s();

    for (;;)
    {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);

        cli();
        sleep_enable();
        sei();

        sleep_cpu();
        sleep_disable();

        /* DESPERTAR CON PULSADOR */

        if (evento_boton ||
            !(PIND & (1U << BOTON)))
        {
            EIMSK &= (uint8_t)~(1U << INT0);
            evento_boton = 0;

            wdt_reset_2s();

            PRR &=
                (uint8_t)~(
                    (1U << PRADC) |
                    (1U << PRUSART0) |
                    (1U << PRTWI) |
                    (1U << PRTIM1)
                );

            iniciar_timer1();
            iniciar_adc();
            iniciar_uart();
            iniciar_i2c();
            iniciar_lcd();

            estado = ACTIVO;
            ultima_actividad = ahora_ms();

            lcd_mostrar(
                "DESPERTO INT0",
                "ESTADO: ACTIVO"
            );

            uart_texto("[DESPERTO INT0]\r\n");

            wdt_reset();

            return;
        }

        /* DESPERTAR CADA 8 SEGUNDOS */

        if (evento_wdt)
        {
            evento_wdt = 0;

            iniciar_adc();
            _delay_us(100);

            ultimo_adc = leer_adc();

            ADCSRA &= (uint8_t)~(1U << ADEN);
            PRR |= (1U << PRADC);

            PORTB |= (1U << AMARILLO);

            _delay_ms(60);

            PORTB &= (uint8_t)~(1U << AMARILLO);
        }
    }
}

/* PROGRAMA PRINCIPAL */

int main(void)
{
    uint32_t t;
    uint32_t ultima_lcd;
    uint32_t inicio_pulsacion = 0;

    uint8_t presion_anterior = 0;
    uint8_t mostrar_standby = 1;
    uint8_t mostrar_seguro = 1;

    char linea[17];

    cli();

    /* LED */

    DDRB |=
        (1U << VERDE) |
        (1U << AMARILLO) |
        (1U << ACTUADOR);

    PORTB =
        (uint8_t)(
            (PORTB | (1U << VERDE)) &
            (uint8_t)~(
                (1U << AMARILLO) |
                (1U << ACTUADOR)
            )
        );

    /* PULSADOR */

    DDRD &= (uint8_t)~(1U << BOTON);
    PORTD |= (1U << BOTON);

    EIMSK &= (uint8_t)~(1U << INT0);

    /* PERIFERICOS */

    iniciar_timer1();
    iniciar_adc();
    iniciar_uart();
    iniciar_i2c();
    iniciar_lcd();

    sei();

    /* CAUSA DEL RESET */

    if (causa_reset & (1U << WDRF))
    {
        lcd_mostrar(
            "SYS RECUPERADO",
            "RESET VIA WDT"
        );

        uart_texto("[RECUPERADO WDT]\r\n");
    }
    else
    {
        lcd_mostrar(
            "ARRANQUE NORMAL",
            "SISTEMA ACTIVO"
        );

        uart_texto("[ARRANQUE NORMAL]\r\n");
    }

    _delay_ms(800);

    t = ahora_ms();

    ultima_actividad = t;
    ultima_lcd = t;

    wdt_reset_2s();
    wdt_reset();

    /* BUCLE PRINCIPAL */

    for (;;)
    {
        uint8_t pulsado;

        t = ahora_ms();

        procesar_uart();

        pulsado =
            (uint8_t)(
                (PIND & (1U << BOTON)) == 0U
            );

        /* DETECTAR NUEVA PULSACION */

        if (pulsado && !presion_anterior)
        {
            inicio_pulsacion = t;
            ultima_actividad = t;

            if (estado == STANDBY ||
                estado == SEGURO)
            {
                estado = ACTIVO;
            }
        }

        presion_anterior = pulsado;

        /* ESTADO ACTIVO */

        if (estado == ACTIVO)
        {
            EIMSK &= (uint8_t)~(1U << INT0);

            if (!(ADCSRA & (1U << ADEN)))
                iniciar_adc();

            PORTB |= (1U << VERDE);
            PORTB &= (uint8_t)~(1U << AMARILLO);

            mostrar_standby = 1;
            mostrar_seguro = 1;

            /* BLOQUEO POR PULSACION LARGA */

            if (pulsado &&
                (uint32_t)(t - inicio_pulsacion)
                    >= 3000UL)
            {
                bloquear_por_wdt();
            }

            /* ACTUALIZACION ADC Y LCD */

            if ((uint32_t)(t - ultima_lcd)
                >= 1000UL)
            {
                ultima_lcd = t;

                ultimo_adc = leer_adc();

                snprintf(
                    linea,
                    sizeof(linea),
                    "ADC: %u",
                    (unsigned)ultimo_adc
                );

                lcd_mostrar(
                    "ESTADO: ACTIVO",
                    linea
                );
            }

            /* PASO A STANDBY */

            if ((uint32_t)(t - ultima_actividad)
                >= 10000UL)
            {
                estado = STANDBY;
                mostrar_standby = 1;
            }
        }

        /* ESTADO STANDBY */

        else if (estado == STANDBY)
        {
            PORTB &= (uint8_t)~(1U << VERDE);
            PORTB |= (1U << AMARILLO);

            if (mostrar_standby)
            {
                mostrar_standby = 0;

                lcd_mostrar(
                    "ESTADO: STANDBY",
                    "Esperando..."
                );

                uart_texto("[STANDBY]\r\n");

                int0_standby();

                ADCSRA &= (uint8_t)~(1U << ADEN);

                PRR |=
                    (1U << PRADC) |
                    (1U << PRSPI) |
                    (1U << PRTIM0) |
                    (1U << PRTIM2);
            }

            /* DESPERTAR DESDE STANDBY */

            if (evento_boton || pulsado)
            {
                evento_boton = 0;

                EIMSK &=
                    (uint8_t)~(1U << INT0);

                estado = ACTIVO;
                ultima_actividad = t;

                iniciar_adc();
            }

            /* PASO A POWER-DOWN */

            else if (
                (uint32_t)(t - ultima_actividad)
                >= 30000UL)
            {
                estado = APAGADO;

                entrar_powerdown();

                ultima_lcd = ahora_ms();

                /* CORRECCION DEL DESPERTAR */
                inicio_pulsacion = ahora_ms();
                presion_anterior = 1;
            }

            /* SLEEP IDLE */

            else
            {
                set_sleep_mode(SLEEP_MODE_IDLE);

                sleep_enable();
                sleep_cpu();
                sleep_disable();
            }
        }

        /* ESTADO APAGADO */

        else if (estado == APAGADO)
        {
            entrar_powerdown();
        }

        /* ESTADO SEGURO */

        else if (estado == SEGURO)
        {
            PORTB &=
                (uint8_t)~(
                    (1U << VERDE) |
                    (1U << AMARILLO) |
                    (1U << ACTUADOR)
                );

            if (mostrar_seguro)
            {
                mostrar_seguro = 0;

                lcd_mostrar(
                    "ESTADO SEGURO",
                    "PULSE DESPERTAR"
                );
            }
        }

        /* ALIMENTAR WDT EN FUNCIONAMIENTO NORMAL */

        wdt_reset();

        _delay_ms(5);
    }
}

