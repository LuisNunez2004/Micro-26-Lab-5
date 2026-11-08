/*
 * lcd_i2c.h
 *
 * Created: 23/10/2026 17:28:30
 *  Author: UTEC-5695
 */ 


#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <util/delay.h>
#include "i2c.h"

#define LCD_I2C_ADDR 0x27  

// Definición de bits del PCF8574
#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04
#define LCD_BL 0x08        // Backlight

void LCD_Write_I2C(uint8_t data) {
	I2C_Start();
	I2C_Write(LCD_I2C_ADDR << 1);
	I2C_Write(data);
	I2C_Stop();
}

void LCD_Pulse_Enable(uint8_t data) {
	LCD_Write_I2C(data | LCD_EN);
	_delay_us(1);
	LCD_Write_I2C(data & ~LCD_EN);
	_delay_us(50);
}

void LCD_Send_Nibble(uint8_t nibble, uint8_t mode) {
	uint8_t data = (nibble & 0xF0) | mode | LCD_BL;
	LCD_Write_I2C(data);
	LCD_Pulse_Enable(data);
}

void LCD_Command(uint8_t cmd) {
	LCD_Send_Nibble(cmd & 0xF0, 0);
	LCD_Send_Nibble((cmd << 4) & 0xF0, 0);
	if (cmd < 4) _delay_ms(2);
	else _delay_us(50);
}

void LCD_Char(uint8_t data_char) {
	LCD_Send_Nibble(data_char & 0xF0, LCD_RS);
	LCD_Send_Nibble((data_char << 4) & 0xF0, LCD_RS);
	_delay_us(50);
}

void LCD_Init(void) {
	I2C_Init();
	_delay_ms(50);
	
	// Secuencia de inicialización en modo 4 bits
	LCD_Send_Nibble(0x30, 0);
	_delay_ms(5);
	LCD_Send_Nibble(0x30, 0);
	_delay_us(150);
	LCD_Send_Nibble(0x30, 0);
	LCD_Send_Nibble(0x20, 0); // Modo 4 bits
	
	LCD_Command(0x28); // 2 líneas, matriz 5x8
	LCD_Command(0x0C); // Display encendido, cursor apagado
	LCD_Command(0x06); // Incremento automático del cursor
	LCD_Command(0x01); // Limpiar pantalla
	_delay_ms(2);
}

void LCD_Clear(void) {
	LCD_Command(0x01);
	_delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
	uint8_t address = (row == 0) ? 0x80 : 0xC0;
	address += col;
	LCD_Command(address);
}

void LCD_String(const char *str) {
	while (*str) {
		LCD_Char(*str++);
	}
}

#endif
