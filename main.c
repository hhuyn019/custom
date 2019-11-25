/*
 * CustomLab.c
 *
 * Created: 11/23/2019 11:06:21 PM
 * Author : Chris
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define input (~PINA & 0xFF)

#define SET_BIT(p,i) ((p) |= (1 << (i)))
#define CLR_BIT(p,i) ((p) &= ~(1 << (i)))
#define GET_BIT(p,i) ((p) & (1 << (i)))

/*-------------------------------------------------------------------------*/

#define DATA_BUS PORTC		// port connected to pins 7-14 of LCD display
#define CONTROL_BUS PORTD	// port connected to pins 4 and 6 of LCD disp.
#define RS 6			// pin number of uC connected to pin 4 of LCD disp.
#define E 7			// pin number of uC connected to pin 6 of LCD disp.

/*-------------------------------------------------------------------------*/

void LCD_init();
void LCD_ClearScreen(void);
void LCD_WriteCommand (unsigned char Command);
void LCD_Cursor (unsigned char column);
void LCD_DisplayString(unsigned char column, const unsigned char* string);
void delay_ms(int miliSec);

void LCD_ClearScreen(void) {
	LCD_WriteCommand(0x01);
}

void LCD_init(void) {

	//wait for 100 ms.
	delay_ms(100);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x0f);
	LCD_WriteCommand(0x01);
	delay_ms(10);
}

void LCD_WriteCommand (unsigned char Command) {
	CLR_BIT(CONTROL_BUS,RS);
	DATA_BUS = Command;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(2); // ClearScreen requires 1.52ms to execute
}

void LCD_WriteData(unsigned char Data) {
	SET_BIT(CONTROL_BUS,RS);
	DATA_BUS = Data;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(1);
}

void LCD_DisplayString( unsigned char column, const unsigned char* string) {
	LCD_ClearScreen();
	unsigned char c = column;
	while(*string) {
		LCD_Cursor(c++);
		LCD_WriteData(*string++);
	}
}

void LCD_Cursor(unsigned char column) {
	if ( column < 17 ) { // 16x1 LCD: column < 9
		// 16x2 LCD: column < 17
		LCD_WriteCommand(0x80 + column - 1);
		} else {
		LCD_WriteCommand(0xB8 + column - 9);	// 16x1 LCD: column - 1
		// 16x2 LCD: column - 9
	}
}

void delay_ms(int miliSec) //for 8 Mhz crystal

{
	int i,j;
	for(i=0;i<miliSec;i++)
	for(j=0;j<775;j++)
	{
		asm("nop");
	}
}

volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1ms
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B 	= 0x0B;	// bit3 = 1: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: prescaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A 	= 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register

	TIMSK1 	= 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1 = 0;

	// TimerISR will be called every _avr_timer_cntcurr milliseconds
	_avr_timer_cntcurr = _avr_timer_M;

	//Enable global interrupts
	SREG |= 0x80;	// 0x80: 1000000
}

void TimerOff() {
	TCCR1B 	= 0x00; // bit3bit2bit1bit0=0000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect)
{
	// CPU automatically calls when TCNT0 == OCR0 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; 			// Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { 	// results in a more efficient compare
		TimerISR(); 				// Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

void Tick() {
	if(input == 0x01) {
		LCD_DisplayString(1, "1");
	}
	if(input == 0x02) {
		LCD_DisplayString(1, "2");
	}
	if(input == 0x03) {
		LCD_DisplayString(1, "3");
	}
	if(input == 0x04) {
		LCD_DisplayString(1, "4");
	}
	if(input == 0x05) {
		LCD_DisplayString(1, "5");
	}
	if(input == 0x06) {
		LCD_DisplayString(1, "6");
	}
	if(input == 0x07) {
		LCD_DisplayString(1, "7");
	}
	if(input == 0x08) {
		LCD_DisplayString(1, "8");
	}
	if(input == 0x09) {
		LCD_DisplayString(1, "9");
	}
	if(input == 0x0A) {
		LCD_DisplayString(1, "A");
	}
	if(input == 0x0B) {
		LCD_DisplayString(1, "B");
	}
	if(input == 0x0C) {
		LCD_DisplayString(1, "C");
	}
	if(input == 0x0D) {
		LCD_DisplayString(1, "D");
	}
	if(input == 0x0E) {
		LCD_DisplayString(1, "E");
	}
	if(input == 0x0F) {
		LCD_DisplayString(1, "F");
	}
	if(input == 0x10) {
		LCD_DisplayString(1, "G");
	}
	if(input == 0x20) {
		LCD_DisplayString(1, "H");
	}
	if(input == 0x30) {
		LCD_DisplayString(1, "I");
	}
	if(input == 0x40) {
		LCD_DisplayString(1, "J");
	}
	if(input == 0x50) {
		LCD_DisplayString(1, "K");
	}
	if(input == 0x60) {
		LCD_DisplayString(1, "L");
	}
	if(input == 0x70) {
		LCD_DisplayString(1, "M");
	}
	if(input == 0x80) {
		LCD_DisplayString(1, "N");
	}
	if(input == 0x90) {
		LCD_DisplayString(1, "O");
	}
	if(input == 0xA0) {
		LCD_DisplayString(1, "P");
	}
	if(input == 0xB0) {
		LCD_DisplayString(1, "Q");
	}
	if(input == 0xC0) {
		LCD_DisplayString(1, "R");
	}
	if(input == 0xD0) {
		LCD_DisplayString(1, "S");
	}
	if(input == 0xE0) {
		LCD_DisplayString(1, "T");
	}
	if(input == 0xF0) {
		LCD_DisplayString(1, "U");
	}
}

int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	TimerSet(100);
	TimerOn();
	LCD_init();
	
    while (1) {
		Tick();
		//LCD_Cursor(1);
		//LCD_WriteData('6');
		while (!TimerFlag);
		TimerFlag = 0;
	}
	return 0;
}
