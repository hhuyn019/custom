/*
 * CustomLab.c
 *
 * Created: 11/23/2019 11:06:21 PM
 * Author : Chris
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
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

typedef enum Direction{NONE, LEFT, RIGHT, UP, DOWN} Direction;
	
#define JOYSTICK_LIMIT_LEFT 100
#define JOYSTICK_LIMIT_RIGHT 1000
#define JOYSTICK_LIMIT_UP 100
#define JOYSTICK_LIMIT_DOWN 1000


typedef struct Joystick_Frame {
	
	unsigned short raw_x;
	unsigned short raw_y;
	unsigned char click;
	
	//These will be filled by 2 calls to Joystick_Process_Raw();
	Direction X_direction;
	Direction Y_direction;
} Joystick_Frame;


void Joystick_Process_Raw(Joystick_Frame* frame);
void Joystick_Read(Joystick_Frame* frame);
void Joystick_Tick();

void Joystick_Process_Raw(Joystick_Frame* frame){
	
	//Set X
	if(frame->raw_x < JOYSTICK_LIMIT_LEFT){frame->X_direction = LEFT;}
	else if(frame->raw_x > JOYSTICK_LIMIT_RIGHT){frame->X_direction = RIGHT;}
	else{frame->X_direction = NONE;}
	//Set Y
	if(frame->raw_y < JOYSTICK_LIMIT_UP){frame->Y_direction = UP;}
	else if(frame->raw_y > JOYSTICK_LIMIT_DOWN){frame->Y_direction = DOWN;}
	else{frame->Y_direction = NONE;}
	//Decide what to do on ambiguous inputs:
	//I have decided to ignore the vertical input as it isn't 
	//time critical to move the block downward
	if(frame->X_direction != NONE && frame->Y_direction != NONE){
		frame->Y_direction = NONE;
	} 
}

void Joystick_Read(Joystick_Frame* frame){
	
	//We will read the Vx and Vy from the 2-potentiometer Joystick;
	
	frame->raw_x = ADC_read(0);		//read 10 bit ADC value on ADC0	
	frame->raw_y = ADC_read(1);		//read ADC1 as well
	frame->click = (~PINA & 0x08) ? 1 : 0; //read the click
}

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

void testDisplayJoystickADC(){
	//I know this code is ugly, its really just a test bench to see 
	//what the X and Y ranges from the joystick and the logic value 
	//from its click.
	
	//display X
	unsigned short tmpADC = currentJoystickFramePtr->raw_x;
	LCD_msg[3] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[2] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[1] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[0] = (tmpADC % 10)+ '0';
	LCD_msg[4] = ' ';
	
	//display Y
	tmpADC = currentJoystickFramePtr->raw_y;
	LCD_msg[8] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[7] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[6] = (tmpADC % 10) + '0';
	tmpADC /= 10;
	LCD_msg[5] = (tmpADC % 10)+ '0';
	LCD_msg[9] = ' ';
	
	//display click
	LCD_msg[10] = currentJoystickFramePtr->click ? '1' :  '0';
	LCD_msg[11] = '\0';
	
	//Write to LCD
	LCD_DisplayString(1, &LCD_msg);

}

int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	ADC_init();
	LCD_init();
	TimerSet(100);
	TimerOn();
	
    while (1) {
		Joystick_Tick();
	    	testDisplayJoystickADC();
		while (!TimerFlag);
		TimerFlag = 0;
	}
	return 0;
}
