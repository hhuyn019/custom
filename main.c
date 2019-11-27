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
#include <util/delay.h>
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

#define DDR_SPI DDRB
#define MOSI 5
#define SCK 7
#define CSN 2
#define SPI_PORT PORTB
#define CE 4

void spi_init(void)
{
	/* Set MOSI, SCK, CSN, and CE as output, all others as input */
	DDR_SPI = ( 1 << MOSI ) | ( 1 << SCK ) | ( 1 << CSN ) | ( 1 << CE );
	/* Enable SPI, set clock rate fck/16 */
	SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | ( 1 << SPR0 );
}

uint8_t spi_transfer(uint8_t data)
{
	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while( !( SPSR & ( 1 << SPIF )));

	return SPDR;
}

#define LCD_PORT 	PORTB
#define LCD_DDR 	DDRB
#define LCD_DC 		PB1
#define LCD_RST 	PB0
#define LCD_CE 		PB4

// User-Defined SPI Settings (configure these macros to use your SPI library and functions)

#define SPI_INIT() 		spi_init()
#define SPI_WRITE(x) 	spi_transfer(x) // Expected to accept and return a byte

// =========================================================================================

#define LCD_DATA 	1
#define LCD_CMD 	0
#define LCD_HEIGHT 	48
#define LCD_WIDTH 	84
#define BLACK 		1
#define WHITE 		0

extern uint8_t buffer[504];

void lcd_init( void );

void lcd_send( uint8_t dataOrCmd, uint8_t byte );

void lcd_gotoXY( uint8_t x, uint8_t y);

void lcd_update( void );

void lcd_clear( void );

void lcd_clearBuffer( void );

void lcd_contrast(uint8_t contrast);

void lcd_putPixel(uint8_t x, uint8_t y, uint8_t bw);

int divideRoundUp(int num, int divisor);

void lcd_drawImage( uint8_t* image, uint8_t x, uint8_t y );

void lcd_drawLine( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void lcd_fillRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void lcd_drawRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void spi_init(void);
uint8_t spi_transfer(uint8_t data);

uint8_t buffer[504];

void lcd_init( void )
{
	// Set outputs
	LCD_DDR |= (1 << LCD_DC) | (1 << LCD_RST) | (1 << LCD_CE);

	// Required Reset
	LCD_PORT &= ~(1 << LCD_RST);
	LCD_PORT |= (1 << LCD_RST);

	// Configure LCD (Refer to datasheet to alter settings)
	lcd_send( LCD_CMD, 0x21 );
	lcd_send( LCD_CMD, 0xB0 );
	lcd_send( LCD_CMD, 0x04 );
	lcd_send( LCD_CMD, 0x14 );
	lcd_send( LCD_CMD, 0x20 );
	lcd_send( LCD_CMD, 0x0C );

	lcd_contrast(55);
}

void lcd_send( uint8_t dataOrCmd, uint8_t byte )
{
	if ( dataOrCmd ) LCD_PORT |= (1 << LCD_DC);
	else LCD_PORT &= ~(1 << LCD_DC);

	LCD_PORT &= ~(1 << LCD_CE);
	SPI_WRITE( byte );
	LCD_PORT |= (1 << LCD_CE);
}

void lcd_gotoXY( uint8_t x, uint8_t y)
{
	lcd_send(0, 0x80 | x);
	lcd_send(0, 0x40 | y);
}

void lcd_update( void )
{
	lcd_gotoXY(0, 0);

	for (int i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		lcd_send(LCD_DATA, buffer[i]);
	}
}

void lcd_clear( void )
{
	lcd_gotoXY(0, 0);
	for (int i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		lcd_send(LCD_DATA, 0x00);
	}
}

void lcd_clearBuffer( void )
{
	for (int i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		buffer[i] = 0;
	}
}

void lcd_contrast(uint8_t contrast)
{
	lcd_send(LCD_CMD, 0x21);
	lcd_send(LCD_CMD, 0x80 | contrast);
	lcd_send(LCD_CMD, 0x20);
}

void lcd_putPixel(uint8_t x, uint8_t y, uint8_t bw)
{
	// Make sure coordinate is within bounds
	if ((x >= 0) && (x < LCD_WIDTH) && (y >= 0) && (y < LCD_HEIGHT))
	{
		uint8_t shift = y % 8;
		
		if (bw) // If black, set the bit.
		buffer[x + (y/8)*LCD_WIDTH] |= 1<<shift;
		else   // If white clear the bit.
		buffer[x + (y/8)*LCD_WIDTH] &= ~(1<<shift);
	}
}

int divideRoundUp(int num, int divisor)
{
	int i, quotient;
	for(i = num, quotient = 0; i > 0; i -= divisor, quotient++);
	return quotient;
}

void lcd_drawImage( uint8_t* image, uint8_t x, uint8_t y )
{
	int row, bit, byteColumn, lineSize, height, width;
	
	height = image[0];
	width = image[1];
	lineSize = divideRoundUp( width, 8 );
	
	for ( row = height - 1; row >= 0; row-- )
	{
		for ( byteColumn = 0; byteColumn < lineSize; byteColumn++ )
		{
			for ( bit = 7; bit >= 0; bit--)
			{
				if ( (image[row*lineSize + byteColumn + 2] & (1 << bit) ) )
				lcd_putPixel( x + (byteColumn * 8) + ( 7 - bit ), y + height - row, BLACK );
			}
		}
	}
}

void lcd_drawLine( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t bw )
{
	#define sign(x) ((x) > 0 ? 1: ((x) == 0 ? 0: (-1)))

	int dx, dy, dxabs, dyabs, i, px, py, sdx, sdy, x, y;

	dx = x2 - x1;
	dy = y2 - y1;
	sdx = sign( dx );
	sdy = sign( dy );
	dxabs = ( dx > 0 ) ? dx : -dx;
	dyabs = ( dy > 0 ) ? dy : -dy;
	x = 0;
	y = 0;
	px = x1;
	py = y1;

	if ( dxabs >= dyabs )
	{
		for ( i = 0; i <= dxabs; i++ )
		{
			y += dyabs;
			if ( y >= dxabs )
			{
				y -= dxabs;
				py += sdy;
			}
			lcd_putPixel( px, py, bw );
			px += sdx;
		}
	}
	else
	{
		for ( i = 0; i <= dyabs; i++ )
		{
			x += dxabs;
			if ( x >= dyabs )
			{
				x -= dyabs;
				px += sdx;
			}
			lcd_putPixel( px, py, bw );
			py += sdy;
		}
	}
}

void lcd_fillRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw )
{
	for ( int i = yLow; i < yHigh + 1; i++ )
	{
		for ( int k = xLeft; k < xRight + 1; k++ )
		lcd_putPixel( k, i, bw );
	}
}

void lcd_drawRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw )
{
	// Lower line
	lcd_drawLine( xLeft, yLow, xRight, yLow, bw );
	
	// Upper line
	lcd_drawLine( xLeft, yHigh, xRight, yHigh, bw );

	// Sinelines
	for ( int i = yLow + 1; i < yHigh; i++ )
	{
		lcd_putPixel( xLeft, i, bw );
		lcd_putPixel( xRight, i, bw );
	}
}

char LCD_msg[33];
unsigned short myADC = 0x0000;
unsigned char tmpA = 0x00;
unsigned char tmpB = 0x00;
unsigned char tmpC = 0x00;
unsigned char tmpD = 0x00;

void ADC_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC);
}

void ADC_channel(unsigned char channel){
	if(channel < 8 && channel >= 0){
		//CLEAR ADMUX2:0
		ADMUX &= 0xF8;
		//Set ADMUX
		ADMUX |= (channel & 0x07);
	}
}

unsigned short ADC_read(unsigned char channel){
	unsigned short myADC = 0x0000;
	ADC_channel(channel);
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	myADC = ADC;
	return myADC;
}

typedef struct Joystick_Frame { // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
	unsigned short raw_x;
	unsigned short raw_y;
	unsigned char click;
	
	//These will be filled by 2 calls to Joystick_Process_Raw();
	Direction X_direction;
	Direction Y_direction;
} Joystick_Frame;

Joystick_Frame* currentJoystickFramePtr;
Joystick_Frame* nextJoystickFramePtr;

void Joystick_Process_Raw(Joystick_Frame* frame);
void Joystick_Read(Joystick_Frame* frame);
void Joystick_Tick();

void Joystick_Process_Raw(Joystick_Frame* frame){ // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
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

void Joystick_Read(Joystick_Frame* frame){ // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
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

void testDisplayJoystickADC(){ // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
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

void Joystick_Tick(){
	//READ + POPULATE always into the next frame before swapping buffer
	Joystick_Read(nextJoystickFramePtr);
	Joystick_Process_Raw(nextJoystickFramePtr);
	//COPY POINTER BEFORE SWAPPING
	Joystick_Frame* temp = currentJoystickFramePtr;
	//SWAP BUFFER
	currentJoystickFramePtr = nextJoystickFramePtr;
	nextJoystickFramePtr = temp;
}


Joystick_Frame* currentJoystickFramePtr1;
Joystick_Frame* nextJoystickFramePtr1;
void Joystick_Tick1(){
	//READ + POPULATE always into the next frame before swapping buffer
	Joystick_Read(nextJoystickFramePtr1);
	Joystick_Process_Raw(nextJoystickFramePtr1);
	//COPY POINTER BEFORE SWAPPING
	Joystick_Frame* temp1 = currentJoystickFramePtr1;
	//SWAP BUFFER
	currentJoystickFramePtr1 = nextJoystickFramePtr1;
	nextJoystickFramePtr1 = temp1;
}
testtt() {
	if(currentJoystickFramePtr1->raw_x < 1000) {
		LCD_DisplayString(1, "RIGHT");
	} else if ((currentJoystickFramePtr1->raw_x < 1000) && (currentJoystickFramePtr1->raw_y < 1000)){
	LCD_DisplayString(1, "UP-RIGHT");
	} else if ((currentJoystickFramePtr1->raw_x < 1000) && (currentJoystickFramePtr1->raw_y < 1000)){
	LCD_DisplayString(1, "UP-RIGHT");
	}
}

uint8_t testImg[] = { 19, 42,
	0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x98, 0x90, 0x50,
	0x00, 0x00, 0x01, 0xdc, 0xd8, 0xf8, 0x71, 0xc0, 0x07, 0xfe,
	0xfc, 0xf8, 0xe7, 0x80, 0x0c, 0x3f, 0xff, 0xff, 0xbf, 0x00,
	0x18, 0x0f, 0xe3, 0x9d, 0xce, 0x00, 0x30, 0x03, 0xf1, 0xcc,
	0xdc, 0x00, 0x70, 0x40, 0xf8, 0xc6, 0x78, 0x00, 0x64, 0x78,
	0x38, 0xe7, 0xf0, 0x00, 0xec, 0x7e, 0x18, 0x7f, 0xc0, 0x00,
	0xcc, 0x3e, 0x1c, 0xf8, 0x00, 0x00, 0xde, 0x3e, 0x1f, 0xc0,
	0x00, 0x00, 0xda, 0x16, 0x1f, 0x00, 0x00, 0x00, 0xce, 0x1e,
	0x18, 0x00, 0x00, 0x00, 0xce, 0x00, 0x38, 0x00, 0x00, 0x00,
	0xe0, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x78, 0x3f, 0xc0, 0x00,
	0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0,
0x00, 0x00, 0x00, 0x00, };

int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	ADC_init();
	LCD_init();
	SPI_INIT();
	lcd_init();
	currentJoystickFramePtr = (Joystick_Frame*) malloc(sizeof(Joystick_Frame));
	nextJoystickFramePtr = (Joystick_Frame*) malloc(sizeof(Joystick_Frame));
	TimerSet(100);
	TimerOn();
	
	lcd_clearBuffer();
	lcd_drawImage( testImg, 0, 0 );
	lcd_drawLine( 5, 30, 80, 30, BLACK );
	lcd_drawRect( 50, 0, 80, 40, BLACK );
	lcd_update();
	
    while (1) {
		Joystick_Tick();
	    testDisplayJoystickADC();
		while (!TimerFlag);
		TimerFlag = 0;
	}
	return 0;
}
