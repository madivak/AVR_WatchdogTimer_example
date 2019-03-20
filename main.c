
/*
 * GPS-UART-BUF.c
 *
 * Created: 25/01/2019 5:37:46 PM
 * Author : Madiva
 */ 
#define F_CPU 1000000UL
#include <string.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ctype.h> //for identifying digits and alphabets
#include <avr/wdt.h>

#include "USART.h"
#include "24c64.h"

#define FOSC 2000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (((FOSC / (BAUD * 16UL))) - 1)

uint8_t EEmemory[22] = "";

static FILE uart0_output = FDEV_SETUP_STREAM(USART0_Transmit, NULL, _FDEV_SETUP_WRITE);
static FILE uart1_output = FDEV_SETUP_STREAM(USART1_Transmit, NULL, _FDEV_SETUP_WRITE);
static FILE uart1_input = FDEV_SETUP_STREAM(NULL, USART1_Receive, _FDEV_SETUP_READ);
static FILE uart0_input = FDEV_SETUP_STREAM(NULL, USART0_Receive, _FDEV_SETUP_READ);

void WDT_off(void);
void WDT_Prescaler_Change(void);
void Blink_LED();

#define LED_ON	PORTD |= (1<<PORTD5)
#define LED_OFF	PORTD &= ~(1<<PORTD5)
#define	wake0GSM	PORTA |= (1<<PORTA5)
#define	wake1GSM	PORTA &= ~(1<<PORTA5)
#define CHOSEN_SECTOR 0
#define BUFFER_SIZE 24

int address;
char company[]	= "+222222222222";//random number

// Function Prototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
// Function Implementation
void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();
	//MCUSR &= ~(1<<WDRF); //clear Watchdog sys rst flag - should be done even if WD is not used
	//WDTCSR &= ~(1<<WDE); //clear WD timer WD Reset Enable
	return;
}
/**********************************
A simple delay routine to wait
between messages given to user
so that he/she gets time to read them
***********************************/
void Wait()
{	uint8_t i;
	for(i=0;i<100;i++)
	_delay_loop_2(0);
}

void setup()
{
	DDRD |= (1<<DDD5); //set PORTA3 as output for LED
	DDRA |= (1<<DDA5); //set PORTB0 AS GSM-power output
	
	WDT_Prescaler_Change();
	
	wake0GSM;
	_delay_ms(4000);
	wake1GSM;
	wdt_reset(); //-------//
	
 	Blink_LED();
}

void Blink_LED()
{
  LED_ON; _delay_ms(2000); LED_OFF; _delay_ms(2000);
}

int main( void )
{	
/**************************SD-CODE-TESTING************************************/
	USART1_Init(MYUBRR);
	USART0_Init(MYUBRR);
	setup();
	
	fdev_close();
	stdin = &uart1_input;
	stdout = &uart0_output;
	
	sei(); //Enable global interrupts by setting the SREG's I-bit
	
	LED_OFF;
	wdt_reset(); //-------//
	printf("System has been reset by the Watchdog timer.......\r\n\r\n");
	_delay_us(5000);
	printf("Starting EEPROM TEST.......\r\n");
	
			EEOpen();	//Init EEPROM
			_delay_loop_2(0);
			for(address=40;address<53;address++)
			{EEWriteByte(address,company[address-40]); }
	
	_delay_ms(200);
			char L;
			EEOpen();
			_delay_loop_2(0);
			L = EEReadByte(44);
	wdt_reset(); //-------//
			
	if (L== 0x032) { Blink_LED(); wdt_reset(); Blink_LED(); wdt_reset();printf("L = %c\r\nValid data retrieved from EEPROM\r\n", L); }  //Blink D11 twice if EEprom is OK
		
	else { Blink_LED(); printf("L = %c\r\nInvalid data retrieved from EEPROM\r\n", L);}
	
	wdt_reset(); 
	printf("Finished EEPROM TEST\r\n"); 
	
	printf("The device will be reset in about 8 seconds.\r\n");
	wdt_reset(); 
	int g = 0;
	while (1)
	{
		g++; _delay_ms(1000);printf("%d second.\r\n",g);
	}
	
	return 0;
}


void WDT_off(void)
{
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE (WD Change Enable) and WDE (WD Reset Enable) */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE); //WDTCSR (WD Timer Control Register)
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
}

void WDT_Prescaler_Change(void)
{
	cli();
	wdt_reset();
	/* Start timed sequence */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Set new prescaler(time-out) value = 64K cycles (~8 s) */
	WDTCSR = (1<<WDE) | (1<<WDP3) | (1<<WDP0);
	sei();
}  