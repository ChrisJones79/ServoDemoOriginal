/************************************************************************/
/*																		*/
/*	main.c	--	Main program file for MiniServo demo					*/
/*																		*/
/************************************************************************/
/*	Author: 	Mark Taylor												*/
/*	Copyright	2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*	This program illustrates a technique for driving RC hobby			*/
/*	servos from a PIC32 microcontroller. This particular 				*/
/*	implementation is for an PIC32MX4 on a Digilent Cerebot 32MX4		*/
/*	board.																*/
/*																		*/
/*	RC hobby servos use a pulse width modulated (PWM) signal			*/
/*	to specify the servo position. The servo is sent digital			*/
/*	pulses at a pulse repetition rate (frequency) of about				*/
/*	50HZ, with the pulse width varying from approximately 1ms			*/
/*	to 2ms. A pulse width of 1.5ms is the nominal center of				*/
/*	rotation.															*/
/*																		*/
/*	This implementation controls up to 8 servos with pulse				*/
/*	widths specified in increments of 0.1us (microsecond). The			*/
/* 	pulses are generated sequentially with 3ms spacing. This			*/
/*	results in a pulse frequency of 41.67Hz for each servo				*/
/*	channel.															*/
/*																		*/
/*	The technique illustrated here uses one timer (Timer2) and one		*/
/*	output compare module (OC2) to time the rising and falling edges of	*/
/*	the servo signals.  Timer2 generates an interrupt each time the 	*/
/*	timer's counter value reaches cntPulseRate, which takes 3ms.  This 	*/
/*	interrupt starts the pulse signal for a servo.  When the timer's	*/
/*	counter value reaches the count specified in OC2R, the servo pulse	*/
/*	is ended in the OC2 interrupt.  At this time, idChanCur is updated	*/
/*	so that during the next Timer2 interrupt, the next servo will be	*/
/*	written to.  The timer prescaler is set to clock the timer at		*/
/*	10Mhz.																*/
/*																		*/
/*	The UpdateServoPos() function was written to automate the movement	*/
/*	of the servos.  The user should remove this function and replace it	*/
/*	with a function that is more specific to the user's requirements.	*/
/*	This could be a function that takes input from an external source, 	*/
/*	such as the PmodJSTK and manipulates the cntWdt(x) values according */
/*	to the PmodJSTK data.												*/
/*																		*/
/*	To better understand this program and what is going on within the 	*/
/*	registers, it is required that the user refers to the Microchip		*/
/*	and Digilent documentation.  Specifically, the following sections:	*/
/*		Digilent Cerebot 32MX4 Board Reference Manual. (Doc: 502-173)	*/
/*		Digilent Cerebot32MX4 Schematic (Doc: 500-173)					*/
/*		Microchip PIC32MX3XX/4XX Family Data Sheet (DS61143F)			*/
/*		Microchip PIC32MX Family Reference Manual (DS61132B)			*/
/*		Microchip Section 8. Interrupts (DS61108D)						*/
/*		Microchip Section 12. I/O Ports (DS61120D)						*/
/*		Microchip Section 14. Timers (DS61105D)							*/
/*		Microchip Section 16. Output Compare (DS61111D)					*/
/*																		*/
/*			*Doc numbers verified at time of writing (09/24/2009)		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*		10/18/2005 (GeneA): created										*/
/*		03/21/2006 (GeneA):	Adapted from Cerebot version				*/
/*		06/20/2006 (GeneA):	Modified for Rev B PCB and changed from		*/
/*			ATmega48 to ATmega168.										*/
/*		05/18/2007 (MarkT): ported to AVR GCC from AVR assembly			*/
/*		09/23/2009 (MarkT): ported to PIC32 from AVR Source.			*/
/*																		*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <p32xxxx.h>
#include <plib.h>
#include <stdint.h>

#include "stdtypes.h"
#include "Cerebot32MX4.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define cntWdtInit 15000

/*	These symbols set the range of motion of the servos.
	The standard servo signal is 1500uS +/- 500uS. This is
	what is put out by most RC transmitter/receivers. Most
	servos will operate over a wider range, but it depends on
	the manufacturer and model what the actual operating range
	will be. Most servos work so that increasing pulse width
	corresponds to clockwise rotation (when viewed from the
	control horn side of the servo). Some manufacturer's servos
	work the opposite, though, with an increasing pulse width
	resulting in couter-clockwise rotation.
	Servo position timing is done using a 10MHz timer clock (0.1uS period)
	and so these numbers specify pulse widths in (1/10)microseconds.*/

#define cntWdtIdle 15000  	//nominal center of travel
#define cntWdtMin 10000 		//left stop (counter-clockwise limit)
#define cntWdtMax 20000		//right stop (clockwise limit)

/*	This symbol specifies the amount to move the servo
	positions for the demo application. */
#define cntWdtDelta 5

/*	This symbol specifies the time delay between servo position
	updates. The update interval will be this value multiplied
	by 3ms. */
#define cmsUpdateInterval 1


//(1/42.67Hz)/(num_servos) -> 24ms/8  -> Timer Clock @ 10MHz -> 30K clocks
#define cntPulseRate 30000


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

unsigned int	cntWdt1, cntWdt2, cntWdt3, cntWdt4, cntWdt5;
unsigned int	cntWdt6, cntWdt7, cntWdt8, idChanCur;


/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Set Up of System Clock							*/
/* ------------------------------------------------------------ */

// Configuration Bit settings for System Clock = 80 MHz
//  Primary Osc w/PLL (XT+,HS+,EC+PLL)
//  Input Divider	2x Divider
//	Multiplier		18x Multiplier
//  WDT disabled
//  Other options are don't cares
#pragma config FNOSC = PRIPLL, POSCMOD = HS, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPBDIV = DIV_2, FPLLODIV = DIV_1 //80MHz
#pragma config FWDTEN = OFF

#define SYSTEM_FREQUENCY	80000000L


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit (void);
void	AppInit (void);
void	StartServoPulse (void);
void	EndServoPulse (void);
void	UpdateServoPos(void);
void	Wait_ms(WORD);

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */

void __ISR(_TIMER_2_VECTOR, ipl7) _Timer2Handler(void) {	

	StartServoPulse();
	UpdateServoPos(); //automate servo movement (remove this and call your own from main loop)

	IFS0CLR	= ( 1 << 8 ); 	// clear interrupt flag for timer 2
}

void __ISR(_OUTPUT_COMPARE_2_VECTOR, ipl7) OC2_IntHandler (void) {
	
	EndServoPulse();

	IFS0CLR	= ( 1 << 10 );	// clear interrupt flag for output compare 2	
}


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/
int main(void) {

	DeviceInit();
	AppInit();

	//main loop
	while(fTrue) {

		//Show a blinking Led to indicate activity
		PORTBINV = (1 << bnLed1);
		Wait_ms(500);

	}// end main loop
	
	return 0;
} //end main()

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes on chip peripheral devices to the default
**		state.
*/
void DeviceInit() {

	int pbFreq;
	
	//Configure the device for maximum performance.
	//This macro sets flash wait states, PBCLK divider and DRM wait states based on the specified
	//clock frequency.  It also turns on the cache mode if available.  Returns the PB frequency
	pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
		
	//Microchip recommends typing unused pins to ground
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	PORTE = 0;
	PORTF = 0;
	PORTG = 0;
	
	TRISA = 0;
	TRISB = 0;
	TRISC = 0;
	TRISD = 0;
	TRISE = 0;
	TRISF = 0;
	TRISG = 0;

	//Set the LEDs as ouputs
	prtLed1_4 &= ~((1 << bnLed1)|(1 << bnLed2)|(1 << bnLed3)|(1 << bnLed4)); //LLV
	trisLed1_4 &= ~((1 << bnLed1)|(1 << bnLed2)|(1 << bnLed3)|(1 << bnLed4)); //output

	//Set up the i/o pins we are using for the servo channels
	//as outputs.
	prtServo1_6 &= ~((1 << bnServo1)|(1 << bnServo2)|(1 << bnServo2)|(1 << bnServo3)|\
						(1 << bnServo4)|(1 << bnServo5)|(1 << bnServo6));
	prtServo7_8 &= ~((1 << bnServo7)|(1 << bnServo8));

	trisServo1_6 &= ~((1 << bnServo1)|(1 << bnServo2)|(1 << bnServo2)|(1 << bnServo3)|\
						(1 << bnServo4)|(1 << bnServo5)|(1 << bnServo6));
	trisServo7_8 &= ~((1 << bnServo7)|(1 << bnServo8));

}//end DeviceInit()

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/
void AppInit(void) {

	//init globals
	cntWdt1 = cntWdtInit;
	cntWdt2 = cntWdtInit;
	cntWdt3 = cntWdtInit;
	cntWdt4 = cntWdtInit;
	cntWdt5 = cntWdtInit;
	cntWdt6 = cntWdtInit;
	cntWdt7 = cntWdtInit;
	cntWdt8 = cntWdtInit;

	idChanCur = 1;


	// Set up the output compare module
	OC2CON	= ( 1 << 1 ) | ( 1 << 0 ); //Timer2 selected as clock source, events toggle OC2 pin
	OC2R	= cntWdtInit; //OC2R sets the pulse duration for the currently selected servo
	//Set up output compare interrupt
	IPC2SET = ( 1 << 20 ) | ( 1 << 19 ) | ( 1 << 18 ) | ( 1 << 17 ); //Priority level: 7, sub: 2
	IFS0CLR = ( 1 << 10 ); //clear output compare 2 interrupt flag
	IEC0SET = ( 1 << 10 ); //enable output compare 2 interrupt

	//Set up timer 2
	TMR2 	= 0; //
	PR2		= cntPulseRate; //set the rate of the timer interrupt
	//Set up timer interrupt
	IPC2SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 0 );
	IFS0CLR	= ( 1 << 8 );
	IEC0SET	= ( 1 << 8 );

	T2CON		= ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 );
	OC2CONSET	= ( 1 << 15 );

	//Start the first servo pulse, interrupts will take over from there
	prtServo1 |= (1 << bnServo1);  //start beginning pulse of first servo.

	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();

}//end AppInit()

/* ------------------------------------------------------------ */
/***	StartServoPulse
**
**	Synopsis:
**		StartServoPulse()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		This function will set the pin high of the appropriate servo.
**		It keeps track of which servo needs to be set next based upon the 
**		current state of the global variable idChanCur.
**		This function is called by the Timer2 interrupt which occurs
**		at approximately 41.67Hz.  This frequecy can be increased to 62.5Hz 
**		by changing cntPulseRate to a minimum of 20000.  This will increase the
** 		work load done by the processor, but offer improved feedback to the servo.
**		Also, increasing the refresh rate will also limit the high-side of the pulse
**		duration, and thereby limit the possible travel range of the servo.
**		Servos require an update frequency of ~50 Hz.  Straying to far from this
**		value may produce adverse results.
**
*/
void StartServoPulse(void) {

	switch (idChanCur) { 
		case 1: 
			prtServo1 |= (1 << bnServo1);
			OC2R = cntWdt1;  //load defined pulse width into compare register
		break; 	
	  
		case 2: 
			prtServo2 |= (1 << bnServo2);
			OC2R = cntWdt2;
		break; 

		case 3: 
			prtServo3 |= (1 << bnServo3);
			OC2R = cntWdt3;
		break; 

		case 4: 
			prtServo4 |= (1 << bnServo4);
			OC2R = cntWdt4;
		break; 

		case 5: 
			prtServo5 |= (1 << bnServo5);
			OC2R = cntWdt5;
		break; 

		case 6: 
			prtServo6 |= (1 << bnServo6);
			OC2R = cntWdt6;
		break; 

		case 7: 
			prtServo7 |= (1 << bnServo7);
			OC2R = cntWdt7;
		break; 

		case 8: 
			prtServo8 |= (1 << bnServo8);
			OC2R = cntWdt8;
		break;  

		default: 
		break; 
   	}//end switch
}//end StartServoPulse function

/* ------------------------------------------------------------ */
/***	EndServoPulse
**
**	Synopsis:
**		EndServoPulse()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		This function ends the pulse on the currently addressed servo.
**		It gets called by the OCR1B compare interrupt.
**
*/
void EndServoPulse(void) {

	switch (idChanCur) { 
		case 1: 
			prtServo1 &= ~(1 << bnServo1);
		break; 	
	  
		case 2: 
			prtServo2 &= ~(1 << bnServo2);
		break; 

		case 3: 
			prtServo3 &= ~(1 << bnServo3);
		break; 

		case 4: 
			prtServo4 &= ~(1 << bnServo4);
		break; 

		case 5: 
			prtServo5 &= ~(1 << bnServo5);
		break; 

		case 6: 
			prtServo6 &= ~(1 << bnServo6);
		break; 

		case 7: 
			prtServo7 &= ~(1 << bnServo7);
		break; 

		case 8: 
			prtServo8 &= ~(1 << bnServo8);
		break;  

		default: 
		break; 
   	} //end switch

	//The current servo's handling is completed.  Allow interrupt
	//	to address next servo.   
   	if(idChanCur == 8) {
   		idChanCur = 1;
	}
	else {
		idChanCur += 1;
	}
	
}  //end EndServoPulse function

/* ------------------------------------------------------------ */
/***	UpdateServoPos
**
**	Synopsis:
**		UpdateServoPos()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		This moves the servos around for the demo.  Add your own functional 
**			code to update the servos' positions.
**
*/
void UpdateServoPos(void) {

	static unsigned int updateDelay;

	if( updateDelay == cmsUpdateInterval)
	{
		if(cntWdt1 == cntWdtMax) { //max reached; reset
			cntWdt1 = cntWdtMin;
			cntWdt2 = cntWdtMin;
			cntWdt3 = cntWdtMin;
			cntWdt4 = cntWdtMin;
			cntWdt5 = cntWdtMin;
			cntWdt6 = cntWdtMin;
			cntWdt7 = cntWdtMin;
			cntWdt8 = cntWdtMin;	
		}  //end if maxed

		else {
			cntWdt1 += cntWdtDelta;
			cntWdt2 += cntWdtDelta;
			cntWdt3 += cntWdtDelta;
			cntWdt4 += cntWdtDelta;
			cntWdt5 += cntWdtDelta;
			cntWdt6 += cntWdtDelta;
			cntWdt7 += cntWdtDelta;
			cntWdt8 += cntWdtDelta;
		} 
		
		updateDelay = 0;  //reset
	}
	else {
		updateDelay += 1; //increment
	}

}  //end UpdateServoPos function

/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of the compiler.  
**		A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete.
**
*/
void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 7270; i ++) {
			_nop(); //Optimization settings can remove empty loops, fill with nops.
		}//end for
		delay -= 1;
	}//end while
}//end Wait_ms()