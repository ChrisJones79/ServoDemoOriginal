/************************************************************************
*																		*
*	Cerebot32MX4.h	--	General Cerebot 32MX4Interface Declarations		*
*																		*
*************************************************************************
*	Author:		Mark Taylor												*
*	Copyright 2009, Digilent Inc.										*
*************************************************************************
*	Module Description:													*
*																		*
*	This header file contains symbol declarations describing ports and	*
*	pins for access to the on-board i/o devices and interface connector	*
*	pins on the Digilent Cerebot 32MX4 board.							*
*																		*
*************************************************************************
*	Revision History:													*
*																		*
*	09/22/2009(MarkT): created											*
*																		*
************************************************************************/


/* ---------------------------------------------------------------
				On-Board I/O Declarations
   --------------------------------------------------------------- */

#define	trisLed1		TRISB
#define	trisLed2		TRISB
#define trisLed3		TRISB
#define	trisLed4		TRISB

#define	trisLed1_4		TRISB

#define	prtLed1	 		PORTB
#define	prtLed2			PORTB
#define	prtLed3			PORTB
#define	prtLed4			PORTB

#define	prtLed1_4		PORTB

#define	bnLed1			10
#define	bnLed2			11
#define	bnLed3 			12
#define	bnLed4 			13


/* ---------------------------------------------------------------
			Symbol Definitions for Servo Connectors
   --------------------------------------------------------------- */

#define	prtServo1		PORTG
#define	prtServo2		PORTG
#define	prtServo3		PORTG
#define	prtServo4		PORTG
#define	prtServo5		PORTG
#define	prtServo6		PORTG
#define	prtServo7		PORTF
#define	prtServo8		PORTF

#define	prtServo1_6 	PORTG
#define	prtServo7_8 	PORTF

#define	trisServo1		TRISG
#define	trisServo2		TRISG
#define	trisServo3		TRISG
#define	trisServo4		TRISG
#define	trisServo5		TRISG
#define	trisServo6		TRISG
#define	trisServo7		TRISF
#define	trisServo8		TRISF

#define	trisServo1_6	TRISG
#define	trisServo7_8	TRISF

#define	bnServo1	12
#define	bnServo2	13
#define	bnServo3	14
#define	bnServo4	15
#define	bnServo5	0
#define	bnServo6	1
#define	bnServo7	0
#define	bnServo8	1

/* ---------------------------------------------------------------
				Interface Connector Declarations
   --------------------------------------------------------------- */

/***************************************************************/







