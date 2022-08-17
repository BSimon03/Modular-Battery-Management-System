//===============================================================
//
//   Projekt sax-extender master
//
//   rst, jan 2022
//
//   Author: Reinhard Steindl
//
//   Update by: Simon Ball
//
//================================================================
#ifndef MANCH_H
#define MANCH_H

#define BAUDRATE 2400  

#ifdef __AVR_ATmega32U4__
#define CLOCK_PR 1
#define TCCR1B_TIMER_START 0x01
#endif // __AVR_ATmega32u4__

#ifdef __AVR_ATtiny261A__
#define CLOCK_PR 8 // Prescaler 8, im code hardcodiert!!!
#define TCCR1B_TIMER_START 0x04
//#define MANCHESTER1 // 2. manchester-übertragung
#endif              // __AVR_ATtiny261A__

//========= definitionen des verwendeten port und pins =============
// zur übertragung in nur eine richtung: vom master oder slave nach unten
// sender: positiv-logik: 1: 111000     0: 000111
// empfänger: ! negativ-logik, wegen invertierender schaltung
//            1: 000111     0: 111000
// im pausen: beim senden hochohmig,  beim empfang liegt 0 an
// verwendet timer1: ocr1b-int und overflow-int
//          zum empfangen pinchange-interrupt

#ifdef __AVR_ATmega32U4__
#define DDRMANCH DDRB // port der übertragungsleitung
#define PORTMANCH PORTB
#define PINMANCH PINB
#define PN_MANCH_SEND (1 << PINB6) // pin zum senden
#define PN_MANCH_REC (1 << PINB5)  // pin zum empfangen
#endif
#ifdef __AVR_ATtiny261A__
#define DDRMANCH DDRB // port der übertragungsleitung
#define PORTMANCH PORTB
#define PINMANCH PINB
#define PN_MANCH_SEND (1 << PINB6) // pin zum senden
#define PN_MANCH_REC (1 << PINB6)  // pin zum empfangen
#endif
#define CLRMANCH (PORTMANCH &= ~PN_MANCH_SEND)
#define SETMANCH (PORTMANCH |= PN_MANCH_SEND)
#define TOGMANCH (PINMANCH = PN_MANCH_SEND)
#define READMANCH (PINMANCH & PN_MANCH_REC)

//=========== 2. schnittstelle ====================================
// zur übertragung "nach oben"
// ! negativ-logik, wegen invertierender schaltung
//            1: 000111     0: 111000
// in pausen: beim senden 1 am port, beim empfang liegt 1 an
// verwendet timer1, wie MANCHESTER
//          zum empfangen externer-interrupt
#ifdef MANCHESTER1
#ifdef __AVR_ATmega32u4__
#define DDRMANCH1 DDRD // port der übertragungsleitung
#define PORTMANCH1 PORTD
#define PINMANCH1 PIND
#define PN_MANCH1 0x01 // pin zum übertrage: z.B. 0x10 => bit 5
#endif                 //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
#define DDRMANCH1 DDRA
#define PORTMANCH1 PORTA
#define PINMANCH1 PINA
#define PN_MANCH1 (1 << PINA2)
#endif //__AVR_ATtiny261A__
#define CLRMANCH1 (PORTMANCH1 &= ~PN_MANCH1)
#define SETMANCH1 (PORTMANCH1 |= PN_MANCH1)
#define TOGMANCH1 (PINMANCH1 = PN_MANCH1)
#define READMANCH1 (PINMANCH1 & PN_MANCH1)
#endif // MANCHESTER1

void manch_init_send(void);
// initializes the manchester-transmitter
// has to be called before manch_send

void manch_send(void);
// copies data to send-buffer and starts sending

void manch_init_receive();
// initializes the manchester-receiver from bottom
// has to be called before the call of manch_receive

uint8_t manch_receive(void);
// copies received data to *data
// returns 0: waitin for data
// returns 1: data received, data in *data
// returns 2: error, receive aborted, has to be restarted

#ifdef MANCHESTER1

void manch_init_receive1();
// initializes the port for receiving from top
// has to be called before the call of manch_receive1

uint8_t manch_receive1(uint16_t *data);
// copies received data to *data
// returns 0: waitin for data
// returns 1: data received, data in *data
// returns 2: error, receive aborted, has to be restarted

#endif // MANCHESTER1

//==========globale variablen================================
#ifdef MANCH_M
	#define EXTERN
	#define REGISTER register
//	#define ASMR3 asm("r3")
#else
	#define EXTERN extern
	#define REGISTER register
#endif 
	// daten zum senden bzw. empfangene daten
	uint16_t EXTERN gl_manch_dat;
#ifdef MANCHESTER1
	uint16_t EXTERN gl_manch_dat1;
#endif // MANCHESTER1


#undef EXTERN

#endif // MANCH_H
