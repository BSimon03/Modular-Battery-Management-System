//===============================================================
//
//   Projekt sax-extender master
//
//   rst, jan 2022
//
//================================================================
#ifndef MANCH_H
#define MANCH_H

#define BAUDRATE 15625
#ifdef __AVR_ATmega32U4__
#define CLOCK_PR 1
#endif // __AVR_ATmega32u4__
#ifdef __AVR_ATtiny261A__
#define CLOCK_PR 256
#define MANCHESTER1 // 2. manchester-übertragung
#endif              // __AVR_ATtiny261a__

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
#elif __AVR_ATtiny261A__
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
#define PN_MANCH1 PINA2
#endif //__AVR_ATtiny261A__
#define CLRMANCH1 (PORTMANCH1 &= ~PN_MANCH1)
#define SETMANCH1 (PORTMANCH1 |= PN_MANCH1)
#define TOGMANCH1 (PINMANCH1 = PN_MANCH1)
#define READMANCH1 (PINMANCH1 & PN_MANCH1)
#endif // MANCHESTER1

//=========== prototypen =======================================
// initialisieren zum senden nach "unten"
// vor jedem senden aufrufen!
void manch_init_send(void);

// copiert daten von data und startet das senden
void manch_send(uint16_t data);

// initialisieren zum empfang von "unten"
// vor jedem empfangen aufrufen
void manch_init_receive();

// return 0: wartet noch auf daten
// return 1: daten empfangen, in *data copiert
// return 2: fehlerhafter empfang, empfang abgebrochen, muss neu gestartet werden
uint8_t manch_receive(uint16_t *data);

#ifdef MANCHESTER1

// zum senden nach "oben"
// initialisiert nur das sendeport.
// danach manch_init_send() aufrufen!
void manch_init_send1(void);

// copiert nur die sendedaten für senden nach "oben",
// danach mach_send() zum starten aufrufen!
void manch_send1(uint16_t data);

// initialisieren zum empfang von "oben"
// vor jedem empfangen aufrufen
void manch_init_receive1();

// return 0: wartet noch auf daten
// return 1: daten empfangen, in *data copiert
// return 2: fehlerhafter empfang, empfang abgebrochen, muss neu gestartet werden
uint8_t manch_receive1(uint16_t *data);

#endif // MANCHESTER1
#endif // MANCH_H