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

#define MANCH_M
#include <avr/io.h>
#include "avr/interrupt.h"
//#include <util/delay.h>
#include "manch_m.h"
#include "status.h"

static uint8_t manch_i;           // manch_i: counter
static uint8_t manch_d, manch_d1; // manch_d: manchester data
static uint8_t manch1_d, manch1_d1;
static uint8_t manch_x; // manch_x: differs betwenn long and short   

uint8_t volatile register manch_res asm("r3"); 

void manch_init_send(void)
{
   DDRMANCH |= PN_MANCH_SEND; // pin als ausgang
   CLRMANCH;                  // und auf 0
#ifdef MANCHESTER1
   DDRMANCH1 |= PN_MANCH1; // pin als ausgang
   SETMANCH1;              // und auf 1
#endif
#ifdef __AVR_ATmega32U4__
   TCCR1A = 0x02; // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18; //  - " -
   OCR1B = F_CPU / BAUDRATE / CLOCK_PR / 2 +10;
   ICR1 = F_CPU / BAUDRATE / CLOCK_PR; // Bitdauern
   TIFR1 = 0xff;                // flags löschen		fx
   TIMSK1 = 0x05;               // ocr1b match und overflow interrupt;
#endif                          //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   TCCR1A = 0x02; // Mode 2, Fast PWM
   TC1H = 0;	// 10-bit register!   
   OCR1B = F_CPU / BAUDRATE / CLOCK_PR / 2;
   OCR1C = F_CPU / BAUDRATE / CLOCK_PR; // Bitdauer
   TIFR = 0xFF;                             // Flags cleared
   TIMSK = 0x24;                            // ocr1b match und overflow interrupt
#endif                                      //__AVR_ATtiny261A__
}

void manch_init_receive()
{
   DDRMANCH &= ~PN_MANCH_REC; // pin als Eingang
   CLRMANCH;                  // no pull-up
#ifdef __AVR_ATmega32U4__
   // timer OCRA für receive error, timeout, wenn keine flanke kommt
   TCCR1A = 0x02;                                // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18;                                //  - " -
   OCR1A = 2 * F_CPU / BAUDRATE / CLOCK_PR + 18; // timeout => bit zu lang
   ICR1 = 0xffff;  // top-wert
   TIFR1 = 0xff;   // flags löschen
   TIMSK1 = 0x02;          // ocr1a match interrupt;
   PCICR = 0x01;           // flankeninterrupt
   PCMSK0 = PN_MANCH_REC; // enable pcint on receive pin
#endif                     //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   TCCR1A = 0x02;                           // mode 2, fast pwm, top is ocr1c
   TC1H = 0;	// 10-bit register!
   OCR1A = 2 * F_CPU / BAUDRATE / CLOCK_PR; // timeout => bit zu lang
   OCR1C = 0xFF;				// top-wert
   TIFR = 0xFF; 				// flags löschen
   TIMSK = 0x40;           // ocr1a match interrupt;
   GIMSK |= 0x20;          // PF_CPU / BAUDRATE / CLOCK_PRCINT1 enable
   PCMSK1 = PN_MANCH_REC; // enable pcint on receive pin
   PCMSK0 = 0;
#endif                     //__AVR_ATtiny261A__
   manch_i = 0;
   manch_res = 0;
}

void manch_send()
{
   manch_res = 0;
   manch_i = 0;
   manch_d = (uint8_t)(gl_manch_dat >> 8) | 0x80; // msb als startbit immer 1
   manch_d1 = (uint8_t)(gl_manch_dat & 0x00ff);
#ifdef MANCHESTER1
   manch1_d = (uint8_t)(gl_manch_dat1 >> 8) | 0x80; // msb als startbit immer 1
   manch1_d1 = (uint8_t)(gl_manch_dat1 & 0x00ff);
#endif //MANCHESTER1
#ifdef __AVR_ATmega32U4__
   TIFR1 = 0xff;   // flags löschen fx
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   TIFR = 0xFF;
#endif //__AVR_ATtiny261A__
#ifdef __AVR_ATtiny261A__
   TC1H = 0;	// 10-bit register!
#endif
   TCNT1 = F_CPU / BAUDRATE / CLOCK_PR - 20; 
   TCCR1B |= TCCR1B_TIMER_START;
}

uint8_t manch_receive()
{
/*
	uint8_t help; // nur einmal einlesen, manch-res könnte sich während der funktion ändern!
	help = manch_res;
   if (help == 1) // daten fertig empfangen
   {
//      gl_manch_dat = manch_d * 256 + manch_d1; // fx
//      stat_led_off();
      return 1;
   }
   else if (help == 0) // noch nicht fertig
   {
      return 0;
   }
   else // error, overflow
   {
//   	stat_led_red();
gl_manch_dat = manch_res;
      return 2;
   }
*/
return manch_res;
}

// fürs empfangen
#ifdef __AVR_ATmega32U4__
ISR(PCINT0_vect)
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
ISR(PCINT_vect)
#endif //__AVR_ATtiny261A__
{
#ifdef __AVR_ATmega32U4__
   uint16_t tim;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   uint8_t tim;
#endif
PINA=0x80;		//fx
   if (manch_i == 0) // anfang
   {
//   	stat_led_green();
#ifdef __AVR_ATmega32U4__
      if (READMANCH == 0) // 1 als startbit, neg. logik! fx
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
      if (READMANCH != 0) // 1 als startbit, neg. logik! fx
#endif 
      {
//			PORTD^=(1<<PIND3);		//fx
#ifdef __AVR_ATtiny261A__
		   TC1H = 0;	// 10-bit register!
#endif
         TCNT1 = 0;
         TCCR1B |= TCCR1B_TIMER_START; // timer starten

         manch_x = 'k';
         manch_i++;
      }
   }
   else
   {
      tim = TCNT1;
#ifdef __AVR_ATtiny261A__
		TC1H = 0;	// 10-bit register!
#endif
      TCNT1 = 0; // reset timer
//flanke nach halben bit:
      if ((F_CPU / BAUDRATE / CLOCK_PR ) / 3 < tim && tim < (F_CPU / BAUDRATE / CLOCK_PR * 2) / 3) //
      {
PINA=0x870;
         if (manch_x == 'l') // weiter
            manch_x = 'k';
         else // bit einlesen!
         {
            manch_d1 = manch_d1 << 1;
            manch_x = 'l';      // nächstes mal einlesen nach ganzem bit
#ifdef __AVR_ATmega32U4__
            if (READMANCH == 1)
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
            if (READMANCH == 0) 
#endif
            {
               manch_d1 |= 0x01;
            }
            else
            {
               manch_d1 &= 0xFE;
            }
            manch_i++;
            if (manch_i == 9) // 1. byte fertig
            {
               manch_d = manch_d1;
            }
            else if (manch_i == 17) // fertig, empfang stoppen
            {
               TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen

#ifdef __AVR_ATmega32U4__
               TIMSK1 = 0x00; // overflow interrupt stoppen
               PCICR = 0x00;  // flankeninterrupt stoppen
#endif                        //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TIMSK = 0x00;            // overflow interrupt stoppen
               GIMSK &= ~(1 << PCIE1);  // flankeninterrupt stoppen
               PCMSK1 &= ~PN_MANCH_REC; // enable pcint on receive pin
#endif                                  //__AVR_ATtiny261A__

               manch_res = 1;
            }
         }
      }
// flanke nach ganzem bit
      else if ((F_CPU / BAUDRATE / CLOCK_PR * 2) / 3 < tim && tim < (F_CPU / BAUDRATE / CLOCK_PR * 4) / 3) //
      {
PINA=0x870;
         if (manch_x == 'l') // bit einlesen
         {
            manch_d1 = manch_d1 << 1;
#ifdef __AVR_ATmega32U4__
            if (READMANCH == 1)
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
            if (READMANCH == 0) 
#endif
            {
               manch_d1 |= 0x01;
            }
            else
            {
               manch_d1 &= 0xFE;
            }
            manch_i++;
            if (manch_i == 9) // 1. byte fertig
            {
               manch_d = manch_d1;
            }
            else if (manch_i == 17) // fertig, empfang stoppen
            {
               TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen

#ifdef __AVR_ATmega32U4__
               TIMSK1 = 0x00; // overflow interrupt stoppen
               PCICR = 0x00;  // flankeninterrupt stoppen
#endif                        //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TIMSK = 0x00;            // overflow interrupt stoppen
               GIMSK &= ~(1 << PCIE1);  // flankeninterrupt stoppen
               PCMSK1 &= ~PN_MANCH_REC; // enable pcint on receive pin
#endif                                  //__AVR_ATtiny261A__
               manch_res = 1;
            }
         }
         else // es hätte nur ein kurzer sein sollen! neu beginnen
         {
            manch_i = 0; // von vorne!
            TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen
            manch_res = 2;
         }
      }
      else // fehler, flanke nicht zum richtigen zeitpunkt! neu beginnen
      {
         manch_i = 0; // von vorne!
         TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen
			manch_res = 3;
      }
   }
}

ISR(TIMER1_COMPA_vect) // timeout, eine erwartete flanke ist nicht gekommen
// gilt für beide empfänger!!
{
//PORTA ^=0x04;
   manch_i = 0; // von vorne!
   TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen
	manch_res = 4;
}

// fürs senden
ISR(TIMER1_OVF_vect)
{
//stat_led_green();
   if (manch_i == 16) // ende
   {
      CLRMANCH;
#ifdef MANCHESTER1
      SETMANCH1;
#endif // MANCHESTER1
	   TCCR1B &= ~TCCR1B_TIMER_START; // timer stoppen
	   manch_res = 1;
   }
   else
   {
      if (manch_i == 8) // 2. byte
      {
         manch_d = manch_d1;
#ifdef MANCHESTER1
         manch1_d = manch1_d1;
#endif // MANCHESTER1
      }
      if (manch_d & 0x80)
         SETMANCH;
      else
         CLRMANCH;
       manch_d = manch_d << 1;
#ifdef MANCHESTER1
      if (manch1_d & 0x80)
         CLRMANCH1;
      else
         SETMANCH1;
      manch1_d = manch1_d << 1;
#endif // MANCHESTER1
      manch_i++;
   }
//stat_led_off();
}

ISR(TIMER1_COMPB_vect)
{
//stat_led_red();
   TOGMANCH; // pin toggeln
#ifdef MANCHESTER1
   // ein paar nop's!
   TOGMANCH1;
#endif // MANCHESTER1
//stat_led_off();
}

#ifdef MANCHESTER1
void manch_init_receive1()
{
   DDRMANCH1 &= ~PN_MANCH1; // pin als Eingang
   CLRMANCH1;               // no pull-up
   // timer OCRA für receive error, timeout, wenn keine flanke kommt

#ifdef __AVR_ATmega32U4__
   ICR1 = 0xffff;
   TIMSK1 = 0x02; // ocr1a match interrupt;
   TCCR1A = 0x02; // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18; //  - " -
   OCR1A = 3 * F_CPU / BAUDRATE / CLOCK_PR + 18;
   EICRA = 0x01 << 2 * (PN_MANCH1 - 1); // ext. interrupt bei jeder flanke
   EIFR = 0xff;                         // clear all ints
   EIMSK = PN_MANCH1;                   // enable interrupt
#endif                                  //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   TCCR1A = 0x02;                           // mode 2, fast pwm, top is ocr1c
   OCR1A = 3 * F_CPU / BAUDRATE / CLOCK_PR; // timeout => bit zu lang
   OCR1C = 0xFF;
   TIMSK = 0x40;  // ocr1a match interrupt;
   MCUCR = 0x01;  // ext. interrupt bei jeder flanke
   GIFR = 0xFF;   // clear all ints
   GIMSK |= 0x80; // enable external interrupt 1
#endif            //__AVR_ATtiny261A__
   manch_i = 0;
   manch_res = 0;
}


uint8_t manch_receive1(uint16_t *data)
{
   if (manch_res == 1) // daten fertig empfangen
   {
      *data = manch_d1 * 256 + manch_d;
      return 1;
   }
   else if (manch_res == 0) // noch nicht fertig
   {
      return 0;
   }
   else // error, overflow
      return 2;
}

// fürs empfangen
ISR(INT1_vect)
{
#ifdef __AVR_ATmega32U4__
   uint16_t tim;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   uint8_t tim;
#endif               //__AVR_ATtiny261A__
   if (manch_i == 0) // anfang
   {
      if (READMANCH1 == 0) // 1 als startbit! (neg.logik)
      {
#ifdef __AVR_ATmega32U4__
         TCCR1B |= 0x01; // timer starten
         TCNT1 = 0x00;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
         TCCR1B |= 0x05; // timer starten
         TCNT1 = 0x00;
#endif //__AVR_ATtiny261A__
         TCNT1 = 0x0000;
         manch_x = 'k';
         manch_i++;
      }
   }
   else
   {
      tim = TCNT1;
#ifdef __AVR_ATmega32U4__
      TCNT1 = 0x0000;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
      TCNT1 = 0x00;
#endif                                                                                                //__AVR_ATtiny261A__
                                                                                                      // reset timer
      if ((F_CPU / BAUDRATE / CLOCK_PR * 2) / 3 < tim && tim < (F_CPU / BAUDRATE / CLOCK_PR * 4) / 3) //
      {
         if (manch_x == 'l') // weiter
            manch_x = 'k';
         else // bit einlesen!
         {
            manch_d1 = manch_d1 << 1;
            manch_x = 'l'; // nächstes mal einlesen nach ganzem bit
            if (READMANCH1)
            {
               manch_d1 |= 0x01;
            }
            else
            {
               manch_d1 &= 0xFE;
            }
            manch_i++;
            if (manch_i == 9) // 1. byte fertig
            {
               manch_d = manch_d1;
            }
            else if (manch_i == 17) // fertig, empfang stoppen
            {
#ifdef __AVR_ATmega32U4__
               TCCR1B &= ~0x01; // timer stoppen
               TIMSK1 = 0x00;   // overflow interrupt stoppen
               EIMSK = 0x00;    // ext. interrupt disable
#endif                          //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TCCR1B &= ~0x05; // timer stoppen
               TIMSK = 0x00;
               GIMSK &= ~0x80; // disable interrupt
#endif                         //__AVR_ATtiny261A__

               manch_res = 1;
            }
         }
      }
      else if ((F_CPU / BAUDRATE / CLOCK_PR * 4) / 3 < tim && tim < (F_CPU / BAUDRATE / CLOCK_PR * 8) / 3) //
      {
         if (manch_x == 'l') // bit einlesen
         {
            manch_d1 = manch_d1 << 1;
            if (READMANCH1)
            {
               manch_d1 |= 0x01;
            }
            else
            {
               manch_d1 &= 0xFE;
            }
            manch_i++;
            if (manch_i == 9) // 1. byte fertig
            {
               manch_d = manch_d1;
            }
            else if (manch_i == 17) // fertig, empfang stoppen
            {
#ifdef __AVR_ATmega32U4__
               TCCR1B &= ~0x01; // timer stoppen
               TIMSK1 = 0x00;   // overflow interrupt stoppen
               EIMSK = 0x00;    // flankeninterrupt stoppen
#endif                          //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TCCR1B &= ~0x05; // timer stoppen
               TIMSK = 0x00;
               GIMSK &= ~0x80; // disable interrupt
#endif                         //__AVR_ATtiny261A__

               manch_res = 1;
            }
         }
         else // es hätte nur ein kurzer sein sollen! neu beginnen
         {
            manch_i = 0; // von vorne!
#ifdef __AVR_ATmega32U4__
            TCCR1B &= ~0x01; // timer stoppen
#endif                       //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
            TCCR1B &= ~0x05; // timer stoppen
#endif                       //__AVR_ATtiny261A__
         }
      }
      else // fehler, flanke nicht zum richtigen zeitpunkt! neu beginnen
      {
         manch_i = 0; // von vorne!
#ifdef __AVR_ATmega32U4__
         TCCR1B &= ~0x01; // timer stoppen
#endif                    //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
         TCCR1B &= ~0x05; // timer stoppen
#endif                    //__AVR_ATtiny261A__
      }
   }
}

#endif // MANCHESTER1
