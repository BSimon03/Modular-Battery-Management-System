//===============================================================
//
//   Projekt sax-extender master
//
//   rst, jan 2022
//
//================================================================

#include <avr/io.h>
#include "avr/interrupt.h"
#include <util/delay.h>
#include "manch_m.h"

volatile uint8_t manch_i;
volatile uint8_t manch_d, manch_d1;
volatile uint8_t manch1_d, manch1_d1;
volatile uint8_t manch_x, manch_res;

void manch_init_send(void)
{
   DDRMANCH |= PN_MANCH_SEND; // pin als ausgang
   CLRMANCH;                  // und auf 0
   TCCR1A = 0x02;             // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18;             //  - " -
   OCR1B = F_CPU / BAUDRATE + 18;
#ifdef __AVR_ATmega32u4__
   ICR1 = 2 * F_CPU / BAUDRATE;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   OCR1C = 2 * F_CPU / BAUDRATE;
#endif //__AVR_ATtiny261A__
#ifdef __AVR_ATmega32u4__
   TIMSK1 = 0x05; // ocr1b match und overflow interrupt;
#endif            //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   TIMSK = 0x05;
#endif //__AVR_ATtiny261A__
   //    TCCR1B|=0x01; //timer starten
}

void manch_init_receive()
{
   DDRMANCH &= ~PN_MANCH_REC; // pin als Eingang
   CLRMANCH;                  // no pull-up
   // timer OCRA für receive error, timeout, wenn keine flanke kommt
   TCCR1A = 0x02;                     // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18;                     //  - " -
   OCR1A = 3 * F_CPU / BAUDRATE + 18; // timeout => bit zu lang
#ifdef __AVR_ATmega32u4__
   ICR1 = 0xffff;
   TIMSK1 = 0x02; // ocr1a match interrupt;
#endif            //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
   OCR1C = 0xFF;
   TIMSK = 0x02;
#endif //__AVR_ATtiny261A__
   manch_i = 0;
   manch_res = 0;
#ifdef __AVR_ATmega32u4__
   PCICR = 0x01; // flankeninterrupt
#endif           //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
// PCINT...
#endif //__AVR_ATtiny261A__

   PCMSK0 = PN_MANCH_REC;
}

void manch_send(uint16_t data)
{
   manch_res = 0;
   manch_i = 0;
   manch_d = (uint8_t)(data >> 8) | 0x80; // msb als startbit immer 1
   manch_d1 = (uint8_t)(data & 0x00ff);
   TCNT1 = F_CPU / BAUDRATE + 5;
#ifdef __AVR_ATmega32u4__
   TIFR1 = 0x00; // flags löschen
#endif           //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
// flags löschen
#endif                            //__AVR_ATtiny261A__
   TCCR1B |= 0x01;                // timer starten
   TCNT1 = F_CPU / BAUDRATE + 20; // nur fürs simulieren!!
}

uint8_t manch_receive(uint16_t *data)
{
   // simul
   // PORTD = data[0];
   if (manch_res == 1) // daten fertig empfangen
   {
      *data = manch_d1 * 256 + manch_d;
#ifdef SIMAVR
      PORTD = 0x10;
      PORTD = data[0];
      PORTD = data[1];
#endif // SIMAVR
      return 1;
   }
   else if (manch_res == 0) // noch nicht fertig
   {
#ifdef SIMAVR
      PORTD = 0x80;
#endif // SIMAVR
      return 0;
   }
   else // error, overflow
      return 2;
}

// fürs empfangen
ISR(PCINT0_vect)
{
   // PORTD = 0x01;
   uint16_t tim;
   if (manch_i == 0) // anfang
   {
      if (READMANCH == 0) // 1 als startbit, neg. logik!
      {
         TCCR1B |= 0x01; // timer starten
         TCNT1 = 0x0000;
         manch_x = 'k';
         manch_i++;
      }
   }
   else
   {
      tim = TCNT1;
      TCNT1 = 0x0000;                                                           // reset timer
      if ((F_CPU / BAUDRATE * 2) / 3 < tim && tim < (F_CPU / BAUDRATE * 4) / 3) //
      {
         if (manch_x == 'l') // weiter
            manch_x = 'k';
         else // bit einlesen!
         {
            // PORTD=0x02;
            manch_d1 = manch_d1 << 1;
            manch_x = 'l'; // nächstes mal einlesen nach ganzem bit
            if (READMANCH)
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
               TCCR1B &= ~0x01; // timer stoppen
#ifdef __AVR_ATmega32u4__
               TIMSK1 = 0x00; // overflow interrupt stoppen
               PCICR = 0x00;  // flankeninterrupt stoppen
#endif                        //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TIMSK = 0x00;
               // PCINT...
#endif //__AVR_ATtiny261A__

               manch_res = 1;
            }
         }
      }
      else if ((F_CPU / BAUDRATE * 4) / 3 < tim && tim < (F_CPU / BAUDRATE * 8) / 3) //
      {
         if (manch_x == 'l') // bit einlesen
         {
            // PORTD=0x04;
            manch_d1 = manch_d1 << 1;
            if (READMANCH)
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
               TCCR1B &= ~0x01; // timer stoppen
#ifdef __AVR_ATmega32u4__
               TIMSK1 = 0x00; // overflow interrupt stoppen
               PCICR = 0x00;  // flankeninterrupt stoppen
#endif                       //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
               TIMSK = 0x00; // overflow interrupt stoppen
               //stop pcint
#endif                        //__AVR_ATtiny261A__
               manch_res = 1;
            }
         }
         else // es hätte nur ein kurzer sein sollen! neu beginnen
         {
            manch_i = 0;     // von vorne!
            TCCR1B &= ~0x01; // timer stoppen
         }
      }
      else // fehler, flanke nicht zum richtigen zeitpunkt! neu beginnen
      {
         // PORTD=0x08;
         manch_i = 0;     // von vorne!
         TCCR1B &= ~0x01; // timer stoppen
      }
   }
   //    PORTD = 0;
}

ISR(TIMER1_COMPA_vect) // timeout, eine erwartete flanke ist nicht gekommen
// gilt für beide empfänger!!
{
   // PORTD = 0x10;
   manch_i = 0;     // von vorne!
   TCCR1B &= ~0x01; // timer stoppen
   // PORTD = 0x00;
}

// fürs senden
ISR(TIMER1_OVF_vect)
{
   if (manch_i == 16) // ende
   {
      TCCR1B &= 0xFE; // timer stoppen
      CLRMANCH;
#ifdef MANCHESTER1
      CLRMANCH1;
#endif // MANCHESTER1
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
#ifdef MANCHESTER1
      if (manch1_d & 0x80)
         SETMANCH1;
      else
         CLRMANCH1;
#endif // MANCHESTER1
      manch_i++;
      manch_d = manch_d << 1;
#ifdef MANCHESTER1
      manch1_d = manch1_d << 1;
#endif // MANCHESTER1
   }
}

ISR(TIMER1_COMPB_vect)
{
   TOGMANCH; // pin toggeln
#ifdef MANCHESTER1
   // ein paar nop's!
   TOGMANCH1;
#endif // MANCHESTER1
}

#ifdef MANCHESTER1
void manch_init_send1(void)
{
   DDRMANCH1 |= PN_MANCH1; // pin als ausgang
   CLRMANCH1;              // und auf 1
}

void manch_init_receive1()
{
   DDRMANCH1 &= ~PN_MANCH1; // pin als Eingang
   CLRMANCH1;               // no pull-up
   // timer OCRA für receive error, timeout, wenn keine flanke kommt
   TCCR1A = 0x02; // mode 14, fast pwm, top is icr1
   TCCR1B = 0x18; //  - " -
   OCR1A = 3 * F_CPU / BAUDRATE + 18;
   #ifdef __AVR_ATmega32u4__
ICR1 = 0xffff;
   TIMSK1 = 0x02; // ocr1a match interrupt;
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
OCR1C=0xFF;
TIMSK =0x02;
#endif //__AVR_ATtiny261A__
   

   manch_i = 0;
   manch_res = 0;
   #ifdef __AVR_ATmega32u4__
EICRA = 0x01 << 2 * (PN_MANCH1 - 1); // ext. interrupt bei jeder flanke
   EIFR = 0xff;                         // clear all ints
   EIMSK = PN_MANCH1;                   // enable interrupt
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
//EXTERNAL INTERRUPT...
#endif //__AVR_ATtiny261A__
   
}

void manch_send1(uint16_t data)
{
   manch1_d = (uint8_t)(data >> 8) | 0x80; // msb als startbit immer 1
   manch1_d1 = (uint8_t)(data & 0x00ff);
}

uint8_t manch_receive1(uint16_t *data)
{
   // simul
   // PORTB = data[0];
   if (manch_res == 1) // daten fertig empfangen
   {
      *data = manch_d1 * 256 + manch_d;
#ifdef SIMAVR
      PORTB = 0x10;
      PORTB = data[0];
      PORTB = data[1];
#endif // SIMAVR
      return 1;
   }
   else if (manch_res == 0) // noch nicht fertig
   {
      // simulieren
      // PORTB=0x80;
      return 0;
   }
   else // error, overflow
      return 2;
}

// fürs empfangen
ISR(INT0_vect)
{
   // PORTB = 0x01;
   uint16_t tim;
   if (manch_i == 0) // anfang
   {
      if (READMANCH1 == 0) // 1 als startbit! (neg.logik)
      {
         TCCR1B |= 0x01; // timer starten
         TCNT1 = 0x0000;
         manch_x = 'k';
         manch_i++;
      }
   }
   else
   {
      tim = TCNT1;
      TCNT1 = 0x0000;                                                           // reset timer
      if ((F_CPU / BAUDRATE * 2) / 3 < tim && tim < (F_CPU / BAUDRATE * 4) / 3) //
      {
         if (manch_x == 'l') // weiter
            manch_x = 'k';
         else // bit einlesen!
         {
            // PORTB=0x02;
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
               TCCR1B &= ~0x01; // timer stoppen
               #ifdef __AVR_ATmega32u4__
TIMSK1 = 0x00;   // overflow interrupt stoppen
               EIMSK = 0x00;    // ext. interrupt disable
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
TIMSK =0x00;
//ext int disable
#endif //__AVR_ATtiny261A__
               
               manch_res = 1;
            }
         }
      }
      else if ((F_CPU / BAUDRATE * 4) / 3 < tim && tim < (F_CPU / BAUDRATE * 8) / 3) //
      {
         if (manch_x == 'l') // bit einlesen
         {
            // PORTB=0x04;
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
               TCCR1B &= ~0x01; // timer stoppen
               #ifdef __AVR_ATmega32u4__
 TIMSK1 = 0x00;   // overflow interrupt stoppen
               EIMSK = 0x00;    // flankeninterrupt stoppen
#endif //__AVR_ATmega32u4
#ifdef __AVR_ATtiny261A__
TIMSK =0x00;
//interrupt stoppen...
#endif //__AVR_ATtiny261A__
              
               manch_res = 1;
            }
         }
         else // es hätte nur ein kurzer sein sollen! neu beginnen
         {
            manch_i = 0;     // von vorne!
            TCCR1B &= ~0x01; // timer stoppen
         }
      }
      else // fehler, flanke nicht zum richtigen zeitpunkt! neu beginnen
      {
         // PORTB=0x08;
         manch_i = 0;     // von vorne!
         TCCR1B &= ~0x01; // timer stoppen
      }
   }
   // PORTB = 0;
}

#endif // MANCHESTER1
