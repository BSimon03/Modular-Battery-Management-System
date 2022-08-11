/*
 * Recieve_Test.c
 *
 * Created: 06.04.2022 08:51:21
 * Author : trist
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "communication.h"
#include "status.h"
#include "manch_m.h"
#include "timer.h"


int main(void)
{
	
	//fx
	DDRD|=(1<<PIND3);
	PORTD&=~(1<<PIND3);
	
	
	uint16_t daten;
	uint8_t state=0;
	uint8_t com_stat=0;
	
	timer_init_timer();
	stat_led_init();
	uint16_t counter=0;
	
	//fx
	sei();
    while (1) 
    {
    	PORTD^=(1<<PIND3);
		
		/*while (1)
		{
			manch_init_send();
			manch_send();
			_delay_ms(500);
		}*/
		//============================Recieve Test==========================
		if(state==0)		//empfangen initialisieren
		{
			manch_init_receive();
			com_stat=0;
			state++;
		}
		
		else if(state==1)		//Daten Empfangen
		{
			com_stat=manch_receive();
			
			if (com_stat==0)		//während auf Daten gewartet wird LED orange blinken
			{
				stat_led_orange();
				if(counter>=1000)
				{				//PORTD^=(1<<PIND3);
					counter=0;
				}
			}
			
			if(com_stat==1)			//wenn Daten erfolgreich Empfangen wurden LED grün
			{
				stat_led_green();
				_delay_ms(100);
//				manch_init_send();
//				manch_send(daten);
//				if(daten==0xFF00)
//				{
//					stat_led_red();	
//				}
				_delay_ms(500);
				state=0;
				//state=0;
			}
			else if (com_stat==2)		//wenn Fehler beim Empfangen LED rot
			{
				stat_led_red();
				state=0;
			}
		}
		
		//Timer für Heartbeat
		timer_add_time();
		counter+=timer_get_timer(HEARTBEAT);
		timer_clear_timer(HEARTBEAT);
    }
}

