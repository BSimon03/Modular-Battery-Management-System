/*********************************************/
/*  file:   balancing.c                      */
/*						                     */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                     */
/*  brief:  Functions for balancing          */
/* 						                     */
/*  Author: Simon Ball                       */
/*********************************************/

#include <avr/io.h>
#include <stdint.h>
#include "balancing.h"

void balancing_init()
{
    BALANCING_DDR |= (1<<BALANCING_PIN);
}

void start_balancing()
{
    BALANCING_PORT |= (1<<BALANCING_PIN);
}

void stop_balancing()
{
    BALANCING_PORT &= ~(1<<BALANCING_PIN);
}