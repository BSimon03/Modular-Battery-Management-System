/*********************************************/
/*  file:   balancing.h                      */
/*						                     */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                     */
/*  brief:  Functions for balancing          */
/* 						                     */
/*  Author: Simon Ball                       */
/*********************************************/

#ifndef BALANCING_H
#define BALANCING_H

#define BALANCING_DDR DDRB
#define BALANCING_PORT PORTB
#define BALANCING_PIN PINB4

void balancing_setup(void);

void start_balancing(void);

void stop_balancing(void);

#endif //BALANCING_H