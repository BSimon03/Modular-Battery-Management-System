/*************************/
/*  communication.h      */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Functions for        */
/*  communication with   */
/*  upper and lower uC   */
/*************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

int handle_upper(int);

int handle_lower(int);