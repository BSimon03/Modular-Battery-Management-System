/***************************/
/*        main.cpp         */
/*    Diploma Thesis BMS   */
/*						             */
/*    ATMEGA32u4 Master    */
/*          V1.0           */
/* 						             */
/* Author: Tristan Horvath */
/***************************/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "init_master.h"

int main(void)
{
  init_master();
  while(1)
  {

  }
}