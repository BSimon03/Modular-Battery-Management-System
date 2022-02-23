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

#include <stdio.h>
#include <stdint.h>
#include "communication.h"

uint16_t bal_com (uint8_t address)
{
    if(address%2==1)
    {
        return COM_BLC_A + 0x10 + address;
    }
    else if (address%2==0)
    {
        return COM_BLC_A + address;
    }
}