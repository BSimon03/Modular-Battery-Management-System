/*************************/
/*  communication.c      */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Functions for        */
/*  communication with   */
/*  upper and lower uC   */
/*************************/

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
uint8_t calc_parity(uint16_t data)
{
    uint8_t par_cnt=0;
    for(int i=0; i<16;i++)
    {
        if(data&(1<<i))
        {
            par_cnt++;
        }
    }
    if(par_cnt%2)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}