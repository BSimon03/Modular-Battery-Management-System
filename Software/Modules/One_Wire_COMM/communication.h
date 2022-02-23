#ifndef COMM_H
#define COMM_H
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
#define REQ_TEMP_G 0xD4AA   //global request for temperature 1 10 10101 0 0101010
#define REQ_VOL_G 0xC633    //global request for voltage 1 10 00110 0 0110011
#define COM_SLP_G 0xFF0F    //global command for sleep 1 11 1111 1 00001111
#define COM_BLC_A 0xA600    //adressed command for balancing 1 01 00110 0 0000000 COM_BLC_A + adress = data

uint16_t bal_com (uint8_t);
//calculate parity for adressed command. argument is address.

#endif //COMM_H
//"I geh ned mit ins gym. I hob duachfall. I hob mi a bissl ogschissn." -kati 23.02.2022