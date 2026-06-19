
/*
 * File:   uart.c
 * Author: 52557
 *
 * Created on 31 August 2019, 11:15 AM
 */

#include "xc.h"
#include "uart.h"

void uart_init(unsigned long baud_val)
{
    ANSELCbits.ANSC7 = 0;
//    ANSELCbits.ANSC6 = 0;
    unsigned short brg_val;

    BAUDCONbits.BRG16 = 1;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;         //Asynchronous
    
    TXSTAbits.TXEN = 1;         //Tx enable
    RCSTAbits.CREN = 1;         //Rx enable
    RCSTAbits.SPEN = 1;
    
    brg_val = _XTAL_FREQ/4/baud_val -1;
    SPBRGL = brg_val & 0xff;
    SPBRGH = brg_val >> 8;
}

unsigned char uart_send_byte(unsigned char data) 
{
    while(!TXSTAbits.TRMT);
    
    TXREG = data;
    return 1;
}

unsigned char uart_recv_byte(void)
{
    while(!PIR1bits.RCIF);
        
    return RCREG;
}