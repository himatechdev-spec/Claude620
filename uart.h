/* 
 * File:   uart.h
 * Author: liweiwang
 *
 * Created on December 11, 2021, 9:59 PM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#ifndef _XTAL_FREQ
#define _XTAL_FREQ           16000000UL
#endif

void uart_init(unsigned long );
unsigned char uart_send_byte(unsigned char );
unsigned char uart_recv_byte(void);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

