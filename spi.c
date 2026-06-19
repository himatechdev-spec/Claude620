/* 
 * File:   spi.c
 * Author: liweiwang
 *
 * Created on December 13, 2021, 9:37 PM
 */

#include "xc.h"
#include "spi.h"

void spi_init(void)
{
    //gpio config
    ANSELCbits.ANSC5 = 0;
    ANSELCbits.ANSC4 = 0;
    ANSELCbits.ANSC3 = 0;    
    ANSELAbits.ANSA5 = 0;
            
    TRISCbits.TRISC3 = 0;                   //RC3 output - sck   
    TRISCbits.TRISC4 = 1;                 //RC4 input - miso   
    TRISCbits.TRISC5 = 0;                   //RC5 output - mosi
    TRISAbits.TRISA5 = 0;                   //RA5 output - ss master not in use
    
    TRISCbits.TRISC0 = 0;                   //RC0 output - chipsel
    TRISCbits.TRISC1 = 0;                   //RC1 output - chipsel
    PORTCbits.RC0 = 1;                      //desel
    PORTCbits.RC1 = 1;                      //desel
    
    SSPSTATbits.SMP = 0;                    //sample at middle of data output time
    SSPCON1bits.CKP = 0;                    //clock idle = low (CPOL=0)
    SSPSTATbits.CKE = 1;                    //transmit on active-to-idle (falling) edge: ADS1118 Mode 0

//    SSPCON1bits.SSPM = 0;                   //spi master fosc/4 = 4 MHz (ADS1118 absolute max)
    SSPCON1bits.SSPM = 1;                   //spi master fosc/16 = 1 MHz (safe margin for ADS1118)

    SSPCON1bits.SSPEN = 1;
}

unsigned char  spi_wr_rd_byte(unsigned char data)
{
    SSPBUF = data;
    
    while(0 == SSPSTATbits.BF) continue;
    return SSPBUF;
}

void spi_send_byte(unsigned char data)
{
    SSPBUF = data;
    while(0 == SSPSTATbits.BF) continue;
}

unsigned char spi_read_byte(void)
{
    SSPBUF = 0xff;                              //dummy data
    
    while(0 == SSPSTATbits.BF) continue;
    return SSPBUF;
}