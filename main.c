/* 
 * File:   main.c
 * Author: liweiwang
 *
 * Created on December 11, 2021, 5:46 PM
 */


// PIC16F1516 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>

#include "uart.h"
#include "spi.h"

/*
 * 
 */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ              16000000UL
#endif
#define BAUDRATE                115200UL

/* Type-K thermocouple table, 0-219°C, 22 segments of 10°C each.
 * Voltages in μV.  tc_base_uV has 23 entries: index 0-21 are segment
 * starts, index 22 (8938 μV) is the 220°C upper bound / out-of-range guard.
 * tc_step_uV[i] is the total voltage span (μV) across the i-th 10°C segment. */
#define TC_NUM_SEGS  22

static const unsigned short tc_base_uV[TC_NUM_SEGS + 1] = {
       0,  397,  798, 1203, 1611, 2022, 2436, 2850,
    3266, 3681, 4095, 4508, 4919, 5327, 5733, 6137,
    6539, 6939, 7338, 7737, 8137, 8537, 8938
};
static const unsigned short tc_step_uV[TC_NUM_SEGS] = {
    397, 401, 405, 408, 411, 414, 414, 416,
    415, 414, 413, 411, 408, 406, 404, 402,
    400, 399, 399, 400, 400, 401
};

#define MODBUS_RDBUF_LEN        14
#define MODBUS_WRBUF_LEN        12
#define MULT_REG_RD_CMD         0x3
#define SING_REG_WR_CMD         0x6
#define MULT_REG_WR_CMD         0x10
#define HMI_STA_NO              0x1
#define HMI_RD_START_ADDR       0x33
#define HMI_WR_START_ADDR       0xb
#define FDMT_STA_NO             0x3
#define MXMT_STA_NO             0x2
#define MT_OP_FREQ_ADDR         0x1000

//MCU write to HMI info - txdata
#define FLOWRATE_IDX            0           //LW11
#define FDMT_FREQ_IDX           1
#define MXMT_FREQ_IDX           2
#define T1_MEAS_IDX             3
#define T2_MEAS_IDX             4
#define T3_MEAS_IDX             5
//#define T4_MEAS_IDX             6           // unuse

#define ERRCODE_IDX             7
#define ERRCODE_ERR0_BIT        0

#define STACODE_IDX             8
#define STACODE_FLCALI_BIT      0
#define STACODE_SS_BIT          1
#define STACODE_HEATCO1_BIT     2
#define STACODE_HEATCO2_BIT     3
#define STACODE_VALVIN_BIT      4
#define STACODE_VALVOT_BIT      5
#define STACODE_MXMTSS_BIT      6
#define STACODE_FDMTSS_BIT      7
#define STACODE_HTCTR_BIT       8

#define FL_CALI_PUL_IDX         9
#define TCJ_A_IDX               10
#define TCJ_B_IDX               11

//MCU read from HMI info - rxdata
#define T1_CALI_IDX             0           //LW51
#define T2_CALI_IDX             1
#define T3_CALI_IDX             2
//#define T4_CALI_IDX             3           // unuse
//#define FL_CALI_VOL_IDX         4           
#define VOL_CALIPUL_RD_IDX      5           
#define TRGT_TEMP_IDX           6           
#define COOKTIM_IDX             7
#define STACODE_RD_IDX          8
#define VOLTOTAL_IDX            9
#define FL_CALIPUL_RD_IDX          10
#define FLOWRATE_RD_IDX            11       //User should use this val instead of motor speeeds
#define FDMT_FREQ_RD_IDX           12       //for debug or preset in config
#define MXMT_FREQ_RD_IDX           13       //for debug or preset in config


void thermo_sens_update(void);
void thermo_sens_init(void);
unsigned short volt_temp_lookup(unsigned short, short);
static unsigned short temp_to_uV(unsigned short);
void modbus_tx_start(unsigned char , unsigned char , unsigned short , unsigned short , volatile unsigned short *);
unsigned char modbus_tx_poll(void);
unsigned short modbusCRC16(const unsigned char *, unsigned short );


volatile unsigned short txdata[MODBUS_WRBUF_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0};
volatile unsigned short fl_pulse_litre = 0,
                        fl_pulse = 0;

unsigned char rxdata[3 +MODBUS_RDBUF_LEN *2 +2] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0}; //(31 -3) = 28 ==> 0->13 x2
volatile unsigned char rxdata_ready_flag = 0;

volatile unsigned char  rsp_type = 0,
                        rsp_fr_len = 0,
                        fr_rx_flag = 0,
                        fl_pulse_cali_flag = 0;
volatile unsigned short timeout_cnt = 0;

/* Non-blocking Modbus transaction timing (1ms ticks, decremented in the
 * TMR2 ISR). settle_cnt replaces the old fixed __delay_ms(60) post-response
 * delay; thermo_due_cnt paces thermo_sens_update() so it isn't called
 * faster than the ADS1118's ~7.8ms (128 SPS) minimum conversion interval,
 * now that the main loop can call it many times while a transaction is
 * in flight instead of once per (previously blocking) pass. */
volatile unsigned short settle_cnt = 0,
                        thermo_due_cnt = 0;
static unsigned char   mb_busy = 0,
                        settle_armed = 0;

/* Cached thermocouple calibration offsets. Updated only from a confirmed
 * complete, CRC-valid rxdata[] frame (see the rxdata_ready_flag block in
 * main()), so thermo_sens_update() never applies a torn/partial read. */
short t1_cali = 0,
      t2_cali = 0,
      t3_cali = 0;

int main(int argc, char** argv) 
{  
//    unsigned char modbus_task_id = HMI_STA_NO;
    
    OSCCONbits.IRCF = 0b1111;               //internal fosc 16M
    
    //Output pins RA0 - 4 RB7 RC2 init
//    ANSELAbits.ANSELA &= ~0x1f;
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELAbits.ANSA3 = 0;
    
    TRISAbits.TRISA0 = 0;
    PORTAbits.RA0 = 0;
    TRISAbits.TRISA1 = 0;
    PORTAbits.RA1 = 0;   
    TRISAbits.TRISA2 = 0;
    PORTAbits.RA2 = 0;
    TRISAbits.TRISA3 = 0;
    PORTAbits.RA3 = 0;
    TRISAbits.TRISA4 = 0;
    PORTAbits.RA4 = 0;
    
    TRISBbits.TRISB7 = 0;
    PORTBbits.RB7 = 0;
    
    ANSELCbits.ANSC2 = 0;
    TRISCbits.TRISC2 = 0;
    PORTCbits.RC2 = 0;
    
    //Input pins RB0 - 5 init
//    ANSELBbits.ANSELB &= ~0x003f;
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    ANSELBbits.ANSB4 = 0;
    ANSELBbits.ANSB5 = 0;
    
    IOCBPbits.IOCBP2 = 1;                   //RB2 IOC RE
    INTCONbits.IOCIE = 1;                   //IOC interrupt enable
    
    //Timer0
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b111;
    OPTION_REGbits.TMR0CS = 0;              //CLK source = Fosc/4
    INTCONbits.TMR0IE = 1;

    //Timer2
//    PR2 = 125;                              //5msec interrupt
    PR2 = 25;                                 //1msec interrupt
//    PR2 = 5;                                 //200usec interrupt
    T2CONbits.T2OUTPS = 0b1001;
    T2CONbits.T2CKPS = 0b10;
    PIE1bits.TMR2IE = 1;                    //TIMER2 interrupt enable
    T2CONbits.TMR2ON = 1;

    PIE1bits.RCIE = 1;                      //UART RX interrupt enable
 
    INTCONbits.PEIE = 1;                    //Peripheral int enable
    INTCONbits.GIE = 1;                     //Global in enable
    
    uart_init(BAUDRATE);

    spi_init();
    thermo_sens_init();
    
    //Read settings from config file; send to hmi
    //txdata = ??
//    modbus_master_send(HMI_STA_NO,MULT_REG_WR_CMD,HMI_WR_START_ADDR,MODBUS_WRBUF_LEN,txdata);
   
    typedef enum {
        SEQ_HMI_READ_START, SEQ_HMI_READ_WAIT,
        SEQ_FLCALI_START,   SEQ_FLCALI_WAIT,
        SEQ_HMI_WRITE_START,SEQ_HMI_WRITE_WAIT,
        SEQ_MXMT_START,     SEQ_MXMT_WAIT,
        SEQ_FDMT_START,     SEQ_FDMT_WAIT
    } main_seq_t;

    main_seq_t seq = SEQ_HMI_READ_START;
    unsigned char pending_flcali = 0;

    while(1)
    {
        CLRWDT();

        if(0 == thermo_due_cnt)
        {
            thermo_sens_update();
            thermo_due_cnt = 10;    //>7.8ms ADS1118 min conversion interval @128SPS
        }

        switch(seq)
        {
            case SEQ_HMI_READ_START:
                modbus_tx_start(HMI_STA_NO,MULT_REG_RD_CMD,HMI_RD_START_ADDR,MODBUS_RDBUF_LEN,0);
                seq = SEQ_HMI_READ_WAIT;
                break;

            case SEQ_HMI_READ_WAIT:
                if(!modbus_tx_poll())
                    break;

                if(rxdata_ready_flag)
                {
                    txdata[ERRCODE_IDX] &= ~(1 << ERRCODE_ERR0_BIT);

                    PORTAbits.RA0 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_MXMTSS_BIT/8] & (1<< STACODE_MXMTSS_BIT%8))? 1: 0;
                    PORTAbits.RA1 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_FDMTSS_BIT/8] & (1<< STACODE_FDMTSS_BIT%8))? 1: 0;

                    PORTAbits.RA4 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_VALVIN_BIT/8] & (1<< STACODE_VALVIN_BIT%8))? 1: 0;
                    PORTCbits.RC2 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_VALVOT_BIT/8] & (1<< STACODE_VALVOT_BIT%8))? 1: 0;

                    if(!(rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_HTCTR_BIT/8] & (1<< STACODE_HTCTR_BIT%8)))
                    {
                        PORTAbits.RA2 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_HEATCO1_BIT/8] & (1<< STACODE_HEATCO1_BIT%8))? 1: 0;
                        PORTAbits.RA3 = (rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_HEATCO2_BIT/8] & (1<< STACODE_HEATCO2_BIT%8))? 1: 0;
                    }
                    else
                    {
                        PORTAbits.RA2 = (((rxdata[3+ TRGT_TEMP_IDX *2] <<8) | rxdata[3+ TRGT_TEMP_IDX *2 +1]) > txdata[T1_MEAS_IDX])? 1:0;
                        PORTAbits.RA3 = (((rxdata[3+ TRGT_TEMP_IDX *2] <<8) | rxdata[3+ TRGT_TEMP_IDX *2 +1]) > txdata[T2_MEAS_IDX])? 1:0;
                    }

                    pending_flcali = 0;
                    if(rxdata[3+ STACODE_RD_IDX*2 +1 -STACODE_FLCALI_BIT/8] & (1<< STACODE_FLCALI_BIT%8))
                    {
                        if(0 == fl_pulse_cali_flag)
                        {
                            fl_pulse_cali_flag = 1;
                            txdata[FL_CALI_PUL_IDX] = 0;
                        }
                    }
                    else
                    {
                        if(fl_pulse_cali_flag)
                        {
                            fl_pulse_cali_flag = 0;
                            pending_flcali = 1;
                        }
                    }

                    if((rxdata[3+ VOL_CALIPUL_RD_IDX *2] <<8) | rxdata[3+ VOL_CALIPUL_RD_IDX *2 +1])
                        fl_pulse_litre = (((unsigned long)rxdata[3+ FL_CALIPUL_RD_IDX *2] <<8) | rxdata[3+ FL_CALIPUL_RD_IDX *2 +1]) *1000 / (((unsigned short)rxdata[3+ VOL_CALIPUL_RD_IDX *2] <<8) | rxdata[3+ VOL_CALIPUL_RD_IDX *2 +1]);

                    txdata[STACODE_IDX] = (PORTAbits.RA0 <<STACODE_MXMTSS_BIT) |
                                          (PORTAbits.RA1 <<STACODE_FDMTSS_BIT) |
                                          (PORTAbits.RA2 <<STACODE_HEATCO1_BIT) |
                                          (PORTAbits.RA3 <<STACODE_HEATCO2_BIT) |
                                          (PORTAbits.RA4 <<STACODE_VALVIN_BIT) |
                                          (PORTCbits.RC2 <<STACODE_VALVOT_BIT);
                    txdata[FDMT_FREQ_IDX] = (unsigned short)(rxdata[3+ FDMT_FREQ_RD_IDX *2] <<8)| rxdata[3+ FDMT_FREQ_RD_IDX *2 +1];
                    txdata[MXMT_FREQ_IDX] = (unsigned short)(rxdata[3+ MXMT_FREQ_RD_IDX *2] <<8)| rxdata[3+ MXMT_FREQ_RD_IDX *2 +1];

                    t1_cali = (short)(((unsigned short)rxdata[3+ T1_CALI_IDX *2] <<8) | rxdata[3+ T1_CALI_IDX *2 +1]);
                    t2_cali = (short)(((unsigned short)rxdata[3+ T2_CALI_IDX *2] <<8) | rxdata[3+ T2_CALI_IDX *2 +1]);
                    t3_cali = (short)(((unsigned short)rxdata[3+ T3_CALI_IDX *2] <<8) | rxdata[3+ T3_CALI_IDX *2 +1]);
                }
                else
                {
                    txdata[ERRCODE_IDX] |= (1 << ERRCODE_ERR0_BIT);
                }

                seq = pending_flcali? SEQ_FLCALI_START : SEQ_HMI_WRITE_START;
                break;

            case SEQ_FLCALI_START:
                modbus_tx_start(HMI_STA_NO,SING_REG_WR_CMD,HMI_RD_START_ADDR +FL_CALIPUL_RD_IDX,1,txdata +FL_CALI_PUL_IDX);
                seq = SEQ_FLCALI_WAIT;
                break;

            case SEQ_FLCALI_WAIT:
                if(modbus_tx_poll())
                    seq = SEQ_HMI_WRITE_START;
                break;

            case SEQ_HMI_WRITE_START:
                modbus_tx_start(HMI_STA_NO,MULT_REG_WR_CMD,HMI_WR_START_ADDR,MODBUS_WRBUF_LEN,txdata);
                seq = SEQ_HMI_WRITE_WAIT;
                break;

            case SEQ_HMI_WRITE_WAIT:
                if(!modbus_tx_poll())
                    break;

                if(PORTAbits.RA0)
                    seq = SEQ_MXMT_START;
                else if(PORTAbits.RA1)
                    seq = SEQ_FDMT_START;
                else
                    seq = SEQ_HMI_READ_START;
                break;

            case SEQ_MXMT_START:
                modbus_tx_start(MXMT_STA_NO,SING_REG_WR_CMD,MT_OP_FREQ_ADDR,1,txdata +MXMT_FREQ_IDX);
                seq = SEQ_MXMT_WAIT;
                break;

            case SEQ_MXMT_WAIT:
                if(!modbus_tx_poll())
                    break;

                seq = PORTAbits.RA1? SEQ_FDMT_START : SEQ_HMI_READ_START;
                break;

            case SEQ_FDMT_START:
                modbus_tx_start(FDMT_STA_NO,SING_REG_WR_CMD,MT_OP_FREQ_ADDR,1,txdata +FDMT_FREQ_IDX);
                seq = SEQ_FDMT_WAIT;
                break;

            case SEQ_FDMT_WAIT:
                if(modbus_tx_poll())
                    seq = SEQ_HMI_READ_START;
                break;
        }
    }
    return (EXIT_SUCCESS);
}

/*Interrupt Routine*/
void __interrupt() isr(void) 
{
    static unsigned char second_cnt = 0,
                         rx_idx = 0;
    unsigned char dummy = 0;
    
    if(INTCONbits.IOCIE && INTCONbits.IOCIF)
    {
        if(IOCBFbits.IOCBF2)
        {
            IOCBFbits.IOCBF2 = 0;

            if(fl_pulse_cali_flag)
                txdata[FL_CALI_PUL_IDX] += 1;
            else
                fl_pulse += 1;
        }
    }
    
    if (PIE1bits.RCIE && PIR1bits.RCIF)
    {
        unsigned char oerr_now = (1 == RCSTAbits.OERR);

        if (oerr_now)
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
            dummy = RCREG;      //discard the indeterminate byte flushed by the CREN reset
            fr_rx_flag = 0;     //abandon the corrupted in-progress frame
            rx_idx = 0;
        }
        else if((fr_rx_flag == 0) && timeout_cnt)
        {
            fr_rx_flag = 1;
            rx_idx = 0;
        }

        if(!oerr_now)
        {
            if(fr_rx_flag)
            {
                timeout_cnt = 1;

                TMR2 = 0;
                PIR1bits.TMR2IF = 0;

                if(rx_idx < sizeof(rxdata))
                {
                    rxdata[rx_idx] = RCREG;
                    rx_idx += 1;
                }
                else
                {
                    dummy = RCREG;
                }
            }
            else
            {
                dummy = RCREG;
            }
        }
    }

    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
               
        second_cnt += 1;
        if(second_cnt >= 61)    //Fosc/4/256/256 , overflow, prescaler
        {
            second_cnt -= 61;
            if(fl_pulse_litre)
            {
                txdata[FLOWRATE_IDX] = fl_pulse *60/fl_pulse_litre;//(unsigned long)fl_pulse *60/fl_pulse_litre; //L per min
//                txdata[ERRCODE_IDX] = fl_pulse *60/fl_pulse_litre;
            }
            fl_pulse = 0;
        
            PORTBbits.RB7 = !PORTBbits.RB7;    //blink heartbeat, ~1 Hz
        }
    }
    
    if (PIE1bits.TMR2IE && PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;

        if(settle_cnt)
            settle_cnt -= 1;

        if(thermo_due_cnt)
            thermo_due_cnt -= 1;

        if(timeout_cnt)
        {
            timeout_cnt -= 1;
            if(0 == timeout_cnt)
            {
                if(fr_rx_flag) //rx stopped
                {
                    fr_rx_flag = 0;
                    
                    if(rxdata[1] == 0x86) 
                    {
//                        if(rxdata[0] == 0x2)
//                            txdata[ERRCODE_IDX] = rxdata[2]<<4;
//                        if(rxdata[0] == 0x3)
//                            txdata[ERRCODE_IDX] = rxdata[2]<<8;
                    }

                    if(rx_idx == rsp_fr_len)
                    {
                        rxdata_ready_flag = 1;
                    }                    
                }
                else //wait rsp timeout
                {
                    
                }
            }
        }
    }
}

/* Builds and transmits one Modbus RTU request, then arms the async
 * wait/settle tracked by modbus_tx_poll(). Non-blocking except for the
 * frame transmission itself (<1ms for these frame sizes at 115200 baud). */
void modbus_tx_start(unsigned char sta_id, unsigned char func, unsigned short addr, unsigned short reg_cnt, volatile unsigned short *data)
{
    unsigned char data_buff[33] = {sta_id,func,addr>>8,addr&0xff,0,0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0},
                  fr_len = 0;
    unsigned short crc = 0;

    switch(func)
    {
        case 0x6:       //write single register
            if (reg_cnt != 1)
                return;
            data_buff[4] = *data >>8;
            data_buff[5] = *data &0xff;

            crc = modbusCRC16(data_buff,6);
            data_buff[6] = crc &0xff;
            data_buff[7] = crc >>8;
            fr_len = 8;
            rsp_fr_len = 8;
            break;
        case 0x10:      //write multiple registers
            if ((reg_cnt <=1)||(reg_cnt >12))   //12 reg max, limited by data_buff[] size
                return;
            data_buff[4] = reg_cnt >>8;
            data_buff[5] = reg_cnt &0xff;
            data_buff[6] = reg_cnt <<1;

            for(unsigned char i=0;i<reg_cnt;++i)
            {
                data_buff[7 +2*i] = *(data +i) >>8;
                data_buff[8 +2*i] = *(data +i) &0xff;
            }

            crc = modbusCRC16(data_buff,7+(reg_cnt<<1));
            data_buff[7+(reg_cnt<<1)] = crc &0xff;
            data_buff[7+(reg_cnt<<1) +1] = crc >>8;
            fr_len = 7+(reg_cnt<<1) +2;
            rsp_fr_len = 6;
            break;
        case 0x3:       //read multiple registers
//            if(reg_cnt >10)       limited by rxdata[] size
//                return;
            data_buff[4] = reg_cnt >>8;
            data_buff[5] = reg_cnt &0xff;

            crc = modbusCRC16(data_buff,6);
            data_buff[6] = crc &0xff;
            data_buff[7] = crc >>8;
            fr_len = 8;
            rsp_fr_len = 3+ (reg_cnt<<1) +2;
            break;
        default:
            return;
    }

    rxdata_ready_flag = 0;

    for(unsigned char i=0;i<fr_len;++i)
        uart_send_byte(data_buff[i]);

    rsp_type = func;
    timeout_cnt = 500;
    settle_armed = 0;
    mb_busy = 1;
}

/* Polls the transaction armed by modbus_tx_start(). Returns 1 once both
 * the response wait (or timeout) and the post-response settle window have
 * completed, 0 while still busy. Returns 1 immediately if no transaction
 * is in flight (including when modbus_tx_start() rejected its arguments
 * and never armed one), matching the old modbus_master_send()'s no-op
 * early-return behavior. Call every main-loop pass; other work (e.g.
 * thermo_sens_update()) can run freely in between calls while busy. */
unsigned char modbus_tx_poll(void)
{
    if(!mb_busy)
        return 1;

    if(timeout_cnt)
        return 0;

    if(!settle_armed)
    {
        if(rxdata_ready_flag && rsp_fr_len >= 2)
        {
            unsigned short calc_crc = modbusCRC16(rxdata, rsp_fr_len - 2);
            unsigned short recv_crc = (unsigned short)rxdata[rsp_fr_len-2] |
                                      ((unsigned short)rxdata[rsp_fr_len-1] << 8);
            if(calc_crc != recv_crc)
                rxdata_ready_flag = 0;
        }
        settle_cnt = 60;
        settle_armed = 1;
        return 0;
    }

    if(settle_cnt)
        return 0;

    mb_busy = 0;
    settle_armed = 0;
    return 1;
}

/* Forward (temp→voltage) lookup used for cold-junction compensation.
 * Returns equivalent EMF in μV for a given integer temperature in °C. */
static unsigned short temp_to_uV(unsigned short temp_C)
{
    unsigned char seg = (unsigned char)(temp_C / 10);
    if(seg >= TC_NUM_SEGS)
        seg = TC_NUM_SEGS - 1;
    return tc_base_uV[seg] + (unsigned short)(tc_step_uV[seg] * (temp_C % 10)) / 10;
}

/* Inverse (voltage→temp) lookup.
 * adc_val : raw 15-bit ADC count (FSR = 256 mV, so 1 LSB ≈ 7.8125 μV).
 * tj      : cold-junction temperature in °C (from ADS1118 internal sensor).
 * Returns thermocouple temperature in °C, or 0 if out of range (>219°C). */
unsigned short volt_temp_lookup(unsigned short adc_val, short tj)
{
    /* Convert ADC count to μV: 256000 μV / 32768 LSB ≈ 7.8125 μV/LSB = *1000 >> 7 */
    unsigned long volt_uV = ((unsigned long)adc_val * 1000UL) >> 7;

    if(tj > 0)
        volt_uV += temp_to_uV((unsigned short)tj);

    if(volt_uV > tc_base_uV[TC_NUM_SEGS])
        return 0;

    /* Scan down to find the 10°C segment containing volt_uV */
    unsigned char seg = TC_NUM_SEGS - 1;
    while(seg > 0 && volt_uV < tc_base_uV[seg])
        --seg;

    unsigned short temp = (unsigned short)seg * 10;
    temp += (unsigned short)((volt_uV - tc_base_uV[seg]) * 10UL / tc_step_uV[seg]);
    return temp;
}

void thermo_sens_update(void)
{   
    unsigned short adc_val = 0;
    static unsigned char ch_num = 0;
    
    switch(ch_num)
    {
        case 0: //environment temp
            PORTCbits.RC1 = 0;
            adc_val = spi_wr_rd_byte(   0b1  <<7 |
                                        0b000<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val|= spi_wr_rd_byte(   0b100<<5 |
                                        0b0  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC1 = 1;
            txdata[TCJ_A_IDX] =  adc_val >> 7;

            PORTCbits.RC0 = 0;
            adc_val  = spi_wr_rd_byte(  0b1  <<7 |
                                        0b000<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val |= spi_wr_rd_byte(  0b100<<5 |
                                        0b0  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC0 = 1;
            txdata[TCJ_B_IDX] =  adc_val >> 7;

            ch_num = 1;
            break;
        case 1: //chn 1 temp
            PORTCbits.RC1 = 0;
            adc_val = spi_wr_rd_byte(   0b1  <<7 |
                                        0b011<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val|= spi_wr_rd_byte(   0b100<<5 |
                                        0b0  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC1 = 1;
            txdata[T2_MEAS_IDX] = volt_temp_lookup(adc_val,txdata[TCJ_A_IDX]) + t2_cali;
 
            PORTCbits.RC0 = 0;
            adc_val = spi_wr_rd_byte(   0b1  <<7 |
                                        0b011<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val|= spi_wr_rd_byte(   0b100<<5 |
                                        0b0  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC0 = 1;
//            txdata[T4_MEAS_IDX] = volt_temp_lookup(adc_val,txdata[TCJ_B_IDX]) +((short)(rxdata[3+ T4_CALI_IDX *2] <<8) | rxdata[3+ T4_CALI_IDX *2 +1]);

            ch_num = 2;
            break;
        case 2: //chn 2 temp
            PORTCbits.RC1 = 0;
            adc_val = spi_wr_rd_byte(   0b1  <<7 |
                                        0b011<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val|= spi_wr_rd_byte(   0b100<<5 |
                                        0b1  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC1 = 1;
            txdata[T1_MEAS_IDX] = volt_temp_lookup(adc_val,txdata[TCJ_A_IDX]) + t1_cali;

            PORTCbits.RC0 = 0;
            adc_val = spi_wr_rd_byte(   0b1  <<7 |
                                        0b011<<4 |
                                        0b111<<1 |
                                        0b1         ) <<8;
            adc_val|= spi_wr_rd_byte(   0b100<<5 |
                                        0b1  <<4 |
                                        0b1  <<3 |
                                        0b01 <<1 |
                                        0b0         );            
            PORTCbits.RC0 = 1;
            txdata[T3_MEAS_IDX] = volt_temp_lookup(adc_val,txdata[TCJ_B_IDX]) + t3_cali;
       
            ch_num = 0;
            break;
        default:
            break;
    }
}

void thermo_sens_init(void)
{
    PORTCbits.RC1 = 0;
    spi_wr_rd_byte( 0b1  <<7 |
                    0b000<<4 |
                    0b010<<1 |
                    0b1         );
    spi_wr_rd_byte( 0b100<<5 |
                    0b1  <<4 |
                    0b1  <<3 |
                    0b01 <<1 |
                    0b0         );
    PORTCbits.RC1 = 1;

    PORTCbits.RC0 = 0;
    spi_wr_rd_byte( 0b1  <<7 |
                    0b000<<4 |
                    0b010<<1 |
                    0b1         );
    spi_wr_rd_byte( 0b100<<5 |
                    0b1  <<4 |
                    0b1  <<3 |
                    0b01 <<1 |
                    0b0         );
    PORTCbits.RC0 = 1;
    __delay_ms(10);
}

unsigned short  modbusCRC16(const unsigned char *data, unsigned short data_len)
{
    static const unsigned short wCRCTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

    unsigned char nTemp;
    unsigned short wCRCWord = 0xFFFF;

   while (data_len--)
   {
      nTemp = *data++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;
}
