/* 
 * File:   spi.h
 * Author: liweiwang
 *
 * Created on December 13, 2021, 9:37 PM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif

void spi_init(void);
unsigned char  spi_wr_rd_byte(unsigned char );
void spi_send_byte(unsigned char );
unsigned char spi_read_byte(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

