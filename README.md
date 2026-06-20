# slht.X — PIC16F1516 Cooking System Firmware

MPLAB X / XC8 firmware for a food-processing controller built around a PIC16F1516 microcontroller.

## Hardware Overview

| Block | Detail |
|---|---|
| MCU | PIC16F1516, 16 MHz internal oscillator |
| HMI & servo comms | RS485 half-duplex, Modbus RTU, 115200 baud |
| Temperature sensing | 2 × ADS1118 16-bit delta-sigma ADC via SPI (1 MHz, Mode 0) |
| Thermocouples | 4 × Type-K (2 per ADS1118); T4 channel reserved/unused |
| Flow metering | Pulse-counting on RB2 (IOC interrupt) |

### GPIO Map

| Pin | Direction | Function |
|---|---|---|
| RA0 | Output | Mixer motor enable (MXMT) |
| RA1 | Output | Feeder motor enable (FDMT) |
| RA2 | Output | Heat control 1 |
| RA3 | Output | Heat control 2 |
| RA4 | Output | Inlet valve |
| RB2 | Input | Flow-meter pulse input (IOC rising edge) |
| RB7 | Output | Heartbeat / status LED |
| RC0 | Output | ADS1118-B chip-select (active low) |
| RC1 | Output | ADS1118-A chip-select (active low) |
| RC2 | Output | Outlet valve |
| RC3 | Output | SPI clock (SCK) |
| RC4 | Input | SPI data in (MISO) |
| RC5 | Output | SPI data out (MOSI) |
| RC6/RC7 | Output/Input | UART TX / RX (RS485 via external transceiver) |

## Software Architecture

```
main.c          — config bits, main loop, ISR, Modbus, temperature
spi.c / spi.h   — MSSP SPI driver
uart.c / uart.h — EUSART driver
```

The single `__interrupt()` ISR handles four sources:
- **IOCBF2** — flow-meter pulse counting
- **RCIF** — UART RX byte accumulation with frame detection
- **TMR0IF** — 1-second tick for flow-rate calculation and LED heartbeat
- **TMR2IF** — 1 ms tick used as inter-character timeout for Modbus frame end detection

## Modbus Register Map

### Registers read from HMI (FC03, starting at 0x0033)

| Index | Register | Description |
|---|---|---|
| 0 | T1_CALI | Thermocouple 1 calibration offset (°C) |
| 1 | T2_CALI | Thermocouple 2 calibration offset (°C) |
| 2 | T3_CALI | Thermocouple 3 calibration offset (°C) |
| 5 | VOL_CALIPUL | Volume per calibration pulse (mL) |
| 6 | TRGT_TEMP | Target temperature setpoint (°C) |
| 7 | COOKTIM | Cook timer |
| 8 | STACODE | Control command bits (see below) |
| 9 | VOLTOTAL | Total volume |
| 10 | FL_CALIPUL | Flow calibration pulse count |
| 11 | FLOWRATE_RD | Flow rate setpoint (L/min) |
| 12 | FDMT_FREQ_RD | Feeder motor frequency |
| 13 | MXMT_FREQ_RD | Mixer motor frequency |

**STACODE bits (received):**

| Bit | Name | Function |
|---|---|---|
| 0 | FLCALI | Start/stop flow calibration |
| 1 | SS | Start/stop |
| 2 | HEATCO1 | Manual heat control 1 |
| 3 | HEATCO2 | Manual heat control 2 |
| 4 | VALVIN | Inlet valve command |
| 5 | VALVOT | Outlet valve command |
| 6 | MXMTSS | Mixer motor start/stop |
| 7 | FDMTSS | Feeder motor start/stop |
| 8 | HTCTR | Auto temperature control mode |

### Registers written to HMI (FC16, starting at 0x000B)

| Index | Register | Description |
|---|---|---|
| 0 | FLOWRATE | Measured flow rate (L/min) |
| 1 | FDMT_FREQ | Feeder motor operating frequency |
| 2 | MXMT_FREQ | Mixer motor operating frequency |
| 3 | T1_MEAS | Thermocouple 1 temperature (°C) |
| 4 | T2_MEAS | Thermocouple 2 temperature (°C) |
| 5 | T3_MEAS | Thermocouple 3 temperature (°C) |
| 7 | ERRCODE | Error flags (bit 0 = HMI comms loss) |
| 8 | STACODE | Active output states |
| 9 | FL_CALI_PUL | Pulse count during flow calibration |
| 10 | TCJ_A | Cold-junction temp, ADS1118-A (°C) |
| 11 | TCJ_B | Cold-junction temp, ADS1118-B (°C) |

## Temperature Sensing

Each ADS1118 is configured for ±256 mV FSR (gain = 8), giving ~7.8125 μV/LSB — suitable for Type-K thermocouple EMF in the 0–220 °C range.

**Cold-junction compensation** uses each ADS1118's built-in temperature sensor (14-bit, 0.03125 °C/LSB, right-shift by 7 to obtain integer °C).

**Voltage-to-temperature conversion** uses a piecewise-linear lookup table with 22 segments of 10 °C each (0–219 °C). All arithmetic is integer-only (μV scale) — no floating-point operations.

```c
// ADC count → μV (0.003 % error vs exact 7.8125 μV/LSB)
unsigned long volt_uV = ((unsigned long)adc_val * 1000UL) >> 7;
```

## Thermocouple Channel Assignment

| Channel | ADS1118 | MUX | txdata index |
|---|---|---|---|
| T1 (CH2) | A (RC1) | AIN2–GND | T1_MEAS_IDX (3) |
| T2 (CH1) | A (RC1) | AIN3–GND | T2_MEAS_IDX (4) |
| T3 (CH2) | B (RC0) | AIN2–GND | T3_MEAS_IDX (5) |
| T4 (CH1) | B (RC0) | AIN3–GND | reserved / unused |

## SPI Configuration

The ADS1118 samples DIN on the **rising** SCLK edge and drives DOUT on the **falling** SCLK edge (SPI Mode 0 / Mode 3). On PIC16 MSSP this requires:

```c
SSPSTATbits.SMP = 0;   // sample at mid-point of data output time
SSPCON1bits.CKP = 0;   // idle clock = low
SSPSTATbits.CKE = 1;   // MOSI driven on falling (active-to-idle) edge
SSPCON1bits.SSPM = 1;  // fosc/16 = 1 MHz (ADS1118 max is 4 MHz)
```

## Modbus CRC Validation

All three function codes (FC03 read, FC06 write-single, FC10 write-multiple) have CRC-16 appended on transmit and validated on receive. A response that fails CRC is discarded (`rxdata_ready_flag` cleared), and `ERRCODE` bit 0 is set on the next HMI write cycle.

## Watchdog Timer

The WDT is enabled in config bits. `CLRWDT()` is called:
- At the top of the main loop
- Inside `modbus_wait_fr_end()` on every iteration, preventing a WDT reset during the up-to-500 ms Modbus wait

## Build

Requires MPLAB X IDE and XC8 compiler. Open `nbproject/project.xml` in MPLAB X and build normally. No external libraries are used.

## Known Design Note

The RS485 transceiver direction pin (DE/RE) is not driven by firmware. This is correct only if the transceiver provides automatic direction control (e.g., half-duplex auto-direction chip). If a standard RS485 transceiver is used, a GPIO must be asserted before `uart_send_byte()` and de-asserted after the last byte completes (`TXSTAbits.TRMT == 1`).
