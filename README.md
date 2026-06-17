# MSP430 BioZ + NFC System

## Overview

This project uses an MSP430 MCU to interface with two devices:

- MAX30002 for bioimpedance (BioZ) measurement  
- ST25DV for NFC communication  

Development was done using Code Composer Studio (CCS) version 12.8.1.

The system is interrupt-driven. External events set flags, and the main loop handles processing.

Main goals:
- bring up MAX30002 BioZ sensing  
- verify FIFO data path  
- prepare communication with a phone through ST25  

---

## System Structure

Two subsystems are connected to the MCU.

### MAX30002
Handles BioZ sensing and produces sampled data.

### ST25DV
Handles NFC communication and phone interaction.

The MSP430 coordinates both.

---

## Interrupt Behavior

Two GPIO interrupts are used:

- P2.1 (MAX INTB) sets `max_flag` when data is ready  
- P2.0 (ST25 GPO) sets `phone_flag` when a phone event occurs  

The ISR only sets flags.  
All processing happens in the main loop.

---

## MAX30002 Operation

The MAX30002 generates BioZ samples and stores them in a FIFO.

Basic sequence:

1. Configure device over SPI  
2. Enable BioZ engine  
3. FIFO begins filling  
4. INTB asserts when data is available  
5. MCU reads STATUS  
6. FIFO is drained  
7. Data is parsed into signed values  

---

### FIFO Format

Each FIFO read returns a 24-bit word.

- Bits [23:18] are tag and status  
- Bits [17:0] are BioZ data  

The data is an 18-bit signed value.

Processing:

- mask lower 18 bits  
- check sign bit  
- sign extend to 32 bits  

---

## ST25DV Operation

The ST25DV is accessed using I2C and is used for communication with a phone.

Features used:

- GPO for phone detection  
- mailbox for data transfer  

---

### ST25 Data Format

ST25 uses 16-bit addressing.

Write operation:
- address high byte  
- address low byte  
- one data byte  

Read operation:
- send address  
- read one byte  

Mailbox is a byte buffer.  
Data is written sequentially one byte at a time.

---

## MSP430 Role

The MSP430:

- controls MAX30002 over SPI  
- controls ST25DV over I2C  
- handles interrupts from both  
- runs at 1 MHz system clock  

All timing depends on this clock.

---

## Test Mode

The code uses a structured bring-up approach.  
Only one test should be enabled at a time.

---

### MAX30002 Tests

1. TEST_MAX_SPI  
   checks basic SPI communication  
   verifies that the device responds  

2. TEST_MAX_ID  
   reads INFO register  
   confirms correct device identity  

3. TEST_MAX_CONFIG  
   writes to registers and reads them back  
   verifies configuration path  

4. TEST_MAX_START  
   configures and starts BioZ engine  
   confirms device enters measurement mode  

5. TEST_MAX_READ  
   reads live FIFO data  
   verifies interrupts, STATUS, and parsing  

---

### ST25DV Tests

1. TEST_ST25_WRITE  
   writes a byte to EEPROM  
   confirms I2C write is working  

2. TEST_ST25_READ  
   writes and reads back a value  
   verifies full read/write operation  

3. TEST_ST25_GPO  
   checks GPO interrupt from phone  
   verifies interrupt path and phone detection  

4. TEST_ST25_MAILBOX  
   writes data to mailbox  
   verifies communication with phone  

---

### Test Notes

- Tests should be run in order  
- Earlier tests validate basic communication  
- Later tests depend on previous functionality  
- Phone is required for GPO and mailbox testing  

---

Next steps:

- Test on hardware
- implement full system behavior  

---

## Notes

MAX30002 requires bit-level parsing.  
ST25DV operates on byte-level transfers.  
System behavior depends heavily on interrupts and timing.
