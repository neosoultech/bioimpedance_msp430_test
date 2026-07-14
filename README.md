# MSP430 BioZ + NFC System

## Overview

This project uses an MSP430FR2433 MCU to interface with two devices:

- MAX30002 for bioimpedance (BioZ) measurement  
- ST25DV-04K for NFC communication  

Development was done using Code Composer Studio (CCS) version 12.8.1.

Each test runs inside an infinite loop. Interrupts set flags that are handled within the active test.



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
8. (Future Rev 2 behavior will use an external BioZ mux to measure the target electrode set and baseline/reference electrode set sequentially.)
---
Current BioZ Configuration: 

- sample rate: 32 sps
- BioZ drive frequency: 8.192 kHz
- drive current: 32 uA
- gain: 10 V/V
- low-noise mode enabled
- analog HPF bypassed
- digital HPF bypassed/disabled
- digital LPF: 4 Hz
- internal 100 MOhm resistive bias enabled on BIP/BIN
- BIP/BIN connected by clearing OPENP and OPENN in CNFG_BMUX

Digital HPF is disabled because eval-kit testing showed that the HPF caused steady/static impedance values to decay toward a false low baseline. For static skin or lesion impedance measurements, the steady impedance value needs to be preserved.

---

### FIFO Format

Each FIFO read returns a 24-bit word.

- Bits [23:4] are the 20-bit signed BioZ sample
- Bit [3] is zero/padding 
- Bits [2:0] are the BTAG/status tag  

The data is an 20-bit signed value.

Processing:

- shift right by 4 to extract bits [23:4]
- mask the lower 20 bits
- check bit 19 as sign bit
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
Mailbox data is written as one sequential I2C write starting at address 0x2008.


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
   writes a byte to EEPROM scratch address 0x01F0  
   confirms basic I2C write is working  

2. TEST_ST25_READ  
   writes and reads back a value from EEPROM scratch address 0x01F0 
   verifies full read/write operation  

3. TEST_ST25_GPO  
   checks GPO interrupt from phone  
   verifies interrupt path and phone detection  

4. TEST_ST25_MAILBOX  
   enables FTM/mailbox
   writes one full mailbox message starting at 0x2008 
   verifies mailbox communication with phone/ST25 NFC Tap

---

### Test Notes

- Tests should be run in order  
- Earlier tests validate basic communication  
- Later tests depend on previous functionality  
- Phone is required for GPO and mailbox testing
- EEPROM tests are only for basic I2C bring-up
- Mailbox testing should use TEST_ST25_MAILBOX
- TEST_ST25_MAILBOX writes to the FTM mailbox, not normal EEPROM NDEF memory
- Use ST25 NFC Tap features for ST25DV / Fast Transfer Mode when checking mailbox data


---

Next steps:

- Test on hardware
- implement full system behavior
- If the code ever appears to freeze or hang during I2C/SPI testing, add timeout handling to the blocking wait loops (`wait_tx_ready()`, `wait_rx_ready()`, SPI TX/RX flag waits, and I2C STOP/START waits) so the MCU can fail gracefully instead of waiting forever.
- Add GPO interrupts for mailbox checking

---

## Notes

MAX30002 requires bit-level parsing.  
ST25DV operates on byte-level transfers.  
System behavior depends heavily on interrupts and timing.
