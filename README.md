## MSP430 BioZ + NFC Power Apps Demo

### Overview

This project uses an MSP430FR2433 MCU to run a wearable bioimpedance demo that interfaces with Microsoft Power Apps through NFC.  
The MCU controls two main devices:
- MAX30002 for bioimpedance (BioZ) measurement
- ST25DV-04K for NFC communication, phone interaction, and NFC energy harvesting support  
Development was done using Code Composer Studio (CCS) version 12.8.1.  
The primary firmware path is RUN_PHONE_NFC_DEMO. In this mode, a phone tap wakes the NFC path, the MCU collects BioZ data, and the result is written to the ST25DV as an NFC-readable NDEF text record for the Power App.

### System Structure

Two subsystems are connected to the MCU.

#### MAX30002

Handles BioZ sensing and produces sampled impedance data.  
The MAX30002 is controlled over SPI.

#### ST25DV

Handles NFC phone interaction and stores the result that the phone reads.  
The ST25DV is controlled over I2C.
The phone app interacts with the ST25DV through NFC.

### Demo Operation

The main demo mode is RUN_PHONE_NFC_DEMO.  
This is the current Power Apps NFC workflow.

Basic sequence:
- Phone taps the ST25DV
- ST25DV asserts GPO
- P2.0 interrupt sets phone_flag
- Main loop enters RUN_PHONE_NFC_DEMO handling
- MCU waits briefly after the NFC trigger
- MAX30002 is configured for BioZ
- ADG884 mux selects target electrodes
- MCU collects a target BioZ average
- ADG884 mux selects baseline/reference electrodes
- MCU collects a baseline BioZ average
- Firmware builds an NDEF text result payload
- MCU writes the payload into ST25DV EEPROM tag memory
- Power Apps reads the updated NFC text record

The demo uses demo_result_pending to separate the scan/measure step from the result-read step.
- demo_result_pending = 0 means the next phone event triggers a new BioZ measurement
- demo_result_pending = 1 means the next phone event is treated as the phone reading the previous result

### Power Apps NFC Payload

The firmware writes a short NDEF text record that Power Apps can read and parse.  
The result payload format is:

```text
BIOZ;mode=result;b=...;d=...;o=...;t=...;r=...;to=...;ro=...;z=...
```

Fields:
- b is percent difference from baseline/reference
- d is raw target/reference difference
- o is approximate ohm difference
- t is target raw magnitude
- r is reference raw magnitude
- to is target approximate ohms
- ro is reference approximate ohms
- z shows whether the displayed difference is inside the deadband

The firmware also defines a scanning payload:

```text
BIOZ;mode=scanning
```

This is used as a simple status-style text payload for the NFC workflow.

### Interrupt Behavior

Two GPIO interrupts are used:
- P2.0 (ST25 GPO) sets phone_flag when a phone event occurs
- P2.1 (MAX INTB) sets max_flag when BioZ data is ready  
The ISR only sets flags.
All processing happens in the main loop.

### MAX30002 Operation

The MAX30002 generates BioZ samples and stores them in a FIFO.  
For the demo, the MCU collects averaged BioZ values from two electrode paths.

Basic BioZ sequence:
- Configure device over SPI
- Enable BioZ engine
- Select target or baseline path using the ADG884 mux
- Reset FIFO and sync measurement timing
- Wait for INTB data-ready events
- Read STATUS
- Drain FIFO
- Parse samples into signed values
- Average settled samples

For the RUN_PHONE_NFC_DEMO mode:
- P2.7 controls the ADG884 mux
- MUX_SEL = 1 selects target electrodes
- MUX_SEL = 0 selects baseline/reference electrodes
- target and baseline are measured sequentially, not simultaneously
- each measurement averages settled BioZ samples after discarding early settling samples

### Current BioZ Configuration:
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
- first 64 samples discarded after path setup
- two settled sample blocks are compared before returning an average

Note: Digital HPF is disabled because eval-kit testing showed that the HPF caused steady/static impedance values to decay toward a false low baseline. For static skin or lesion impedance measurements, the steady impedance value needs to be preserved.

### BioZ Averaging Algorithm

The demo does not use the first BioZ values immediately after switching electrode paths.  
This is handled inside max30002_collect_bioz_average().
The goal is to avoid using samples that may still be settling after the mux changes paths, the FIFO is reset, or the BioZ timing is restarted.

For each target or baseline measurement, the algorithm works like this:
- Wait for max_flag from the MAX30002 INTB interrupt
- Read STATUS to confirm that BioZ FIFO data is available
- Read FIFO samples while valid data is present
- Ignore FIFO empty tags
- Reset FIFO and restart collection if overflow occurs
- Parse only valid BioZ samples
- Discard the first 64 valid samples after the selected path is prepared
- Collect the next block of settled samples into sum1
- Collect a second block of settled samples into sum2
- Average each block separately
- Compare the two block averages
- If the two block averages are too different, return only the second block average
- If the two block averages agree, return the average of both blocks

In the current demo, the function is called with a sample goal of 16.  
This means each target or baseline reading uses:
- 64 discarded settling samples
- 16 samples for the first settled block
- 16 samples for the second settled block

The two-block comparison is used as a simple stability check.  
If the first settled block is still affected by contact settling or switching artifacts, it may be noticeably different from the second block.
The firmware compares avg1 and avg2 using BIOZ_BLOCK_MISMATCH_LIMIT.
If the difference is greater than 500 raw counts, the algorithm assumes the later block is more reliable and returns avg2.
If the difference is 500 raw counts or less, the algorithm returns the average of avg1 and avg2.

Basic logic:

```text
Discard first 64 valid samples
Collect 16 valid samples into block 1
Collect 16 valid samples into block 2
avg1 = block 1 average
avg2 = block 2 average
if abs(avg1 - avg2) > 500:
    return avg2
else:
    return (avg1 + avg2) / 2
```

The same averaging process is used once for the target electrodes and once for the baseline/reference electrodes.
Those two final averaged values are then used to build the NFC result payload for Power Apps.

### FIFO Format

Each FIFO read returns a 24-bit word.
- Bits [23:4] are the 20-bit signed BioZ sample
- Bit [3] is zero/padding
- Bits [2:0] are the BTAG/status tag  
The data is a 20-bit signed value.  
Processing:
- shift right by 4 to extract bits [23:4]
- mask the lower 20 bits
- check bit 19 as sign bit
- sign extend to 32 bits

### ST25DV Operation

The ST25DV is accessed using I2C and is used as the NFC interface for the phone.  
Features used:
- GPO for phone detection
- EEPROM user memory for NDEF text records
- I2C password/session access for system configuration
- dynamic and static energy harvesting configuration helpers

#### ST25 Data Format

ST25 uses 16-bit addressing.  
Write operation:
- address high byte
- address low byte
- one data byte or a sequential block of data  
Read operation:
- send address
- read one byte  
NDEF text records are written into normal ST25 EEPROM tag memory starting at address 0x0004.
The firmware wraps the NDEF record in TLV format before writing.
EEPROM writes are split into 4-byte chunks with retry and I2C bus recovery.

### MSP430 Role

The MSP430:
- handles phone tap interrupts from the ST25DV
- controls MAX30002 over SPI
- controls ST25DV over I2C
- controls ADG884 target/baseline BioZ mux using P2.7 on the custom patch PCB
- collects target and baseline BioZ averages
- builds the Power Apps NFC text payload
- writes the NDEF result to ST25DV EEPROM
- runs at 1 MHz system clock  
All timing depends on this clock.
I2C and SPI are both configured for 100 kHz.

### Test Mode

The code uses a structured bring-up approach.
Only one test should be enabled at a time.

#### Main Demo Mode
- RUN_PHONE_NFC_DEMO
uses a phone tap on ST25 GPO to trigger a target/baseline BioZ measurement
writes the result as an NDEF text record to ST25DV EEPROM
uses demo_result_pending so the next tap is treated as the phone result-read window

#### MAX30002 Tests
- TEST_MAX_SPI
checks basic SPI communication
verifies that the device responds
- TEST_MAX_ID
reads INFO register
confirms correct device identity
- TEST_MAX_CONFIG
writes to registers and reads them back
verifies configuration path
- TEST_MAX_START
configures and starts BioZ engine
confirms device enters measurement mode
- TEST_MAX_READ
reads live FIFO data
verifies interrupts, STATUS, and parsing
- TEST_MAX_2SPOT
uses the ADG884 mux to automatically measure target and baseline/reference electrode sets
- TEST_MAX_2SPOT_MANUAL
eval-kit/manual workflow; user places electrodes on one skin area, presses S1, moves electrodes to a second area, then presses S1 again
- TEST_BUILD_BIOZ_MSG_ONLY
builds the BioZ NFC result message without running the full phone demo path

#### ST25DV Tests
- TEST_ST25_WRITE
writes a byte to EEPROM scratch address 0x01F0
confirms basic I2C write is working
- TEST_ST25_READ
writes and reads back a value from EEPROM scratch address 0x01F0
verifies full read/write operation
- TEST_ST25_GPO
checks GPO interrupt from phone
verifies interrupt path and phone detection

#### Test Notes
- Tests should be run in order
- Earlier tests validate basic communication
- Later tests depend on previous functionality
- Phone is required for GPO and Power Apps demo testing
- EEPROM scratch tests are only for basic I2C bring-up
- RUN_PHONE_NFC_DEMO writes NDEF text records to normal ST25DV EEPROM tag memory
- The current demo does not use the FTM mailbox path
- USE_BIOZ_MUX enables the ADG884 mux for custom patch workflows
- USE_EXP430_S1 enables the EXP430 S1 button for the manual eval-board workflow
- USE_MSP_FCLK is disabled because the MAX30001EVSYS eval board already provides the 32 kHz clock

### LED Behavior

The LED is used as a simple status indicator.
- LED pulses once if transmission or measurement step occurs successfully
- For most test functions (excluding RUN_PHONE_NFC DEMO): LED blinks on and off if measurement is occurring
- LED stays solid if error
- LED is off if IDLE/SLEEP

### Notes

MAX30002 requires bit-level parsing.
ST25DV operates on byte-level transfers.
System behavior depends heavily on interrupts and timing.
I2C timeout and bus recovery helpers are included to avoid getting stuck after NACKs or stale bus states.
The current Power Apps demo uses EEPROM/NDEF writes, not the ST25DV mailbox.
