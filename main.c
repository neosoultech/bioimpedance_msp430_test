                                // Overview
// Test code for a wearable, phone-powered bio-impedance patch sensor powered by energy harvesting.
// Parts include ST25DV-04K (NFC chip), MSP430FR2433 (MCU), and MAX30002 (Bio-impedance Sensor)
// Code is WIP, all tests are modular and not meant to be run at same time.
// MAX30002 communicates via SPI with MCU, while ST25 utilizes I2C.

                                // MAX30002 Information
// SPI interface
// ACLK (clock speed) at ~32KHz REFO
// - Data flow: BioZ measurement --> sample placed into FIFO --> FIFO reaches threshold --> INTB goes low on P2.1 falling edge
// --> PORT2 ISR --> max_flag = 1 --> main loop detects max_flag --> MCU reads STATUS --> MCU drains FIFO --> MCU parses bioZ data
// Note: FCLK must be present or no data generated, FIFO must be drainer to prevent overflow, data interrupt-driven
//
// MAX30002 FIFO FORMAT:
// 24 bit word: bits [23:4] are 20-bit signed BioZ sample (2SC), bit [3]: zero padding, bits [2:0]: BTAG status tag
// BTAG: 000 valid, 001 over/under range, 010 valid EOF, 011 over/under EOF
// Note: Extract bits [23:4], mask 20 bits, and sign extend bit 19.


                                // ST25 Information
// I2C interface
// - Data flow: Phone tap --> ST26 asserts GPO on P2.0 falling edge --> PORT2 ISR --> phone_flag = 1 --> main loop detects phone_flag
// --> MCU reads/writes mailbox or EEPROM --> data transmitted to/from phone.
// Note: GPO requires pull-up resistor, mailbox must be enabled before use
//
// ST25 MAILBOX FORMAT:
// I2C mailbox message write must be one sequential write starting at 0x2008:
// [addr_MSB = 0x20][addr_LSB = 0x08][data0][data1]...[dataN]
// ST25DV automatically updates mailbox length/status after a successful message write.


                            // MSP430 Information
// CPU clock: 1MHz (SMCLK)
// Timing assumptions based on 1MHz: __delay_cycles(x) --> x cycles = x microseconds
// Peripheral timings: I2C clock and SPI clock = 100KHz (UCBOBRW and UCAOBRW)
//


                            //PREEXISTING WIRING (ST25)
//  P1.0: LED (status/debug indicator)
//  P1.2: SDA (serial data wire for I2C)
//  P1.3: SCL (clock wire for I2C)
//  P2.0: GPO (ST25 GPO input. Sets phone_flag)

                            //PREEXISTING WIRING (MAX)
//  P1.4: MOSI (master to slave data line)
//  P1.5: MISO (slave to master data line)
//  P1.6: SCLK (clock wire for SPI)
//  P1.7: CS (Chip select to initiate comms)
//  P2.1: INTB (FIFO/data-ready interrupt. Sets max_flag)
//  P2.2: FCLK (clock output ~32KHz ACLK to MAX30002)

                            // LED Notes:
// LED pulses once if transmission occurs successfully.
// LED blinks on and off if measurement is occurring.
// LED stays solid if error
// LED is off if IDLE/SLEEP.

#include <msp430.h>
#include <stdint.h>
#define USE_LPM              0 // 0 = Normal (debug), 1 = low power

                                // INTERRUPT HANDLER
// external interrupts from P2.0 (ST25 GPO): Indicates phone interaction, sets phone_flag
// external interrupts from P2.1 (MAX30002 INTB): Indicates data ready / FIFO event, sets max_flag
volatile int32_t debug_bioz = 0; // debugging
volatile uint32_t sample_count = 0; // debugging

volatile int phone_flag = 0;
volatile int max_flag = 0;
#pragma vector=PORT2_VECTOR
__interrupt void Port_2_ISR(void)
{
    if (P2IFG & BIT0) { // if GPO interrupt occurs on P2.0
        phone_flag = 1; // enable phone flag
        P2IFG &= ~BIT0; // clear interrupt flag
    }

    if (P2IFG & BIT1) { // if INTB interrupt occurs on P2.1
        max_flag = 1; // enable max flag
        P2IFG &= ~BIT1; // clear interrupt flag
    }
#if USE_LPM
    __bic_SR_register_on_exit(LPM0_bits); // exiting LPM upon interrupt
#endif
}
                                            //

                            // MAX30002 REGISTER MAP //
// MAX30002 config bit masks
#define MAX_REG_STATUS      0x01
#define MAX_REG_EN_INT      0x02
#define MAX_REG_EN_INT2     0x03
#define MAX_REG_MNGR_INT    0x04
#define MAX_REG_MNGR_DYN    0x05
#define MAX_REG_SW_RST      0x08
#define MAX_REG_SYNCH       0x09
#define MAX_REG_FIFO_RST    0x0A
#define MAX_REG_INFO        0x0F
#define MAX_REG_CNFG_GEN    0x10
#define MAX_REG_CNFG_BMUX   0x17
#define MAX_REG_CNFG_BIOZ   0x18
#define MAX_REG_FIFO_BURST  0x22
#define MAX_REG_FIFO        0x23

//STATUS
#define MAX_STATUS_BINT   (1UL << 19)  // BioZ FIFO interrupt
#define MAX_STATUS_BOVF   (1UL << 18)

// EN_INT bits
#define MAX_EN_INT_BINT     (1UL << 19) // enable BIOZ FIFO interrupt on external INTB pin
#define MAX_EN_INT_BOVF     (1UL << 18) // enable BIOZ FIFO overflow interrupt on INTB pin
#define MAX_INTB_DISABLED        0x000000 // D[1:0] = 00
#define MAX_INTB_CMOS            0x000001 // D[1:0] = 01
#define MAX_INTB_OPEN_DRAIN      0x000002 // D[1:0] = 10
#define MAX_INTB_OD_PULLUP       0x000003 // D[1:0] = 11 --> 125k pullup, default
#define MAX_CFG_EN_INT_START  (MAX_EN_INT_BINT | MAX_EN_INT_BOVF | MAX_INTB_OPEN_DRAIN) // EN_INT starting settings

    // CNFG_GEN
#define MAX_CNFG_GEN_EN_BIOZ     (1UL << 18) // enable BioZ channel
#define MAX_CNFG_GEN_EN_RBIAS    (2UL << 4)  // EN_RBIAS[1:0] = 10, BioZ resistive bias enabled
#define MAX_CNFG_GEN_RBIASV_100M (1UL << 2)  // RBIASV[1:0] = 01, 100M ohm bias
#define MAX_CNFG_GEN_RBIASP      (1UL << 1)  // enable bias on BIP
#define MAX_CNFG_GEN_RBIASN      (1UL << 0)  // enable bias on BIN
// CNFG_GEN starting settings for BIOZ, RBIAS, 100M, RBIASP, and RBIASN //
#define MAX_CFG_CNFG_GEN_START   (MAX_CNFG_GEN_EN_BIOZ      | \
                                   MAX_CNFG_GEN_EN_RBIAS    | \
                                   MAX_CNFG_GEN_RBIASV_100M | \
                                   MAX_CNFG_GEN_RBIASP      | \
                                   MAX_CNFG_GEN_RBIASN)
    // CNFG_BMUX
#define MAX_BMUX_OPENP_BIT        (1UL << 21) // bit is 1 when BIP isolated
#define MAX_BMUX_OPENN_BIT        (1UL << 20) // bit is 1 when BIN isolated

    // MNGR_INT
#define MAX_MNGR_INT_CLR_SAMP    (1UL << 2) // self clear SAMP interrupt after 1/4 data rate cycle
#define MAX_MNGR_INT_BFIT_1_SAMP (0UL << 16) // interrupt at 1 unread FIFO sample
#define MAX_CFG_MNGR_INT_START   (MAX_MNGR_INT_BFIT_1_SAMP | MAX_MNGR_INT_CLR_SAMP) // MNGR start settings

    // MAX30002 CNFG_BIOZ field values for near-10kHz BioZ start
#define MAX_BIOZ_RATE_32SPS      (1UL << 23)
#define MAX_BIOZ_AHPF_BYPASS   (6UL << 20)
#define MAX_BIOZ_USE_INTERNAL_BIASGEN (0UL << 19)
#define MAX_BIOZ_LOW_NOISE       (1UL << 18)
#define MAX_BIOZ_GAIN_10         (0UL << 16)
#define MAX_BIOZ_DHPF_0P05HZ     (1UL << 14)
#define MAX_BIOZ_DLPF_4HZ        (1UL << 12)
#define MAX_BIOZ_FCGEN_8192HZ    (4UL << 8)
#define MAX_BIOZ_CGMON_OFF       (0UL << 7)
#define MAX_BIOZ_CGMAG_TEST      (3UL << 4)
#define MAX_BIOZ_PHOFF_0         (0UL << 0)
#define MAX_CFG_CNFG_BIOZ_START  (\
    MAX_BIOZ_RATE_32SPS             | /* 32 samples per second to save power */ \
    MAX_BIOZ_AHPF_BYPASS             |/* AHPF[2:0] = 110, bypass analog HPF. Measurements are more accurate on low freq with HPF disabled. */ \
    MAX_BIOZ_USE_INTERNAL_BIASGEN   | /* Use MAX30002 internal bias generator instead of external resistor */ \
    MAX_BIOZ_LOW_NOISE              | /* Low noise mode (cleaner data, but more power) */ \
    MAX_BIOZ_GAIN_10                | /* 10 V/V gain; testing lowest 1st */ \
    MAX_BIOZ_DHPF_0P05HZ            | /* Digital HPF to preserve slower trends */ \
    MAX_BIOZ_DLPF_4HZ               | /* Digital LPF to reduce noise and output bandwidth */ \
    MAX_BIOZ_FCGEN_8192HZ           | /* 8.192KHz generated current frequency, closest to 10KHz (wanted previously) */ \
    MAX_BIOZ_CGMON_OFF              | /* Current generator compliance monitor OFF. Diagnostic for later */ \
    MAX_BIOZ_CGMAG_TEST             | /* 32uA injected current magnitude. Using middle low value. */ \
    MAX_BIOZ_PHOFF_0)                 /* No phase offset */

                                    //

                             // ST25DV REGISTER MAP //

// ST25DV I2C addresses
// MSP430 uses 7-bit slave addresses.
// ST25DV device select code: 1010 E2 1 1 R/W
// E2 = 0 -> user memory, dynamic registers, mailbox
// E2 = 1 -> system configuration area
#define ST25DV_ADDR_USER              0x53
#define ST25DV_ADDR_SYSTEM            0x57

// ST25DV user EEPROM test address
#define ST25DV_TEST_EEPROM_ADDR       0x01F0

// ST25DV static system configuration register, E2 = 1
#define ST25DV_REG_MB_MODE            0x000D

// ST25DV I2C password command address, E2 = 1
#define ST25DV_I2C_PWD_ADDR           0x0900
#define ST25DV_PRESENT_PWD_CODE       0x09

// ST25DV dynamic registers, E2 = 0
#define ST25DV_REG_I2C_SSO_DYN        0x2004
#define ST25DV_REG_MB_CTRL_DYN        0x2006
#define ST25DV_REG_MB_LEN_DYN         0x2007

// ST25DV mailbox memory, E2 = 0
#define ST25DV_MAILBOX_BASE           0x2008
#define ST25DV_MAILBOX_MAX_LEN        256

// ST25DV bit masks
#define ST25DV_MB_MODE_ENABLE         0x01

// MB_CTRL_Dyn bits
#define ST25DV_MB_CTRL_MB_EN          0x01
#define ST25DV_MB_CTRL_HOST_PUT_MSG   0x02
#define ST25DV_MB_CTRL_RF_PUT_MSG     0x04

// I2C_SSO_Dyn bits
#define ST25DV_I2C_SSO_OPEN           0x01


                                        //

                      //TEST MODE MACROS. ONLY 1 AT A TIME CAN BE ENABLED. //
#define TEST_MODE            1
// if test mode enabled...
#define TEST_ST25_WRITE      1
#define TEST_ST25_READ       0
#define TEST_ST25_GPO        0
#define TEST_ST25_MAILBOX    0

#define TEST_MAX_SPI         0
#define TEST_MAX_ID          0
#define TEST_MAX_CONFIG      0
#define TEST_MAX_START       0
#define TEST_MAX_READ        0

                                        //

                        // OTHER MACROS (DON'T CHANGE)
#define LED_ON()    (P1OUT |= BIT0) // LED on indicates error.
#define LED_OFF()   (P1OUT &= ~BIT0) // LED off indicates IDLE/SLEEP
#define LED_TOGGLE() (P1OUT ^= BIT0) // macro to toggle LED on and off. --> Indicates that sensing/transmission occuring
#define MAX_CS_LOW() (P1OUT &= ~BIT7) //select MAX30002, CS active low
#define MAX_CS_HIGH()(P1OUT |= BIT7) // deselect MAX30002
                                        //

                        // FUNCTION DECLARATIONS
int st25dv_write(uint8_t slave, uint16_t mem_addr, uint8_t data);
void i2c_init();
void i2c_pins_init();
void error_handler(void);
void led_blink_measuring(void);
void led_pulse_transmission(void);
void led_idle(void);
uint8_t st25dv_read(uint8_t slave, uint16_t addr);
void gpo_init();
int mailbox_write(uint8_t *message, unsigned int length);
int check_nack();
void wait_tx_ready();
int i2c_send_and_check(uint8_t byte);
int enable_mailbox(void);
int st25dv_present_i2c_password(void);
int st25dv_i2c_session_is_open(void);
int st25dv_mailbox_is_free(void);
int st25dv_write_sequence(uint8_t slave, uint16_t mem_addr, uint8_t *data, unsigned int length);
void max_int_init(void);
void wait_rx_ready(void);
void spi_init(void);
void spi_pins_init(void);
uint8_t spi_transfer(uint8_t data);
uint32_t max30002_read_reg(uint8_t reg);
void max30002_write_reg(uint8_t reg, uint32_t data);
void max30002_reset(void);
void max30002_fifo_reset(void);
uint32_t max30002_read_info(void);
uint32_t max30002_read_status(void);
uint32_t max30002_read_fifo(void);
void max30002_synch(void);
void fclk_init(void);
int32_t bioz_parse(uint32_t raw);
                                        //

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	 PM5CTL0 &= ~LOCKLPM5;       // Unlock GPIO ports to activate I/O pins

	// LED
	P1DIR |= BIT0; // setting pin 1.0 as LED output (1)
	P1OUT &= ~BIT0; // setting pin 1.0 to a 0 as value

	// init
	i2c_pins_init();
	i2c_init();
	spi_pins_init();
	spi_init();
	fclk_init();
	gpo_init();
	max_int_init();
#if TEST_ST25_MAILBOX
	if (!enable_mailbox()) { // only enable FTM/mailbox when running mailbox test. No FTM during EEPROM tests
	    error_handler();
	}
#endif
	__enable_interrupt(); // enabling global interrupts


	// TEST FUNCTIONS
	while(1) {
#if TEST_MODE // if test mode enabled


    #if TEST_ST25_WRITE // TEST 1: MCU WRITE --> ST25
	    if (st25dv_write(ST25DV_ADDR_USER, ST25DV_TEST_EEPROM_ADDR, 0xAA)) { // sending test byte 0xAA to address 0x01F0 on ST25DV (slave addr 0x53). Checking to see if successful (ST25DV will respond)
	                led_pulse_transmission(); // pulse LED if transmission success
	                }
	            else {
	                error_handler(); // if not successful (ERR), LED is solid forever
	            }

    #elif TEST_ST25_READ // TEST 2: MCU READ <-- ST25
	    uint8_t val; // declare value variable
	    st25dv_write(ST25DV_ADDR_USER, ST25DV_TEST_EEPROM_ADDR, 0xAA); // write 0xAA to 0x01F0 on st25dv
	    __delay_cycles(1000000); // 1s to wait for it to store in mem
	    val = st25dv_read(ST25DV_ADDR_USER, ST25DV_TEST_EEPROM_ADDR); // reading it back from 0x01F0
	    if (val == 0xAA){
	        led_pulse_transmission(); // pulse LED for success
	    }
	    else {
	        error_handler(); // solid for failure
	    }
	    __delay_cycles(1000000);

    #elif TEST_ST25_GPO // TEST 3: SEE IF ST25 SENDS PHONE DETECT SIGNAL ON GPO
	    if (phone_flag == 1) { // if detect phone
	        phone_flag = 0; // clear interrupt event after handling
	        led_pulse_transmission(); // pulse LED
	    }
	    else {
	        led_idle(); // LED stays off
	    }

    #elif TEST_ST25_MAILBOX // TEST 4: Phone tap, MCU writes to mailbox, LED blinks to confirm
	    if (phone_flag) {
	            phone_flag = 0; // clear interrupt
	            uint8_t msg[] = {
	                0xD1, // NDEF header byte to tell phone it's NFC text
	                0x01, // length of type field
	                0x15, // payload length (1 status byte + 2 "en" + 18 message length)

	                'T', // type field T for text

	                0x02, 'e', 'n', // status byte, then language code (english, 2 chars)

	                'H','e','l','l','o',' ', // message
	                'f','r','o','m',' ',
	                'M','S','P','4','3','0','!'
	            };

	            if (mailbox_write(msg, sizeof(msg))) { // write to mailbox
	               led_pulse_transmission(); // if success, pulse
	            }
	                else {
	                   error_handler();
	                }
	            __delay_cycles(200000);
	                }
	                else { // if no phone flag
	                    led_idle();
	                }

    #elif TEST_MAX_SPI // TEST 5: Basic MAX30002 SPI connection test. Checks to see if reads anything from INFO
	    uint32_t info;
	    max30002_reset(); // RESET to start from known state
	    (void)max30002_read_status(); // INFO should not be first command after power-up/reset
	    __delay_cycles(10000);

	    info = max30002_read_info();
	    if ((info != 0x000000) && (info != 0xFFFFFF)) { // check to see if anything there
	        led_pulse_transmission(); // Pulse LED; SPI returned something plausible
	    }
	    else {
	        error_handler(); // likely SPI wiring or mode issue. SOLID LED
	    }
	    __delay_cycles(1000000);

	 #elif TEST_MAX_ID // TEST 6: More in depth MAX30002 SPI communication test. Reads/compares info register.
	    uint32_t info;
	    max30002_reset(); // RESET to start from known state
	    (void)max30002_read_status(); // read status first as dummy transaction. Cannot read info command right away per data sheet.
	    __delay_cycles(10000); // 10ms delay

	    info = max30002_read_info(); // read info register just as a test for SPI communication
	    if ((info & 0xF03000) == 0x502000) { // D[23:20] should be 0101 (5), D[13:12] should be 10 (fixed INFO/part-ID related bits)
	        led_pulse_transmission(); // pulse LED if what read looks valid
	    }
	    else {
	        error_handler(); // solid LED
	    }
	    __delay_cycles(1000000); // 1 second between test

    #elif TEST_MAX_CONFIG // TEST 7: Write to and read from MAX30002 config registers
	    uint32_t en_int_read;
	    max30002_reset(); // RESET to start from known state
	    (void)max30002_read_status(); // read status first as dummy transaction
	    __delay_cycles(10000); // 10ms delay

	    max30002_write_reg(MAX_REG_EN_INT, 0x000002); // writing to low bits of EN_INT register. Open drain NMOS without internal pull up
	    __delay_cycles(10000); // 10 ms delay
	    en_int_read = max30002_read_reg(MAX_REG_EN_INT); // read what we just wrote
	    if ((en_int_read & 0x000003) == MAX_INTB_OPEN_DRAIN) { // lower bits should be '10 (2 decimal)' for open drain
	        led_pulse_transmission(); // config write/read worked
	    }
	    else {
	        error_handler(); // solid LED if error
	    }
	    __delay_cycles(1000000); // 1 second between tests

    #elif TEST_MAX_START // TEST 8: Configure and start MAX30002 BioZ operation
	    uint32_t bmux;
	    uint32_t en_int_read;
	    uint32_t gen_read;
	    uint32_t mngr_int_read;
	    uint32_t bmux_read;
	    uint32_t bioz_read;

	    max30002_reset(); // reset to known state
	    (void)max30002_read_status(); // dummy transaction. Can't do INFO as first command after reset.
	    __delay_cycles(10000); // 10 ms delay

	    max30002_write_reg(MAX_REG_EN_INT, MAX_CFG_EN_INT_START); // Configure INTB: allow BioZ FIFO interrupt & overflow to drive INTB, use open drain INTB output
	    max30002_write_reg(MAX_REG_MNGR_INT, MAX_CFG_MNGR_INT_START); // Configure interrupt manager so interrupt if >=1 BioZ sample in FIFO, and interrupt flag clears auto.
	    max30002_write_reg(MAX_REG_CNFG_BIOZ, MAX_CFG_CNFG_BIOZ_START); // Configure BioZ channel: 32sps, 8.192Khz, low noise, 10V/V gain, midrange current magnitude

	    bmux = max30002_read_reg(MAX_REG_CNFG_BMUX); // read BMUX config
	    bmux &= ~(MAX_BMUX_OPENP_BIT | MAX_BMUX_OPENN_BIT); // clear OPENP AND OPENN so BIP and BIN connected to BioZ channel
	    max30002_write_reg(MAX_REG_CNFG_BMUX, bmux); // Connected BIP & BIN to BioZ channel

	    max30002_write_reg(MAX_REG_CNFG_GEN, MAX_CFG_CNFG_GEN_START); // Enable BioZ & internal lead biasing. Requires EN_BIOZ to be asserted same time (done)
	    __delay_cycles(100000); // 0.1s startup

	    // Reset FIFO
	    max30002_fifo_reset();
	    max30002_synch();
	    __delay_cycles(100000); // allow BioZ to begin running

	    en_int_read   = max30002_read_reg(MAX_REG_EN_INT); // reading registers
	    mngr_int_read = max30002_read_reg(MAX_REG_MNGR_INT);
	    gen_read      = max30002_read_reg(MAX_REG_CNFG_GEN);
	    bmux_read     = max30002_read_reg(MAX_REG_CNFG_BMUX);
	    bioz_read     = max30002_read_reg(MAX_REG_CNFG_BIOZ);

	    if (((en_int_read & MAX_CFG_EN_INT_START) == MAX_CFG_EN_INT_START) && // check that the registers have been altered
	                ((mngr_int_read & MAX_CFG_MNGR_INT_START) == MAX_CFG_MNGR_INT_START) &&
	                ((gen_read & MAX_CFG_CNFG_GEN_START) == MAX_CFG_CNFG_GEN_START) &&
	                ((bmux_read & (MAX_BMUX_OPENP_BIT | MAX_BMUX_OPENN_BIT)) == 0) &&
	                (bioz_read == MAX_CFG_CNFG_BIOZ_START)) {
	        int i;
	        for (i = 0; i < 6; i++) {
	                        led_blink_measuring(); // Blink while measuring if success
	                    }

	                    led_idle(); // stop blinking
	                }
	                else {
	                    error_handler(); // solid LED if config/start failed
	    }
	    __delay_cycles(1000000); // 1 second between tests

    #elif TEST_MAX_READ

	    uint32_t status;
	    uint32_t raw;
	    int32_t bioz;

	    max30002_reset(); // reset before use, read status as first command
	    (void)max30002_read_status();
	    __delay_cycles(10000);

	    max30002_write_reg(MAX_REG_EN_INT, MAX_CFG_EN_INT_START); // setup for MAX
	    max30002_write_reg(MAX_REG_MNGR_INT, MAX_CFG_MNGR_INT_START);

	    uint32_t bmux = max30002_read_reg(MAX_REG_CNFG_BMUX);
	        bmux &= ~(MAX_BMUX_OPENP_BIT | MAX_BMUX_OPENN_BIT);
	        max30002_write_reg(MAX_REG_CNFG_BMUX, bmux);

	        max30002_write_reg(MAX_REG_CNFG_BIOZ, MAX_CFG_CNFG_BIOZ_START);
	            max30002_write_reg(MAX_REG_CNFG_GEN, MAX_CFG_CNFG_GEN_START);

	            __delay_cycles(100000); // delay

	                max30002_fifo_reset(); // reset FIFO to prepare for measurement
	                max30002_synch();

	                __delay_cycles(100000);
// READ LOOP:
	                while(1) {
	                    if (max_flag) { // set by ISR on INTB
	                        max_flag = 0; // reset flag
	                        status = max30002_read_status(); // read its status

	                        if (status & MAX_STATUS_BOVF) { // checking first for global overflow
	                            max30002_fifo_reset();
	                            continue; // go back to waiting for next MAX interrupt
	                        }
	                        while (status & MAX_STATUS_BINT) { // if FIFO here
	                            raw = max30002_read_fifo(); // read it

	                            uint8_t tag = raw & 0x07;  // extract BTAG bits (bottom 3)
	                            if (tag == 0x07) { // 111: FIFO overflow
	                            max30002_fifo_reset();
	                            break;
	                            }
	                            if (tag == 0x06) { // 110: Empty
	                                break; // skip to get more data
	                            }
	                            if (tag == 0x00 || tag == 0x02) { // Cases of guaranteed valid data
	                                bioz = bioz_parse(raw);
	                                debug_bioz = bioz;
	                                sample_count++; // debug; count samples recieved
	                                LED_TOGGLE(); // if successful read, blink if FIFO producing data
	                            }
	                            else if (tag == 0x01 || tag == 0x03){ // Over-under. Cases of questionably valid data
	                                bioz = bioz_parse(raw);
	                                debug_bioz = bioz;
	                            }
	                            if (tag == 0x02 || tag == 0x03) { // if EOF, stop draining FIFO
	                                break;
	                            }
	                            status = max30002_read_status(); // check if there are more samples to read
	                        }
	                      }
	                    }

    #endif // ends test chain
    #endif // end program

#if USE_LPM
	__bis_SR_register(LPM0_bits | GIE); // enter low power mode, enable interrupts
#else
	    __delay_cycles(200000); // delay by 0.2s
#endif
	}
}

// END TEST FUNCTIONS

                            // HELPER FUNCTIONS

// I2C / ST25DV

void i2c_pins_init() { //initializing communication pins on MSP430
    P1SEL0 |= BIT2 | BIT3;  // to select I2C, SEL0 needs have bits 2 and 3 as 1.
    P1SEL1 &= ~(BIT2 | BIT3); // to select I2C, SEL1 needs to be 0 for bits 2 and 3.
}

void i2c_init() { // initializing I2C sequence
    UCB0CTLW0 = UCSWRST; // completely reset I2C registers. Going to set bits within CTL (control register)

    UCB0CTLW0 |= UCMODE_3 | // enabling I2C mode (11)
                 UCMST    | // enabling master mode (generating clock, initializing comms)
                 UCSSEL__SMCLK; // enable clock

    UCB0BRW = 10; // clock speed set to 100 KHz
    UCB0CTLW0 &= ~UCSWRST; // enabling system
}

void gpo_init() {
    P2DIR &= ~BIT0; // P2.0 is GPO input from ST25DV
    P2REN |= BIT0; // turn on internal resistor since GPO pin open drain
    P2OUT |= BIT0; // enable pull-up (from ST25DV datasheet, GPO is active low)
    P2IES |= BIT0; // interrupt edge set to falling edge (high to low)
    P2IFG &= ~BIT0; // clearing interrupt flag
    P2IE  |= BIT0; // enable interrupts

    if (!(P2IN & BIT0)) {
        phone_flag = 1; // if phone/GPO alr active before interrupts enabled
    }
}

int check_nack(void) { // helper function to check NACK
    if (UCB0IFG & UCNACKIFG) { //bitwise check in interrupt flag reg to see if NACK bit set
        UCB0IFG &= ~UCNACKIFG; // if NACK flag set, clear it (prevents unwanted behavior in future)
        UCB0CTLW0 |= UCTXSTP; // send STOP command to system (transmission did not work)
        while (UCB0CTLW0 & UCTXSTP); // wait for STOP command to go through before ending loop
        return 1; // NACK occurred
    }
    return 0; // no NACK
}

void wait_tx_ready(void) { // helper function to wait until TX buf ready
    while (!(UCB0IFG & UCTXIFG0));
}

void wait_rx_ready(void) { // helper function to wait until RX buf ready
    while (!(UCB0IFG & UCRXIFG0));
}

int i2c_send_and_check(uint8_t byte) { // sends a byte and checks to see if recieved
        UCB0TXBUF = byte; // get byte
        wait_tx_ready(); // waiting here until byte done sending
        return check_nack(); // 1 = error, 0 = OK
}
int st25dv_write_sequence(uint8_t slave, uint16_t mem_addr, uint8_t * data, unsigned int length) { // helper function to write
    unsigned int i;
    if (data == 0 || length == 0) {
        return 0; // error if nothing
    }
    UCB0I2CSA = slave; // setting st25dv as slave
    UCB0CTLW0 |= UCTR | UCTXSTT; // enabling control bits to transmit
    wait_tx_ready(); // wait for tx

    if (check_nack()) return 0; // if a NACK, then error

        if (i2c_send_and_check((mem_addr >> 8) & 0xFF)) return 0; // send two address bytes
        if (i2c_send_and_check(mem_addr & 0xFF)) return 0;        //

        for (i = 0; i < length; i++) { // send all data bytes during same I2C transaction
            if (i2c_send_and_check(data[i])) return 0;
        }

        UCB0CTLW0 |= UCTXSTP; // stop transmitting
        while (UCB0CTLW0 & UCTXSTP);

        return 1; // success
}
int st25dv_present_i2c_password(void) // helper function to present default I2C PW to ST25
{
    uint8_t seq[17];
    unsigned int i;

    // Factory default I2C password is 0000000000000000h.
    // Present password sequence:
    // 8 password bytes, validation code 0x09, same 8 password bytes again.
    for (i = 0; i < 8; i++) {
        seq[i] = 0x00; // writing 0000...h
    }

    seq[8] = ST25DV_PRESENT_PWD_CODE; // 0x09 validation code

    for (i = 0; i < 8; i++) {
        seq[9 + i] = 0x00; // repeating PW bytes again
    }

    return st25dv_write_sequence(ST25DV_ADDR_SYSTEM, // write this
                                 ST25DV_I2C_PWD_ADDR,
                                 seq,
                                 sizeof(seq));
}

int st25dv_i2c_session_is_open(void) // helper function to ensure password worked
{
    uint8_t sso;

    sso = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_I2C_SSO_DYN); // read to ensure I2C security session open

    if (sso & ST25DV_I2C_SSO_OPEN) {
        return 1;
    }

    return 0;
}


int st25dv_write(uint8_t slave, uint16_t mem_addr, uint8_t data) { // function to send a test byte. mem_addr is 16bit address, uint8_t slave is the 8 bit slave address, and uint8_t data is the test byte of data. Returns 1 if success, 0 if not.
// SETUP
    UCB0I2CSA = slave; // Target I2C device with slave address 'slave'
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmission mode for master, then transmit START condition in master mode
    wait_tx_ready(); // waiting here until transit buffer flag set
    if (check_nack() == 1) return 0; // if NACK, stop and report failure
// DATA TRANSMISSION
    if (i2c_send_and_check((mem_addr >> 8) & 0xFF)) return 0; // extract/send high byte of 16 bit addr
    if (i2c_send_and_check(mem_addr & 0xFF)) return 0; // extract/send lower byte of 16 bit addr
    if (i2c_send_and_check(data)) return 0; // send a byte of data.s
// STOPPING DATA TRANSMISSION
    UCB0CTLW0 |= UCTXSTP; // stop transmitting
    while (UCB0CTLW0 & UCTXSTP); // wait for STOP command to go through before ending loop
        return 1; // returns a 1 if success
}

uint8_t st25dv_read(uint8_t slave, uint16_t mem_addr) { // MCU gets address and asks ST25DV to send data
    uint8_t data; // declaring data that will be passed from ST25DV
    // writing address
        UCB0I2CSA = slave; // Target I2C device with slave address 'slave'
        UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmission mode for master, then transmit START condition in master mode
        wait_tx_ready(); // waiting here until transit buffer flag set
        if (check_nack() == 1) return 0xFF; // if NACK, stop and report failure
        if (i2c_send_and_check((mem_addr >> 8) & 0xFF)) return 0xFF; // extract/send high byte of 16 bit addr
        if (i2c_send_and_check(mem_addr & 0xFF)) return 0xFF; // extract/send lower byte of 16 bit addr
        // switch to read mode
        UCB0CTLW0 &= ~UCTR; // transmission mode disabled, read enabled
        UCB0CTLW0 |= UCTXSTT; // START
        while (UCB0CTLW0 & UCTXSTT); // wait for START to finish
        // get 1 byte
        UCB0CTLW0 |= UCTXSTP; // only get 1 byte then stop
        wait_rx_ready();
        data = UCB0RXBUF; // get data from buffer
        while (UCB0CTLW0 & UCTXSTP); // wait for STOP command to go through before ending loop

        return data;
    }

int enable_mailbox (void) { // helper function to enable FTM mailbox
     uint8_t mb_mode;

     if (!st25dv_present_i2c_password()) { // sending default, all zero password
             return 0;
         }
     if (!st25dv_i2c_session_is_open()) { // confirm I2C security session opened
             return 0;
         }
     if (!st25dv_write(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN, 0x00)) { // Disable dynamic FTM before changing static system config. This is harmless if FTM is already disabled.
         return 0;
     }
     mb_mode = st25dv_read(ST25DV_ADDR_SYSTEM, ST25DV_REG_MB_MODE); // Read static MB_MODE register
     if ((mb_mode & ST25DV_MB_MODE_ENABLE) == 0) { // if FTM auth not enabled, enable it.
             mb_mode |= ST25DV_MB_MODE_ENABLE; // set MB_MODE bit to allow Fast Transfer Mode/mailbox use
             if (!st25dv_write(ST25DV_ADDR_SYSTEM, ST25DV_REG_MB_MODE, mb_mode)) { // writing FTM enable
                         return 0;
                     }
             __delay_cycles(10000);
                }
                if (!st25dv_write(ST25DV_ADDR_USER,                 // Enable FTM dynamically for runtime mailbox use.
                                  ST25DV_REG_MB_CTRL_DYN,
                                  ST25DV_MB_CTRL_MB_EN)) {
                    return 0;
                }
                    return 1;
}

int st25dv_mailbox_is_free(void) {
    uint8_t ctrl;
    ctrl = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN to see if MB free
    if ((ctrl & ST25DV_MB_CTRL_MB_EN) == 0) { // MB must be enabled
        return 0;
    }
    if (ctrl & ST25DV_MB_CTRL_HOST_PUT_MSG) { // if host put something in mailbox
        return 0;
    }
    if (ctrl & ST25DV_MB_CTRL_RF_PUT_MSG) { // if slave put something in mailbox
        return 0;
    }
    return 1; // mailbox is free
}

int mailbox_write(uint8_t *message, unsigned int length) { // write message bytes into ST25DV mailbox buffer
    if (message == 0) { // null pointer check, not checking message contents
        return 0;
    }
    if (length == 0 || length > ST25DV_MAILBOX_MAX_LEN) { // if overflow or empty
        return 0;
    }
    if (!st25dv_mailbox_is_free()) { // check to see if mailbox free
        return 0;
    }
    return st25dv_write_sequence(ST25DV_ADDR_USER, ST25DV_MAILBOX_BASE, message, length); // write to mailbox if free. Sending whole message as one sequential write.
    }

// SPI/MAX30002
void spi_pins_init(void) {
    P1SEL0 |= BIT4 | BIT5 | BIT6; // to select SPI, SEL0 needs to have bits 4,5,6 as 1.
    P1SEL1 &= ~(BIT4 | BIT5 | BIT6); // to select SPI, SEL1 needs to have bits
    P1DIR |= BIT7; // bit 7 is output (chip select)
    MAX_CS_HIGH(); // deselect MAX30002 to prevent accidental SPI commands during startup
}

void spi_init(void) { // clock idle low
    UCA0CTLW0 = UCSWRST; // completely reset SPI
    UCA0CTLW0 |= UCMODE_0 | // enabling SPI mode
                 UCSYNC   | // synchronous mode
                 UCMST    | // master mode
                 UCMSB    | // MSB 1st
                 UCCKPH   | // data captured on 1st clock edge, changed at following edge. Data sampled on rising edge.
                 UCSSEL__SMCLK; // enable clock

    UCA0BRW = 10; // clock speed set to 100kHz
    UCA0CTLW0 &= ~UCSWRST;   // begin SPI
}

void max_int_init() {
    P2DIR &= ~BIT1; // set P2.1 as interrupt input from MAX30002
    P2REN |= BIT1; // turn on internal resistor
    P2OUT |= BIT1; // enable pull-up

    P2IES |= BIT1; // detect interrupt on falling edge (1 to 0)
    P2IFG &= ~BIT1; // clear stale interrupt flag
    P2IE |= BIT1; // enable P2.1 interrupt

    if (!(P2IN & BIT1)) { // if P2.1 reads as 0, INTB is alr active, so set max_flag manually
            max_flag = 1;
        }

}

void fclk_init(void) { // helper function to set up ACLK between MCU and MAX30002
    CSCTL4 |= SELA__REFOCLK; // set ACLK source to REFO (~32 KHz)

    P2DIR |= BIT2; // 2.2 becomes output
    P2SEL0 &= ~BIT2; // setting P2SEL0.2 value to 0
    P2SEL1 |= BIT2; // P2SEL1.2 value set to 1. Both of these lines make (Sel1,Sel0) as (1,0) to select ACLK output
}

uint8_t spi_transfer(uint8_t data) { // helper function to transfer over SPI (both ways at same time)
    while (!(UCA0IFG & UCTXIFG)); // wait until TX buffer ready
    UCA0TXBUF = data; // put data into TX buffer
    while (!(UCA0IFG & UCRXIFG)); // while no RX interrupts
    return UCA0RXBUF; // get information coming into RX buffer
}

uint32_t max30002_read_reg(uint8_t reg) { // helper function to read data from a register from max30002
    uint32_t data = 0;
    MAX_CS_LOW(); // enable MAX30002
    spi_transfer(((reg & 0x7F) << 1) | 0x01); // Send command byte. Format is 7 bit address + R/W indicator
    data |= ((uint32_t)spi_transfer(0x00) << 16); // getting top data byte
    data |= ((uint32_t)spi_transfer(0x00) << 8); // getting middle data byte
    data |= ((uint32_t)spi_transfer(0x00)); // getting bottom data byte
    MAX_CS_HIGH(); // disable MAX30002

    return data; // what we read from register 'reg'
}

void max30002_write_reg(uint8_t reg, uint32_t data) { // Helper function to write to MAX30002 reg
    MAX_CS_LOW(); // enable MAX30002
    spi_transfer(((reg & 0x7F) << 1) | 0x00); // Send command byte. Same as before, but 0 LSB for write
    spi_transfer((data >> 16) & 0xFF); // top byte of data
    spi_transfer((data >> 8) & 0xFF); // middle byte of data
    spi_transfer((data) & 0xFF); // bottom byte of data
    MAX_CS_HIGH(); // disable MAX30002
}

void max30002_reset(void) { // resets max30002
    max30002_write_reg(MAX_REG_SW_RST, 0x000000);
    __delay_cycles(100000); // 0.1 s delay
}

void max30002_synch(void) { // Begins new BIOZ operations on clock edge. Clears FIFO memory
    max30002_write_reg(MAX_REG_SYNCH, 0x000000);
    __delay_cycles(10000); // 10 ms delay
}

void max30002_fifo_reset(void) { // Begins new BIOZ operations by resetting FIFO. Operations of BIOZ active circuitry not impacted. (No settling/transients)
    max30002_write_reg(MAX_REG_FIFO_RST, 0x000000);
    __delay_cycles(10000); // 10ms delay
}

uint32_t max30002_read_status(void) { // how to determine why MAX interrupted
    return max30002_read_reg(MAX_REG_STATUS);
}

uint32_t max30002_read_info(void) { // gives interface/part/revision information
    return max30002_read_reg(MAX_REG_INFO);
}

uint32_t max30002_read_fifo(void) { // reads BIOZ FIFO at register 0x23. Uses same 32-clock read sequence.
    return max30002_read_reg(MAX_REG_FIFO);
}

int32_t bioz_parse(uint32_t raw) { // helper function to parse bioZ data. Bits [2:0] are status bits and bits [23:4] are the data.
    int32_t data = (int32_t)(raw >> 4)  & 0xFFFFF; // extract 20-bit signed BioZ sample
    if (data & 0x80000) // if bit 19 is 1 (negative)
        data |= 0xFFF00000; // sign-extend 20-bit value to 32-bit signed int
    return (int32_t) data; // returning the parsed data bits
}

// LED STATUS INDICATORS
void led_blink_measuring (void) { // blinking LED = measurement
    LED_TOGGLE();
    __delay_cycles (200000); // 0.2 seconds at 1MHz
}
void led_pulse_transmission (void) { // short pulse = successful event
    LED_ON();
    __delay_cycles(300000); // 0.3 second pulse
    LED_OFF();
}
void led_idle(void) { // LED off = idle/sleeping
    LED_OFF();
}
void error_handler (void) { // ensures LED is solid during error
    LED_ON();

    while(1); // LED stays permanently on with error
}
