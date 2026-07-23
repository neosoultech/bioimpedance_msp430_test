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
// Note: FCLK must be present or no data generated, FIFO must be drained to prevent overflow, data interrupt-driven
// For eval board (MAX30001EVSYS), a 32kHz crystal oscillator is already provided, so FCLK on MSP is disabled.

// MAX30002 FIFO FORMAT:
// 24 bit word: bits [23:4] are 20-bit signed BioZ sample (2SC), bit [3]: zero padding, bits [2:0]: BTAG status tag
// BTAG: 000 valid, 001 over/under range, 010 valid EOF, 011 over/under EOF
// Note: Extract bits [23:4], mask 20 bits, and sign extend bit 19.


                                // ST25 Information
// I2C interface
// - Data flow: Phone tap --> ST25 asserts GPO on P2.0 falling edge --> PORT2 ISR --> phone_flag = 1 --> main loop detects phone_flag
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
//  P2.7: MUX_SEL on custom patch PCB only. Controls ADG884 target/reference mux.
//        Not used in TEST_MAX_2SPOT_MANUAL eval-board workflow.


                            // LED Notes:
// LED pulses once if transmission occurs successfully.
// LED blinks on and off if measurement is occurring.
// LED stays solid if error
// LED is off if IDLE/SLEEP.

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#define USE_LPM              0 // 0 = Normal (debug), 1 = low power
#define BIOZ_LOG_N 128
                                // INTERRUPT HANDLER
// external interrupts from P2.0 (ST25 GPO): Indicates phone interaction, sets phone_flag
// external interrupts from P2.1 (MAX30002 INTB): Indicates data ready / FIFO event, sets max_flag
volatile int32_t debug_bioz = 0; // debugging
volatile uint32_t sample_count = 0; // debugging
volatile int32_t debug_target_bioz = 0;
volatile int32_t bioz_log_value[BIOZ_LOG_N] = {0};
volatile uint16_t bioz_log_i = 0;
volatile uint8_t bioz_log_done = 0;
volatile int32_t debug_print_count = 0;
volatile int32_t debug_baseline_bioz = 0;
volatile int32_t debug_bioz_diff = 0;
volatile uint32_t debug_info = 0;
volatile uint32_t debug_en_int_read = 0;
volatile uint8_t debug_config_pass = 0;
volatile uint32_t debug_en_int_start = 0;
volatile uint32_t debug_mngr_int_start = 0;
volatile uint32_t debug_gen_start = 0;
volatile uint32_t debug_bmux_start = 0;
volatile uint32_t debug_bioz_start = 0;
volatile uint8_t debug_start_pass = 0;
volatile uint32_t debug_bovf_count = 0;
volatile uint8_t demo_msg[180] = {0};
volatile uint16_t demo_msg_len = 0;
volatile int32_t demo_fake_target = 6285;
volatile int32_t demo_fake_reference = 5316;
volatile uint8_t debug_st25_ctrl = 0;
volatile uint8_t debug_st25_sso = 0;
volatile uint8_t debug_st25_mb_mode = 0;
volatile uint8_t debug_st25_fail_step = 0;
volatile uint8_t debug_ctrl_user = 0;
volatile uint8_t debug_ctrl_data = 0;
volatile uint8_t debug_st25_ctrl_after_write = 0; // MB_CTRL_DYN after mailbox write
volatile uint8_t debug_st25_mb_len_after_write = 0; // MB_LEN_DYN after mailbox write
volatile uint8_t debug_st25_mailbox_test_addr = 0; // 0x53 or 0x2D used for mailbox message write
volatile unsigned int debug_eeprom_offset = 0; // EEPROM chunk offset during NDEF write
volatile unsigned int debug_eeprom_total_len = 0; // total TLV length being written
volatile unsigned int debug_eeprom_chunk_len = 0; // current chunk length
volatile uint8_t debug_eeprom_fail_reason = 0; // 1=len/null fail, 2=chunk write fail
volatile uint8_t demo_result_pending = 0; // 0 = next tap measures/writes result, 1 = next tap resets tag to rea
volatile uint8_t debug_max_collect_failed = 0;
volatile uint32_t debug_max_wait_timeout_count = 0;
volatile uint8_t debug_st25_eh_ctrl_before = 0;
volatile uint8_t debug_st25_eh_ctrl_after = 0;
volatile uint8_t debug_st25_eh_mode_before = 0;
volatile uint8_t debug_st25_eh_mode_after = 0;


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
#define MAX_BIOZ_DHPF_BYPASS     (0UL << 14)
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
    MAX_BIOZ_DHPF_BYPASS            | /* Digital HPF disabled (was destroying steady signal) */ \
    MAX_BIOZ_DLPF_4HZ               | /* Digital LPF to reduce noise and output bandwidth */ \
    MAX_BIOZ_FCGEN_8192HZ           | /* 8.192KHz generated current frequency, closest to 10KHz (wanted previously) */ \
    MAX_BIOZ_CGMON_OFF              | /* Current generator compliance monitor OFF. Diagnostic for later */ \
    MAX_BIOZ_CGMAG_TEST             | /* 32uA injected current magnitude. Using middle low value. */ \
    MAX_BIOZ_PHOFF_0)                 /* No phase offset */

    // MAX30002 MUX SELECT (MCU CHOOSES). MAX is still a single BioZ channel. The ADG884 muxes select whether MAX30002 BioZ path connects to target or reference electrode set.
    // Sequential, not simultaneous
    // P2.7 controls ADG884 IN1/IN2 pins.
    // MUX_SEL = 1 selects TARGET electrodes, while MUX_SEL = 0 selects BASELINE electrodes
#define BIOZ_MUX_SEL_BIT    BIT7
#define BIOZ_SELECT_TARGET()    (P2OUT |= BIOZ_MUX_SEL_BIT)
#define BIOZ_SELECT_BASELINE()  (P2OUT &= ~BIOZ_MUX_SEL_BIT)
#define BIOZ_DISCARD_SAMPLES 64
#define BIOZ_BLOCK_MISMATCH_LIMIT 500

                                    //

                             // ST25DV REGISTER MAP //

// ST25DV I2C addresses
// MSP430 uses 7-bit slave addresses.
// ST25DV device select code: 1010 E2 1 1 R/W
// E2 = 0 -> user memory, dynamic registers, mailbox
// E2 = 1 -> system configuration area
#define ST25DV_ADDR_USER              0x53
#define ST25DV_ADDR_SYSTEM            0x57
#define ST25DV_ADDR_DATA              0x2D // ST25DV third I2C address; likely used for data/FTM/mailbox functions

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

// ST25DV energy harvesting dynamic register

#define ST25DV_REG_EH_CTRL_DYN 0x2002
#define ST25DV_EH_CTRL_DYN_EH_EN BIT0
#define ST25DV_EH_CTRL_DYN_EH_ON BIT1
#define ST25DV_EH_CTRL_DYN_FIELD_ON BIT2
#define ST25DV_EH_CTRL_DYN_VCC_ON BIT3
#define ST25DV_REG_EH_MODE             0x0002  // system area register
#define ST25DV_EH_MODE_ENABLE_AUTO     0x00    // bit0 cleared = auto EH enabled
#define ST25DV_EH_MODE_DISABLE_AUTO    BIT0    // bit0 set = auto EH disabled

// URL info
#define BIOZ_URL_SCANNING "apps.powerapps.com/play/acef55d5-b582-4b23-9945-7312b9c0f7d5?tenantId=d196a604-d993-4028-90df-e223d68126d2&mode=scanning"
#define BIOZ_URL_RESULT_BASE "apps.powerapps.com/play/acef55d5-b582-4b23-9945-7312b9c0f7d5?tenantId=d196a604-d993-4028-90df-e223d68126d2&mode=result&b="

                                        //

                      //TEST MODE MACROS. ONLY 1 AT A TIME CAN BE ENABLED. //
// PASSED TESTS: TEST_MAX_SPI, TEST_MAX_ID, TEST_MAX_CONFIG, TEST_MAX_START, TEST_MAX_READ, TEST_MAX_2SPOT_MANUAL, TEST_BUILD_BIOZ_MSG_ONLY, TEST_ST25_READ, TEST_ST25_WRITE, TEST_ST25_GPO
#define TEST_MODE                 1
// if test mode enabled...
#define TEST_ST25_WRITE           0
#define TEST_ST25_READ            0
#define TEST_ST25_GPO             0
#define TEST_ST25_MAILBOX         1
#define TEST_ST25_ADDR_DEBUG      0
#define TEST_ST25_MB_CTRL_WRITE   0

#define TEST_MAX_SPI              0
#define TEST_MAX_ID               0
#define TEST_MAX_CONFIG           0
#define TEST_MAX_START            0
#define TEST_MAX_READ             0
#define TEST_MAX_2SPOT            0
#define TEST_MAX_2SPOT_MANUAL     0
#define TEST_DEMO_S1_BIOZ_TO_ST25 0
#define TEST_BUILD_BIOZ_MSG_ONLY  0


#define USE_BIOZ_MUX        (TEST_MAX_2SPOT || TEST_MAX_START || TEST_MAX_READ)
#define USE_EXP430_S1       (TEST_MAX_2SPOT_MANUAL || TEST_DEMO_S1_BIOZ_TO_ST25)
#define USE_MSP_FCLK          0
#define DEMO_WRITE_READY_ON_BOOT 1


                                        //

                        // OTHER MACROS (DON'T CHANGE)
#define LED_ON()      (P1OUT |= BIT0) // LED on indicates error.
#define LED_OFF()     (P1OUT &= ~BIT0) // LED off indicates IDLE/SLEEP
#define LED_TOGGLE()  (P1OUT ^= BIT0) // macro to toggle LED on and off. --> Indicates that sensing/transmission occuring
#define MAX_CS_LOW()  (P1OUT &= ~BIT7) //select MAX30002, CS active low
#define MAX_CS_HIGH() (P1OUT |= BIT7) // deselect MAX30002
#define BUTTON_S1_BIT BIT3            // S1 button on EXP430 is BIT3
                                        //

                        // FUNCTION DECLARATIONS
int st25dv_write(uint8_t slave, uint16_t mem_addr, uint8_t data);
int st25dv_enable_energy_harvesting_dynamic(void);
void i2c_init();
void i2c_pins_init();
void i2c_bus_recover(void);
void error_handler(void);
void led_blink_measuring(void);
void led_pulse_transmission(void);
void led_idle(void);
void led_double_pulse(void);
uint8_t st25dv_read(uint8_t slave, uint16_t addr);
void gpo_init();
int mailbox_write(uint8_t *message, unsigned int length);
int check_nack();
void wait_tx_ready();
int i2c_send_and_check(uint8_t byte);
int enable_mailbox(void);
int st25dv_present_i2c_password(void);
int st25dv_i2c_session_is_open(void);
int st25dv_write_ndef_text_eeprom(uint8_t *ndef_msg, unsigned int ndef_len); // Writes a complete NDEF message into normal ST25 EEPROM tag memory
int st25dv_mailbox_is_free(void);
int st25dv_enable_energy_harvesting_static(void);
int wait_tx_ready_timeout(void);
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
void bioz_mux_init(void);
void max30002_configure_bioz_once(void);
void bioz_prepare_selected_path(void);
int32_t max30002_collect_bioz_average(uint16_t sample_goal);
void button_s1_init(void);
void wait_for_s1_press(void);
uint16_t append_str(uint8_t *buf, uint16_t idx, uint16_t max, const char *s);
uint16_t append_int32(uint8_t *buf, uint16_t idx, uint16_t max, int32_t value);
uint16_t append_signed_x10(uint8_t *buf, uint16_t idx, uint16_t max, int32_t value_x10);
uint16_t build_bioz_ndef_text_msg(uint8_t *out, uint16_t max_len, int32_t target_raw, int32_t reference_raw);
uint16_t append_uint32_no_sign(uint8_t *buf, uint16_t idx, uint16_t max, uint32_t value); // Appends an unsigned integer as ASCII text with no plus sign
uint16_t append_url_int32(uint8_t buf[], uint16_t idx, uint16_t max, int32_t value);
uint16_t append_url_x10(uint8_t buf[], uint16_t idx, uint16_t max, int32_t value_x10);
uint16_t build_ready_url_ndef(uint8_t out[], uint16_t max_len);
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
	i2c_bus_recover(); // force I2C bus/pins back to clean idle state after previous stuck transaction
	spi_pins_init();
	spi_init();
	#if USE_BIOZ_MUX
	    bioz_mux_init();
	#endif
    #if USE_MSP_FCLK
	fclk_init();
    #endif
	#if USE_EXP430_S1
	    button_s1_init();
	#endif
	gpo_init();
	max_int_init();
#if DEMO_WRITE_READY_ON_BOOT
    {
        uint8_t msg[260];
        uint16_t msg_len;

        debug_st25_fail_step = 80; // writing ready URL at boot

        msg_len = build_ready_url_ndef(msg, sizeof(msg));

        if (msg_len == 0) {
            debug_st25_fail_step = 81; // ready URL builder failed at boot
            error_handler();
        }

        if (st25dv_write_ndef_text_eeprom(msg, msg_len)) {
            demo_result_pending = 0;
            debug_st25_fail_step = 0;
            led_double_pulse(); // boot ready reset complete
        }
        else {
            debug_st25_fail_step = 82; // ready URL EEPROM write failed at boot
            error_handler();
        }

        phone_flag = 0; // clear any stale GPO event from startup
        P2IFG &= ~BIT0;
    }
#endif
#if (TEST_DEMO_S1_BIOZ_TO_ST25 || TEST_DEMO_PHONE_TRIGGER_BIOZ)
    if (!enable_mailbox()) { // enable FTM/mailbox when demo needs ST25 mailbox
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

#elif TEST_ST25_MAILBOX // Phone tap demo: ready -> result -> auto reset ready using phone_flag only
        if (phone_flag) {
            phone_flag = 0;

            uint8_t msg[260];
            uint16_t msg_len;

            if (demo_result_pending == 0) {
                // First tap: user is on ready/alignment page.
                // Wait a bit so phone finishes reading the ready URL, then measure and write result.

                int32_t target_avg;
                int32_t reference_avg;

                // TARGET MEASUREMENT
                debug_st25_fail_step = 60; // entered first tap / measure-result path

                __delay_cycles(2000000); // 2 sec delay after NFC trigger


                debug_st25_fail_step = 65; // about to configure MAX

                debug_max_collect_failed = 0;
                debug_max_wait_timeout_count = 0;
                max_flag = 0;

                max30002_configure_bioz_once();

                debug_st25_fail_step = 66; // MAX configured, preparing selected path
                BIOZ_SELECT_TARGET();
                bioz_prepare_selected_path();

                debug_st25_fail_step = 67; // collecting live MAX BioZ average

                target_avg = max30002_collect_bioz_average(16);

                if (debug_max_collect_failed) {
                    debug_st25_fail_step = 68; // MAX BioZ collection timed out
                    error_handler();
                }
                // BASELINE MEASUREMENT
                debug_max_collect_failed = 0;
                debug_max_wait_timeout_count = 0;
                max_flag = 0;
                debug_st25_fail_step = 69; // selecting baseline path
                BIOZ_SELECT_BASELINE();

                bioz_prepare_selected_path();

                debug_st25_fail_step = 70; // collecting baseline BioZ average

                reference_avg = max30002_collect_bioz_average(16);

                if (debug_max_collect_failed) {
                    debug_st25_fail_step = 71; // baseline MAX BioZ collection timed out
                    error_handler();
                }


                debug_target_bioz = target_avg;
                debug_baseline_bioz = reference_avg;
                debug_bioz_diff = target_avg - reference_avg;

                debug_st25_fail_step = 67; // BioZ collected, about to build result URL

                msg_len = build_bioz_ndef_text_msg(msg, sizeof(msg), target_avg, reference_avg);

                if (msg_len == 0) {
                    debug_st25_fail_step = 61; // result URL builder failed
                    error_handler();
                }

                debug_st25_fail_step = 62; // result URL built, about to write EEPROM

                if (st25dv_write_ndef_text_eeprom(msg, msg_len)) {
                    __delay_cycles(100000);

                    demo_result_pending = 1; // next tap resets to ready
                    phone_flag = 0; // clear stale phone event that may have occurred during processing
                    P2IFG &= ~BIT0; // clear stale GPO interrupt flag

                    debug_st25_fail_step = 0;
                    led_pulse_transmission(); // one pulse = result saved
                }
                else {
                    debug_st25_fail_step = 63; // result EEPROM write failed
                    error_handler();
                }
            }
            else {
                // Second tap: phone is opening the result page.
                // Wait long enough for user/phone to load result, then reset tag to ready.

                debug_st25_fail_step = 70; // entered second tap / reset-ready path

                __delay_cycles(5000000); // 5 sec delay so phone can open/read result URL

                msg_len = build_ready_url_ndef(msg, sizeof(msg));

                if (msg_len == 0) {
                    debug_st25_fail_step = 71; // ready URL builder failed
                    error_handler();
                }

                debug_st25_fail_step = 72; // ready URL built, about to write EEPROM

                if (st25dv_write_ndef_text_eeprom(msg, msg_len)) {
                    __delay_cycles(100000);

                    demo_result_pending = 0; // next tap measures again
                    phone_flag = 0; // clear stale phone event
                    P2IFG &= ~BIT0; // clear stale GPO interrupt flag

                    debug_st25_fail_step = 0;
                    led_double_pulse(); // two pulses = reset to ready
                }

                else {
                    debug_st25_fail_step = 73; // ready EEPROM write failed
                    error_handler();
                }
            }
        }
        else {
            led_idle();
        }

#elif TEST_ST25_ADDR_DEBUG
    // Debug test: compare mailbox dynamic register reads through 0x53 and 0x2D.
    // This helps determine which ST25 I2C address is valid for mailbox/FTM status.

    debug_ctrl_user = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN using 0x53
    debug_ctrl_data = st25dv_read(ST25DV_ADDR_DATA, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN using 0x2D

    led_pulse_transmission(); // pulse only means the debug read code ran

    __delay_cycles(1000000); // wait 1 second before repeating

#elif TEST_ST25_MB_CTRL_WRITE
    // Debug test: try to enable the dynamic mailbox bit using the known-good 0x53 user/dynamic-register address.
    // This test does not write an actual mailbox message yet.
    // It only checks whether MB_CTRL_DYN bit0, MB_EN, can be set and read back.

    debug_st25_fail_step = 19; // marker: test started and is about to write MB_CTRL_DYN
    debug_ctrl_user = 0; // clear previous user-address debug read
    debug_ctrl_data = 0; // clear previous data-address debug read

    if (!st25dv_write(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN, ST25DV_MB_CTRL_MB_EN)) { // try to set MB_EN bit using 0x53
        debug_st25_fail_step = 20; // write to MB_CTRL_DYN through 0x53 failed
        error_handler();
    }

    debug_st25_fail_step = 23; // marker: write returned success, now reading MB_CTRL_DYN

    __delay_cycles(10000); // small delay after dynamic register write

    debug_ctrl_user = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN using 0x53
    debug_ctrl_data = st25dv_read(ST25DV_ADDR_DATA, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN using 0x2D only for comparison

    if (debug_ctrl_user == 0xFF) { // 0xFF is suspicious because st25dv_read returns 0xFF on failure
        debug_st25_fail_step = 22; // read through 0x53 returned invalid/all-ones
        error_handler();
    }

    if (debug_ctrl_user & ST25DV_MB_CTRL_MB_EN) { // if mailbox enable bit is now set
        debug_st25_fail_step = 0; // success
        led_pulse_transmission(); // pulse means MB_EN set successfully
    }
    else {
        debug_st25_fail_step = 21; // write happened, but MB_EN did not stay set
        error_handler();
    }

    __delay_cycles(1000000);


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
	    debug_info = info;
	    if ((info & 0xF03000) == 0x501000) { // MAX30001 INFO fixed bits: D[23:20] = 0101 and D[13:12] = 01. Mask 0xF03000 should equal 0x501000. INFO is not valid as the first command after SW_RST.

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
	    debug_en_int_read = en_int_read;
	    if ((en_int_read & 0x000003) == MAX_INTB_OPEN_DRAIN) { // lower bits should be '10 (2 decimal)' for open drain
	        debug_config_pass = 1;
	        led_pulse_transmission(); // config write/read worked
	    }
	    else {
	        debug_config_pass = 2;
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
	    BIOZ_SELECT_TARGET();
	    __delay_cycles(50000); // let mux/electrode path settle

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

	    debug_en_int_start = en_int_read;
	    debug_mngr_int_start = mngr_int_read;
	    debug_gen_start = gen_read;
	    debug_bmux_start = bmux_read;
	    debug_bioz_start = bioz_read;

	    if (((en_int_read & MAX_CFG_EN_INT_START) == MAX_CFG_EN_INT_START) && // check that the registers have been altered
	                ((mngr_int_read & MAX_CFG_MNGR_INT_START) == MAX_CFG_MNGR_INT_START) &&
	                ((gen_read & MAX_CFG_CNFG_GEN_START) == MAX_CFG_CNFG_GEN_START) &&
	                ((bmux_read & (MAX_BMUX_OPENP_BIT | MAX_BMUX_OPENN_BIT)) == 0) &&
	                (bioz_read == MAX_CFG_CNFG_BIOZ_START)) {
	        int i;
	        for (i = 0; i < 6; i++) {
	                        debug_start_pass = 1;
	                        led_blink_measuring(); // Blink while measuring if success
	                    }

	                    led_idle(); // stop blinking
	                }
	                else {
	                    debug_start_pass = 2;
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
	    BIOZ_SELECT_TARGET();
	    __delay_cycles(50000); // let mux/electrode path settle

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

	                debug_print_count = 0;

// READ LOOP:
	                while(1) {
	                    if (max_flag) { // set by ISR on INTB
	                        max_flag = 0; // reset flag
	                        status = max30002_read_status(); // read its status

	                        if (status & MAX_STATUS_BOVF) { // checking first for global overflow
	                            debug_bovf_count++;
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
	                                if (bioz_log_i < BIOZ_LOG_N) {
	                                    bioz_log_value[bioz_log_i] = bioz;
	                                    bioz_log_i++;

	                                    if (bioz_log_i >= BIOZ_LOG_N) {
	                                        bioz_log_done = 1;
	                                    }
	                                }
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

    #elif TEST_MAX_2SPOT
	// Custom patch test: ADG884 mux automatically switches between target and baseline/reference electrodes.
	int32_t target_avg;
	int32_t baseline_avg;
	int32_t diff;

	max30002_configure_bioz_once(); // configure BioZ

	BIOZ_SELECT_TARGET();           // target selected
	bioz_prepare_selected_path();   // prepare for BioZ measurement
	target_avg = max30002_collect_bioz_average(16); // average of 16 parsed raw BioZ samples
	BIOZ_SELECT_BASELINE();         // baseline selected
	bioz_prepare_selected_path();   // prepare for BioZ measurement
	baseline_avg = max30002_collect_bioz_average(16);  // average of 16 parsed raw BioZ samples
	diff = target_avg - baseline_avg;   // difference between target and baseline
	debug_target_bioz = target_avg;     // debug for CCS window
	debug_baseline_bioz = baseline_avg; // debug for CCS window
	debug_bioz_diff = diff;             // debug for CCS window

	led_pulse_transmission(); // indicates one full target & baseline cycle completed
	__delay_cycles(1000000);

    #elif TEST_MAX_2SPOT_MANUAL
	// Eval-kit/manual test: user places electrodes on target, presses S1,
	// then moves electrodes to baseline/reference area and presses S1 again.
	// Does not use ADG884 mux.
	int32_t target_avg;
	int32_t baseline_avg;
	int32_t diff;

	max30002_configure_bioz_once(); // configure BioZ startup
// STEP 1: Place electrodes on target. Press S1 when ready.
	wait_for_s1_press();
	bioz_prepare_selected_path();
	target_avg = max30002_collect_bioz_average(16);  // average of 16 measurements
	debug_target_bioz = target_avg; // debug
	led_pulse_transmission(); // pulse for success in target measurement
// STEP 2: Move same electrodes to the baseline area. Press S1 when ready.
	wait_for_s1_press();
	bioz_prepare_selected_path();
	baseline_avg = max30002_collect_bioz_average(16);
	debug_baseline_bioz = baseline_avg; // average of 16 measurements

	diff = target_avg - baseline_avg; // difference b/w target and baseline
	debug_bioz_diff = diff; // debug

	led_pulse_transmission(); // pulse for success in baseline measurement
	__delay_cycles(1000000);

#elif TEST_DEMO_S1_BIOZ_TO_ST25 // Demo: S1 press #1 measures target, S1 press #2 measures reference, then result is written to ST25DV mailbox for phone read.
    int32_t target_avg;
    int32_t reference_avg;
    int32_t diff;
    uint8_t msg[180]; // buffer for final NDEF message
    uint16_t msg_len; // length of message

    max30002_configure_bioz_once(); // configure BioZ startup once

    wait_for_s1_press(); // STEP 1: Place electrodes on target. Press S1 when ready.
    bioz_prepare_selected_path();
    target_avg = max30002_collect_bioz_average(16); // average of 16
    debug_target_bioz = target_avg; // debug
    led_pulse_transmission(); // pulse for target complete

    wait_for_s1_press(); // STEP 2: Move same electrodes to reference area. Press S1 when ready.
    bioz_prepare_selected_path();
    reference_avg = max30002_collect_bioz_average(16);
    debug_baseline_bioz = reference_avg; // debug

    diff = target_avg - reference_avg;
    debug_bioz_diff = diff; // debug

    msg_len = build_bioz_ndef_text_msg(msg, sizeof(msg), target_avg, reference_avg); // using helper function to build a message to phone
    if (msg_len == 0) { // if empty
        error_handler();
    }

    if (mailbox_write(msg, msg_len)) { // write complete NDEF to ST25 mailbox
        led_pulse_transmission(); // pulse means ST25 mailbox write worked
    }
    else {
        error_handler(); // if error, solid LED
    }

    __delay_cycles(1000000); // wait 1 sec



#elif TEST_BUILD_BIOZ_MSG_ONLY
    // No-hardware test.
    // This only tests whether the BIOZ NDEF text message is built correctly.
    // It does not use MAX hardware and does not use ST25 hardware.

    demo_msg_len = build_bioz_ndef_text_msg((uint8_t *)demo_msg,
                                            sizeof(demo_msg),
                                            demo_fake_target,
                                            demo_fake_reference);

    if (demo_msg_len == 0) {
        error_handler();
    }

    led_pulse_transmission();

    while (1) {
        // Hold here so CCS can inspect demo_msg and demo_msg_len.
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

int wait_tx_ready_timeout(void)
{
    uint32_t timeout = 100000;

    while (!(UCB0IFG & UCTXIFG0)) {
        if (UCB0IFG & UCNACKIFG) {
            UCB0IFG &= ~UCNACKIFG;
            UCB0CTLW0 |= UCTXSTP;
            while (UCB0CTLW0 & UCTXSTP);
            return 0;
        }

        if (--timeout == 0) {
            UCB0CTLW0 |= UCTXSTP;
            while (UCB0CTLW0 & UCTXSTP);
            return 0;
        }
    }

    return 1;
}

void wait_rx_ready(void) { // helper function to wait until RX buf ready
    while (!(UCB0IFG & UCRXIFG0));
}

void i2c_bus_recover(void)
{
    UCB0CTLW0 = UCSWRST; // hold I2C peripheral in reset so it releases control of pins

    P1SEL0 &= ~(BIT2 | BIT3); // temporarily make P1.2/P1.3 GPIO, not I2C
    P1SEL1 &= ~(BIT2 | BIT3);

    P1DIR &= ~(BIT2 | BIT3); // make SDA/SCL inputs
    P1REN |= BIT2 | BIT3; // enable internal resistors
    P1OUT |= BIT2 | BIT3; // make resistors pullups

    __delay_cycles(10000); // short settle delay

    P1IFG &= ~(BIT2 | BIT3); // clear any stale flags just in case

    i2c_pins_init(); // put pins back into I2C mode
    i2c_init(); // reinitialize I2C peripheral
}

int i2c_send_and_check(uint8_t byte) { // sends a byte and checks to see if recieved
    UCB0TXBUF = byte;
    if (!wait_tx_ready_timeout()) {
        return 1;
    }
    return check_nack();
}
int st25dv_write_sequence(uint8_t slave, uint16_t mem_addr, uint8_t * data, unsigned int length) { // helper function to write
    unsigned int i;
    if (data == 0 || length == 0) {
        return 0; // error if nothing
    }
    UCB0I2CSA = slave; // setting st25dv as slave
    UCB0CTLW0 |= UCTR | UCTXSTT; // enabling control bits to transmit
    if (!wait_tx_ready_timeout()) return 0; // wait for tx with timeout

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
    debug_st25_sso = sso; // save I2C_SSO_DYN value for CCS watch

    if (sso & ST25DV_I2C_SSO_OPEN) { // if I2C security session is open
        return 1;
    }

    return 0;
}


int st25dv_write(uint8_t slave, uint16_t mem_addr, uint8_t data) { // function to send a test byte. mem_addr is 16bit address, uint8_t slave is the 8 bit slave address, and uint8_t data is the test byte of data. Returns 1 if success, 0 if not.
// SETUP
    UCB0I2CSA = slave; // Target I2C device with slave address 'slave'
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmission mode for master, then transmit START condition in master mode
    if (!wait_tx_ready_timeout()) return 0; // wait for TX buffer with timeout
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

int st25dv_enable_energy_harvesting_dynamic(void)
{
    uint8_t eh_ctrl;

    debug_st25_fail_step = 90; // starting EH dynamic enable

    eh_ctrl = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_EH_CTRL_DYN);
    debug_st25_eh_ctrl_before = eh_ctrl;

    if (eh_ctrl == 0xFF) {
        debug_st25_fail_step = 91; // EH_CTRL_DYN read failed
        return 0;
    }

    eh_ctrl |= ST25DV_EH_CTRL_DYN_EH_EN;

    if (!st25dv_write(ST25DV_ADDR_USER, ST25DV_REG_EH_CTRL_DYN, eh_ctrl)) {
        debug_st25_fail_step = 92; // EH_CTRL_DYN write failed
        return 0;
    }

    __delay_cycles(10000);

    eh_ctrl = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_EH_CTRL_DYN);
    debug_st25_eh_ctrl_after = eh_ctrl;

    if (eh_ctrl == 0xFF) {
        debug_st25_fail_step = 93; // EH_CTRL_DYN readback failed
        return 0;
    }

    if ((eh_ctrl & ST25DV_EH_CTRL_DYN_EH_EN) == 0) {
        debug_st25_fail_step = 94; // EH_EN bit did not stay set
        return 0;
    }

    debug_st25_fail_step = 0;
    return 1;
}
int st25dv_enable_energy_harvesting_static(void)
{
    uint8_t eh_mode;

    debug_st25_fail_step = 95; // starting static EH enable

    if (!st25dv_present_i2c_password()) {
        debug_st25_fail_step = 96; // failed to present I2C password
        return 0;
    }

    if (!st25dv_i2c_session_is_open()) {
        debug_st25_fail_step = 97; // I2C security session did not open
        return 0;
    }

    eh_mode = st25dv_read(ST25DV_ADDR_SYSTEM, ST25DV_REG_EH_MODE);
    debug_st25_eh_mode_before = eh_mode;

    if (eh_mode == 0xFF) {
        debug_st25_fail_step = 98; // EH_MODE read failed
        return 0;
    }

    /*
     * Static EH_MODE bit0 is reversed:
     * bit0 = 0 means automatic energy harvesting enabled.
     * bit0 = 1 means automatic energy harvesting disabled.
     */
    eh_mode &= ~ST25DV_EH_MODE_DISABLE_AUTO;

    if (!st25dv_write(ST25DV_ADDR_SYSTEM, ST25DV_REG_EH_MODE, eh_mode)) {
        debug_st25_fail_step = 99; // EH_MODE write failed
        return 0;
    }

    __delay_cycles(20000); // allow system EEPROM/config write time

    eh_mode = st25dv_read(ST25DV_ADDR_SYSTEM, ST25DV_REG_EH_MODE);
    debug_st25_eh_mode_after = eh_mode;

    if (eh_mode == 0xFF) {
        debug_st25_fail_step = 100; // EH_MODE readback failed
        return 0;
    }

    if (eh_mode & ST25DV_EH_MODE_DISABLE_AUTO) {
        debug_st25_fail_step = 101; // EH_MODE bit0 still set, auto EH not enabled
        return 0;
    }

    debug_st25_fail_step = 0;
    return 1;
}

int enable_mailbox (void) { // helper function to enable FTM mailbox
     uint8_t mb_mode;

     debug_st25_fail_step = 0; // clear previous ST25 mailbox failure before starting

     if (!st25dv_present_i2c_password()) { // sending default, all zero password
             debug_st25_fail_step = 1; // failed while presenting I2C password
             return 0;
         }

     if (!st25dv_i2c_session_is_open()) { // confirm I2C security session opened
             debug_st25_fail_step = 2; // I2C password sent, but I2C security session did not open
             return 0;
         }

     if (!st25dv_write(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN, 0x00)) { // Disable dynamic mailbox before changing static mailbox mode
         debug_st25_fail_step = 3; // failed while disabling dynamic mailbox
         return 0;
     }

     __delay_cycles(10000); // short delay after disabling dynamic mailbox

     mb_mode = st25dv_read(ST25DV_ADDR_SYSTEM, ST25DV_REG_MB_MODE); // Read static MB_MODE register from system memory
     debug_st25_mb_mode = mb_mode; // save MB_MODE value for CCS watch

     // Important:
     // If mb_mode reads as 0xFF, do NOT treat that as success and do NOT preserve all bits.
     // Force MB_MODE to known-good mailbox-enabled value instead.
     // ST25DV_MB_MODE_ENABLE is bit0, so write 0x01.
     mb_mode = ST25DV_MB_MODE_ENABLE; // force mailbox mode enabled, do not preserve suspicious 0xFF value

     if (!st25dv_write(ST25DV_ADDR_SYSTEM, ST25DV_REG_MB_MODE, mb_mode)) { // write static MB_MODE = 0x01
         debug_st25_fail_step = 4; // failed while writing static MB_MODE
         return 0;
     }

     __delay_cycles(20000); // allow system EEPROM/config write time

     mb_mode = st25dv_read(ST25DV_ADDR_SYSTEM, ST25DV_REG_MB_MODE); // read MB_MODE back after write
     debug_st25_mb_mode = mb_mode; // save readback for CCS watch

     if ((mb_mode & ST25DV_MB_MODE_ENABLE) == 0) { // confirm MB_MODE bit0 stayed enabled
         debug_st25_fail_step = 11; // MB_MODE write did not stick
         return 0;
     }

     if (!st25dv_write(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN, ST25DV_MB_CTRL_MB_EN)) { // Enable mailbox dynamically at runtime
         debug_st25_fail_step = 5; // failed while enabling dynamic mailbox
         return 0;
     }

     __delay_cycles(10000); // short delay after enabling dynamic mailbox

     debug_st25_ctrl = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN); // read dynamic mailbox control for CCS watch

     if ((debug_st25_ctrl & ST25DV_MB_CTRL_MB_EN) == 0) { // confirm dynamic mailbox enable bit is set
         debug_st25_fail_step = 6; // dynamic mailbox enable bit is not set
         return 0;
     }

     debug_st25_fail_step = 0; // mailbox enable completed successfully
     return 1;
}

int st25dv_mailbox_is_free(void) {
    uint8_t ctrl;

    ctrl = st25dv_read(ST25DV_ADDR_USER, ST25DV_REG_MB_CTRL_DYN); // read MB_CTRL_DYN to see if MB free using user address
    debug_st25_ctrl = ctrl; // save MB_CTRL_DYN value for CCS watch

    if (ctrl == 0xFF) { // 0xFF is suspicious here and likely means read failed/all-ones
        debug_st25_fail_step = 10; // suspicious MB_CTRL_DYN read using 0x53
        return 0;
    }

    if ((ctrl & ST25DV_MB_CTRL_MB_EN) == 0) { // MB must be enabled
        debug_st25_fail_step = 6; // mailbox dynamic enable bit is not set
        return 0;
    }

    if (ctrl & ST25DV_MB_CTRL_HOST_PUT_MSG) { // if host put something in mailbox
        debug_st25_fail_step = 7; // host message already pending
        return 0;
    }

    if (ctrl & ST25DV_MB_CTRL_RF_PUT_MSG) { // if slave put something in mailbox
        debug_st25_fail_step = 8; // RF message already pending
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

int st25dv_write_ndef_text_eeprom(uint8_t *ndef_msg, unsigned int ndef_len)
{
    uint8_t tlv[272]; // buffer for NDEF TLV wrapper plus NDEF message
    unsigned int i; // loop counter
    unsigned int total_len; // total bytes to write to EEPROM
    unsigned int offset; // current EEPROM write offset
    unsigned int chunk_len; // number of bytes to write in current chunk

    debug_eeprom_fail_reason = 0;
    debug_eeprom_offset = 0;
    debug_eeprom_total_len = 0;
    debug_eeprom_chunk_len = 0;

    if (ndef_msg == 0) { // reject null pointer
        debug_eeprom_fail_reason = 1;
        return 0;
    }

    if (ndef_len == 0 || ndef_len > 259) { // keep message safely inside local buffer
        debug_eeprom_fail_reason = 1;
        return 0;
    }

    tlv[0] = 0x03; // NDEF TLV tag
    tlv[1] = (uint8_t)ndef_len; // NDEF message length

    for (i = 0; i < ndef_len; i++) { // copy NDEF record into TLV buffer
        tlv[2 + i] = ndef_msg[i];
    }


    tlv[2 + ndef_len] = 0xFE; // terminator TLV

    total_len = ndef_len + 3; // TLV tag + length + NDEF message + terminator

    while ((total_len % 4) != 0) { // pad to 4-byte boundary so final write is not a partial chunk
        tlv[total_len] = 0x00; // harmless padding after terminator
        total_len++;
    }

    debug_eeprom_total_len = total_len;


    offset = 0;

    while (offset < total_len) {
        chunk_len = total_len - offset;

        if (chunk_len > 4) {
            chunk_len = 4;
        }

        debug_eeprom_offset = offset;
        debug_eeprom_chunk_len = chunk_len;


        {
            uint8_t attempt;
            uint8_t chunk_ok = 0;

            for (attempt = 0; attempt < 5; attempt++) {
                if (st25dv_write_sequence(ST25DV_ADDR_USER, 0x0004 + offset, &tlv[offset], chunk_len)) {
                    chunk_ok = 1;
                    break;
                }

                i2c_bus_recover();          // recover if ST25 NACKed or bus got weird
                __delay_cycles(200000);     // wait 200 ms before retrying same chunk
            }

            if (!chunk_ok) {
                debug_eeprom_fail_reason = 2;
                return 0;
            }
        }

        __delay_cycles(100000); // 100 ms after successful EEPROM chunk write
        offset += chunk_len;
    }

    return 1;
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

void bioz_mux_init(void) { // helper function to choose between ADG884 pins. P2.7 is BIOZ_MUX_SEL output
    P2SEL0 &= ~BIOZ_MUX_SEL_BIT; // GPIO MODE
    P2SEL1 &= ~BIOZ_MUX_SEL_BIT; // GPIO MODE
    BIOZ_SELECT_TARGET(); // default to target electrodes
    P2DIR |= BIOZ_MUX_SEL_BIT; // make P2.7 output
    P2REN &= ~BIOZ_MUX_SEL_BIT; // disable internal pull resistor
    P2IE &= ~BIOZ_MUX_SEL_BIT; // no interrupt on mux sel pin
    P2IFG &= ~BIOZ_MUX_SEL_BIT; // clear any stale flags
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

void max30002_configure_bioz_once(void) { // configures the MAX30002 for bioimpedance sensing. Only needs to occur once
    uint32_t bmux;

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
}

void bioz_prepare_selected_path(void) { // helper function to get ready for BioZ sample
    max_flag = 0; // clear stale interrupt flag
    __delay_cycles(50000); // Allow selected electrode path/contact to settle.
    max30002_fifo_reset(); // clear old samples
    max30002_synch(); // restart bioZ sample timing & alignment
    // Let BioZ samples settle after restart.
    __delay_cycles(1000000);        // 1.0 s

}

int32_t max30002_collect_bioz_average(uint16_t sample_goal) // takes average of n x 2 samples; if the n first trials and the n second trials are not similar, it will discard the outlier.
{
    uint32_t status;
    uint32_t raw;
    uint8_t tag;
    int32_t bioz;
    uint32_t wait_timeout = 0;

    int64_t sum1 = 0;
    int64_t sum2 = 0;

    uint16_t count1 = 0;
    uint16_t count2 = 0;
    uint16_t discard_count = 0;

    int32_t avg1;
    int32_t avg2;
    int32_t diff;

    while (count2 < sample_goal) {
        if (!max_flag) {
            wait_timeout++;

            if (wait_timeout > 3000000) { // timeout if MAX interrupt never arrives
                debug_max_collect_failed = 1;
                debug_max_wait_timeout_count = wait_timeout;
                return 0;
            }

            continue;
        }

        wait_timeout = 0;

        max_flag = 0;
        status = max30002_read_status();

        if (status & MAX_STATUS_BOVF) { // if FIFO overflow happened, clear FIFO and keep trying
            debug_bovf_count++;
            max30002_fifo_reset();
            max30002_synch();

            discard_count = 0; // restart discard after overflow
            count1 = 0;
            count2 = 0;
            sum1 = 0;
            sum2 = 0;

            continue;
        }

        while ((status & MAX_STATUS_BINT) && (count2 < sample_goal)) {
            raw = max30002_read_fifo(); // getting raw data from MAX
            tag = raw & 0x07; // get FIFO overflow tag

            if (tag == 0x07) { // FIFO overflow tag
                debug_bovf_count++;
                max30002_fifo_reset();
                max30002_synch();

                discard_count = 0; // restart discard after overflow
                count1 = 0;
                count2 = 0;
                sum1 = 0;
                sum2 = 0;

                break;
            }

            if (tag == 0x06) { // if FIFO empty, try again
                break;
            }

            if (tag == 0x00 || tag == 0x02) { // valid sample or valid EOF
                bioz = bioz_parse(raw); // parse data
                debug_bioz = bioz; // debug bioz
                sample_count++; // for debug

                if (discard_count < BIOZ_DISCARD_SAMPLES) { // Throw away early settling samples
                    discard_count++;
                }
                else if (count1 < sample_goal) { // first settled block
                    sum1 += bioz; // sum added
                    count1++; // counter incremented
                }
                else { // second settled block
                    sum2 += bioz; // sum added
                    count2++; // counter incremented
                }
            }

            if (tag == 0x01 || tag == 0x03) { // over/under range
                bioz = bioz_parse(raw);
                debug_bioz = bioz;
            }

            if (tag == 0x02 || tag == 0x03) { // EOF tags
                break;
            }

            status = max30002_read_status();
        }
    }

    if (count1 == 0 || count2 == 0) { // error if no count
        return 0;
    }

    avg1 = (int32_t)(sum1 / count1); // averaging the first 16 samples
    avg2 = (int32_t)(sum2 / count2); // averaging the second 16 samples

    diff = avg1 - avg2; // compare first settled block to second settled block
    if (diff < 0) {
        diff = -diff; // take ||
    }

    if (diff > BIOZ_BLOCK_MISMATCH_LIMIT) { // first block likely still settling or contact artifact if difference is over 500
        return avg2; // use later block only
    }

    return (avg1 + avg2) / 2; // blocks agree, use both
}

uint16_t append_str(uint8_t *buf, uint16_t idx, uint16_t max, const char *s) // appends a normal null-terminated C string into byte buffer
{
    while (*s && idx < max) { // copy until end of buffer full
        buf[idx++] = (uint8_t)(*s++); // copy character by character into buffer
    }
    return idx; // return updated buffer index.
}

uint16_t append_int32(uint8_t *buf, uint16_t idx, uint16_t max, int32_t value) // signed 32-bit integer into readable ASCII. i.e 10 to +10
{
    char temp[12]; // temp storage for digits
    uint8_t i = 0; // how many digits stored in temp
    uint8_t j;

    if (value < 0) { // if neg num
        if (idx < max) { // if there is room in buffer
            buf[idx++] = '-'; // append minus sign
        }
        value = -value; // get positive val to extract digits
    }
    else {
        if (idx < max) { // if value positive
            buf[idx++] = '+'; // append + sign
        }
    }

    if (value == 0) { // if zero
        if (idx < max) {
            buf[idx++] = '0'; // append zero
        }
        return idx; // return updated index
    }

    while (value > 0 && i < sizeof(temp)) { // extract digits from R to L
        temp[i++] = (char)('0' + (value % 10)); // LSD stored as ASCII
        value /= 10; // drop LSD
    }

    for (j = 0; j < i; j++) { // copy digits in reverse order so num reads right
        if (idx < max) {
            buf[idx++] = temp[i - 1 - j];
        }
    }

    return idx; // return updated index in buffer
}

uint16_t append_uint32_no_sign(uint8_t *buf, uint16_t idx, uint16_t max, uint32_t value)     // Appends an unsigned integer as readable ASCII text. Example: value = 6285 becomes "6285". This is used for Target and Reference because those do not need plus signs.
{
    char temp[12]; // Temporary reversed digit storage
    uint8_t i = 0; // Counts how many digits were stored in temp
    uint8_t j; // Loop index used when copying digits back in correct order
    if (value == 0) { // Special case because digit extraction loop would not run for zero
        if (idx < max) { // Make sure there is buffer room
            buf[idx++] = '0'; // Append ASCII zero
        }
        return idx; // Return updated index
    }
    while (value > 0 && i < sizeof(temp)) { // Extract digits from right to left
        temp[i++] = (char)('0' + (value % 10)); // Store current least significant digit as ASCII
        value /= 10; // Drop the least significant digit
    }
    for (j = 0; j < i; j++) { // Copy digits back in reverse order so number reads correctly
        if (idx < max) { // Make sure there is buffer room
            buf[idx++] = temp[i - 1 - j]; // Append next digit in correct order
        }
    }
    return idx; // Return updated buffer index
}

uint16_t append_signed_x10(uint8_t *buf, uint16_t idx, uint16_t max, int32_t value_x10) // append signed fixed pt num with a decimal place. i.e. value_x10 = 55 goes to +5.5
{
    int32_t whole; // whole num part
    int32_t frac; // fraction
    char temp[12]; // temp digit storage reversed
    uint8_t i = 0;
    uint8_t j;

    if (value_x10 < 0) { // negative
        if (idx < max) {
            buf[idx++] = '-'; // append -
        }
        value_x10 = -value_x10;
    }
    else {
        if (idx < max) { // positive
            buf[idx++] = '+'; // append positive
        }
    }

    whole = value_x10 / 10; // whole num part
    frac = value_x10 % 10; // remainder gives decimal digit

    if (whole == 0) { // if whole num is zero
        if (idx < max) {
            buf[idx++] = '0'; // append zero before decimal pt
        }
    }
    else { // if whole num is nonzero
        while (whole > 0 && i < sizeof(temp)) { // extract whole num digits from R to L
            temp[i++] = (char)('0' + (whole % 10));
            whole /= 10;
        }

        for (j = 0; j < i; j++) { // copy digits back in correct order
            if (idx < max) {
                buf[idx++] = temp[i - 1 - j];
            }
        }
    }

    if (idx < max) { // ensure buffer room
        buf[idx++] = '.'; // decimal pt append
    }

    if (idx < max) {
        buf[idx++] = (uint8_t)('0' + frac); // append one decial digit
    }

    return idx;
}

uint16_t append_url_int32(uint8_t buf[], uint16_t idx, uint16_t max, int32_t value)
{
    char temp[12];
    uint8_t i = 0;
    uint8_t j;

    if (value < 0) {
        if (idx < max) {
            buf[idx++] = '-';
        }
        value = -value;
    }

    if (value == 0) {
        if (idx < max) {
            buf[idx++] = '0';
        }
        return idx;
    }

    while (value > 0 && i < sizeof(temp)) {
        temp[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    for (j = 0; j < i; j++) {
        if (idx < max) {
            buf[idx++] = temp[i - 1 - j];
        }
    }

    return idx;
}

uint16_t append_url_x10(uint8_t buf[], uint16_t idx, uint16_t max, int32_t value_x10)
{
    int32_t whole;
    int32_t frac;

    if (value_x10 < 0) {
        if (idx < max) {
            buf[idx++] = '-';
        }
        value_x10 = -value_x10;
    }

    whole = value_x10 / 10;
    frac = value_x10 % 10;

    idx = append_url_int32(buf, idx, max, whole);

    if (idx < max) {
        buf[idx++] = '.';
    }

    if (idx < max) {
        buf[idx++] = (uint8_t)('0' + frac);
    }

    return idx;
}

uint16_t build_bioz_ndef_text_msg(uint8_t out[], uint16_t max_len, int32_t target_raw, int32_t reference_raw)
{
    uint8_t uri[256];
    uint16_t ui = 0;
    uint16_t i;
    uint16_t payload_len;
    uint16_t total_len;
    int32_t diff;
    int32_t percent;
    int32_t zdiff_x10;

    diff = target_raw - reference_raw;

    if (reference_raw != 0) {
        percent = (int32_t)(((int64_t)diff * 100) / reference_raw);
    }
    else {
        percent = 0;
    }

    zdiff_x10 = (int32_t)(((int64_t)diff * 900) / 15946);

    uri[ui++] = 0x04; // URI prefix code for https://

    ui = append_str(uri, ui, sizeof(uri), BIOZ_URL_RESULT_BASE);
    ui = append_url_int32(uri, ui, sizeof(uri), percent);

    ui = append_str(uri, ui, sizeof(uri), "&d=");
    ui = append_url_int32(uri, ui, sizeof(uri), diff);

    ui = append_str(uri, ui, sizeof(uri), "&o=");
    ui = append_url_x10(uri, ui, sizeof(uri), zdiff_x10);

    ui = append_str(uri, ui, sizeof(uri), "&t=");
    ui = append_url_int32(uri, ui, sizeof(uri), target_raw);

    ui = append_str(uri, ui, sizeof(uri), "&r=");
    ui = append_url_int32(uri, ui, sizeof(uri), reference_raw);

    payload_len = ui;
    total_len = 4 + payload_len;

    if (total_len > max_len || payload_len > 255) {
        return 0;
    }

    out[0] = 0xD1; // NDEF short record, MB/ME/SR set, well-known type
    out[1] = 0x01; // type length
    out[2] = (uint8_t)payload_len; // URI payload length
    out[3] = 'U'; // URI record type

    for (i = 0; i < payload_len; i++) {
        out[4 + i] = uri[i];
    }

    return total_len;
}

uint16_t build_ready_url_ndef(uint8_t out[], uint16_t max_len) // program back to ready mode before demo
{
    uint8_t uri[220];
    uint16_t ui = 0;
    uint16_t i;
    uint16_t payload_len;
    uint16_t total_len;

    uri[ui++] = 0x04; // URI prefix code for https://

    ui = append_str(uri, ui, sizeof(uri), BIOZ_URL_SCANNING);

    payload_len = ui;
    total_len = 4 + payload_len;

    if (total_len > max_len || payload_len > 255) {
        return 0;
    }

    out[0] = 0xD1;
    out[1] = 0x01;
    out[2] = (uint8_t)payload_len;
    out[3] = 'U';

    for (i = 0; i < payload_len; i++) {
        out[4 + i] = uri[i];
    }

    return total_len;
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

void led_double_pulse(void) // single pulse = result written, double = tag reset to ready
{
    LED_ON();
    __delay_cycles(180000);
    LED_OFF();
    __delay_cycles(180000);
    LED_ON();
    __delay_cycles(180000);
    LED_OFF();
}
// EXP430 buttons
void button_s1_init(void) { // helper function to initialize buttons on EXP430FR2433 dev kit
    // S1 = P2.3
    // active-low buttons w/ internal pullups

    P2SEL0 &= ~(BUTTON_S1_BIT); // GPIO mode
    P2SEL1 &= ~(BUTTON_S1_BIT); // GPIO mode
    P2DIR &= ~(BUTTON_S1_BIT); // // S1 as input
    P2REN |= (BUTTON_S1_BIT); // turn on resistors
    P2OUT |= (BUTTON_S1_BIT); // turn on pullups
    P2IE &= ~(BUTTON_S1_BIT); // turn off hardware interrupts
    P2IFG &= ~(BUTTON_S1_BIT); // clear any interrupt flags
}
void wait_for_s1_press(void) { // blocks execution until button S1 fully pressed & released
    while (P2IN & BUTTON_S1_BIT); // BUTTON_S1_BIT is 0 if pressed
    __delay_cycles(50000); // 50ms debounce
    while (!(P2IN & BUTTON_S1_BIT));
    __delay_cycles(50000);
}
