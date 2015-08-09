// rttr_tx_node.ino - "The Ratter" TX Node
// ---------------------------------------------------------------------
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.

// #include <Wire.h>         // Used for serial comms.
#include <stdint.h>         // Enable fixed width integers.
#include <avr/wdt.h>        // Inc. Inline Macros for WDT
#include <avr/sleep.h>      // Inc. Inline Macros for Sleep Modes
#include <avr/interrupt.h>  // Required for interrupts.
#include <avr/power.h>      // Power reduction management.
#include <avr/delay.h>      // Delay functions
#include "spi.h"            // Include my spi library.
#include "rfm69w.h"         // Include my rfm69w library
#include "rfm69w_reg.h"     // Register reference for rfm69w
#include "rttr.h"           // Rttr RX/TX Node Common functions

#define F_CPU 8000000UL // 8MHz - Used for delay timing
#define DEBUG  // Enables Debug code. Comment out to disable debug code.

// Function Declarations
void setup_int();
void setupRFM();
void powerSave();
void gotosleep();
void setup_wdt();
void trap_set();

typedef Spi SPIx;        // Create Global instance of the Spi Class
RFM69W<SPIx> RFM;        // Create Global instance of RFM69W Class
volatile uint8_t intFlag = 0x00;  // Setup a flag for the interrupt.
volatile uint8_t wdtFlag = 0x00;  // Setup a flag for WDT interrupt.

void setup() {
    RFM.setReg();  // Setup the registers & initial mode for the RFM69
    setupRFM();    // Application Specific Settings RFM69W
    // DEV NOTE: This delay has no function, program executes correctly without it.
    //_delay_ms(100); // DEBUG - 1000ms works, 500ms works, 100ms works, 0 works
    cli();         // Disable interrupts whilst setting them up.
    setup_int();   // Setup Interrupts
    // setup_wdt();   // Setup WDT Timeout Interrupt
    sei();         // Enable interrupts
    return;
}
void powerSave() {
    power_adc_disable();    // Not using ADC
    power_twi_disable();    // Not using I2C
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();
    // power_spi_disable();     // Todo: Might need for Rx Mode later.
    power_usart0_disable();     // Disable by default, enable if needed.
}
void setupRFM() {
    // Write Custom Setup Values to registers
    // TODO: There are defaults that the the RFM library loads can this be merge to
    //       reduce the number of changes. No need to change a value and change it back.

    // Data Modulation
    // - Packet Mode, OOK, No Shaping
    RFM.singleByteWrite(RegDataModul, 0x08);

    // DIO0 Mapping - Starup value, want to change during operation
    //              - depending on mode
    // TODO: Confirm best initial state.
    RFM.singleByteWrite(RegDioMapping1, 0x00);

    // Packet Config - Set Fixed Length 8 bytes
    // singleByteWrite(RegDataModul,0x10);  // Def Fixed Packet(Default)
    // Set Fixed Packet Length to 8 bytes.
    RFM.singleByteWrite(RegPayloadLength, 0x08);

    // Set Carrier Frequency to 867,999,975.2 Hz
    RFM.singleByteWrite(RegFrfMsb, 0xd9);
    RFM.singleByteWrite(RegFrfMid, 0x00);
    RFM.singleByteWrite(RegFrfLsb, 0x24);

    // Set DIO4/5, Disable Clk Out - None of these used/connected
    RFM.singleByteWrite(RegDioMapping2, 0x07);
}
void setup_wdt() {
    cli(); // Disable interrupts during setup.
    // Clear WDRF - Watchdog System Reset Flag to allow WDE to be cleared later.
    MCUSR &= ~(1 << WDRF);
    /*
    To perform adjustments to WDE & prescaler bits:
    - Set the WDCE - watchdog change enable bit
    - Make adjustments within 4 clock cycles.
    */

    WDTCSR |= (1 << WDCE)| (1 << WDE);  // Set WDCE

    /*
    WDTCSR Register
    - Bit 7 WDIF - Watchdog Interrupt Flag
    - Bit 6 WDIE - Watchdog Interrupt Enable
    - Bit 5 WDP3 - Watchdog Timer Prescaler 3
    - Bit 4 WDCE - Watchdog Change Enable
    - Bit 3 WDE  - Watchdog System Reset Enable
    - Bit 2 WDP2 - Watchdog Timer Prescaler 2
    - Bit 1 WDP1 - Watchdog Timer Prescaler 1
    - Bit 0 WDP0 - Watchdog Timer Prescaler 0

    Setup for Interrupt Mode, 8 second timeout
    ------------------------------------------
    Prescaler Settings for Timeout of approx 8 seconds.
    WDP3 = 1, WDP2 = 0, WDP1 = 0, WDP0 = 1

    To Set Interrupt Mode
    WDE = 0, WDIE = 1 // Toggle WDIE later to enable/disable interrupt mode.

    Other Flags
    WDCE = 1
    WDIF = 1 // Normally cleared by HW, write 1 to clear

    Note: In this mode the system reset is disabled.
    */

    // Set Watch1dog Timer for Interrupt Mode, 8 second timeout.
    // WDT is enabled at this point.
    WDTCSR = (1 << WDP3)|(0 << WDP2)|(0 << WDP1)|(1 << WDP0)|
             (0 << WDE)|(1 << WDIE)|  // Set WDT for interrupt mode
             (1 << WDCE)|(1 << WDIF);
    sei(); // Enable interrupts
}
void gotosleep() {
    // Select the sleep mode to be use and enable it.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    /*
    Dev Note:
        // This sequence....
        sleep_enable();
        sleep_cpu();
        sleep_disable();

        // ... can be replace by this equivalent function.
        sleep_mode();
    */

    // Disable ADC to save power during sleep.
    ADCSRA &= ~(1 << ADEN);
    cli();
    sleep_enable();

    // TODO: Reset WDT Count. Might not make a huge difference.
    // wdt_reset(); // Reset the watchdog timer for full sleep cycle

    // Timing critical - must be done right before entering sleep mode.
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
    sei();
    // Dev Note: Do I need to be reenabling this?
    ADCSRA |= (1 << ADEN);
}
void setup_int() {
    /*---------------------
     PIN CHANGE INTERRUPTS
    ----------------------*/

    // Set PB1 as interrupt input.
    DDRB &= ~(1 << DDB1);
    // Not using internal pullup on PB1 as want to trigger on a logic 1
    // RFM69W DIO0 is logic 0 until set. Pull down resistor not used.
    PCICR |= (1 << PCIE0);  // Enable PCMSK0 covering PCINT[7:0]
    PCMSK0 |= (1 << PCINT1);  // Set mask to only interrupt on PCINT1

    /*---------------------
     EXTERNAL INTERRUPTS
    ----------------------*/

    DDRD &= ~(1 << DDD2);     // Set PD2 (INT0) as an input
    PORTD |= (1 << PORTD2);   // Enable internal pullup resistor
    EICRA |= (1 << ISC00);    // Set INT0 to trigger on any change
    EIMSK &= ~(1 << INT0);    // Disable INT0.
}
ISR(PCINT0_vect) {      // PCINT0 is vector for PCINT[7:0]
    // Dev Note: Serial.println() cmds can't be used in an ISR.
    /*
        The ISR will set a flag that can be tested by the main loop.
        The interrupt is triggered by DIO0 on RFM69W.
        Setting a local flag via this interrupt allows the monitoring of
        RFM69W without the need to constantly read the register statuses
        over the SPI bus.
    */
    intFlag = 0xff;  // Set interrupt flag.
}

ISR(INT0_vect) {  // Triggers on INT0 (See EICRA Reg for Trigger Setup)
    // The ISR does nothing. The act of triggering the ISR wakes the
    // MCU from its deep sleep and resumes program execution.

    // DEV NOTE: Disable Interrupt to prevent trigger
    //     jitter causing multiple executions of ISR
    EIMSK &= ~(1 << INT0);  // Disable INT0
}

ISR(WDT_vect) {         // Runs when WDT timeout is reached
    // Dev Note: This ISR is intended only for waking
    // the mcu from a sleep mode. Speed of the ISR is not important in this case.
    wdt_disable();
}
void ping(int8_t msg) {  // DEV Note: This is development code
    // Load selected data into FIFO Register for transmission

    // Sends teststring stored in array via RFM69W
    // Workout how many characters there are to send.
    // 1 is deducted from count to remove the trailing null char.
    uint8_t tststr[] = "ERROR!";  // Case Default
    uint8_t tststr0[] = "Hello_"; // Case 0
    uint8_t tststr1[] = "World!"; // Case 1
    // Dev Note: Should this result in 2 packets being received?
    //           Only detecting one.
    // - Update: Since fixed length packets are used the only the data
    //           that can be contained in the packet is sent.
    //           It appears the act of sending a single packet clears
    //           any residual data in the FIFO. If there is data for a
    //           follow on packet it should be sent as a separate Tx
    //           operation.
    uint8_t tststr2[] = "0123456789ABCDEF"; // Case 2
    uint8_t tststr3[] = "Reset!"; // Case 3
    uint8_t tststr4[] = "DEBUG!"; // Case 4

    // TODO: Can this be refactored?
    switch (msg) {
    case 0:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr0)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr0[arrayChar]);
        break;
    case 1:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr1)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr1[arrayChar]);
        break;
    case 2:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr2)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr2[arrayChar]);
        break;
    case 3:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr3)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr3[arrayChar]);
        break;
    case 4:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr4)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr4[arrayChar]);
        break;
    default:
        for (uint8_t arrayChar = 0; arrayChar < (sizeof(tststr)-1); arrayChar++)
            RFM.singleByteWrite(RegFifo, tststr[arrayChar]);
    }
}
void transmit(int8_t pkt) {       // Transmit Packet
    // The SPI communication and registers have been set by setup()

    // The RFM69W should be in Sleep mode.
    // Load bytes to transmit into the FIFO register.
    ping(pkt);  // Pass an int to select which msg to send.
    // Data will be sent once the conditions for Tx have been met.
    // In packet mode & data already in the FIFO buffer this should
    // happen as soon as Tx mode is enabled.
    RFM.modeTransmit();  // Data being sent at 4.8kbps

    // Should not have to worry about Tx Startup delay (800uS) as the
    // program waits here until the packet sent flag is raised.

    while (!(RFM.singleByteRead(RegIrqFlags2) & 0x08)) {
        // Keeps checking to see if the packet sent bit is set.
        // Once packet send is confirmed the program will continue.
        // The reason for the loop here is that to save power the
        // transmitter needs to be turned off as soon as possible after
        // the program has finished with it.
    }
    RFM.modeSleep();  // Return to Sleep mode to save power.
    // Having a delay appears to make the program execute more
    // consistantly. Why?? Sleep Delay??
    _delay_ms(1000); // 800uS delay, wait for RFM69W to go back to sleep
}

uint8_t trap_set(uint8_t count) {
    static uint8_t reset_count = 0; // Setup persistant counter, initially 0.
    // Read PortD2 to get sensor state
    // 0 = set, 1 = triggered (Rat in Trap)
    if (PIND & ( 1 << PIND2)) {  // Try this in place of the line below.     <<<<<<< This needs work
    //if (PIND2 == 1) {       // TODO: Check this is detecting correctly
        reset_count = 0;
    } else {
        reset_count = reset_count + 1;
        transmit(4);                // Transmit DEBUG packet
        if (reset_count == 3) {
            transmit(3);            // Send packet indicating Reset
            count = 0;
            reset_count = 0;
            EIMSK |= (1 << INT0);   // Enable INT0
            gotosleep();            // Sleep; continue on INT0 ISR
            transmit(0);            // Send Hello packet
            count = 0;
        }
    }
    return count;
}
void loop() {                      // Main Program Loop
    static uint8_t count = 0x00;   // Persistant counter, initially 0.
    count = trap_set(count);       // Check if trap set
    count++;                // Increment Loop Counter
    if (count == 0xff) {    // Tx new packet after 255 cycles
        transmit(0);        // Send Hello packet
        count = 0x00;
    } else {
        setup_wdt();        // Enable WDT interrupt.
        gotosleep();        // Sleep; wake & continue on WDT timeout.
    }
}

