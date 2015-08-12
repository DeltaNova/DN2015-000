// rttr_rx_node.ino - "The Ratter" RX Node
// ---------------------------------------------------------------------
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.

#include <Wire.h>       // Used for serial comms.
#include <stdint.h>     // Enable fixed width integers.
#include <avr/wdt.h>    // Includes Inline Macros for working with the WDT
#include <avr/sleep.h>  // Includes Inline Macros for working with Sleep Modes
#include <avr/interrupt.h>  // Required for interrupts.
#include <avr/power.h>  // Power reduction management.
#include "spi.h"        // Include my spi library.
#include "rfm69w.h"     // Include my rfm69w library
#include "rfm69w_reg.h"     // Register reference for rfm69w

#define DEBUG  // Enables Debugging code. Comment out to disable debug code.

// Function Declarations
void setup_int();
void listen();
void setup_mode();
void setup_mode2();
void setupRFM();
void powerSave();
void gotosleep();
void setup_wdt();

typedef Spi SPIx;                // Create Global instance of the Spi Class
RFM69W<SPIx> RFM;        // Create Global instance of RFM69W Class
volatile uint8_t packet_count = 0;  // Setup a flag for monitoring the interrupt.
volatile uint8_t wdtFlag = 0x00;  // Setup a flag for monitoring WDT interrupt.
//uint8_t mode = 0x00;     // Node startup mode. Rx Default.

void setup() {

    // DEV Note: Will startup power requirements benefit from reordering of the
    //           powerSave(), RFM.setReg(), setupRFM() functions?
    //           The RFM module is likely to be the biggest power draw until it
    //           makes it into sleep mode.

    RFM.setReg();  // Setup the registers & initial mode for the RFM69
    setupRFM();    // Application Specific Settings RFM69W
    setup_mode();  // Determine the startup mode from status of PB0.
    setup_int();   // Setup Interrupts
    //setup_wdt();   // Setup WDT Timeout Interrupt
    sei();  // Enable interrupts
    return;
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

void setup_mode() {
    /*
    // Set PB0 as Tx/Rx Mode select input
    DDRB &= ~(1 << DDB0);
    // No internal pullup on PB0, hardwired to VCC (Tx) or GND (Rx).
    PORTB &= ~(1 << PORTB0);
    // Configure the node startup mode as a Tx or Rx.
    if (PINB & (1 << PINB0)) {
        // Tx Mode Selected
        mode = 0xff;  // Change node mode
        // RFM69W configured to startup in sleep mode and will wake to
        // transmit as required.
        // TODO: Check interrupt settings / DIO0 map for sleep mode
    } else */{
        // Rx Mode Selected
        #ifdef DEBUG
        //power_usart0_enable();// Enable Serial comms for Rx Mode.
        Serial.begin(19200);  // Setup Serial Comms
        Serial.println("Rx Mode");  // DEBUG: Print "Rx Mode"
        #endif
        RFM.modeReceive();
    }
    return;
}

void setup_wdt() {
    // Clear WDRF - Watchdog System Reset Flag to allow WDE to be cleared later.
    MCUSR &= ~(1<<WDRF);
    /*
    To perform adjustments to WDE & prescaler bits:
    - Set the WDCE - watchdog change enable bit
    - Make adjustments within 4 clock cycles.
    */

    WDTCSR |= (1<<WDCE)| (1<<WDE); // Set WDCE

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
    WDTCSR = (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)|
             (0<<WDE)|(1<<WDIE)|
             (1<<WDCE)|(1<<WDIF);
}

void gotosleep(){
    // Select the sleep mode to be use and enable it.
    // In this case the Power-down mode is selected.
    //MCUCR |= (1<<SM1)|(1<<SE);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Todo: Does turning off the external interrupts INT1,INT0 reduce sleep power?
    //EIMSK = 0x00;

    /*
    Dev Note:
        // This sequence....
        sleep_enable();
        sleep_cpu();
        sleep_disable();

        // ... can be replace by this equivalent function.
        sleep_mode();
    */

    ADCSRA &= ~(1<<ADEN);
    cli();
    sleep_enable();

    // TODO: Reset WDT Count. Might not make a huge difference for this application.
    //wdt_reset(); // Reset the watchdog timer for full sleep cycle
    sleep_bod_disable(); // Timing critical - must be done right before endtering sleep mode.
    sei();
    sleep_cpu();
    sleep_disable();
    sei();
    ADCSRA |= (1<<ADEN);
}

void setup_int() {
    // Set PB1 as interrupt input.
    DDRB &= ~(1 << DDB1);
    // Not using internal pullup on PB1 as want to trigger on a logic 1
    // RFM69W DIO0 is logic 0 until set. Pull down resistor not used.
    PCICR |= (1 << PCIE0);  // Enable PCMSK0 covering PCINT[7:0]
    PCMSK0 |= (1 << PCINT1);  // Set mask to only interrupt on PCINT1
}

ISR(PCINT0_vect) {  // PCINT0 is vector for PCINT[7:0]
    // Dev Note: Serial.println() cmds can't be used in an ISR.
    /*
        The ISR will incrementthe packet_count that can be tested by the main loop.
        The interrupt is triggered by DIO0 on RFM69W.
        Setting a local flag via this interrupt allows the monitoring of
        RFM69W without the need to constantly read the register statuses
        over the SPI bus.
    */
    //packet_count = packet_count + 1;  // Increment packet_count.
    packet_count += 1;
}

ISR(WDT_vect) { // Runs when WDT timeout is reached
    // Dev Note: This ISR is intended only for waking
    // the mcu from a sleep mode. Speed of the ISR is not important in this case.
}

void listen() {
    Serial.println(packet_count);
    // Listens for an incomming packet via RFM69W
    // Read the Payload Ready bit from RegIrqFlags2 to see if any data
    uint8_t char_count = 0;
    while ((char_count < 7) && (RFM.singleByteRead(RegIrqFlags2) & 0x04)){
    //while (RFM.singleByteRead(RegIrqFlags2) & 0x04) { // True whilst FIFO still contains data.
        // Read 8 Byte Packet

        //while (char_count < 7) {
            Serial.print(RFM.singleByteRead(RegFifo));
            Serial.print(" ");
            char_count++;
        }
    //while (RFM.singleByteRead(RegIrqFlags2) & 0x04){
        Serial.println(RFM.singleByteRead(RegFifo));
        //Serial.println(" ");
        // End 8 Byte Packet
        //--packet_count;
        packet_count -= 1;
        Serial.println(packet_count);
    //}
    //packet_count = packet_count - 1; // Decrement Packet Count

}

void loop() {
    //Serial.println("Loop");
    while (packet_count != 0) {
        listen();
    }
}

