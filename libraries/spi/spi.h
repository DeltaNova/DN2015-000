// spi.h - Header file for spi.cpp
// Serial Periferal Interface for Atmega328 using the default portB.
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.

#ifndef SPI_H //If spi.h hasn't been included elsewhere, define it now.
#define SPI_H

#include <avr/io.h> // Needed for register references
//#include <cstdint> // Fixed width integer support (C++11)
#include <stdint.h> // Fixed width integer support (Arduino IDE 1.0.5 Compatable)
/*----------------------------------------------------------------------

SPCR - SPI Control Register
SPCR = SPIE,SPE,DORD,MSTR,CPOL,CPHA,SPR1,SPR0

SPSR - SPI Status Register
SPSR = SPIF,WCOL,-,-,-,-,-,SPI2X

SPDR - SPI Data Register
SPDR = MSB,-,-,-,-,-,-,LSB

DDRB - Data Direction Register for PortB
DDRB = DDB7,DDB6,DDB5(SCK),DDB4(MISO),DDB3(MOSI),DDB2(SS),DDB1,DDB0
----------------------------------------------------------------------*/

class Spi
{
    // private members listed first as class members are private by
    // default (without the access specifier).
    private: // Can only be accessed by functions in this class

    public: // Can be accessed by any function in the program
        Spi() // default constructor
        {
                // Anything that needs to be executed when the class
                // is instantiated.
        }
        void InitMaster();
        void InitSlave();

        void InterruptEnable() { SPCR |= (1<<SPIE); }
        void InterruptDisable() { SPCR &= ~(1<<SPIE); }
        void EnableSPI() { SPCR |= (1<<SPE); }
        void DisableSPI() { SPCR &= ~(1<<SPE); }
        void MSBFirst() { SPCR |= (1<<DORD); }
        void LSBFirst() { SPCR &= ~(1<<DORD); }
        void SelectSlave() { PORTB &= ~(1<<PORTB2); }
        void DeselectSlave() { PORTB |= (1<<PORTB2); }

        /*
        Sets one of the SPI Modes below.
        Returns 1 if error.

        SPI Modes
        0 - CPOL=0, CPHA=0
        1 - CPOL=0, CPHA=1
        2 - CPOL=1, CPHA=0
        3 - CPOL=1, CPHA=1
        */
        int8_t SetMode(int8_t Mode);

        /*
        Mode - Clock Frequency  [fosc - freq of uC crystal]
        0      fosc/4 (default)
        1      fosc/16
        2      fosc/64
        3      fosc/128
        4      fosc/2
        5      fosc/8
        6      fosc/32
        7      fosc/64
        */
        int8_t SetClock(int8_t ClockMode);

        uint8_t Transiever(uint8_t data);
};

#endif // End of ifndef SPI_H
