// spi.cpp - also see header spi.h
// Serial Periferal Interface for Atmega328
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.
#include "spi.h"

void Spi::InitSlave() 
{ 
    SPCR &= ~(1<<MSTR);
    //Set SCK,MOSI,SS as inputs
    DDRB &= ~((1<<DDB5)|(1<<DDB3)|(1<<DDB2));
    // Set MISO as output
    DDRB |= (1<<DDB4);
}
 
void Spi::InitMaster() 
{ 
    SPCR |= (1<<MSTR);
    // Set SCK,MOSI,SS as outputs
    DDRB |= (1<<DDB5)|(1<<DDB3)|(1<<DDB2);
    // Set MISO as input
    DDRB &= ~(1<<DDB4);
    // Set SS line HIGH (slave not enabled)
    PORTB |= (1<<PORTB2);
}
 
int8_t Spi::SetMode(int8_t Mode)
{
    /* 
       Sets one of the SPI Modes below.
       Returns 1 if error. 
       
       SPI Modes
        0 - CPOL=0, CPHA=0
        1 - CPOL=0, CPHA=1
        2 - CPOL=1, CPHA=0
        3 - CPOL=1, CPHA=1
    */
    int8_t return_code = 0;
    
    switch(Mode)
    {
        case 0:
            SPCR &= ~((1<<CPOL) | (1<<CPHA));
            break;
        case 1:
            SPCR &= ~(1<<CPOL);
            SPCR |= (1<<CPHA);
            break;
        case 2:
            SPCR |= (1<<CPOL);
            SPCR &= ~(1<<CPHA);
            break;
        case 3:
            SPCR |= (1<<CPOL)|(1<<CPHA);
            break;
        default:
            return_code = 1;
    }
    return return_code;
}

int8_t Spi::SetClock(int8_t ClockMode)
{
    /*
     fosc - frequency of crystal/resonator of uC. 
     
     Mode - Clock Frequency
     0          fosc/4 (default)
     1          fosc/16
     2          fosc/64
     3          fosc/128
     4          fosc/2
     5          fosc/8
     6          fosc/32
     7          fosc/64
      
    */
    int8_t return_code = 0;
    switch(ClockMode)
    {
        case 0: // fosc/4
            SPSR &= ~(1<<SPI2X);
            SPCR &= ~((1<<SPR1)|(1<<SPR0));
            break;
        case 1: // fosc/16
            SPSR &= ~(1<<SPI2X);
            SPCR &= ~(1<<SPR1);
            SPCR |= (1<<SPR0);
            break;
        case 2: // fosc/64
            SPSR &= ~(1<<SPI2X);
            SPCR |= (1<<SPR1);
            SPCR &= ~(1<<SPR0);
            break;
        case 3: // fosc/128
            SPSR &= ~(1<<SPI2X);
            SPCR |= (1<<SPR1)|(1<<SPR0);
            break;
        case 4: // fosc/2
            SPSR |= (1<<SPI2X);
            SPCR &= ~((1<<SPR1)|(1<<SPR0));
            break;
        case 5: // fosc/8
            SPSR |= (1<<SPI2X);
            SPCR &= ~(1<<SPR1);
            SPCR |= (1<<SPR0);
            break;
        case 6: // fosc/32
            SPSR |= (1<<SPI2X);
            SPCR |= (1<<SPR1);
            SPCR &= ~(1<<SPR0);
            break;
        case 7: // fosc/64
            SPSR |= (1<<SPI2X);
            SPCR |= (1<<SPR1)|(1<<SPR0);
            break;
        default:
            return_code = 1;
    }
    return return_code;
}

uint8_t Spi::Transiever(uint8_t data) // Spi member function
{
    // Load data into SPDR initiating transfer
    SPDR = data;
    //Wait until transmission complete
    while(!(SPSR &(1<<SPIF)));
    // Return received data.
    return(SPDR);
}
