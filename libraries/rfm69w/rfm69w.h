// rfm69w.h - Header file for rfm69w.cpp
// Library to use the rfm69w transeiver.
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.
//#include "spi.h"
#include <stdint.h> // Enable fixed width integers.
#include "rfm69w_reg.h" // Register Addresses as a separate file

#ifndef RFM69W_H // Header guards
#define rfm69w_h
#define RFM69W_H

template< typename SPI_T >
class RFM69W
{
    protected: // accessed by member functions and derived classes.
        SPI_T SPI; // Reference to an instance of the SPI interface.

    private: // accessed by member functions but not derived classes.
        void setDefaultReg(); // Load Register Recommended Defaults
    public: // accessed by anybody.
        //RFM69W(Spi& SPIx) : SPI(SPIx) //Default constructor
        RFM69W() // Default Constructor
        {
            SPI.InitMaster(); // Initialise as Master SPI Node
            SPI.SetClock(1); // Change the SPI Clock rate.
            SPI.EnableSPI(); // Enable SPI Communication.
            // DEV NOTE: It doesn't seem possible to load the default
            // register values from within the constructor.
        }
        ~RFM69W(){}  // Destructor
        void setReg(); // Wrapper to load Custom & Recommended Defaults
        uint8_t singleByteRead(uint8_t byteAddr);
        void singleByteWrite(uint8_t byteAddr, uint8_t dataByte);
        void setNodeAdr(uint8_t Adr); // Set the Node Address to be included with data packet.
        void calOsc(); // Calibrate RC Oscillator

        // TODO: Add function to adjust transmit power.

        void modeTransmit();
        void modeReceive();
        void modeSleep();
        void modeStandby();
};

//------------------

template< typename SPI_T >
void RFM69W<SPI_T>::setDefaultReg()
{
    // Write the recommended default values into device registers
    singleByteWrite(RegLna,0x88);
    singleByteWrite(RegRxBw,0x55);
    singleByteWrite(RegAfcBw,0x8b);
    singleByteWrite(RegDioMapping2,0x07);
    singleByteWrite(RegRssiThresh,0xe4);
    singleByteWrite(RegSyncValue1,0x01);
    singleByteWrite(RegSyncValue2,0x01);
    singleByteWrite(RegSyncValue3,0x01);
    singleByteWrite(RegSyncValue4,0x01);
    singleByteWrite(RegSyncValue5,0x01);
    singleByteWrite(RegSyncValue6,0x01);
    singleByteWrite(RegSyncValue7,0x01);
    singleByteWrite(RegSyncValue8,0x01);
    singleByteWrite(RegFifoThresh,0x8f);
    singleByteWrite(RegTestDagc,0x30);
};
template< typename SPI_T >
void RFM69W<SPI_T>::setNodeAdr(uint8_t Adr)
{
    singleByteWrite(RegNodeAdrs, Adr);
};
template< typename SPI_T >
void RFM69W<SPI_T>::calOsc()
{
    // TODO: Calibrate RC Oscillator
    singleByteWrite(RegOsc1,0x80); // Trigger cal operation
    // Reg will read 0x01 whilst cal in progress.
    // Reg will read 0x41 when cal complete.
};
template< typename SPI_T >
void RFM69W<SPI_T>::setReg()
{
    // Wrapper for register setup
    setDefaultReg();
    modeSleep(); // Set to sleep mode
};
template< typename SPI_T >
uint8_t RFM69W<SPI_T>::singleByteRead(uint8_t byteAddr)
{
    // Read a single byte from an address over the SPI interface
    uint8_t readByte; // Place to store returned data
    SPI.SelectSlave();// Set Slave Select line low
    SPI.Transiever(byteAddr); // Request data from address
    readByte = SPI.Transiever(0); // Pad the shift register to get data.
    SPI.DeselectSlave();//Set Slave Select line high
    return readByte;
};
template< typename SPI_T >
void RFM69W<SPI_T>::singleByteWrite(uint8_t byteAddr, uint8_t dataByte)
{
    // Write a single byte to an address over the SPI interface.
    SPI.SelectSlave();// Slave Select line low
    // In this instance we are not handling any data the slave returns.
    // To write, the MSB of the address must be 1; MSB 0 to read.
    SPI.Transiever(byteAddr|0x80); // Send the address byte.
    SPI.Transiever(dataByte);   // Send the data byte.
    SPI.DeselectSlave(); // Slave Select line high
    return;
};
template< typename SPI_T >
void RFM69W<SPI_T>::modeTransmit()
{   // Set Op Mode
    singleByteWrite(RegOpMode,0x0C);
    // Set DIO Map
    // DIO0 will indicate PacketSent in Tx Mode
    singleByteWrite(RegDioMapping1,0x00);
};
template< typename SPI_T >
void RFM69W<SPI_T>::modeReceive()
{
    // Set Op Mode
    singleByteWrite(RegOpMode,0x10);
    // Set DIO Map
    // DIO0 will indicate PayloadReady in Rx Mode
    singleByteWrite(RegDioMapping1,0x40);
};
template< typename SPI_T >
void RFM69W<SPI_T>::modeSleep()
{
    singleByteWrite(RegOpMode,0x00);
};
template< typename SPI_T >
void RFM69W<SPI_T>::modeStandby()
{
    singleByteWrite(RegOpMode,0x04);
};

#endif //RFM69W_H
