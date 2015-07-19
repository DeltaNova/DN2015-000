// rfm69w_reg.h - Header file for use with rfm69w.h rfm69w.cpp
// Register list for use with the rfm69w transeiver.
// Author: M. Tunstall
// NOTE: This is heavily commented for my own learning/reference.
#ifndef RFM69W_REG_H
#define RFM69W_REG_H
        
enum
{
    /*
        REFERENCE NOTE:
        The name of an unscoped enumeration may be ommitted: 
        such declaration only introduces the enumberators into the 
        enclosing scope.
        e.g.
        // defines A = 0, B = 1, C = 0, D = 2
        enum { A, B, C=0, D=A+2 }; 
    */
    /*
        DEV NOTE:
        By using the unscoped enumeration it is possible to create
        references to the hardware registers of the rfm69w.
    */
    //Register addresses
    RegFifo         = 0x00, // FIFO read/write access
    RegOpMode       = 0x01, // Op modes of the transceiver
    RegDataModul    = 0x02, // Data op mode & Modulation Settings
    RegBitRateMsb   = 0x03, // Bit Rate Setting, MSB
    RegBitRateLsb   = 0x04, // Bit Rate Setting, LSB
    RegFdevMsb      = 0x05, // Freq Division Setting MSB
    RegFdevLsb      = 0x06, // Freq Division Setting LSB
    RegFrfMsb       = 0x07, // RF Carrier Freq, MSB
    RegFrfMid       = 0x08, // RF Carrier Freq, Intermediate Bits
    RegFrfLsb       = 0x09, // RF Carriet Freq, LSB
    RegOsc1         = 0x0a, // RC Oscillators Settings
    RegAfcCtrl      = 0x0b, // AFC control in low modulation index situations
    //Reserved0C    = 0x0c, // RESERVED
    RegListen1      = 0x0d, // Listen Mode Settings
    RegListen2      = 0x0e, // Listen Mode idle duration
    RegListen3      = 0x0f, // Listen Mode Rx duration
    RegVersion      = 0x10, // Version??
    RegPaLevel      = 0x11, // PA Selection & Output Pwr Ctrl
    RegPaRamp       = 0x12, // Control of PA Ramp time (FSK Mode)
    RegOcp          = 0x13, // Over Current Protection Control
    //Reserved14    = 0x14, // RESERVED
    //Reserved15    = 0x15, // RESERVED
    //Reserved16    = 0x16, // RESERVED
    //Reserved17    = 0x17, // RESERVED
    RegLna          = 0x18, // LNA Settings
    RegRxBw         = 0x19, // Channel Filter BW Control
    RegAfcBw        = 0x1a, // Channel Filter BW control during AFC routine
    RegOokPeak      = 0x1b, // OOK demodulator selection & ctrl (peak mode)
    RegOokAvg       = 0x1c, // Avg threshold control of the OOK demodulator
    RegOokFix       = 0x1d, // Fixed threshold control of the OOK demodulator
    RegAfcFei       = 0x1e, // AFC & FEI control and status
    RegAfcMsb       = 0x1f, // MSB of the freq correction of the AFC
    RegAfcLsb       = 0x20, // LSB of the freq correction of the AFC
    RegFeiMSB       = 0x21, // MSB of the calculated freq error
    RegFeiLSB       = 0x22, // LSB of the calculated freq error
    RegRssiConfig   = 0x23, // RSSI related settings
    ResRssiValue    = 0x24, // RSSI value in dBm
    RegDioMapping1  = 0x25, // Mapping of pins DIO0 - DIO3
    RegDioMapping2  = 0x26, // Mapping of pins DIO4 - DIO5, ClkOut Freq.
    RegIrqFlags1    = 0x27, // Status reg: PLL Lock State, Timeout, RSSI > Threshold
    RegIrqFlags2    = 0x28, // Status reg: FIFO handling flags
    RegRssiThresh   = 0x29, // RSSI Threshold control
    RegRxTimeout1   = 0x2a, // Timout duration between Rx request and RSSI detection
    RegRxTimeout2   = 0x2b, // Timeout duration between RSSI detection and PayloadReady
    RegPreambleMsb  = 0x2c, // Preamble length, MSB
    RegPreambleLsb  = 0x2d, // Premable length, LSB
    RegSyncConfig   = 0x2e, // Sync Word recognition control
    
    RegSyncValue1   = 0x2f, // Sync World Byte1
    RegSyncValue2   = 0x30, // Sync World Byte2
    RegSyncValue3   = 0x31, // Sync World Byte3
    RegSyncValue4   = 0x32, // Sync World Byte4
    RegSyncValue5   = 0x33, // Sync World Byte5
    RegSyncValue6   = 0x34, // Sync World Byte6
    RegSyncValue7   = 0x35, // Sync World Byte7
    RegSyncValue8   = 0x36, // Sync World Byte8
    
    RegPacketConfig1 = 0x37, // Packet Mode Settings
    RegPayloadLength = 0x38, // Payload Length Settings
    
    RegNodeAdrs      = 0x39, // Node Address
    RegBroadcastAdrs = 0x3a, // Broadcase Address
    RegAutoModes     = 0x3b, // Auto modes settings
    RegFifoThresh    = 0x3c, // FIFO theshold, TX start condition
    RegPacketConfig2 = 0x3d, // Packet Mode Settings
    
    RegAesKey1      = 0x3e, // AES Cypher Key Byte1
    RegAesKey2      = 0x3f, // AES Cypher Key Byte2
    RegAesKey3      = 0x40, // AES Cypher Key Byte3
    RegAesKey4      = 0x41, // AES Cypher Key Byte4
    RegAesKey5      = 0x42, // AES Cypher Key Byte5
    RegAesKey6      = 0x43, // AES Cypher Key Byte6
    RegAesKey7      = 0x44, // AES Cypher Key Byte7
    RegAesKey8      = 0x45, // AES Cypher Key Byte8
    RegAesKey9      = 0x46, // AES Cypher Key Byte9
    RegAesKey10     = 0x47, // AES Cypher Key Byte10
    RegAesKey11     = 0x48, // AES Cypher Key Byte11
    RegAesKey12     = 0x49, // AES Cypher Key Byte12
    RegAesKey13     = 0x4a, // AES Cypher Key Byte13
    RegAesKey14     = 0x4b, // AES Cypher Key Byte14
    RegAesKey15     = 0x4c, // AES Cypher Key Byte15
    RegAesKey16     = 0x4d, // AES Cypher Key Byte16
    
    RegTemp1        = 0x4e, // Temperature Sensor Control
    RegTemp2        = 0x4f, // Temperature readout
    // RegTest      = 0x50+ // Internal Test Register
    RegTestLna      = 0x58, // Sensitivity Boost
    RegTestPa1      = 0x5a, // High Power PA settings
    RegTestPa2      = 0x5c, // High Power PA settings
    RegTestDagc     = 0x6f, // Fading Margin Improvement
    RegTestAfc      = 0x71  // AFC ofset for low modulation index AFC
};
#endif
