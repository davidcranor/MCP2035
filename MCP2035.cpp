/******************************************************************************
Microchip MCP2035 Bodycom IC driver for Maple

By David Cranor
cranor@mit.edu
8/20/2013

 * The MIT License
 *
 * Copyright (c) 2013, MIT Media Lab
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#include "MCP2035.h"
#include <wirish/wirish.h>
#include <stdint.h>

MCP2035::MCP2035()
{

}

void MCP2035::begin(uint8_t io, uint8_t clk, uint8_t cs)
{
    ioPin = io;
    clkPin = clk;
    csPin = cs;

    //Configure pins
    pinMode(ioPin, INPUT);
    pinMode(clkPin, INPUT);
    pinMode(csPin, OUTPUT);

    //Turn off chip select
    digitalWrite(csPin, HIGH);

    delayMicroseconds(20);

    //Start up in accordance with procedure in datasheet.
    setup();
    
}

void MCP2035::setup()
{
    //Initial settings from mastery of datasheet.  Some register settings must be enabled from beginning, check register docs.


    writeRegister(CONFIG_REGISTER_0, 0b10101110);
    delay(1);

    writeRegister(CONFIG_REGISTER_1, 0b11000001);
    delay(1);

    writeRegister(CONFIG_REGISTER_2, 0b00000000);
    delay(1);

    writeRegister(CONFIG_REGISTER_3, 0b00000000);
    delay(1);

    writeRegister(CONFIG_REGISTER_4, 0b00000000);
    delay(1);

    writeRegister(CONFIG_REGISTER_5, 0b01000000);
    delay(1);

    updateColumnParity();
    delay(1);

        uint8_t data;

        data = readRegister(CONFIG_REGISTER_0);
        SerialUSB.println(data, 2);

        data = readRegister(CONFIG_REGISTER_1);
        SerialUSB.println(data, 2);

        data = readRegister(CONFIG_REGISTER_2);
        SerialUSB.println(data, 2);

        data = readRegister(CONFIG_REGISTER_3);
        SerialUSB.println(data, 2);

        data = readRegister(CONFIG_REGISTER_4);
        SerialUSB.println(data, 2);

        data = readRegister(CONFIG_REGISTER_5);
        SerialUSB.println(data, 2);

        data = readRegister(COLUMN_PARITY_REGISTER);
        SerialUSB.println(data, 2);

        SerialUSB.println();

        data = readRegister(STATUS_REGISTER);
        SerialUSB.println(data, 2);


    while(1)
    {

        // delay(100);
    }
    



}

void MCP2035::enableModulationClamp()
{
    
}

void MCP2035::disableModulationClamp()
{
    
}

void MCP2035::goToSleep()
{
    
}

void MCP2035::AGCPreserveEnable()
{
    
}

void MCP2035::softReset()
{
    
}

void MCP2035::setOutputEnableHighTime(uint8_t timeSetting)
{
    
}

void MCP2035::setOutputEnableLowTime(uint8_t timeSetting)
{
    
}

uint8_t MCP2035::checkAlert()
{
    uint8_t alertStatus = 0;
    return alertStatus;
}

void MCP2035::setInputChannelStatus(uint8_t status)
{
    
}

void MCP2035::setLFDATAOutputType(uint8_t type)
{
    
}

void MCP2035::setTuningCap(uint8_t valueInPicoFarads)
{
    //Needs to:
    //read register that contains this setting
    //compute new register byte that includes desired setting
    //recompute row parity bit
    //re-write the entire register, including row parity bit
    //adjust column parity register accordingly
    //check to make sure that there aren't any parity errors
}

void MCP2035::setRSSIMosfetStatus(uint8_t status)
{
    
}

void MCP2035::setCarrierClockDivide(uint8_t setting)
{
    
}

void MCP2035::setSensitivityReduction(uint8_t setting)
{
    
}

void MCP2035::setDemodulatorOutput(uint8_t status)
{
    
}

void MCP2035::setMinimumModulationDepth(uint8_t depth)
{
    
}

uint16_t MCP2035::checkColumnParityRegister()
{
    uint16_t columnParityStatus = 0;
    return columnParityStatus;
}

uint8_t MCP2035::isInputChannelActive()
{
    uint8_t inputChannelStatus = 0;
    return inputChannelStatus;
}

uint8_t MCP2035::isAGCActive()
{
    uint8_t AGCChannelStatus = 0;
    return AGCChannelStatus;
}

uint8_t MCP2035::didInputCauseWakeUp()
{
    uint8_t didInputCauseWakeup = 0;
    return didInputCauseWakeup;
}

uint8_t MCP2035::checkAlarmTimeout()
{
    uint8_t alarmTimeoutStatus = 0;
    return alarmTimeoutStatus;
}

uint8_t MCP2035::checkParity()
{
    uint8_t parityStatus = 0;
    return parityStatus;
}

void MCP2035::sendCommand(uint8_t command)
{
    //Bit-bang three-wire SPI, ahoy!
    //See page 45 of datasheet.

    //Drive CLK/!ALERT pin low so no false clocks after !CS drops
    pinMode(clkPin, OUTPUT);
    digitalWrite(clkPin, LOW);

    //Drop !CS
    digitalWrite(csPin, LOW);

    //Change IO pin to output so can clock data to MCP2035
    pinMode(ioPin, OUTPUT);

    //Clock command out MSB first
    for(uint8_t i = 3; i > 0; i--)
    {
        //Write the bit
        //if( command >> (i - 1) & 0x01) == 1)
        if( ((command >> (i - 1)) & 0x01 ) == 1)
        {
            digitalWrite(ioPin, HIGH); 
        } else {
            digitalWrite(ioPin, LOW);
        }

        //Wait
        delayMicroseconds(SPI_DELAY);

        //Raise CLK
        digitalWrite(clkPin, HIGH);

        //Give MCP2035 some time to read bit
        delayMicroseconds(SPI_DELAY);

        //Lower CLK
        digitalWrite(clkPin, LOW);

        //Wait, on to the next bit
        delayMicroseconds(SPI_DELAY);
    }

    //Clock dummy bits out
    for(uint8_t i = 13; i > 0; i--)
    {
        //Write the bit
        digitalWrite(ioPin, LOW); 

        //Wait
        delayMicroseconds(SPI_DELAY);

        //Raise CLK
        digitalWrite(clkPin, HIGH);

        //Give MCP2035 some time to read bit
        delayMicroseconds(SPI_DELAY);

        //Lower CLK
        digitalWrite(clkPin, LOW);

        //Wait, on to the next bit
        delayMicroseconds(SPI_DELAY);
    }

    //Return IO pin to input so can receive data from MCP2035
    pinMode(ioPin, INPUT);

    //Raise !CS to complete write
    digitalWrite(csPin, HIGH);

    //Return CLK/!ALERT pin to input
    pinMode(clkPin, INPUT);
}

void MCP2035::writeRegister(uint8_t addressToWrite, uint8_t data)
{
    //Uses SPI to write all bits into the desired register
    //See page 45 of datasheet.

    //Build the packet
    uint16_t packet = 0;

    //Add command
    packet = packet | (WRITE_REGISTER << 13);

    //Add address
    packet = packet | (addressToWrite << 9);

    //Add data
    packet = packet | (data << 1);

    //Add row parity bit
    packet = packet | (computeRowParity(data) );



    //Bit-bang three-wire SPI, ahoy!

    //Drive CLK/!ALERT pin low so no false clocks after !CS drops
    pinMode(clkPin, OUTPUT);
    digitalWrite(clkPin, LOW);

    //Drop !CS
    digitalWrite(csPin, LOW);

    //Change IO pin to output so can clock data to MCP2035
    pinMode(ioPin, OUTPUT);

    //Clock packet out MSB first
    for(uint8_t i = 16; i > 0; i--)
    {
        //Write the bit
        if( ((packet >> (i - 1)) & 0x01 ) == 1)
        {
            digitalWrite(ioPin, HIGH); 
        } else {
            digitalWrite(ioPin, LOW);
        }

        //Wait
        delayMicroseconds(SPI_DELAY);

        //Raise CLK
        digitalWrite(clkPin, HIGH);

        //Give MCP2035 some time to read bit
        delayMicroseconds(SPI_DELAY);

        //Lower CLK
        digitalWrite(clkPin, LOW);

        //Wait, on to the next bit
        delayMicroseconds(SPI_DELAY);
    }

    //Return IO pin to input so can receive data from MCP2035
    pinMode(ioPin, INPUT);

    //Raise !CS to complete write
    digitalWrite(csPin, HIGH);

    //Return CLK/!ALERT pin to input
    pinMode(clkPin, INPUT);



}

uint8_t MCP2035::readRegister(uint8_t addressToRead)
{
    //Uses SPI to write all bits into the desired register
    //See page 46 of datasheet.

    uint16_t receivedPacket = 0;
    uint8_t data = 0;
    uint8_t parityBit;

    //Bit-bang three-wire SPI, ahoy!

    //Drive CLK/!ALERT pin low so no false clocks after !CS drops
    pinMode(clkPin, OUTPUT);
    digitalWrite(clkPin, LOW);

    //Drop !CS
    digitalWrite(csPin, LOW);

    //Change IO pin to output so can clock data to MCP2035
    pinMode(ioPin, OUTPUT);

    //READ sequence is command, address, and dummy data

    //Build the packet
    uint16_t packet = 0;

    //Add command
    packet = packet | (READ_REGISTER << 13);

    //Add address
    packet = packet | (addressToRead << 9);

    //Add dummy data
    packet = packet | (0xAA << 1);

    //Add row parity bit
    packet = packet | (computeRowParity(0xAA) );


    //Clock out READ packet
    for(uint8_t i = 16; i > 0; i--)
    {
        //Write the bit
        if( ((packet >> (i - 1)) & 0x01 ) == 1)
        {
            digitalWrite(ioPin, HIGH); 
        } else {
            digitalWrite(ioPin, LOW);
        }

        //Wait
        delayMicroseconds(SPI_DELAY);

        //Raise CLK
        digitalWrite(clkPin, HIGH);

        //Give MCP2035 some time to read bit
        delayMicroseconds(SPI_DELAY);

        //Lower CLK
        digitalWrite(clkPin, LOW);

        //Wait, on to the next bit
        delayMicroseconds(SPI_DELAY);
    }

    //Return IO pin to input so can receive data from MCP2035
    pinMode(ioPin, INPUT);

    //Raise !CS to complete write
    digitalWrite(csPin, HIGH);

    //Time to listen for data

    delayMicroseconds(SPI_DELAY);

    //Here we go again
    digitalWrite(csPin, LOW);

    //Clock in 16 bits.  7 dummy, 8 configuration, and 1 parity
    for(uint8_t i = 16; i > 0; i--)
    {
        uint8_t currentBit;

        //Raise CLK
        digitalWrite(clkPin, HIGH);

        //Read the bit and add it to packet
        currentBit = digitalRead(ioPin);

        if(currentBit == 1)
        {
            receivedPacket = receivedPacket | (1 << (i - 1));
        }

        //Lower CLK to get ready for another bit
        digitalWrite(clkPin, LOW);

        //Wait for it to be set
        delayMicroseconds(SPI_DELAY);
    }

    if(DEBUG == 1)
    {
        SerialUSB.print("Received Packet: ");
        SerialUSB.println(receivedPacket, 2); 
    }

    //Raise !CS to complete read
    digitalWrite(csPin, HIGH);

    //Return CLK/!ALERT pin to input
    pinMode(clkPin, INPUT);


    //Parse the received packet
    parityBit = receivedPacket & 1;

    if(DEBUG == 1)
    {
        SerialUSB.print("Parity bit: ");
        SerialUSB.println(parityBit, 2);
    }


    for(uint8_t i = 1; i < 9; i++)
    {
        if( (receivedPacket >> i) & 1)
        {
           data = data | (1 << (i -1) ); 
        }
        
    }

    if(DEBUG == 1)
    {
        SerialUSB.print("Data: ");
        SerialUSB.println(data, 2);
        SerialUSB.println();
    }

    //Return the data (or parity error)
    if(parityBit == computeRowParity(data))
    {
        return data;
    } else {
        return -1;
    }
}

//See MCP2035 data sheet page 49, note 2 
uint8_t MCP2035::computeRowParity(uint16_t data)
{
    //Parity bit needs to be set such that the row contains an odd number of set bits.
    uint8_t parityBit = 1;

    for(uint8_t i = 16; i > 0; i--)
    {
        if( ((data >> (i - 1)) & 1 ) == 1)
        {
            parityBit = !parityBit; 
        }
    }
    return parityBit;
}

////See MCP2035 data sheet page 55
void MCP2035::updateColumnParity()
{
        uint16_t columnParityRegister = 0;
        uint8_t columnParaties[8] = {1, 1, 1, 1, 1, 1, 1, 1};

        uint8_t configRegisters[6];

        //Read register data
        configRegisters[0] = readRegister(CONFIG_REGISTER_0);
        configRegisters[1] = readRegister(CONFIG_REGISTER_1);
        configRegisters[2] = readRegister(CONFIG_REGISTER_2);
        configRegisters[3] = readRegister(CONFIG_REGISTER_3);
        configRegisters[4] = readRegister(CONFIG_REGISTER_4);
        configRegisters[5] = readRegister(CONFIG_REGISTER_5);

        //Compute column parity array
        for(uint8_t i = 8; i > 0; i--)
        {

            for(uint8_t j = 0; j < 6; j++)
            {
                if( (configRegisters[j] >> (i - 1) & 1 ) == 1)
                {
                    columnParaties[i-1] = !columnParaties[i-1];
                }
            }
        }

        //Convert column parity array back into an 8-bit number
        for(uint8_t i = 8; i > 0; i--)
        {
            //SerialUSB.print(columnParaties[i-1]);
            //SerialUSB.print( (columnParaties[i-1] << i) & 1);

            if( columnParaties[i-1] == 1)
            {
                columnParityRegister = columnParityRegister | (1 << (i - 1));
            }
             
        }

        // SerialUSB.println();
        // SerialUSB.println(columnParityRegister, 2);

        //Write column parity register
        writeRegister(COLUMN_PARITY_REGISTER, columnParityRegister);
}

