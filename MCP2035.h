/******************************************************************************
Driver for Microchip MCP2035 Bodycom IC

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

#ifndef _MCP2035_H_
#define _MCP2035_H_

#include <wirish/wirish.h>
#include <stdint.h>

#define DEBUG 0

#define SPI_DELAY 1

//SPI Commands
//See MCP2035 datahseet, page 47
#define MODULATION_CLAMP_ENABLE 0b000
#define MODULATION_CLAMP_DISABLE 0b001
#define GO_TO_SLEEP 0b010
#define AGC_PRESERVE_ENABLE 0b011
#define AGC_PRESERVE_DISABLE 0b100
#define SOFT_RESET 0b101
#define READ_REGISTER 0b110
#define WRITE_REGISTER 0b111

//Config Register 0
//See MCP2035 datahseet, page 50

//Address
#define CONFIG_REGISTER_0 0b0000

//Settings
#define OUTPUT_ENABLE_FILTER_HIGH_DISABLED 0b00
#define OUTPUT_ENABLE_FILTER_HIGH_1MS 0b01
#define OUTPUT_ENABLE_FILTER_HIGH_2MS 0b10
#define OUTPUT_ENABLE_FILTER_HIGH_4MS 0b11

#define OUTPUT_ENABLE_FILTER_LOW_DISABLED 0b00
#define OUTPUT_ENABLE_FILTER_LOW_1MS 0b01
#define OUTPUT_ENABLE_FILTER_LOW_2MS 0b10
#define OUTPUT_ENABLE_FILTER_LOW_4MS 0b11

#define ALERT_TRIGGERED_BY_PARITY_ERROR_OR_ALARM_TIMER 0b1
#define ALERT_TRIGGERED_BY_PARITY_ERROR 0b0

#define INPUT_CHANNEL_DISABLE 0b1
#define INPUT_CHANNEL_ENABLE 0b0

//Config Register 1
//See MCP2035 datahseet, page 51

//Address
#define CONFIG_REGISTER_1 0b0001

//Settings
#define LFDATA_OUTPUT_DEMODULATED 0b00
#define LFDATA_OUTPUT_CARRIER 0b01
#define LFDATA_OUTPUT_RSSI 0b10

//Config Register 2
//See MCP2035 datahseet, page 51

//Address
#define CONFIG_REGISTER_2 0b0010

//Settings
#define RSSI_PULL_DOWN_ON 0b1
#define RSSI_PULL_DOWN_OFF 0b0

#define CARRIER_CLOCK_DIVIDE_4 0b1
#define CARRIER_CLOCK_DIVIDE_1 0b0

//Config Register 3 
//See MCP2035 datahseet, page 52

//Address
#define CONFIG_REGISTER_3 0b0011

//Config Register 4 
//See MCP2035 datahseet, page 53

//Address
#define CONFIG_REGISTER_4 0b0100

//Settings
#define INPUT_SENSITIVITY_REDUCTION_0_DB 0b0000
#define INPUT_SENSITIVITY_REDUCTION_2_DB 0b0001
#define INPUT_SENSITIVITY_REDUCTION_4_DB 0b0010
#define INPUT_SENSITIVITY_REDUCTION_6_DB 0b0011
#define INPUT_SENSITIVITY_REDUCTION_8_DB 0b0100
#define INPUT_SENSITIVITY_REDUCTION_10_DB 0b0101
#define INPUT_SENSITIVITY_REDUCTION_12_DB 0b0110
#define INPUT_SENSITIVITY_REDUCTION_14_DB 0b0111
#define INPUT_SENSITIVITY_REDUCTION_16_DB 0b1000
#define INPUT_SENSITIVITY_REDUCTION_18_DB 0b1001
#define INPUT_SENSITIVITY_REDUCTION_20_DB 0b1010
#define INPUT_SENSITIVITY_REDUCTION_22_DB 0b1011
#define INPUT_SENSITIVITY_REDUCTION_24_DB 0b1100
#define INPUT_SENSITIVITY_REDUCTION_26_DB 0b1101
#define INPUT_SENSITIVITY_REDUCTION_28_DB 0b1110
#define INPUT_SENSITIVITY_REDUCTION_30_DB 0b1111

//Config Register 5
//See MCP2035 datahseet, page 54

//Address
#define CONFIG_REGISTER_5 0b0101

//Settings
#define DEMOD_OUTPUT_AGC_DEPENDENT_ENABLE 0b1
#define DEMOD_OUTPUT_AGC_DEPENDENT_DISABLE 0b0

#define MIN_MOD_DEPTH_33_PCT 0b00
#define MIN_MOD_DEPTH_60_PCT 0b01
#define MIN_MOD_DEPTH_14_PCT 0b10
#define MIN_MOD_DEPTH_8_PCT 0b11

//Column Parity Bit Register 6
//See MCP2035 datahseet, page 55

//Address
#define COLUMN_PARITY_REGISTER 0b0110

//Status Register 7 - ALL OF THESE ARE READ ONLY
//See MCP2035 datahseet, page 56

//Address
#define STATUS_REGISTER 0b0111

//Settings
#define INPUT_CHANELL_ACTIVE 0b1
#define INPUT_CHANNEL_INACTIVE 0b0

#define AGC_ACTIVE 0b1
#define AGC_INACTIVE 0b0

#define INPUT_CHANNEL_CAUSED_WAKEUP 0b1
#define INPUT_CHANNEL_NO_WAKEUP 0b0

#define ALARM_TIME_OUT 0b1
#define NO_ALARM_TIME_OUT 0b0

#define PARITY_ERROR 0b1
#define NO_PARITY_ERROR 0b0

class MCP2035 {
    
    //HardwareSPI* spiPort;
    uint8_t ioPin;
    uint8_t clkPin;
    uint8_t csPin;
    
public:

    MCP2035();
    
    //Gonna try my own three wire SPI bit-bang implementation
    void begin(uint8_t io, uint8_t clk, uint8_t cs);

    void setup();   

    void enableModulationClamp();
    void disableModulationClamp();
    void goToSleep();
    void AGCPreserveEnable();
    void softReset();
    
    void setOutputEnableHighTime(uint8_t timeSetting);
    void setOutputEnableLowTime(uint8_t timeSetting);
    uint8_t checkAlert();
    void setInputChannelStatus(uint8_t status);
    
    void setLFDATAOutputType(uint8_t type);
    void setTuningCap(uint8_t valueInPicoFarads);
    void setRSSIMosfetStatus(uint8_t status);
    void setCarrierClockDivide(uint8_t setting);
    
    void setSensitivityReduction(uint8_t setting);
    
    void setDemodulatorOutput(uint8_t status);
    void setMinimumModulationDepth(uint8_t depth);
    uint16_t checkColumnParityRegister();
    
    uint8_t isInputChannelActive();
    uint8_t isAGCActive();
    uint8_t didInputCauseWakeUp();
    uint8_t checkAlarmTimeout();
    uint8_t checkParity();


private:
    void sendCommand(uint8_t command);
    void writeRegister(uint8_t addressToWrite, uint8_t data);
    uint8_t readRegister(uint8_t addressToRead);
    uint8_t computeRowParity(uint16_t data);
    void updateColumnParity();

};

#endif  /* _MCP2035_H_ */
