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
	spiPort = NULL;
}

void MCP2035::begin(HardwareSPI* port)
{
    spiPort = port;

	//Needs to started up in accordance with procedure in datasheet.
	setup();
	
}

void MCP2035::setup()
{
	//Initial settings from mastery of datasheet.  Some register settings must be enabled from beginning, check register docs.
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

void MCP2035::writeRegister(uint8_t addressToWrite, uint16_t data)
{
	//Uses SPI port to write all 9 bits into the desired register
}

uint8_t MCP2035::readRegister(uint8_t addressToRead)
{
	//placeholder, but notice 16 bit variable since registers contain 9 bits
	uint16_t readValue = 0;
	return readValue;
}

//See MCP2035 data sheet page 49, note 2 
uint8_t MCP2035::computeRowParity(uint8_t data)
{
	uint8_t parityBit = 0;
	return parityBit;
}

////See MCP2035 data sheet page 55
void MCP2035::updateColumnParity()
{
	//remember to do all 9 bits
}

