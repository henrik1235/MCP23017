/**************************************************************************
  This Arduino library is for the I2C port expander MCP23017.
  
  The library for the MCP23017 works as follows:
  - Operates in byte mode (not in sequential mode (IOCON.SEQOP)
  - with IOCON.BANK = 0 
    => (address pointer toggles between associated A/B register pairs)

		Address			Address			Access to:		Define
	IOCON.BANK = 1	IOCON.BANK = 0
		00h				00h				IODIRA			MCP_IODIRA
		10h				01h				IODIRB			MCP_IODIRB
		01h				02h				IPOLA			MCP_IPOLA
		11h				03h				IPOLB			MCP_IPOLB
		02h				04h				GPINTENA		MCP_GPINTENA
		12h				05h				GPINTENB		MCP_GPINTENB
		03h				06h				DEFVALA			MCP_DEFVALA
		13h				07h				DEFVALB			MCP_DEFVALB
		04h				08h				INTCONA			MCP_INTCONA
		14h				09h				INTCONB			MCP_INTCONB
		05h				0Ah				IOCON			MCP_IOCON
		15h				0Bh				IOCON			MCP_IOCON
		06h				0Ch				GPPUA			MCP_GPPUA
		16h				0Dh				GPPUB			MCP_GPPUB
		07h				0Eh				INTFA			MCP_INTFA
		17h				0Fh				INTFB			MCP_INTFB
		08h				10h				INTCAPA			MCP_INTCAPA
		18h				11h				INTCAPB			MCP_INTCAPB
		09h				12h				GPIOA			MCP_GPIOA
		19h				13h				GPIOB			MCP_GPIOB
		0Ah				14h				OLATA			MCP_OLATA
		1Ah				15h				OLATB			MCP_OLATB
		
	This library reads all data as values in 16 bit mode. (HighByte PortB, LowByte PortA)
	
  Author Rainer Wieland
  Modifications by Henrik Kunzelmann (2017)
  
  BSD license, all text above must be included in any redistribution
**************************************************************************/


#ifndef _NewMCP23017_H_
#define _NewMCP23017_H_

#include <Arduino.h>
#include <Wire.h>

class MCP23017 {
public:
	uint8_t begin();
	uint8_t begin(uint8_t addr);

	uint16_t getPinModes();
	uint8_t setPinsInput(uint16_t pins);
	uint8_t setPinsOutput(uint16_t pins);

	boolean getPullUp(uint8_t pin);
	uint16_t setPullUp(uint8_t pin, boolean enable);

	boolean getPolarity(uint8_t pin);
	uint16_t setPolarity(uint8_t pin, boolean invert);

	uint16_t readGPIO();
	uint8_t  writeGPIO(uint16_t value);

	// Arduino friendly methods
	void pinMode(uint8_t pin, uint8_t mode);

	uint8_t digitalRead(uint8_t pin);
	void digitalWrite(uint8_t pin, uint8_t value);
private:
	uint8_t addr;

	uint16_t readRegisters(uint8_t reg);
	uint8_t  writeRegisters(uint8_t reg, uint16_t value);
};

#define MCP23017_BASEADDRESS ((uint8_t)0x20u)
#define MCP23017_ADDRESS_MAX ((uint8_t)0x07u)

//REGISTER ADRESSES with ICON.BANK = 0
#define	MCP_IODIRA		0x00
#define	MCP_IODIRB		0x01
#define	MCP_IPOLA		0x02
#define	MCP_IPOLB		0x03
#define	MCP_GPINTENA	0x04
#define	MCP_GPINTENB	0x05

#define	MCP_DEFVALA		0x06
#define	MCP_DEFVALB		0x07
#define	MCP_INTCONA		0x08
#define	MCP_INTCONB		0x09
#define	MCP_IOCONA		0x0A
#define	MCP_IOCONB		0x0B

#define	MCP_GPPUA		0x0C
#define	MCP_GPPUB		0x0D

#define	MCP_INTFA		0x0E
#define	MCP_INTFB		0x0F
#define	MCP_INTCAPA		0x10
#define	MCP_INTCAPB		0x11

#define	MCP_GPIOA		0x12
#define MCP_GPIOB		0x13
#define MCP_OLATA		0x14
#define	MCP_OLATB		0x15

#define MCP_ERROR		0x1B // Register not in use

#define MCP_PIN_COUNT   16
#endif
