/**************************************************************************
  This library is for the I2C-Portexpander MCP23017
  In this library the MCP23017:
  - Operate in Byte mode (not in Sequential mode (IOCON.SEQOP)
  - (Byte mode with IOCON.BANK = 0
	 => ( address  pointer do toggle between associated A/B register pairs)
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
	this library read all Data as values in 16 bit HighByte PortB, LowByte PortA
  Author Rainer Wieland
  BSD license, all text above must be included in any redistribution
**************************************************************************/

#include "MCP23017.h"

uint8_t MCP23017::begin() {
	return begin(0);
}

uint8_t MCP23017::begin(uint8_t mcpDeviceAddr) {
	addr = MCP23017_BASEADDRESS | min(mcpDeviceAddr, MCP23017_ADDRESS_MAX);

	// Set all pins as input (default)
	return setPinsInput(0xFFFFu);
}

//Configuration-Routines
// +---Port B--- +  +---Port A--- +
// |             |  |             |
// 0 0 0 0 0 0 0 0  0 0 0 0 0 0 0 0 XXX
// 0 0 0 0 0 0 0 0  0 0 0 0 0 0 0 1 PIN_0
// 0 0 0 0 0 0 0 0  0 0 0 0 0 0 1 0 PIN_1
// 0 0 0 0 0 0 0 0  1 0 0 0 0 0 0 0 PIN_7
// 0 0 0 0 0 0 0 1  0 0 0 0 0 0 0 0 PIN_8
// 1 0 0 0 0 0 0 0  0 0 0 0 0 0 0 0 PIN_15

uint16_t MCP23017::readRegisters(uint8_t reg) {
	// read a Register in as 16Bit
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(addr, 2u);
	uint16_t value = Wire.read() | (Wire.read() << 8);
	return value;
}

uint8_t MCP23017::writeRegisters(uint8_t reg, uint16_t value) {
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write((uint8_t)(value & 0xFF));
	Wire.write((uint8_t)(value >> 8));
	return Wire.endTransmission();
}

// I/O DIRECTION REGISTER
uint16_t MCP23017::getPinModes() {
	/*************************************************************/
	/* Controls the direction of the data I/O.                   */
	/* When a bit is set, the corresponding pin becomes an       */
	/* input.  When  a  bit  is  clear,  the  corresponding  pin */
	/* becomes an output.                                        */
	/*************************************************************/
	return readRegisters(MCP_IODIRA);
}

uint8_t MCP23017::setPinsInput(uint16_t pins) {
	/*************************************************************/
	/* Controls the direction of the data I/O.                   */
	/* When a bit is set, the corresponding pin becomes an       */
	/* input.  When  a  bit  is  clear,  the  corresponding  pin */
	/* becomes an output.                                        */
	/*************************************************************/
	return writeRegisters(MCP_IODIRA, readRegisters(MCP_IODIRA) | pins);
}

uint8_t MCP23017::setPinsOutput(uint16_t pins) {
	/*************************************************************/
	/* Controls the direction of the data I/O.                   */
	/* When a bit is set, the corresponding pin becomes an       */
	/* input.  When  a  bit  is  clear,  the  corresponding  pin */
	/* becomes an output.                                        */
	/*************************************************************/
	return writeRegisters(MCP_IODIRA, readRegisters(MCP_IODIRA) & ~pins);
}

// INPUT POLARITY REGISTER
boolean MCP23017::getPolarity(uint8_t pin) {
	return (readRegisters(MCP_IPOLA) & BIT(pin) != 0);
}

uint16_t MCP23017::setPolarity(uint8_t pin, boolean invert) {
	/**************************************************************/
	/* This register allows the user to configure the polarity on */
	/* the corresponding GPIO port bits.                          */
	/* If a bit is set, the corresponding GPIO register bit will  */
	/* reflect the inverted value on the pin.                     */
	/**************************************************************/
	uint16_t polarity = readRegisters(MCP_IPOLA);

	if (invert)
		polarity |= BIT(pin);
	else
		polarity &= ~BIT(pin);

	writeRegisters(MCP_IPOLA, polarity);
	return polarity;
}

// INTERRUPT-ON-CHANGE CONTROL REGISTER
/********************************************************************/
/* The  GPINTEN  register  controls  the  interrupt-on-change       */
/* feature for each pin.                                            */
/* If  a  bit  is  set,  the  corresponding  pin  is  enabled  for  */
/* interrupt-on-change.  The  DEFVAL  and  INTCON                   */
/* registers  must  also  be  configured  if  any  pins  are        */
/* enabled for interrupt-on-change.                                 */
/********************************************************************/

// DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE
/********************************************************************/
/* The  default  comparison  value  is  configured  in  the         */
/* DEFVAL  register.  If  enabled  (via  GPINTEN  and               */
/* INTCON) to compare against the DEFVAL register, an               */
/* opposite  value  on  the  associated  pin  will  cause  an       */
/* interrupt to occur.                                              */
/********************************************************************/

// INTERRUPT CONTROL REGISTER
/********************************************************************/
/* The INTCON register controls how the associated pin              */
/* value is compared for the interrupt-on-change feature.           */
/* If a bit is set, the corresponding I/O pin is compared           */
/* against the associated bit in the DEFVAL register. If a          */
/* bit value is clear, the corresponding I/O pin is compared        */
/* against the previous value.                                      */
/********************************************************************/

// INTERRUPT FLAG REGISTER
/********************************************************************/
/* The INTF register reflects the interrupt condition on the        */
/* port pins of any pin that is enabled for interrupts via the      */
/* GPINTEN  register.  A  ‘set’  bit  indicates  that  the          */
/* associated pin caused the interrupt.                             */
/* This register is ‘read-only’. Writes to this register will be    */
/* ignored.                                                         */
/********************************************************************/

// INTERRUPT CAPTURE REGISTER
/********************************************************************/
/* The INTCAP register captures the GPIO port value at              */
/* the  time  the  interrupt  occurred.  The  register  is  ‘read   */
/* only’ and is updated only when an interrupt occurs. The          */
/* register  will  remain  unchanged  until  the  interrupt  is     */
/* cleared via a read of INTCAP or GPIO.                            */
/********************************************************************/


// IOCON – I/O EXPANDER CONFIGURATION REGISTER 
/************************************************************************************************************/
/* bit7	BANK: Controls how the registers are addressed 														*/
/* 		1 = The registers associated with each port are separated into different banks						*/
/* 		0 = The registers are in the same bank (addresses are sequential)									*/
/* bit6 MIRROR: INT Pins Mirror bit														  					*/
/* 		1 = The INT pins are internally connected														  	*/
/* 		0 = The INT pins are not connected. INTA is associated with PortA and INTB is associated with PortB	*/
/* bit5 SEQOP: Sequential Operation mode bit.														  		*/
/* 		1 = Sequential operation disabled, address pointer does not increment.						 		*/
/* 		0 = Sequential operation enabled, address pointer increments.								 		*/
/* bit4 DISSLW: Slew Rate control bit for SDA output.												 		*/
/* 		1 = Slew rate disabled.														  				 		*/
/* 		0 = Slew rate enabled.														  				 		*/
/* bit3 HAEN: Hardware Address Enable bit (MCP23S17 only).											 		*/
/* 		Address pins are always enabled on MCP23017.												 		*/
/* 		1 = Enables the MCP23S17 address pins.														 		*/
/* 		0 = Disables the MCP23S17 address pins.														 		*/
/* bit2 ODR: This bit configures the INT pin as an open-drain output.								 		*/
/* 		1 = Open-drain output (overrides the INTPOL bit).											 		*/
/* 		0 = Active driver output (INTPOL bit sets the polarity).									 		*/
/* bit1 INTPOL: This bit sets the polarity of the INT output pin.									 		*/
/* 		1 = Active-high.														  					 		*/
/* 		0 = Active-low.														  						 		*/
/* bit 0 Unimplemented: Read as ‘0’.														  		 		*/
/************************************************************************************************************/


//PULL-UP RESISTOR CONFIGURATION REGISTER
boolean MCP23017::getPullUp(uint8_t pin) {
	/************************************************************/
	/* The GPPU register controls the pull-up resistors for the */
	/* port pins. If a bit is set and the corresponding pin is  */
	/* configured as an input, the corresponding port pin is    */
	/* internally pulled up with a 100 kOhm resistor.           */
	/************************************************************/
	return (readRegisters(MCP_GPPUA) & BIT(pin)) != 0;
}

/********************************************************************/
/* The GPPU register controls the pull-up resistors for the         */
/* port pins. If a bit is set and the corresponding pin is          */
/* configured as an input, the corresponding port pin is            */
/* internally pulled up with a 100 kOhm resistor.                   */
/********************************************************************/
uint16_t MCP23017::setPullUp(uint8_t pin, boolean enable) {
	if (pin >= MCP_PIN_COUNT)
		return 0;

	/************************************************************/
	/* The GPPU register controls the pull-up resistors for the */
	/* port pins. If a bit is set and the corresponding pin is  */
	/* configured as an input, the corresponding port pin is    */
	/* internally pulled up with a 100 kOhm resistor.           */
	/************************************************************/
	uint16_t pullUps = readRegisters(MCP_GPPUA);

	//HIGH => PullUp 
	//LOW  => No PullUp
	if (enable)
		pullUps |= BIT(pin);
	else
		pullUps &= ~BIT(pin);

	// write the new PullUpValue
	writeRegisters(MCP_GPPUA, pullUps);

	// read the new PullUpValue
	return readRegisters(MCP_GPPUA);
}

//PORT REGISTER
/********************************************************************/
/* The  GPIO  register  reflects  the  value  on  the  port.        */
/* Reading from this register reads the port. Writing to this       */
/* register modifies the Output Latch (OLAT) register.              */
/********************************************************************/
uint16_t MCP23017::readGPIO() {
	// read GPIOA and GPIOB in 16Bit Mode
	return readRegisters(MCP_GPIOA);
}

uint8_t MCP23017::writeGPIO(uint16_t value) {
	/**************************************************************/
	/* Writing to the GPIOn register actually causes a write to   */
	/* the  latches  (OLATn).  Writing  to  the  OLATn  register  */
	/* forces the associated output drivers to drive to the level */
	/* in  OLATn.  Pins  configured  as  inputs  turn  off  the   */
	/* associated output driver and put it in high-impedance.     */
	/**************************************************************/
	/* => Write only on GPIOs needed, Latch not necesarry         */
	/**************************************************************/
	return writeRegisters(MCP_GPIOA, value);
}

void MCP23017::pinMode(uint8_t pin, uint8_t mode) {
	if (pin >= MCP_PIN_COUNT)
		return;

	if (mode == INPUT_PULLUP) {
		setPinsInput(BIT(pin));
		setPullUp(pin, true);
	}
	else if (mode == INPUT) {
		setPinsInput(BIT(pin));
		setPullUp(pin, false);
	}
	else if (mode == OUTPUT) 
		setPinsOutput(BIT(pin));
}

void MCP23017::digitalWrite(uint8_t pin, uint8_t value) {
	if (pin >= MCP_PIN_COUNT)
		return;

	uint16_t gpioab = readGPIO();

	// set the pin
	if (value)
		gpioab |= BIT(pin);
	else
		gpioab &= ~BIT(pin);

	writeGPIO(gpioab);
}

uint8_t MCP23017::digitalRead(uint8_t pin) {
	if (pin >= MCP_PIN_COUNT)
		return LOW;

	if (readGPIO() & BIT(pin))
		return HIGH;
	else
		return LOW;
}

