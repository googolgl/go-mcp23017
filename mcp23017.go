/*
Copyright (c) 2021
Author: Pavlo Lytvynoff

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN i2cECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

package mcp23017

import (
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"

	"github.com/googolgl/go-i2c"
)

type (
	// RegAddres is register addresses
	regAddres byte
	// RegOption is configuration register bits
	regOption byte
	// Pins set valid pins, max - 16
	Pins []string
	// MCP23017 is a Driver for the MCP23017 16-Bit I/O Expander with Serial Interface
	MCP23017 struct {
		i2c *i2c.Options
		//name string
		sync.Mutex
	}
	//ModeBanks - structure
	ModeBanks struct {
		m    *MCP23017
		a    bool
		b    bool
		xFF  bool
		pins Pins
	}
)

const (
	// DefI2CAdr - default address for controller (0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27)
	DefI2CAdr byte = 0x20
)

const (
	// IODIR - I/O direction register
	IODIR regAddres = iota // 0x00
	// IPOL - Input polarity port register (0x01)
	IPOL
	// GPINTEN - Interrupt-on-change pins (0x02)
	GPINTEN
	// DEFVAL - Default value register (0x03)
	DEFVAL
	// INTCON - Interrupt-on-change control register (0x04)
	INTCON
	// IOCON - I/O expander configuration register (0x05)
	IOCON
	// GPPU - GPIO pull-up resistor register (0x06)
	GPPU
	// INTF - Interrupt flag register (0x07)
	INTF
	// INTCAP - Interrupt captured value for port register (0x08)
	INTCAP
	// GPIO - General purpose I/O port register (0x09)
	GPIO
	// OLAT - Output latch register 0 (0x10)
	OLAT
)

const (
	// INTCC - Read as 0
	// Unimplemented: Read as 0
	INTCC regOption = 1 << iota // Bit 0

	// INTPOL - This bit sets the polarity of the INT output pin
	// 1 = Active-high
	// 0 = Active-lo
	INTPOL // Bit 1

	// ODR - This bit configures the INT pin as an open-drain output
	// 1 = Open-drain output (overrides the INTPOL bit.)
	// 0 =  Active driver output (INTPOL bit sets the polarity.)
	ODR // Bit 2

	// HAEN - Hardware Address Enable bit (MCP23S17 only)
	// 1 =  Enables the MCP23S17 address pins
	// 0 =  Disables the MCP23S17 address pins
	HAEN // Bit 3

	// DISSLW - Slew Rate control bit for SDA output
	// 1 =  Slew rate disabled
	// 0 =  Slew rate enabled
	DISSLW // Bit 4

	// SEQOP - Sequential Operation mode bit
	// 1 =  Sequential operation disabled, address pointer does not increment
	// 0 =  Sequential operation enabled, address pointer increments
	SEQOP // Bit 5

	// MIRROR - INT Pins Mirror bit
	// 1 = The INT pins are internally connected
	// 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
	MIRROR // Bit 6

	// BANK - Controls how the registers are addressed
	// 1 = The registers associated with each port are separated into different banks
	// 0 = The registers are in the same bank (addresses are sequential)
	BANK // Bit 7
)

var (
	pinBit = [8]byte{1, 2, 4, 8, 16, 32, 64, 128}
)

// AllPins - return all pins in all banks
func AllPins() Pins {
	return Pins{
		"A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7",
		"B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7",
	}
}

// New creates the new MCP23017 driver with specified i2c interface and options
func New(i2c *i2c.Options, opt ...regOption) (*MCP23017, error) {
	adr := i2c.GetAddr()
	if adr == 0 {
		return nil, fmt.Errorf(`I2C device is not initiated`)
	}

	mcp := &MCP23017{
		i2c: i2c,
		//name: "Controller" + fmt.Sprintf("-0x%x", adr),
	}

	iocon := BANK
	if opt != nil {
		for _, v := range opt {
			iocon |= v
		}
	}

	//in default mode IOCON.BANK = 0 and IOCON addres = 0x10
	if err := mcp.writeReg(OLAT, byte(iocon)); err != nil {
		return nil, err
	}

	// set inputs mode on all pins
	if err := mcp.writeReg(IODIR, 0xff); err != nil {
		return nil, err
	}
	if err := mcp.writeReg(IODIR|16, 0xff); err != nil {
		return nil, err
	}

	// Turn off interrupt triggers
	if err := mcp.writeReg(GPINTEN, 0x00); err != nil {
		return nil, err
	}
	if err := mcp.writeReg(GPINTEN|16, 0x00); err != nil {
		return nil, err
	}

	// Turn off pull up resistors
	if err := mcp.writeReg(GPPU, 0x00); err != nil {
		return nil, err
	}
	if err := mcp.writeReg(GPPU|16, 0x00); err != nil {
		return nil, err
	}

	return mcp, nil
}

// readReg reads and returns a register, given its address.
func (mcp *MCP23017) readReg(ra regAddres) (byte, error) {
	mcp.Lock()
	defer mcp.Unlock()
	return mcp.i2c.ReadRegU8(byte(ra))
}

// writeReg writes a register, given its address and the value to write.
func (mcp *MCP23017) writeReg(ra regAddres, val byte) error {
	mcp.Lock()
	defer mcp.Unlock()
	return mcp.i2c.WriteRegU8(byte(ra), val)
}

// Get - values by pins
func (mcp *MCP23017) Get(p Pins) (map[string]uint8, error) {
	pinLevels := make(map[string]uint8)

	uBanks, err := checkValidPins(p)
	if err != nil {
		return nil, err
	}

	var gpioA, gpioB byte

	if uBanks.a {
		gpioA, err = mcp.readReg(GPIO)
		if err != nil {
			return nil, err
		}
	}

	if uBanks.b {
		gpioB, err = mcp.readReg(GPIO | 16)
		if err != nil {
			return nil, err
		}
	}

	for _, v := range p {
		pin, _ := strconv.Atoi(v[1:])
		if strings.HasPrefix(v, "A") {
			pinLevels[v] = (gpioA >> uint8(pin)) & 1
		} else {
			pinLevels[v] = (gpioB >> uint8(pin)) & 1
		}
	}

	return pinLevels, nil
}

// Set - params
func (mcp *MCP23017) Set(p Pins) *ModeBanks {
	mBanks, err := checkValidPins(p)
	if err != nil {
		log.Fatalln(err)
		return nil
	}

	mBanks.m = mcp

	return &mBanks
}

func (mb *ModeBanks) modeSet(r, w regAddres) error {
	var gpioA, gpioB byte
	var err error

	if mb.a {
		gpioA, err = mb.m.readReg(r)
		if err != nil {
			return err
		}
	}

	if mb.b {
		gpioB, err = mb.m.readReg(r | 16)
		if err != nil {
			return err
		}
	}

	for _, v := range mb.pins {
		pin, _ := strconv.Atoi(v[1:])
		if strings.HasPrefix(v, "A") {
			if mb.xFF {
				gpioA |= pinBit[pin]
			} else {
				gpioA &^= pinBit[pin]
			}
		} else {
			if mb.xFF {
				gpioB |= pinBit[pin]
			} else {
				gpioB &^= pinBit[pin]
			}
		}
	}

	if mb.a {
		if err := mb.m.writeReg(w, gpioA); err != nil {
			return err
		}
	}

	if mb.b {
		if err := mb.m.writeReg(w|16, gpioB); err != nil {
			return err
		}
	}

	return nil
}

//INPUT - set input mode for pins
func (mb *ModeBanks) INPUT() error {
	mb.xFF = true
	return mb.modeSet(IODIR, IODIR)
}

//OUTPUT - set output mode for pins
func (mb *ModeBanks) OUTPUT() error {
	mb.xFF = false
	return mb.modeSet(IODIR, IODIR)
}

//HIGH - set high mode for pins
func (mb *ModeBanks) HIGH() error {
	mb.xFF = true
	return mb.modeSet(OLAT, OLAT)
}

//LOW - set low mode for pins
func (mb *ModeBanks) LOW() error {
	mb.xFF = false
	return mb.modeSet(OLAT, OLAT)
}

//PULLUP - turn on pull up resistors
func (mb *ModeBanks) PULLUP() error {
	mb.xFF = true
	return mb.modeSet(GPPU, GPPU)
}

//PULLDOWN - turn off pull up resistors
func (mb *ModeBanks) PULLDOWN() error {
	mb.xFF = false
	return mb.modeSet(GPPU, GPPU)
}

func checkValidPins(p Pins) (ModeBanks, error) {
	mbanks := ModeBanks{
		a: false,
		b: false,
	}

	if len(p) > 16 {
		return mbanks, fmt.Errorf("Out of range pins, max - 16")
	}

	for _, v := range p {
		for _, vv := range AllPins() {
			if strings.ToUpper(v) == vv {
				// check used banks
				if strings.HasPrefix(vv, "A") {
					mbanks.a = true
				} else {
					mbanks.b = true
				}
			}
		}
		if !mbanks.a && !mbanks.b {
			return mbanks, fmt.Errorf("Invalid pin: %v", v)
		}
	}

	mbanks.pins = p

	return mbanks, nil
}

// Close - return default settings
func (mcp *MCP23017) Close() error {
	if err := mcp.writeReg(IOCON, 0x00); err != nil {
		return err
	}
	return nil
}
