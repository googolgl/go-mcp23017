MCP23017 16-Bit I/O Expander with Serial Interface Driver
============================================================

[![GoDoc](https://godoc.org/github.com/googolgl/go-mcp23017?status.svg)](https://godoc.org/github.com/googolgl/go-mcp23017)
[![MIT License](http://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

MCP23017 is a popular controller among Arduino and Raspberry PI developers.
The 16-Bit I/O Expander with Serial Interface Driver will drive up to 16 pins over I2C with only 2 pins.
![image](https://raw.github.com/googolgl/go-mcp23017/master/mcp23017.jpg)

Here is a library written in [Go programming language](https://golang.org/) for Raspberry PI and counterparts.

Golang usage
------------


```go
package main

import (
	"log"
	"time"

	"github.com/googolgl/go-i2c"
	"github.com/googolgl/go-mcp23017"
)

func main() {
    // Create new connection to i2c-bus on 1 line with address 0x40.
    // Use i2cdetect utility to find device address over the i2c-bus
    i2c, err := i2c.New(mcp23017.DefI2CAdr, 1)
    if err != nil {
        log.Fatal(err)
    }

    mcp, err := mcp23017.New(i2c)
    if err != nil {
        log.Fatal(err)
    }

    // Sets all pins to INPUT mode
    mcp.Set(mcp23017.AllPins()).INPUT()
    
    // Gets values all INPUT pins
    v, err := mcp.Get(mcp23017.AllPins())
    if err != nil {
    	log.Printf(err.Error())
    }
    log.Println(v)

    // Turn on pull up resistors for A0, A1, A2 and B7 pins
    mcp.Set(mcp23017.Pins{"A0", "A1", "A2", "B7"}).PULLUP()

    LedPin := mcp23017.Pins{"B0", "B1", "B2", "B3"}
    
    // Sets pins to OUTPUT mode
    mcp.Set(LedPin).OUTPUT()

    // Run cycle
    for {
        if err := mcp.Set(LedPin).HIGH(); err != nil {
		log.Printf(err.Error())
	}
        time.Sleep(3 * time.Second)

        if err := mcp.Set(LedPin).LOW(); err != nil {
		log.Printf(err.Error())
	}
        time.Sleep(3 * time.Second)
    }
}
```


Getting help
------------

GoDoc [documentation](http://godoc.org/github.com/googolgl/go-mcp23017)

Installation
------------

```bash
$ go get -u github.com/googolgl/go-mcp23017
```

Troubleshooting
--------------

- *How to obtain fresh Golang installation to RPi device (either any RPi clone):*
If your RaspberryPI golang installation taken by default from repository is outdated, you may consider
to install actual golang manually from official Golang [site](https://golang.org/dl/). Download
tar.gz file containing arm64 in the name. Follow installation instructions.

- *How to enable I2C bus on RPi device:*
If you employ RaspberryPI, use raspi-config utility to activate i2c-bus on the OS level.
Go to "Interfacing Options" menu, to active I2C bus.
Probably you will need to reboot to load i2c kernel module.
Finally you should have device like /dev/i2c-1 present in the system.

- *How to find I2C bus allocation and device address:*
Use i2cdetect utility in format "i2cdetect -y X", where X may vary from 0 to 5 or more,
to discover address occupied by peripheral device. To install utility you should run
`apt install i2c-tools` on debian-kind system. `i2cdetect -y 1` sample output:
	```
	     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
	00:          -- -- -- -- -- -- -- -- -- -- -- -- --
	10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	70: -- -- -- -- -- -- 76 --    
	```

Change i2c bus baudrate
------------

```
modprobe -r i2c_bcm2708
modprobe i2c_bcm2708 baudrate=1200000
```

Datasheets for supported devices
------------

MCP23017 / MCP23S17 :
http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf


Contact
-------

Please use [Github issue tracker](https://github.com/googolgl/go-mcp23017/issues) for filing bugs or feature requests.


License
-------

Go-mcp23017 is licensed under MIT License.
