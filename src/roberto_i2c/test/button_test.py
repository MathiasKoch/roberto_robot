#!/usr/bin/python
import os, time, select
import smbus

PCF8574=0x38

b=smbus.SMBus(4)

irq = open('/sys/class/gpio/gpio204/value', 'r') # open interrupt pin
epoll = select.epoll()
epoll.register(irq, select.EPOLLIN|select.EPOLLET) #register poll method and triggered events
last_irq = 0

pins_init = b.read_byte(PCF8574)


while True:
    events = epoll.poll() #this line reads blocking
    for fileno, event in events:
        if fileno == irq.fileno(): #check if interrupt comes from our handle
			pins = b.read_byte(PCF8574)
			if pins != pins_init:
				if (pins & 0x01) != (pins_init & 0x01):
					print "Pin 1 triggered"
				if (pins & 0x02) != (pins_init & 0x02):
					print "Pin 2 triggered"
				if (pins & 0x04) != (pins_init & 0x04):
					print "Pin 3 triggered"
				if (pins & 0x08) != (pins_init & 0x08):
					print "Pin 4 triggered"
				if (pins & 0x10) != (pins_init & 0x10):
					print "Shutdown triggered!"
				if (pins & 0x20) != (pins_init & 0x20):
					print "Pin 6 triggered"
				if (pins & 0x40) != (pins_init & 0x40):
					print "Pin 7 triggered"
				if (pins & 0x80) != (pins_init & 0x80):
					print "Pin 8 triggered"