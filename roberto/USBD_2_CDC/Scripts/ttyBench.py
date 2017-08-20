#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ttybench.py
# This file is part of minmal-cdc
#
# Copyright (C) 2015 - Kevin Laeufer <kevin.laeufer@rwth-aachen.de>
#
# minmal-cdc is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# minmal-cdc is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with minmal-cdc. If not, see <http://www.gnu.org/licenses/>.
#

import serial, threading, time

def is_coherent(seq):
	#print(seq)
	seq.sort()
	return seq == range(min(seq), max(seq)+1)

class TtyBench(object):
	def __init__(self):

		self.tx_data_size = 256
		self.tx_buffer_min  = 2048
		self.tx_max = 16

		# timeout of 0 means non blocking
		self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
		self.serial2 = serial.Serial('/dev/ttyACM1', 115200, timeout=0)

		self.tx_data = bytes([(ii % 256) for ii in range(0,self.tx_data_size)])

		self.rx_data = ""

		self.rx_count = 0
		self.tx_count = 0
		self.reading = False
		self.writing = False

		self.thread_read = threading.Thread(target=self.reader)
		self.thread_read.setDaemon(True)
		self.thread_write = threading.Thread(target=self.writer)
		self.thread_write.setDaemon(True)

	def run(self):
		self.reading = True
		self.thread_read.start()
		time.sleep(0.1)
		self.writing = True
		self.thread_write.start()
		time.sleep(1)
		self.writing = False
		time.sleep(0.1)
		self.reading = False
		#self.rx_data = map(int, filter(None, self.rx_data.translate(None, '[ ]').split(',')))
		#if not is_coherent(self.rx_data):
	#		print("Data not coherent!")
			#print(self.rx_data)
	#	else:
	#		print("Data verified and is coherent")
		print(self.rx_data)
		self.thread_write.join()
		self.thread_read.join()
		print("tx_count: {} bit".format(self.tx_count * 8))
		print("rx_count: {} bit".format(len(self.rx_data) * 8))
		print("{} MBps".format(self.tx_count*1e-6))

	def writer(self):
		while self.writing and self.tx_count < self.tx_max:
			if self.serial.out_waiting < self.tx_buffer_min:
				n = self.serial.write(self.tx_data)
				self.tx_count += n


	def reader(self):
		while self.reading:
			self.rx_data = self.rx_data + self.serial2.read(1)
			n = self.serial2.in_waiting
			if n:
				self.rx_data = self.rx_data + self.serial2.read(n)

	def close(self):
		self.serial.close()
		self.serial2.close()

if __name__ == "__main__":
	tb = TtyBench()
	tb.run()
	tb.close()