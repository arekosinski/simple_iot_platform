#!/usr/bin/env python3

import sys
import serial
import time

from os.path import expanduser

uart_speed = 460800
uart_port = '/dev/ttyAMA0'

msg_logger_file = "{homedir}/radio_msg.log".format(homedir=expanduser("~"))

def decode_msg(line):
	msg = dict()
	msg["timestamp"] = time.time()
	msg["source_msg"] = line
	try:
		msg["device_id"] = line[0:2]
		msg["measure_code"] = line[2:3]
		msg["measure_value"] = line[3:line.find('|',0)]
		msg["cycle_number"] = line[line.find('|',0)+1:len(line)].strip()
	except:
		pass
	return msg

if __name__ == '__main__':
	serial_proxy = serial.Serial(uart_port, uart_speed, timeout=1)
	serial_proxy.flush()
	
	radio_log = open(msg_logger_file,'a')
	
	while True:
		if serial_proxy.in_waiting > 0:
			try:
				line = serial_proxy.readline().decode('ascii').rstrip()
				print("MSG received: {msg}".format(msg=line))
				if ('|' in line):
					msg_decoded = decode_msg(line)
					print("MSG decoded: {msgd}".format(msgd=msg_decoded))
					radio_log.write("{msgd}\n".format(msgd=msg_decoded))
				sys.stdout.flush()
				radio_log.flush()
			except:
				print("failed to handle message")
	radio_log.close()
