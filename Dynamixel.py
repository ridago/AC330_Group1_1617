'''
Dynamixel.py
Created by: Fredrik Rentsch
Date:		05.02.17

Class for communicating with the Dynamixel AX-18A servo 
'''

import time
from serial import Serial
import RPi.GPIO as GPIO

class AX18A:
	# AX-12A constants
	instruction_packet = {
		'ping': 0x01,
		'read_data': 0x02,
		'write_data': 0x03,
		'reg_write': 0x04,
		'action': 0x05,
		'reset': 0x06,
		'sync_write': 0x83
	}

	# AX-12A return error bits
	return_error = {
		1: "Inpput Voltage",
		2: "Angle Limit",
		4: "Overheating",
		8: "Range",
		16: "Checksum",
		32: "Overload",
		64: "Instruction"
	}
	return_delay = 0.0002

	instruction_read_length = 4
	instruction_ping_length = 2
	broadcast_ID = 0xFE

	# GPIO communication pins (BCM)
	GPIO_direction = 18
	GPIO_TX = 14
	GPIO_RX = 15
	GPIO_direction_TX = 1
	GPIO_direction_RX = 0

	port = None

	class AX18A_error(Exception) : pass

	def __init__(self, ID):
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(AX18A.GPIO_direction, GPIO.OUT)
		if(AX18A.port == None):
			AX18A.port = Serial("/dev/ttyAMA0", baudrate=1000000, timeout=0.01)

		self.ID = ID
		# Setup default register values
		self.register = [
			0x12,
			0x00, 
			0x00,
			self.ID,
			0x01,
			0xFA,
			0x00,
			0x00,
			0xFF,
			0x03,
			0x46,
			0x3C,
			0x8C,
			0xD7,
			0x03,
			0x02,
			0x24,
			0x24,
			0x00,
			0x00,
			0x01,
			0x01,
			0x20,
			0x20,
			0x00,
			0x00,
			0x00,
			0x00,
			0xD7,
			0x03,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x20,
			0x00
		]

	@staticmethod
	def wait(s):
		# Function for waiting s seconds
		start_time = time.perf_counter()
		target_time = start_time+s
		current_time = time.perf_counter()
		while(current_time < target_time):
			current_time = time.perf_counter()

	@staticmethod
	def checksum(data):
		# Function to calculate checksum for an instruction packet, data.
		# data must be the bytearray that is the instruction or status packet
		list_length = len(data)
		if (list_length < 6):
			raise AX18A.AX18A_error("Checksum: Data too short")

		# Add ID, length, instruction and each parameter into checksum
		# exclude the checksum byte from the addition, although this should 
		# be 0 anyways
		inverted_sum = data[2]
		for byte in data[3:list_length-1]:
			print("checksum: adding ", byte, "to ", inverted_sum)
			inverted_sum += byte

		# Return lowest byte
		return_value = (inverted_sum^0xFF) & 0xFF
		return return_value

	@staticmethod
	def set_direction(direction):
		GPIO.output(AX18A.GPIO_direction, direction)

	@staticmethod
	def get_status_packet():
		AX18A.set_direction(AX18A.GPIO_direction_RX)
		AX18A.wait(0.01)
		print("In waiting: ", AX18A.port.inWaiting()) # debugging
		# Read 5 first bytes [0xFF, 0xFF, id, length, error]
		reply = AX18A.port.read(5)

		print("reply: ", reply) # debugging

		try:
			assert (reply[0] == 0xFF) and (reply[1] != 0xFF)
		except IndexError:
			raise AX18A.AX18A_error("get_status_packet: Timeout error")
		except AssertionError:
			raise AX18A.AX18A_error("get_status_packet: Start signal incorrect")

		length = reply[3] - 2 	# Convert length byte to indicate number of parameters
		error = reply[4]		# Get the error byte

		if (error != 0):
			raise AX18A.AX18A_error("get_status_packet: Received error: ", AX18A.return_error[error])

		parameters = AX18A.port.read(length)
		received_checksum = AX18A.port.read(1)
		print("Paramters: ", parameters) # debugging

		status_packet = reply + parameters + received_checksum 	# Assemble status packet
		calculated_checksum = AX18A.checksum(status_packet)		# Get expected checksum

		print("Calculated checksum", calculated_checksum) # debugging
		print("Received checksum", received_checksum) # debugging

		if (calculated_checksum == received_checksum[0]):
			return status_packet
		else:
			raise AX18A.AX18A_error("get_status_packet: Checksum error")

	def get_instruction_start(self, length, instruction):
		# Method to create a bytearray object for instruction set
		# length should be N_parameters + 2. total instruction length
		# is therefore length+4, adding start, ID and length bytes 
		# Returns value_error if length is too short
		if (length < 2):
			raise AX18A.AX18A_error("get_instruction_start: Length must be at least 2")

		instruction_start = bytearray(length+4)
		try:
			instruction_start[0] = 0xFF
			instruction_start[1] = 0xFF
			instruction_start[2] = self.ID
			instruction_start[3] = length
			instruction_start[4] = AX18A.instruction_packet[instruction]
		except ValueError:
			raise AX18A.AX18A_error("get_instruction_error: A value was larger than one byte")
		except KeyError:
			raise AX18A.AX18A_error("get_instruction_error: Instruction key not valid")
		else:
			return instruction_start


	def ping(self):
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		out_data = self.get_instruction_start(AX18A.instruction_ping_length, 'ping')
		if (out_data == AX18A.value_error):
			return AX18A.value_error

		out_data[5] = AX18A.checksum(out_data)

		# Write the instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX)

		# Read status packet
		status_packet = AX18A.get_status_packet()







	def read_data(self, address, length):
		# Method to send the read data instruction to servo
		# address is register start address for reading
		# length is number of register addresses to read from
		# Returns parameters read

		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()
		# Assemble the instruction packet
		try:
			out_data = self.get_instruction_start(AX18A.instruction_read_length, 'read_data')
			out_data[5] = address
			out_data[6] = length
			out_data[7] = AX18A.checksum(out_data)
		except ValueError:
			raise AX18A.AX18A_error("read_data: A value was larger than one byte")

		# Write the instruction packet
		AX18A.port.write(out_data)
		print("out waiting: ", AX18A.port.out_waiting) # debugging
		AX18A.set_direction(AX18A.GPIO_direction_RX)
		#AX18A.wait(AX18A.return_delay)

		# Read status packet
		status_packet = AX18A.get_status_packet()
		# Extract parameters from status_packet
		try:
			status_length = status_packet[3]
			param_idx = 5+status_length-2
			parameters = status_packet[5:param_idx]
			print("param_idx: ", param_idx) # debugging
		except IndexError:
			raise AX18A.AX18A_error("read_data: length error")
		else:
			return parameters

