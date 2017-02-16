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
	instruction = {
		'ping': 0x01,
		'read_data': 0x02,
		'write_data': 0x03,
		'reg_write': 0x04,
		'action': 0x05,
		'reset': 0x06,
		'sync_write': 0x83
	}
	broadcast_ID = 0xFE

	address = {
		'model_number_l': 0x00,
		'model_number_h': 0x01,
		'version_of_firmware': 0x02,
		'id': 0x03,
		'baud_rate': 0x04,
		'return_delay_time': 0x05,
		'cw_angle_limit_l': 0x06,
		'cw_angle_limit_h': 0x07,
		'ccw_angle_limit_l': 0x08,
		'ccw_angle_limit_h': 0x09,
		'the_highest_limit_temperature': 0x0B,
		'the_lowest_limit_voltage': 0x0C,
		'the_highest_limit_voltage': 0x0D,
		'max_torque_l': 0x0E,
		'max_torque_h': 0x0F,
		'status_return_level': 0x10,
		'alarm_led': 0x11,
		'alarm_shutdown': 0x12,
		'torque_enable': 0x18,
		'led': 0x19,
		'cw_compliance_margin': 0x1A,
		'ccw_compliance_margin': 0x1B,
		'cw_compliance_slope': 0x1C,
		'ccw_compliance_slope': 0x1D,
		'goal_position_l': 0x1E,
		'goal_position_h': 0x1F,
		'moving_speed_l': 0x20,
		'moving_speed_h': 0x21,
		'torque_limit_l': 0x22,
		'torque_limit_h': 0x23,
		'present_position_l': 0x24,
		'present_position_h': 0x25,
		'present_speed_l': 0x26,
		'present_speed_h': 0x27,
		'present_load_l': 0x28,
		'present_load_h': 0x29,
		'present_voltage': 0x2A,
		'present_temperature': 0x2B,
		'registered': 0x2C,
		'moving': 0x2E,
		'lock': 0x2F,
		'punch_l': 0x30,
		'punch_h': 0x31
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

	# Dictionary containing error codes,
	# short description as key
	error_code = {
		'servo': 256,
		'timeout': 257,
		'start_signal': 258,
		'checksum': 259,
		'parameter': 260
	}

	# Dictionary to get description of error codes
	error_mask = {
		256: "Servo returned error",
		257: "Timeout error",
		258: "Start signal error",
		259: "Checksum error",
		260: "Parameter error"
	}

	# Standard speed values
	speed = {
		'slow': 0x00FF,
		'medium': 0x0200,
		'fast': 0x0399
	}

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
			AX18A.port = Serial("/dev/ttyAMA0", baudrate=1000000, timeout=0.1)
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
			0x00,
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
			0x00,
			0x00,
			0x00,
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
			0x00,
			0x20,
			0x00
		]

	@staticmethod
	def wait(s):
		# Method for waiting s seconds
		#	s: seconds to wait, can be floating point

		start_time = time.perf_counter()		# Get initial time
		target_time = start_time+s 				# Calculate target time to hit

		current_time = time.perf_counter() 		# Get current time
		while(current_time < target_time):		# Check if target time reached
			current_time = time.perf_counter()	# If target time not reached,
												# get current time again

	@staticmethod
	def checksum(data):
		# Methyod to calculate checksum for an instruction packet, data.
		# data must be the bytearray that is the instruction or status packet
		list_length = len(data)
		if (list_length < 6):
			raise AX18A.AX18A_error(error_code['parameter'], "Checksum: Data too short")

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
		# Small method to set direction pin to RX or TX
		#	direction: either AX18A.GPIO_direction_RX
		#				or AX18A.GPIO_direction_TX

		GPIO.output(AX18A.GPIO_direction, direction)

	@staticmethod
	def get_status_packet():
		# Method to get incomming status packet

		# Set direction pin
		#AX18A.set_direction(AX18A.GPIO_direction_RX)
		AX18A.wait(0.01)
		print("In waiting: ", AX18A.port.inWaiting()) # debugging

		# Read 5 first bytes [0xFF, 0xFF, id, length, error]
		reply = AX18A.port.read(5)

		print("reply: ", reply) # debugging

		# Check that input is present and correct
		try:
			assert (reply[0] == 0xFF) and (reply[1] == 0xFF)
		except IndexError:
			raise AX18A.AX18A_error(error_code['timeout'], "get_status_packet: Timeout error")
		except AssertionError:
			raise AX18A.AX18A_error(error_code['start_signal'], "get_status_packet: Start signal incorrect")

		length = reply[3] - 2 	# Convert length byte to indicate number of parameters
		error = reply[4]		# Get the error byte

		if (error != 0):
			raise AX18A.AX18A_error(error_code['servo'], "get_status_packet: Received error: ", AX18A.return_error[error])

		parameters = AX18A.port.read(length)
		received_checksum = AX18A.port.read(1)
		print("Paramters: ", parameters) # debugging

		status_packet = reply + parameters + received_checksum 	# Assemble status packet
		calculated_checksum = AX18A.checksum(status_packet)		# Get expected checksum

		# Check correct checksum and return status packet if correct
		if (calculated_checksum == received_checksum[0]):
			return status_packet
		else:
			raise AX18A.AX18A_error(error_code['checksum'], "get_status_packet: Checksum error")

	@staticmethod
	def get_parameters_from_status_packet(status_packet):
		# Method to extract the parameters from a status packet
		#	status_packet: status packet to extract from
		# Returns list of parameters

		try:
			status_length = status_packet[3]			# Excract length value
			checksum_idx = 5+status_length-2			# Get index of checksum value
			parameters = status_packet[5:checksum_idx]	# Get parameters
			print("checksum_idx: ", checksum_idx) # debugging
		except IndexError:
			raise AX18A.AX18A_error(error_code['parameter'], "get_parameters_from_status_packet: length error")
		else:
			return parameters

	def get_instruction_packet(self, instruction, parameters):
		# Method to create a bytearray object for instruction packet
		#	instruction: String matching a key from the AX18A.instruction dictionary
		#	parameters: All parameters to be included in the instruction packet
		# returns the instruction packet (bytearray)

		# Get length value (number of parameters * 2)
		nParams = len(parameters)
		length = nParams+2

		instruction_packet = bytearray(length+4)	# Create instruction packet bytearray
		try:
			instruction_packet[0] = 0xFF			# Start byte one
			instruction_packet[1] = 0xFF			# Start byte two
			instruction_packet[2] = self.ID 		# Servo ID
			instruction_packet[3] = length 			# Length value (Nparameters+2)
			instruction_packet[4] = AX18A.instruction[instruction] # Instruction value
			# Add all parameter bytes
			i = 5
			for param in parameters:
				instruction_packet[i] = param
				i += 1
			# Add checksum to final byte
			instruction_packet[i] = AX18A.checksum(instruction_packet)

		except ValueError:
			raise AX18A.AX18A_error(error_code['parameter'], "get_instruction_packet: A value was larger than one byte")
		except KeyError:
			raise AX18A.AX18A_error(error_code['parameter'], "get_instruction_packet: Instruction key not valid")
		else:
			return instruction_packet


	def ping(self):
		# Method to send the ping instruction to servo
		# returns status packet received from servo

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		# Assemble instruction packet
		out_data = self.get_instruction_packet('ping', ())

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

		# Read status packet
		status_packet = AX18A.get_status_packet()
		return status_packet



	def read_data(self, address, length):
		# Method to send the read data instruction to servo
		# 	address: 	register start address for reading
		# 	length: 	is number of register addresses to read from
		# Returns parameters read

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		# Assemble instruction packet
		out_data = self.get_instruction_packet('read_data', (address, length))

		# Write instruction packet
		AX18A.port.write(out_data)
		#print("out waiting: ", AX18A.port.out_waiting) # debugging
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX
		#AX18A.wait(AX18A.return_delay)

		# Read status packet
		status_packet = AX18A.get_status_packet()
		# Extract parameters from status_packet
		parameters = AX18A.get_parameters_from_status_packet(status_packet)
		return parameters

	def write_data(self, address, parameters):
		# Method to send write data instruction to servo
		# 	address: 	register start address for writing
		# 	parameters:	is a list of all values to be written
		# return status packet received

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		nParams = len(parameters)

		# Assemble instruction packet
		parameters_full = (address,) + tuple(parameters)
		out_data = self.get_instruction_packet('write_data', parameters_full)

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

		# Read status packet if not broadcast ID
		if (self.ID != AX18A.broadcast_ID):
			status_packet = AX18A.get_status_packet()
			# Update register values if status packet returned
			register[address:(address+nParams)] = parameters
		else:
			status_packet = 0

		return status_packet

	def reg_write(self, address, parameters):
		# Method to send reg_write instruction to servo
		# 	address: 	register start address for writing
		# 	parameters:	is a list of all values to be written
		# return status packet received

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		nParams = len(parameters)

		# Assemble instruction packet
		parameters_full = (address,) + tuple(parameters)
		out_data = self.get_instruction_packet('reg_write', parameters_full)

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

		# TODO Check if reg_write returns status packet

		# Read status packet if not broadcast ID
		if (self.ID != AX18A.broadcast_ID):
			status_packet = AX18A.get_status_packet()
			# Update register values if status packet returned
			register[address:(address+nParams)] = parameters
		else:
			status_packet = 0

		return status_packet

	def action(self):
		# Method to send action instruction to servo.
		# This method is only recommended to be used when
		# instance ID is broadcasting ID
		# returns status packet received

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		# Assemble instruction packet
		out_data = self.get_instruction_packet('action', ())

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

		# TODO Check if action returns status packet

		# Read status packet if not broadcast ID
		if (self.ID != AX18A.broadcast_ID):
			status_packet = AX18A.get_status_packet()
		else:
			status_packet = 0

		return status_packet

	def reset(self):
		# Method to send the reset instruction to servo
		# returns status packet received

		# Set direction pin to TX and flush all serial input
		AX18A.set_direction(AX18A.GPIO_direction_TX)
		AX18A.port.flushInput()

		# Assemble instruction packet
		out_data = self.get_instruction_packet('reset', ())

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

		# Read status packet
		status_packet = AX18A.get_status_packet()
		return status_packet

	def sync_write(self, servos, address, *args):
		# Method to send the sync_write command.
		# Only allowed if instance ID is broadcasting ID
		# 	servos: 	a tuple of the AX18A objects representing each servo
		#	address: 	starting address for writing
		#	*args: 		a set of paramaters where each parameter is a tupple
		#				containing the values to be written
		# Nothing is returned, since broadcasting ID is used

		# Check that broadcasting ID is being used
		if (self.ID != AX18A.broadcast_ID):
			raise AX18A.AX18A_error(error_code['parameter'], "sync_write: instance ID must be broadcasting ID (0xFE)")

		# Check that servos length adds up with *args length
		nServos = len(servos)
		if (nServos != len(args)):
			raise AX18A.AX18A_error(error_code['parameter'], "sync_write: number of servos not equal to number of write tuples")

		# Get number of parameters from number of variables in first arg tuple
		nParams = len(args[0])

		# Assemble parameters list
		parameters = [address, nParams] # Initial parameters
		# Add parameters for each servo
		for i, servo in enumerate(args):
			# Check that all tuples are same length
			if (len(servo) != nParams):
				raise AX18A.AX18A_error(error_code['parameter'], "sync_write: all servos must have the same amount of data to write")
			# Add ID of servo to parameters
			parameters.append(servos[i].ID)
			print("Adding servo ID: ", servos[i].ID) # Debug
			# Add each parameters for servo
			for data in servo:
				parameters.append(data)

		print("sync_write: parameters: ", parameters)

		# Assemble instruction packet
		out_data = self.get_instruction_packet('sync_write', parameters)

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

	def update_register(self):
		# Method to read the whole servo register into the local register list

		# Read all values from servo
		rx_register = self.read_data(0x00, 50)

		# Update register list
		for i, value in enumerate(rx_register):
			self.register[i] = value

	def move(self, angle, speed, method="normal"):
		# Method to send a move command to the servo
		#	angle: 	numeric value from 30 to 330 degrees (servo angle limits)
		#	speed: 	either string matching the speed dictionary or
		#			rpm number from 0 to 113.5
		#	method:	normal or reg; decides if write_data or reg_write is used

		# Check angle parameter
		if (angle < 30 or angle > 330):
			raise AX18A.AX18A_error(error_code['parameter'], "move: angle must be between 30 and 330")

		if (isinstance(speed, str)):
			try:
				speed_value = self.speed[speed]
			except KeyError:
				raise AX18A.AX18A_error(error_code['parameter'], "move: speed must be number or one of the valid speed options")
		else:
			# Check that speed value is in range
			if (speed_value < 0 or speed_value > 113.5):
				raise AX18A.AX18A_error(error_code['parameter'], "move: speed number must be between 0 and 113.5")
			# Get servo equivalent value
			speed_value = int(speed/0.111)

		

		# Get lowest and highest byte of angle value
		dynamixel_angle = angle-30
		angle_value = int(dynamixel_angle*3.41)
		angle_l = angle_value & 0xFF
		angle_h = angle_value >> 8

		# Get lowest and highest byte of speed value
		speed_l = speed_value & 0xFF
		speed_h = speed_value >> 8

		# Write using either write_data or reg_write instruction
		if (method == "reg"):
			self.reg_write(AX18A.address['goal_position_l'], (angle_l, angle_h, speed_l, speed_h))
		else:
			self.write_data(AX18A.address['goal_position_l'], (angle_l, angle_h, speed_l, speed_h))

	def set_angle_limit(angle_limit, direction):
		# Method to set angle limit of servo
		#	angle_limit:	numeric value from 30 to 330 degrees 
		#					(servo angle limits)
		#	direction:		string with value cw or ccw

		# Check angle parameter
		if (angle_limit < 30 or angle_limit > 330):
			raise AX18A.AX18A_error(error_code['parameter'], "set_angle_limit: angle_limit must be between 30 and 330")

		# Get lowest and highest byte of angle_limit value
		dynamixel_angle = angle_limit-30
		angle_value = int(dynamixel_angle*3.41)
		angle_l = angle_value & 0xFF
		angle_h = angle_value >> 8

		# Get register address from direction
		if (direction == "cw" or direction == "CW"):
			address = AX18A.address['cw_angle_limit_l']
		elif (direction == "ccw" or direction = "CCW"):
			address = AX18A.address['ccw_angle_limit_l']
		else:
			raise AX18A.AX18A_error(error_code['parameter'], "set_angle_limit: direction must be either cw or ccw")

		# Write angle limit
		self.write_data(address, (angle_l, angle_h))

