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
	broadcasting_id = 0xFE

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
		1: "Input Voltage",
		2: "Angle Limit",
		4: "Overheating",
		8: "Range",
		16: "Checksum",
		32: "Overload",
		64: "Instruction"
	}

	return_error_value = {
		'Input Voltage': 1,
		'Angle Limit': 2,
		'Overheating': 4,
		'Range': 8,
		'Checksum': 16,
		'Overload': 32,
		'Instruction': 64
	}

	# Standard speed values
	slow = 5
	medium = 15
	fast = 25

	CW = 1
	CCW = 0

	# GPIO communication pins (BCM)
	GPIO_direction = 18
	GPIO_TX = 14
	GPIO_RX = 15
	# Constants for setting direction
	GPIO_direction_TX = 1
	GPIO_direction_RX = 0

	port = None			# Common port object
	retry_delay = 0.01

	class CommError(Exception) : pass
	class ServoError(Exception) : pass
	class TimeoutError(CommError) : pass
	class StartSignalError(CommError) : pass
	class ChecksumError(CommError) : pass
	class ParameterError(Exception) : pass

# ---------------------------------------------
# ---------- INIT AND SETUP METHODS -----------
# ---------------------------------------------

	def __init__(self, ID):
		# Setup GPIO and port if not already set up
		if (AX18A.port == None):
			GPIO.setwarnings(False)
			GPIO.setmode(GPIO.BCM)
			GPIO.setup(AX18A.GPIO_direction, GPIO.OUT)
			AX18A.port = Serial("/dev/ttyAMA0", baudrate=1000000, timeout=0.1)

		# Set own ID
		self.ID = ID
		
		if (self.ID != AX18A.broadcasting_id):
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

			# Get actual register values from servo
			self.update_register()

	def update_register(self):
		# Method to read the whole servo register into the local register list

		# Read all values from servo
		rx_register = self.read_data(0x00, 50)

		# Update register list
		for i, value in enumerate(rx_register):
			self.register[i] = value

# ---------------------------------------------
# ----------- EASE OF LIFE METHODS ------------
# ---------------------------------------------

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
		# Method to calculate checksum for an instruction packet, data.
		# data must be the bytearray that is the instruction or status packet

		list_length = len(data)
		if (list_length < 6):
			raise AX18A.ParameterError(0xFF, "Checksum: Data too short")

		# Add ID, length, instruction and each parameter into checksum
		# exclude the checksum byte from the addition, although this should 
		# be 0 anyways
		inverted_sum = data[2]
		for byte in data[3:list_length-1]:
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
		AX18A.set_direction(AX18A.GPIO_direction_RX)

		# Read 5 first bytes [0xFF, 0xFF, id, length, error]
		reply = AX18A.port.read(5)

		# Check that input is present and correct
		try:
			assert (reply[0] == 0xFF) and (reply[1] == 0xFF)
		except IndexError:
			raise AX18A.TimeoutError(0xFF, "get_status_packet: Timeout error")
		except AssertionError:
			raise AX18A.StartSignalError(0xFF, "get_status_packet: Start signal incorrect")

		length = reply[3] - 2 	# Convert length byte to indicate number of parameters
		error = reply[4]		# Get the error byte

		if (error != 0):
			# Create error tuple
			error_tuple = AX18A.get_error_tuple(error)

			# Raise error with error tuple attached
			error_message = "get_status_packet: Received error: " + str(error_tuple)
			raise AX18A.ServoError(0xFF, error_message, error)

		# Read parameters
		parameters = AX18A.port.read(length)
		# Make sure parameters properly read
		if (len(parameters) < length):
			raise AX18A.TimeoutError(0xFF, "get_status_packet: Timeout error")
		# Read checksum
		received_checksum = AX18A.port.read(1)
		# Make sure checksum properly read
		if (len(received_checksum) < 1):
			raise AX18A.TimeoutError(0xFF, "get_status_packet: Timeout error")

		status_packet = reply + parameters + received_checksum 	# Assemble status packet
		calculated_checksum = AX18A.checksum(status_packet)		# Get expected checksum

		# Check correct checksum and return status packet if correct
		if (calculated_checksum == received_checksum[0]):
			return status_packet
		else:
			raise AX18A.ChecksumError(0xFF, "get_status_packet: Checksum error")

	@staticmethod
	def get_parameters_from_status_packet(status_packet):
		# Method to extract the parameters from a status packet
		#	status_packet: status packet to extract from
		# Returns list of parameters

		try:
			status_length = status_packet[3]			# Excract length value
			checksum_idx = 5+status_length-2			# Get index of checksum value
			parameters = status_packet[5:checksum_idx]	# Get parameters
		except IndexError:
			raise AX18A.ParameterError(0xFF, "get_parameters_from_status_packet: length error")
		else:
			return parameters

	@staticmethod
	def get_error_tuple(error_byte):
		# Method that returns the names of all errors matching the
		# bits in error_byte variable
		#	error_byte:	number between 0 and 127 where each bit represents an error

		error_list = []
		for error_bit, error_name in AX18A.return_error.items():
			if ((error_bit & error_byte) == error_bit):
				error_list.append(error_name)

		error_tuple = tuple(error_list)
		return error_tuple

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
			raise AX18A.ParameterError(self.ID, "get_instruction_packet: A value was larger than one byte")
		except KeyError:
			raise AX18A.ParameterError(self.ID, "get_instruction_packet: Instruction key not valid")
		else:
			return instruction_packet

# ---------------------------------------------
# ----------- COMMUNICATION METHODS -----------
# ---------------------------------------------

	def ping(self):
		# Method to send the ping instruction to servo
		# returns status packet received from servo

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
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
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err
				# Wait a set time before next attempt
				AX18A.wait(AX18A.retry_delay)



	def read_data(self, address, length):
		# Method to send the read data instruction to servo
		# 	address: 	register start address for reading
		# 	length: 	is number of register addresses to read from
		# Returns parameters read. If only one parameter read, value
		# is returned in stead of tuple

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
				# Set direction pin to TX and flush all serial input
				AX18A.set_direction(AX18A.GPIO_direction_TX)
				AX18A.port.flushInput()

				# Assemble instruction packet
				out_data = self.get_instruction_packet('read_data', (address, length))

				# Write instruction packet
				AX18A.port.write(out_data)

				# Read status packet
				status_packet = AX18A.get_status_packet()
				# Extract parameters from status_packet
				parameters = AX18A.get_parameters_from_status_packet(status_packet)

				# If only one parameter read, return that value, otherwise return tuple
				if (length == 1):
					return parameters[0]
				else:
					return parameters
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err
				# Wait a set time before next attempt
				AX18A.wait(AX18A.retry_delay)

	def write_data(self, address, *parameters):
		# Method to send write data instruction to servo
		# Updated the register list if successfull write
		# 	address: 	register start address for writing
		# 	parameters:	is a list of all values to be written
		# return status packet received

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
				# Set direction pin to TX and flush all serial input
				AX18A.set_direction(AX18A.GPIO_direction_TX)
				AX18A.port.flushInput()

				nParams = len(parameters)

				# Assemble instruction packet
				parameters_full = (address,) + tuple(parameters)
				out_data = self.get_instruction_packet('write_data', parameters_full)

				# Write instruction packet
				AX18A.port.write(out_data)

				# Read status packet if not broadcast ID
				if (self.ID != AX18A.broadcasting_id):
					status_packet = AX18A.get_status_packet()
					# Update register values if status packet returned
					self.register[address:(address+nParams)] = parameters
				else:
					status_packet = 0

				return status_packet
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err
				# Wait a set time before next attempt
				AX18A.wait(AX18A.retry_delay)

	def reg_write(self, address, *parameters):
		# Method to send reg_write instruction to servo
		# Updated the register list if successfull write
		# 	address: 	register start address for writing
		# 	parameters:	is a list of all values to be written
		# return status packet received

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
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

				# Read status packet if not broadcast ID
				if (self.ID != AX18A.broadcasting_id):
					status_packet = AX18A.get_status_packet()
					# Update register values if status packet returned
					self.register[address:(address+nParams)] = parameters
				else:
					status_packet = 0

				return status_packet
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err

	def action(self):
		# Method to send action instruction to servo.
		# This method is only recommended to be used when
		# instance ID is broadcasting ID
		# returns status packet received

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
				# Set direction pin to TX and flush all serial input
				AX18A.set_direction(AX18A.GPIO_direction_TX)
				AX18A.port.flushInput()

				# Assemble instruction packet
				out_data = self.get_instruction_packet('action', ())

				# Write instruction packet
				AX18A.port.write(out_data)
				AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

				# Read status packet if not broadcast ID
				if (self.ID != AX18A.broadcasting_id):
					status_packet = AX18A.get_status_packet()
				else:
					status_packet = 0

				return status_packet
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err

	def reset(self):
		# Method to send the reset instruction to servo
		# returns status packet received

		# Try three times in case of communication error
		attempts = 0
		while (attempts < 3):
			try:
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
				
				# Set ID to 1 after reset
				self.ID = 0x01

				return status_packet
			except (AX18A.CommError) as err:
				attempts += 1
				if (attempts >= 3):
					err.args = (self.id, err.args[1])
					raise err

	def sync_write(self, servos, address, *args):
		# Method to send the sync_write command.
		# Only allowed if instance ID is broadcasting ID
		# 	servos: 	a tuple of the AX18A objects representing each servo
		#	address: 	starting address for writing
		#	*args: 		a set of paramaters where each parameter is a tupple
		#				containing the values to be written
		# Nothing is returned, since broadcasting ID is used
		# TESTING HAS SHOWN SYNC_WRITE NOT TO WORK, DESPITE MENTIONED IN DOCUMENTATION

		# Check that broadcasting ID is being used
		if (self.ID != AX18A.broadcasting_id):
			raise AX18A.ParameterError(self.ID, "sync_write: instance ID must be broadcasting ID (0xFE)")

		# Check that servos length adds up with *args length
		nServos = len(servos)
		if (nServos != len(args)):
			raise AX18A.ParameterError(self.ID, "sync_write: number of servos not equal to number of write tuples")

		# Get number of parameters from number of variables in first arg tuple
		nParams = len(args[0])

		# Assemble parameters list
		parameters = [address, nParams] # Initial parameters
		# Add parameters for each servo
		for i, servo in enumerate(args):
			# Check that all tuples are same length
			if (len(servo) != nParams):
				raise AX18A.ParameterError(self.ID, "sync_write: all servos must have the same amount of data to write")
			# Add ID of servo to parameters
			parameters.append(servos[i].ID)
			# Add each parameters for servo
			for data in servo:
				parameters.append(data)

		# Assemble instruction packet
		out_data = self.get_instruction_packet('sync_write', parameters)

		# Write instruction packet
		AX18A.port.write(out_data)
		AX18A.set_direction(AX18A.GPIO_direction_RX) # Set direction pin back to RX

# ---------------------------------------------
# ---------------- SET METHODS ----------------
# ---------------------------------------------

	def move(self, angle, speed=medium, method="normal"):
		# Method to send a move command to the servo
		#	angle: 	numeric value from 30 to 330 degrees (servo angle limits)
		#	speed: 	rpm number from 0 to 113.5, can use AX18A.slow, medium or fast
		#	method:	normal or reg; decides if write_data or reg_write is used

		# Check angle parameter
		if (angle < 30 or angle > 330):
			raise AX18A.ParameterError(self.ID, "move: angle must be between 30 and 330")

		# Check that speed value is in range
		if (speed < 0 or speed > 113.5):
			raise AX18A.ParameterError(self.ID, "move: speed number must be between 0 and 113.5")

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
			self.reg_write(AX18A.address['goal_position_l'], angle_l, angle_h, speed_l, speed_h)
		else:
			self.write_data(AX18A.address['goal_position_l'], angle_l, angle_h, speed_l, speed_h)

	def set_angle_limit(self, angle_limit, direction):
		# Method to set angle limit of servo
		#	angle_limit:	numeric value from 30 to 330 degrees 
		#					(servo angle limits)
		#	direction:		AX18A.CW or AX18A.CCW

		# Check angle parameter
		if (angle_limit < 30 or angle_limit > 330):
			raise AX18A.ParameterError(self.ID, "set_angle_limit: angle_limit must be between 30 and 330")

		# Get lowest and highest byte of angle_limit value
		dynamixel_angle = angle_limit-30
		angle_value = int(dynamixel_angle*3.41)
		angle_l = angle_value & 0xFF
		angle_h = angle_value >> 8

		# Get register address from direction
		if (direction == AX18A.CW):
			address = AX18A.address['cw_angle_limit_l']
		elif (direction == AX18A.CCW):
			address = AX18A.address['ccw_angle_limit_l']
		else:
			raise AX18A.ParameterError(self.ID, "set_angle_limit: direction must be either AX18A.CW or AX18A.CCW")

		# Write angle limit
		self.write_data(address, angle_l, angle_h)

	def set_id(self, new_id):
		# Method to set the ID of the servo, also changes id of instance
		#	new_id:	new servo ID

		# Check id parameter
		if (new_id<0 or new_id>252):
			raise AX18A.ParameterError(self.ID, "set_id: new_id must be in range 0-252")

		# Write id to servo register
		self.write_data(AX18A.address['id'], new_id)
		# Change instance id
		self.ID = new_id

	def set_max_torque(self, max_torque):
		# Method to set the Max Torque of the servo (EEPROM area)
		#	max_torque:	percentage value for max torque 0-100%

		# Check max_torque parameter
		if (max_torque < 0 or max_torque > 100):
			raise AX18A.ParameterError(self.ID, "set_max_torque: max_torque must be in range 0-100")

		# Get lowest and highest byte of torque value
		torque_value = int(max_torque*10.23)
		torque_l = torque_value & 0xFF
		torque_h = torque_value >> 8

		# Write to servo max torque registers
		self.write_data(AX18A.address['max_torque_l'], torque_l, torque_h)

	def set_alarm(self, alarm, *args):
		# Method to set which error the LED or shutdown should be activated for
		#	alarm:	string led or shutdown
		#	*args: 	either a single 8-bit number between 0 and 127 representing the error bits
		#			or strings representing each error

		# Get number of parameters
		nParams = len(args)

		# Check that parameters have been given
		if (nParams == 0):
			raise AX18A.ParameterError(self.ID, )

		# Check if number given as parameter
		if (isinstance(args[0], int)):
			alarm_value = args[0]
			# Check correct input
			if (not alarm_value in range(0, 128)):
				raise AX18A.ParameterError(self.ID, "set_alarm: error value must be in range 0-127")
		else:
			alarm_value = 0
			# Loop through each given error and assemble alarm value
			try:
				for error_str in args:
					alarm_value = alarm_value | AX18A.return_error_value[error_str]
			except KeyError:
				# Raise error if incorrect string given
				raise AX18A.ParameterError(self.ID, "set_alarm: at least one error name was incorrect")

		if (alarm == "led" or alarm == "LED"):
			# Write value to register
			self.write_data(AX18A.address['alarm_led'], alarm_value)
		elif (alarm == "shutdown" or alarm == "SHUTDOWN"):
			# Write value to register
			self.write_data(AX18A.address['alarm_shutdown'], alarm_value)
		else:
			raise AX18A.ParameterError(self.ID, "set_alarm: alarm must be either led or shutdown")

	def set_torque_enable(self, torque_enable):
		# Method to set torque enable. Torque enable makes the servo generate
		# torque to stay in place
		# 	torque_enable:	boolean true or false

		# Set value to write (1 for enable 0 for disable)
		if (torque_enable):
			write_value = 1
		else:
			write_value = 0

		# Write to register
		self.write_data(AX18A.address['torque_enable'], write_value)

	def set_led(self, led):
		# Method to turn led on or off
		#	led: boolean true or false

		# Set value to write (1 for on 0 for off)
		if (led):
			write_value = 1
		else:
			write_value = 0

		# Write to register
		self.write_data(AX18A.address['led'], write_value)
			

	def set_compliance(self, margin, slope, direction):
		# Method to set compliance margin and slope for selected direction
		#	margin:		compliance margin in degrees, from 0 to 73 degrees
		#	slope: 		value in range 1-7 representing slope level (see documentation)
		#	direction:	AX18A.CW or AX18A.CCW

		# Check parameters
		if (margin < 0 or margin > 73):
			raise AX18A.ParameterError(self.ID, "set_compliance: margin must be in range 0-73")

		if (not slope in range(1,8)):
			raise AX18A.ParameterError(self.ID, "set_compliance: slope must be in range 1-7")

		# Calculate register values
		margin_value = int(margin/0.29)
		slope_value = 2**slope
			
		# Write to registers corresponding to selected direction
		if (direction == AX18A.CW):
			self.write_data(AX18A.address['cw_compliance_margin'], margin_value)
			self.write_data(AX18A.address['cw_compliance_slope'], slope_value)
		elif (direction == AX18A.CCW):
			self.write_data(AX18A.address['ccw_compliance_margin'], margin_value)
			self.write_data(AX18A.address['ccw_compliance_slope'], slope_value)
		else:
			raise AX18A.ParameterError(self.ID, "set_compliance: direction must be AX18A.CW or AX18A.CCW")


	def set_torque_limit(self, torque_limit):
		# Method to set the servo Torque Limit (RAM area)
		# if servo shuts itself down this must be set to turn it back on
		#	torque_limit:	percentage value for torque limit 0-100%

		# Check torque_limit parameter
		if (torque_limit < 0 or torque_limit > 100):
			raise AX18A.ParameterError(self.ID, "set_torque_limit: torque_limit must be in range 0-100")

		# Get lowest and highest byte of torque value
		torque_value = int(torque_limit*10.23)
		torque_l = torque_value & 0xFF
		torque_h = torque_value >> 8

		# Write to servo max torque registers
		self.write_data(AX18A.address['torque_limit_l'], torque_l, torque_h)

	def set_punch(self, punch):
		# Method to set servo punch (minimum current to drive motor)
		# Documentation does not specify anything about this value
		# and parameter is therefore simply the register value
		#	punch:	value between 0x03FF and 0x0000

		if (not punch in range(0x0000, 0x0400)):
			raise AX18A.ParameterError(self.ID, "set_punch: punch must be in range 0x0000-0x03FF")

		# Get lowest and highest byte of punch
		punch_l = punch & 0xFF
		punch_h = punch >> 8

		# Write to servo punch registers
		self.write_data(AX18A.address['punch_l'], punch_l, punch_h)

# ---------------------------------------------
# ---------------- GET METHODS ----------------
# ---------------------------------------------

	def get_angle_limit(self, direction):
		# Method to get the angle limit in degrees for the selected direction
		# Does not communicate with servo, since local register values for
		# EEPROM should always be correct
		#	direction:	AX18A.CW or AX18A.CCW

		# Get lowest and highest byte from register
		if (direction == AX18A.CW):
			angle_l = self.register[AX18A.address['cw_angle_limit_l']]
			angle_h = self.register[AX18A.address['cw_angle_limit_h']]
		elif (direction == AX18A.CCW):
			angle_l = self.register[AX18A.address['ccw_angle_limit_l']]
			angle_h = self.register[AX18A.address['ccw_angle_limit_h']]
		else:
			raise AX18A.ParameterError(self.ID, "get_angle_limit: direction must be either AX18A.CW or AX18A.CCW")

		angle_full = angle_h << 8 | angle_l 	# Assemble 16bit variable (10bit max)
		dynamixel_angle = angle_full/3.41		# Get angle as described in Dynamixel documentation
		angle_limit = dynamixel_angle + 30		# Convert to angle that makes more sense

		return angle_limit

	def get_max_torque(self):
		# Method to get max torque in percentage value
		# Does not communicate with servo, since local register values
		# for EEPROM should always be correct

		# Get lowest and highest byte from register
		torque_l = self.register[AX18A.address['max_torque_l']]
		torque_h = self.register[AX18A.address['max_torque_h']]

		torque_full = torque_h << 8 | torque_l 	# Assemble 16bit variable (10bit max)
		max_torque = torque_full/10.23 			# Convert to percentage value

		return max_torque

	def get_alarm(self, alarm):
		# Method to get alarm led value
		# first value returned is the register value, consecutive values
		# are the names of each error.
		# Des not communicate with servo, since local register values
		# should be correct at all times
		#	alarm:	led or shutdown

		# Get register value for correct alarm
		if (alarm == "led" or alarm == "LED"):
			error_value = self.register[AX18A.address['alarm_led']]
		elif (alarm == "shutdown" or alarm == "SHUTDOWN"):
			error_value = self.register[AX18A.address['alarm_shutdown']]
		else:
			raise AX18A.ParameterError(self.ID, "get_alarm: alarm must be either led or shutdown")

		# Get error name tuple
		error_tuple = AX18A.get_error_tuple(error_value)

		return_tuple = (error_value,) + error_tuple
		return return_tuple

	def get_torque_enable(self):
		# Method to get torque_enable status
		# Reads from servo as this value is dynamic

		# Read from servo 
		torque_enable = self.read_data(AX18A.address['torque_enable'], 1)

		# Update local register
		self.register[AX18A.address['torque_enable']] = torque_enable

		return torque_enable

	def get_compliance(self, direction):
		# Method to get compliance slope and margin
		# Does not communicate with servo, since local register values
		# should be correct at all times
		#	direction: 	AX18A.CW or AX18A.CCW

		# Get register values
		if (direction == AX18A.CW):
			margin_value = self.register[AX18A.address['cw_compliance_margin']]
			slope_value = self.register[AX18A.address['cw_compliance_slope']]
		elif (direction == AX18A.CCW):
			margin_value = self.register[AX18A.address['ccw_compliance_margin']]
			slope_value = self.register[AX18A.address['ccw_compliance_slope']]
		else:
			raise AX18A.ParameterError(self.ID, "get_compliance: direction must be either AX18A.CW or AX18A.CCW")

		# Convert to values of correct unit
		margin = margin_value*0.29
		slope = 1
		slope_value = slope_value>>1
		while(slope_value > 1):
			slope_value = slope_value>>1
			slope+=1

		return (margin, slope)

	def get_torque_limit(self):
		# Method to get torque limit from servo
		# This is dynamic according to the datasheet and will therefore
		# communicate with servo. (Not sure if the datasheet is actually correct though)
		# Torque limit is returned as a percentage value in range 0-100

		# Read from servo
		(torque_l, torque_h) = self.read_data(AX18A.address['torque_limit_l'], 2)

		# Assemble full value
		torque_value = torque_h << 8
		torque_value = torque_value | torque_l

		# Update local register
		self.register[AX18A.address['torque_limit_l']] = torque_l
		self.register[AX18A.address['torque_limit_h']] = torque_h

		# Calculate percentage value
		percentage_value = torque_value / 10.23
		
		return percentage_value

	def get_position(self, time="current"):
		# Method to get present or goal position of servo in degrees (30-330 degrees)
		# Reads from servo and updates local register
		#	time: 	current or goal, representing which position value to get
		# Returns position in degrees

		if (time == "current"):
			register_address_l = AX18A.address['present_position_l']
			register_address_h = AX18A.address['present_position_h']
		elif (time == "goal"):
			register_address_l = AX18A.address['goal_position_l']
			register_address_h = AX18A.address['goal_position_h']
		else:
			raise AX18A.ParameterError(self.ID, "get_position: time must be either current or goal")

		# Read from servo
		(position_l, position_h) = self.read_data(register_address_l, 2)

		# Assemble full value
		position_value = position_h << 8
		position_value = position_value | position_l

		# Update local register
		self.register[register_address_l] = position_l
		self.register[register_address_h] = position_h

		# Calculate angle value
		dynamixel_angle = position_value/3.41
		angle = dynamixel_angle+30

		return angle

	def get_speed(self):
		# Method to get present speed from servo in rpm (0-113.5 rpm)
		# Reads from servo and updates local register
		# Returns speed in rpm

		# Read from servo
		(speed_l, speed_h) = self.read_data(AX18A.address['present_speed_l'], 2)

		# Assemble full value
		speed_value = speed_h << 8
		speed_value = speed_value | speed_l

		# Update local register
		self.register[AX18A.address['present_speed_l']] = speed_l
		self.register[AX18A.address['present_speed_h']] = speed_h

		# Calculate rpm value
		rpm = speed_value*0.111

		return rpm

	def get_load(self):
		# Method to get present load from servo in percent of max load
		# Reads from servo and updates local register
		# Returns positive values for CW and negative for CCW

		# Read from servo
		(load_l, load_h) = self.read_data(AX18A.address['present_load_l'], 2)

		# Assemble full value
		load_value = load_h << 8
		load_value = load_value | load_l

		# Update local register
		self.register[AX18A.address['present_load_l']] = load_l
		self.register[AX18A.address['present_load_h']] = load_h

		# Calculate percentage value
		load = (load_value&0x3FF)/10.23
		# Set correct sign (by multiplying with -1 if direction bit is 0)
		load = load*(-1+2*(load_value>>10))

		return load

	def get_volt(self):
		# Method to get present voltage from servo
		# Reads from servo and updates local register
		# Returns present voltage in volts

		# Read from servo
		volt_value = self.read_data(AX18A.address['present_voltage'], 1)

		# Update local register
		self.register[AX18A.address['present_voltage']] = volt_value

		# Calculate voltage value
		volt = volt_value/10

		return volt

	def get_temperature(self):
		# Method to get present temperature from servo (in degrees C)
		# Reads from servo and updates local register
		# Returns temperature in degrees C

		# Read from servo 
		temperature = self.read_data(AX18A.address['present_temperature'], 1)

		# Update local register
		self.register[AX18A.address['present_temperature']] = temperature

		# Register value is exactly the same as actual value
		return int(temperature)

	def get_registered(self):
		# Method to check if servo has waiting reg_write command
		# Reads from servo and updates local register
		# Returns True if command waiting, False otherwise

		# Read from servo
		registered = self.read_data(AX18A.address['registered'], 1)

		# Update local register
		self.register[AX18A.address['registered']] = registered

		return bool(registered)

	def get_moving(self):
		# Method to check if servo is moving
		# Reads from servo and updates local register
		# Returns True if moving, False if not

		# Read from servo
		moving = self.read_data(AX18A.address['moving'], 1)

		# Update local register
		self.register[AX18A.address['moving']] = moving

		return bool(moving)

	def get_punch(self):
		# Method to get punch value from servo register
		# Due to lack of documentation this is just the value store in register
		# Does not communicate with servo, since local register value
		# should be correct at all times

		# Read from servo
		(punch_l, punch_h) = self.read_data(AX18A.address['punch_l'], 2)

		# Update local register
		self.register[AX18A.address['punch_l']] = punch_l
		self.register[AX18A.address['punch_l']] = punch_h

		# Assemble full value
		punch_value = punch_h << 8
		punch_value = punch_value | punch_l

		return punch_value

# ---------------------------------------------
# ---------- AUTOMATED FUNCTIONALITY ----------
# ---------------------------------------------

