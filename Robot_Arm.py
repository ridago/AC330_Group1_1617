from Dynamixel import AX18A

class Arm:

	grip_id = 2
	hand_rot_id = 3
	hand_l_id = 4
	hand_r_id = 5
	elbow_l_id = 6
	elbow_r_id = 7
	rot_id = 8

	def __init__(self):

		# Setup servos
		self.grip = AX18A(Arm.grip_id)
		self.hand_rot = AX18A(Arm.hand_rot_id)
		self.hand_l = AX18A(Arm.hand_l_id)
		self.hand_r = AX18A(Arm.hand_r_id)
		self.elbow_l = AX18A(Arm.elbow_l_id)
		self.elbow_r = AX18A(Arm.elbow_r_id)
		self.rot = AX18A(Arm.rot_id)
		self.brod = AX18A(AX18A.broadcasting_id) # Broadcasting id for sending action instruction

		self.position = self.get_position()
		self.rotation = self.get_rotation()

		self.saved_positions = Arm.get_saved_positions()

	@staticmethod
	def get_saved_positions():
		try:
			f = open("positions.txt", 'r')
		except FileNotFoundError:
			f = open("positions.txt", 'w')
			f.close()
			f = open("positions.txt", 'r')

		file_str = f.read()
		f.close()

		str_lst = file_str.splitlines()
		pos_dict = {}
		for s in str_lst:
			sk, sv = s.split(":")
			pos_dict[sk] = sv

		return pos_dict

	def save_current_position(self, name):
		# Adds the current positions to the saved positions dictionary with name as key
		# The value of a position is a string with each angle value comma separated
		# The order of values is: rot, elbow, hand, hand rot, grip (not and angle)
		# Finally saves the new dictionary to positions file

		angles = self.get_all_angles()
		pos_str = "%f,%f,%f,%f,%f" % (angles['rot'], angles['elbow'], angles['hand'], angles['hand_rot'], angles['grip'])
		self.saved_positions[name] = pos_str

		all_pos_str = ""
		for key, val in self.saved_positions.items():
			all_pos_str += name + ":" + val + "\n"

		f = open("positions.txt", 'w')
		f.write(all_pos_str)
		f.close

	def move_to_position(self, position):
		# Move arm to position in saved position dictionary
		# Return True if success

		try:
			pos_str = self.saved_positions[position]
		except KeyError:
			print("move_to_position: invalid position")
			return False

		try:
			angle_str_lst = pos_str.split(",")
			rot_angle = float(angle_str_lst[0])
			elbow_angle = float(angle_str_lst[1])
			hand_angle = float(angle_str_lst[2])
			hand_rot_angle = float(angle_str_lst[3])
			grip_angle = float(angle_str_lst[4])
		except:
			print("move_to_position: corrupted positions file")
			return False

		self.move_joint('rot', rot_angle)
		self.move_joint('elbow', elbow_angle)
		self.move_joint('hand', hand_angle)
		self.move_joint('hand_rot', hand_rot_angle)
		self.move_joint('grip', grip_angle)

	def move_joint(self, joint, angle, speed=AX18A.medium):
		# Joint is string with possible values:
		#	rot, elbow, hand, hand_rot, grip
		# returns True if successfull joint move

		servo_angle_l = angle+180
		servo_angle_r = 360 - servo_angle_l
		if (joint == 'rot'):
			servo_l = self.rot
			servo_r = None
		elif (joint == 'elbow'):
			servo_angle_l = -angle+180
			servo_angle_r = 360-servo_angle_l
			servo_l = self.elbow_l
			servo_r = self.elbow_r
		elif (joint == 'hand'):
			servo_l = self.hand_l
			servo_r = self.hand_r
		elif (joint == 'hand_rot'):
			servo_l = self.hand_rot
			servo_r = None
		elif (joint == 'grip'):
			servo_angle_l = (angle/1.1)+252
			servo_l = self.grip
			servo_r = None
		else:
			print("move_joint: invalid joint input")
			return False

		if (servo_r == None):
			try:
				servo_l.move(servo_angle_l, speed)
				return True
			except AX18A.ParameterError:
				print("move_joint: invalid input")
			except AX18A.CommError as err:
				print("Failed to communicate with servo: ", err.args[0], "Error message: ", err.args[1])
			except AX18A.ServoError as err:
				print("Servo ", err.args[0], " returned error: ", err.args[1])
		else:
			try:
				servo_l.move(servo_angle_l, speed, method="reg")
				servo_r.move(servo_angle_r, speed, method="reg")
				self.brod.action()
				return True
			except AX18A.ParameterError:
				print("move_joint: invalid input")
			except AX18A.CommError as err:
				print("Failed to communicate with servo: ", err.args[0], "Error message: ", err.args[1])
			except AX18A.ServoError as err:
				print("Servo ", err.args[0], " returned error: ", err.args[1])

		# If method has reached this point, servo has not successfully moved
		return False


	def get_position(self):
		x = 0
		y = 0
		z = 0
		return (x, y, z)

	def get_rotation(self):
		x_rot = 0
		y_rot = 0
		z_rot = 0
		return (x_rot, y_rot, z_rot)

	def rest(self):
		try:
			self.grip.move(255, method="reg")
			self.hand_rot.move(180, method="reg")
			self.hand_l.move(60, speed=AX18A.slow, method="reg")
			self.hand_r.move(300, speed=AX18A.slow, method="reg")
			self.elbow_l.move(62, speed=AX18A.slow, method="reg")
			self.elbow_r.move(298, speed=AX18A.slow, method="reg")
			self.rot.move(180, method="reg")
		except AX18A.CommError as err:
			print("Failed to communicate with servo: ", err.args[0], "Error message: ", err.args[1])
		except AX18A.ServoError as err:
			print("Servo ", err.args[0], " returned error: ", err.args[1])

		self.brod.action()
		AX18A.wait(8) # Wait 8 seconds before turning torque off
		self.brod.set_torque_enable(False)

	def get_all_angles(self):
		rot_angle = round(self.rot.get_position()-180, 2)
		hand_angle = round(self.hand_l.get_position()-180, 2)
		elbow_angle = round(180-self.elbow_l.get_position(), 2)
		hand_rot_angle = round(self.hand_rot.get_position()-180, 2)
		grip_disp = round(self.grip.get_position()-252, 2)*1.1

		angles_dict = {"rot":rot_angle, "hand":hand_angle, "elbow":elbow_angle, "hand_rot":hand_rot_angle, "grip":grip_disp}
		return angles_dict