'''
AX18A_Test.py
Created by: Fredrik Rentsch
Date:		26.02.17

Test file for AX18A class. Making sure each method works as expected
'''

from Dynamixel import AX18A

delay_time = 0.01

# 01.01
print("-----01.01-----")
attempts = 0
while(attempts<3):
	try:
		test = AX18A(3)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("01.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			raise Exception("01.01: Initialization failed")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Check a set of register values
if (test.register[0x18] == 0 and 
	test.register[0x19] == 0 and 
	test.register[0x1A] == 1 and
	test.register[0x1C] == 32 and
	test.register[0x2C] == 0 and
	test.register[0x31] == 0):
	print("01.01: passed")
else:
	print("01.01: failed")

print("-----01.02-----")
brod = AX18A(AX18A.broadcast_ID)
try:
	brod.register
except AttributeError:
	print("01.02: passed")
else:
	print("01.02: failed")

print("-----02.01-----")
curr_pos = test.register[AX18A.address['present_position_h']] << 8 + test.register[AX18A.address['present_position_l']]
print("Move servo")
AX18A.wait(5) # Wait five seconds for servo to be moved
attempts = 0
while(attempts < 3):
	try:
		test.update_register()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("02.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			raise Exception("02.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

new_pos = test.register[AX18A.address['present_position_h']] << 8 + test.register[AX18A.address['present_position_l']]

if (new_pos != curr_pos):
	print("02.01: passed")
else:
	print("02.01: failed")

print("-----03.01-----")
attempts = 0
while(attempts < 3):
	input("Press enter when starting stop watch")
	wait(10)
	print("10 seconds should now have passed")
	attempts += 1
	print("Test done ", attempts, "time(s)")

print("-----04.01-----")
checksum = AX18A.checksum(bytearray(0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0x00))
if (checksum == 0xCC):
	print("04.01: passed")
else:
	print("04.01: failed. Received checksum: ", hex(checksum))

print("-----04.02-----")
checksum = AX18A.checksum(bytearray(0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0x00))
if (checksum == 0xCC):
	print("04.02: passed")
else:
	print("04.02: failed. Received checksum: ", hex(checksum))

print("-----07.01-----")
params = AX18A.get_parameters_from_status_packet((0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xD8))
if (params == (0x20,)):
	print("07.01: passed")
else: 
	print("07.01: failed. Received parameters: ", params)

print("-----07.02-----")
params = AX18A.get_parameters_from_status_packet((0xFF, 0xFF, 0x01, 0x04, 0x00, 0xFF, 0x03, 0xF8))
if (params == (0xFF, 0x03)):
	print("07.02: passed")
else: 
	print("07.02: failed. Received parameters: ", params)


print("-----08.01-----")
errors = AX18A.get_error_tuple(0x7F)
excepted_errors = ("Input Voltage", "Angle Limit", "Overheating", "Range", "Checksum", "Overload", "Instruction")
passed = True
for err in excepted_errors:
	if(not any(err in test_err for test_err in errors)):
		passed = False
if (passed):
	print("08.01: passed")
else:
	print("08.01: failed. Received errors: ", errors)


print("-----08.02-----")
errors = AX18A.get_error_tuple(0x05)
excepted_errors = ("Input Voltage", "Overheating")
passed = True
for err in excepted_errors:
	if(not any(err in test_err for test_err in errors)):
		passed = False
if (passed):
	print("08.02: passed")
else:
	print("08.02: failed. Received errors: ", errors)

print("-----09.01-----")
instr_pckt = test.get_instruction_packet('read_data', (0x03, 0x01))
if (instr_pckt == (0xFF, 0xFF, 0x03, 0x04, 0x03, 0x01, 0xF4)):
	print("09.01: passed")
else:
	print("09.01: failed. Received instruction packet: ", instr_pckt)

print("-----10.01-----")
attempts = 0
while(attempts < 3):
	try:
		ping_result = test.ping()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("10.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("10.01: Communication error")
			ping_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (ping_result == (0xFF, 0xFF, 0x03, 0x02, 0x00, 0xFA)):
	print("10.01: passed")
else:
	print("10.01: failed. Received status_packet: ", ping_result)

print("-----11.01-----")
attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x03, 1)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("11.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("11.01: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (read_result == 0x03):
	print("11.01: passed")
else:
	print("11.01: failed. Received data: ", read_result)


print("-----11.02-----")
attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x08, 2)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("11.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("11.02: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (read_result == 0x03):
	print("11.02: passed")
else:
	print("11.02: failed. Received data: ", read_result)

print("-----12.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.write_data(0x08, 0xAA, 0x02)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("12.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("12.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x08, 2)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("12.01: Attempt 2_", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("12.01: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (read_result == (0xAA, 0x02)):
	print("12.01: passed")
else:
	print("12.01: failed. Received data: ", read_result)

print("-----13.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.reg_write(0x08, 0xFF, 0x03)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("13.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("13.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

brod.action()

attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x08, 2)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("13.01: Attempt 2_", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("13.01: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (read_result == (0xFF, 0x03)):
	print("13.01: passed")
else:
	print("13.01: failed. Received data: ", read_result)

print("-----15.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.reset()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("15.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("15.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x03, 1)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("15.01: Attempt 2_", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("15.01: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (test.ID == 0x01 and read_result == 0x01):
	print("15.01: passed")
else:
	print("15.01: failed. ID: ", test.ID, "Received ID: ", read_result)


print("-----17.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(180, AX18A.fast)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("17.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("17.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("17.01: test complete")
else:
	print("17.01: failed")

print("-----17.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(270, AX18A.slow, method="reg")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("17.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("17.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

brod.action()

if (passed):
	print("17.02: test complete")
else:
	print("17.02: failed")


print("-----18.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(90, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("18.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("18.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['cw_angle_limit_h']
reg_val = (test.register[reg_address] << 8) + test.register[reg_address-1]

if (passed):
	print("18.01: test complete. Received data: ", reg_val)
else:
	print("18.01: failed")

print("-----18.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(30, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("18.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("18.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x06, 2)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("18.02: Attempt 2_", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("18.02: Communication error")
			read_result = 0
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("18.02: test complete. Received data: ", read_result)
else:
	print("18.02: failed")

print("-----18.03-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(270, AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("18.03: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("18.03: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['ccw_angle_limit_h']
reg_val = (test.register[reg_address] << 8) + test.register[reg_address-1]

if (passed):
	print("18.03: test complete. Received data: ", reg_val)
else:
	print("18.03: failed")

print("-----18.04-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(330, AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("18.04: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("18.04: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['ccw_angle_limit_h']
reg_val = (test.register[reg_address] << 8) + test.register[reg_address-1]

if (passed):
	print("18.04: test complete. Received data: ", reg_val)
else:
	print("18.04: failed")


print("-----19.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.set_id(3)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("19.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("19.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

attempts = 0
while(attempts < 3):
	try:
		read_result = test.read_data(0x03, 1)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("19.01: Attempt 2_", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("19.01: Communication error")
			read_result = 0
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (test.ID == 0x03 and read_result == 0x03):
	print("19.01: passed")
else:
	print("19.01: failed. ID: ", test.ID, "Received ID: ", read_result)


print("-----20.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_max_torque(50)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("20.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("20.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['max_torque_h']
reg_val = (test.register[reg_address] << 8) + test.register[reg_address-1]

if (passed):
	print("20.01: test complete. Received data: ", reg_val)
else:
	print("20.01: failed")

print("-----21.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.set_alarm("led", "Angle Limit", "Range")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("21.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("21.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['alarm_led']
reg_val = test.register[reg_address]

if (reg_val == 0x0A):
	print("21.01: passed")
else:
	print("21.01: failed. Received data: ", reg_val)

print("-----21.02-----")
attempts = 0
while(attempts < 3):
	try:
		test.set_alarm("shutdown", "Overheating", "Input Voltage")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("21.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("21.02: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['alarm_shutdown']
reg_val = test.register[reg_address]

if (reg_val == 0x05):
	print("21.02: passed")
else:
	print("21.02: failed. Received data: ", reg_val)


print("-----22.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_enable(True)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("22.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("22.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['torque_enable']
reg_val = test.register[reg_address]

correct_input = False
while(not correct_input):
	user_input = input("22.01: Is servo movable?(y/n): ")
	if ("y" in user_input):
		correct_input = True
		passed = False
	elif ("n" in user_input):
		correct_input = True
	else:
		print("22.01: Incorrect input. Type y or n")

if (passed and reg_val == 0x01):
	print("22.01: passed")
else:
	print("22.01: failed. Received data: ", reg_val)

print("-----22.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_enable(False)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("22.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("22.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['torque_enable']
reg_val = test.register[reg_address]

correct_input = False
while(not correct_input):
	user_input = input("22.02: Is servo movable?(y/n): ")
	if ("y" in user_input):
		correct_input = True
	elif ("n" in user_input):
		correct_input = True
		passed = False
	else:
		print("22.02: Incorrect input. Type y or n")

if (passed and reg_val == 0x00):
	print("22.02: passed")
else:
	print("22.02: failed. Received data: ", reg_val)


print("-----23.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_led(True)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("23.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("23.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['led']
reg_val = test.register[reg_address]

correct_input = False
while(not correct_input):
	user_input = input("23.01: Is LED on?(y/n): ")
	if ("y" in user_input):
		correct_input = True
	elif ("n" in user_input):
		correct_input = True
		passed = False
	else:
		print("23.01: Incorrect input. Type y or n")

if (passed and reg_val == 0x01):
	print("23.01: passed")
else:
	print("23.01: failed. Received data: ", reg_val)


print("-----23.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_led(True)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("23.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("23.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['led']
reg_val = test.register[reg_address]

correct_input = False
while(not correct_input):
	user_input = input("23.02: Is LED on?(y/n): ")
	if ("y" in user_input):
		correct_input = True
		passed = False
	elif ("n" in user_input):
		correct_input = True
	else:
		print("23.02: Incorrect input. Type y or n")

if (passed and reg_val == 0x00):
	print("23.02: passed")
else:
	print("23.02: failed. Received data: ", reg_val)



print("-----24.01-----")
# First move servo to 270 degrees 
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(270)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("24.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("24.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break


attempts = 0
while(attempts < 3):
	try:
		test.set_compliance(20, 1, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("24.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("24.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Move servo to 90 degrees (CW motion)
attempts = 0
while(attempts < 3):
	try:
		test.move(90)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("24.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("24.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['cw_compliance_margin']
reg_address2 = AX18A.address['cw_compliance_slope']
reg_val = (test.register[reg_address], test.register[reg_address2])

correct_input = False
while(not correct_input):
	user_input = input("24.01: Roughly 20 degree error?(y/n): ")
	if ("y" in user_input):
		correct_input = True
	elif ("n" in user_input):
		correct_input = True
		passed = False
	else:
		print("24.01: Incorrect input. Type y or n")

if (passed and reg_val == (0x14, 0x02)):
	print("24.01: passed")
else:
	print("24.01: failed. Received data: ", reg_val)


print("-----24.02-----")
passed = True
attempts = 0
while(attempts < 3):
	try:
		test.set_compliance(20, 7, AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("24.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("24.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Move servo to 270 degrees (CCW motion)
attempts = 0
while(attempts < 3):
	try:
		test.move(270)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("24.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("24.02: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['ccw_compliance_margin']
reg_address2 = AX18A.address['ccw_compliance_slope']
reg_val = (test.register[reg_address], test.register[reg_address2])

correct_input = False
while(not correct_input):
	user_input = input("24.02: Roughly 20 degree error?(y/n)(Remember aditional tests at this point): ")
	if ("y" in user_input):
		correct_input = True
	elif ("n" in user_input):
		correct_input = True
		passed = False
	else:
		print("24.02: Incorrect input. Type y or n")

if (passed and reg_val == (0x14, 0x02)):
	print("24.02: passed")
else:
	print("24.02: failed. Received data: ", reg_val)

# Reset compliance
while(attempts < 3):
	try:
		test.set_compliance(0, 5, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("Intermission: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("Intermission: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		test.set_compliance(0, 5, AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("Intermission_2: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("Intermission_2: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break


print("-----25.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_limit(5)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("25.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("25.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['torque_limit_h']
reg_val = test.register[reg_address] << 8 + test.register[reg_address-1]

# Move servo to 90 degrees 
attempts = 0
while(attempts < 3):
	try:
		test.move(90)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("25.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("25.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

correct_input = False
while(not correct_input):
	user_input = input("25.01: Servo moving?(y/n): ")
	if ("y" in user_input):
		correct_input = True
		passed = False
	elif ("n" in user_input):
		correct_input = True
	else:
		print("25.01: Incorrect input. Type y or n")

if (passed):
	print("25.01: passed. Received data: ", reg_val)
else:
	print("25.01: failed. Received data: ", reg_val)


print("-----25.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_limit(100)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("25.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("25.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['torque_limit_h']
reg_val = test.register[reg_address] << 8 + test.register[reg_address-1]

correct_input = False
while(not correct_input):
	user_input = input("25.02: Servo moving?(y/n): ")
	if ("y" in user_input):
		correct_input = True
	elif ("n" in user_input):
		correct_input = True
		passed = False
	else:
		print("25.02: Incorrect input. Type y or n")

if (passed):
	print("25.02: passed. Received data: ", reg_val)
else:
	print("25.02: failed. Received data: ", reg_val)


print("-----26.01-----")
attempts = 0
while(attempts < 3):
	try:
		test.set_punch(0x0200)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("26.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("26.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

reg_address = AX18A.address['punch_h']
reg_val = test.register[reg_address] << 8 + test.register[reg_address-1]

if (reg_val == 0x0200):
	print("26.01: passed")
else:
	print("26.01: failed. Received data: ", reg_val)


print("-----27.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(90, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("27.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("27.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_angle_limit(AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("27.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("27.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("27.01: test complete. Received data: ", return_val)
else:
	print("27.01: failed")


print("-----27.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_angle_limit(270, AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("27.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("27.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_angle_limit(AX18A.CCW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("27.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("27.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("27.02: test complete. Received data: ", return_val)
else:
	print("27.02: failed")


print("-----28.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_max_torque(70)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("28.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("28.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_max_torque()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("28.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("28.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("28.01: test complete. Received data: ", return_val)
else:
	print("28.01: failed")


print("-----29.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_alarm("led", "Angle Limit", "Range")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("29.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("29.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_alarm("led")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("29.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("29.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Check correct errors returned
for err in return_val:
	if(not any(err in test_err for test_err in ("Angle Limit", "Range"))):
		passed = False

if (passed):
	print("29.01: passed")
else:
	print("29.01: failed. Received data: ", return_val)

print("-----29.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_alarm("shutdown", "Overheating", "Input Voltage")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("29.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("29.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_alarm("shutdown")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("29.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("29.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Check correct errors returned
for err in return_val:
	if(not any(err in test_err for test_err in ("Overheating", "Input Voltage"))):
		passed = False

if (passed):
	print("29.02: passed")
else:
	print("29.02: failed. Received data: ", return_val)


print("-----30.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_enable(True)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("30.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("30.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_torque_enable()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("30.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("30.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed and return_val = True):
	print("30.01: passed")
else:
	print("30.01: failed. Received data: ", return_val)


print("-----31.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_compliance(5, 5, AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("31.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("31.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_compliance(AX18A.CW)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("31.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("31.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("31.01: test complete. Received data: ", return_val)
else:
	print("31.01: failed")

print("-----32.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.set_torque_limit(60)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("32.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("32.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_torque_limit()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("32.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("32.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("32.01: test complete. Received data: ", return_val)
else:
	print("32.01: failed")


print("-----33.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(120)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("33.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("33.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_position(time="goal")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("33.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("33.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("33.01: test complete. Received data: ", return_val)
else:
	print("33.01: failed")


print("-----33.02-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(180)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("33.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("33.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val = test.get_position()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("33.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("33.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("33.02: test complete. Received data: ", return_val)
else:
	print("33.02: failed")


print("-----34.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		test.move(60, speed=10)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("34.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("34.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Let servo start moving
AX18A.wait(0.4)

while(attempts < 3):
	try:
		return_val = test.get_speed()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("34.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("34.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("34.01: test complete. Received data: ", return_val)
else:
	print("34.01: failed")


print("-----35.01-----")
attempts = 0
passed = True
# Enable torque
while(attempts < 3):
	try:
		test.set_torque_enable(True)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("35.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("35.01: Communication error")
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

input("Provide force in CCW direction. Press enter to continue ")

while(attempts < 3):
	try:
		return_val = test.get_load()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("35.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("35.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("35.01: test complete. Received data: ", return_val)
else:
	print("35.01: failed")


print("-----35.02-----")
attempts = 0
passed = True

input("Provide force in CW direction. Press enter to continue ")

while(attempts < 3):
	try:
		return_val = test.get_load()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("35.02: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("35.02: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("35.02: test complete. Received data: ", return_val)
else:
	print("35.02: failed")


print("-----36.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		return_val = test.get_volt()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("36.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("36.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("36.01: test complete. Received data: ", return_val)
else:
	print("36.01: failed")

print("-----37.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		return_val = test.get_temperature()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("37.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("37.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed):
	print("37.01: test complete. Received data: ", return_val)
else:
	print("37.01: failed")


print("-----38.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		return_val = test.get_registered()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("38.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("38.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		test.move(180, method="reg")
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("38.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("38.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		return_val2 = test.get_registered()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("38.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("38.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed and return_val == False and return_val2 = True):
	print("38.01: passed")
else:
	print("38.01: failed. Received data: ", return_val, " and ", return_val2)

# Activate action to not have it waiting
brod.action()
AX18A.wait(2)

print("-----39.01-----")
attempts = 0
passed = True
while(attempts < 3):
	try:
		return_val = test.get_moving()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("39.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("39.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

while(attempts < 3):
	try:
		test.move(270)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("39.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("39.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

# Let servo start moving
AX18A.wait(0.4)

while(attempts < 3):
	try:
		return_val2 = test.get_moving()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("39.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("39.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed and return_val == False and return_val2 = True):
	print("39.01: passed")
else:
	print("39.01: failed. Received data: ", return_val, " and ", return_val2)


print("-----40.01-----")
attempts = 0
passed = True
# Enable torque
while(attempts < 3):
	try:
		test.set_punch(0x0030)
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("40.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("40.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

input("Provide force in CCW direction. Press enter to continue ")

while(attempts < 3):
	try:
		return_val = test.get_punch()
	except (AX18A.ServoError, AX18A.CommError) as err:
		attempts += 1
		print("40.01: Attempt ", attempts, " failed. Error: ", err.args[1])
		if (attempts == 2):
			print("40.01: Communication error")
			passed = False
		# Have a tiny delay before next try
		AX18A.wait(delay_time)
	else:
		break

if (passed and return_val == 0x0030):
	print("40.01: passed")
else:
	print("40.01: failed. Received data: ", return_val)