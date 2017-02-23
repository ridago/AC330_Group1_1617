from Dynamixel import AX18A

attempts = 0
while (attempts < 4):
	try:
		grip = AX18A(2)
		handRot = AX18A(3)
		handL = AX18A(4)
		handR = AX18A(5)
		elbowL = AX18A(6)
		elbowR = AX18A(7)
		rot = AX18A(8)
		brod = AX18A(AX18A.broadcast_ID)
	except AX18A.AX18A_error:
		attempts += 1
		print("Initialization failed, trying again")
	else:
		attempts = 5

handRot.move(180, AX18A.fast, method="reg")

elbowL.move(110, AX18A.slow, method="reg")
elbowR.move(360-110, AX18A.slow, method="reg")
brod.action()
AX18A.wait(1)

handL.move(90, AX18A.slow, method="reg")
handR.move(360-90, AX18A.slow, method ="reg")
brod.action()
AX18A.wait(2)

rot.move(230, AX18A.medium)
AX18A.wait(1)
rot.move(310, AX18A.medium)
AX18A.wait(1)
rot.move(270, AX18A.medium)
AX18A.wait(1)

handRot.move(250, AX18A.fast)
AX18A.wait(1)
handRot.move(110, AX18A.fast)
AX18A.wait(1)
handRot.move(180, AX18A.fast)
AX18A.wait(1)

grip.move(320, AX18A.fast)
AX18A.wait(1)
grip.move(256, AX18A.fast)
AX18A.wait(3)

elbowL.move(64, AX18A.slow, method="reg")
elbowR.move(360-64, AX18A.slow, method="reg")
handL.move(61, AX18A.slow, method="reg")
handR.move(360-61, AX18A.slow, method="reg")
brod.action()