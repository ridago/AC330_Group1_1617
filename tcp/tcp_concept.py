import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))

theValue = 5
closeConnection = False

while(1):
	s.listen(1)

	(conn, addr) = s.accept()
	print("Connection address: ", addr)
	while 1:
		data = conn.recv(BUFFER_SIZE)
		if (not data):
			break
		print("received data: ", data)

		try:
			if (data[0] == 0xFF and data[1] == 0xFF):
				data_length = data[2]
				print("Data length: ", data_length)
				if (data_length != (len(data)-3)):
					print("Wrong data length")
					break
				command = data[3]
			else:
				print("Wrong message received")
				break
		except IndexError:
			print("Not enough data received")
			break

		if (command == 2):
			theValue = data[4]
		elif (command == 3):
			closeConnection = True

		checksum = 2+theValue
		checksum = checksum & 0xFF		

		send_data = bytes((0xFF, 0xFF, 0x02, theValue, checksum))

		conn.send(send_data)
	conn.close()
	if (closeConnection):
		break
	print("loop completed")