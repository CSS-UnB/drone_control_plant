import serial

portPath ='/dev/ttyUSB0'
baud = 115200
file_name = "data.txt"
count = 0
number_of_samples = 1000

ser = serial.Serial(portPath, baud)
writer = open(file_name, 'w')

line = ser.readline()
while line == -1:
	pass

print(line)

raw_input("Press Enter to continue...")

ser.write('1')

while(count < number_of_samples):
	line = ser.readline()
	print(line)
	if line != 'done':
		writer.write(line)
	else:
		break
	count += 1

ser.close()
writer.close()

