import serial

portPath ='/dev/ttyUSB0'
baud = 115200
data_file = "data.txt"
prbs_file = "prbs.txt"
orientation_file = "orientation.txt"
count = 0
number_of_samples = 200

ser = serial.Serial(portPath, baud)

# reads file to list
prbs = open(prbs_file, 'r')
prbs_list = prbs.read().split()[:number_of_samples]
prbs.close()
print('len(prbs_list) = ' + str(len(prbs_list)))

# opens file to write
orientation = open(orientation_file, 'w')

# waits to talk to arduino
print(ser.readline())

raw_input("Press Enter to send PRBS data...")
# sends list length and then each element
for sample in prbs_list:
	ser.write(sample)
	print(ser.readline())

# waits for received confirmation
print(ser.readline())

# starts communication
for sample in prbs_list:
	#print(ser.readline())
	orientation.write(ser.readline())

ser.close()
orientation.close()
