import serial

portPath ='/dev/ttyUSB1'
baud = 115200
data_file = "data.txt"
prbs_file = "prbs.txt"
count = 0
number_of_samples = 250

ser = serial.Serial(portPath, baud)

# reads file to list
prbs = open(prbs_file, 'r')
prbs_list = prbs.read().split()[:number_of_samples]
prbs.close()
print('len(prbs_list) = ' + str(len(prbs_list)))

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
	print(ser.readline())

ser.close()
'''
writer = open(data_file, 'w')

writer.close()
'''
