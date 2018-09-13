import serial

portPath ='/dev/ttyUSB0'
baud = 115200
data_file = "data.txt"
prbs_file = "prbs.txt"
orientation_file = "orientation.txt"
count = 0
number_of_samples = 200
pwm_limits_id = [60, 58]



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

#sends identification pwm limits
ser.write(chr(pwm_limits_id[0]))
print(ser.readline())
ser.write(chr(pwm_limits_id[1]))
print(ser.readline())

# sends binary each element
for sample in prbs_list:
	ser.write(sample)
	print(ser.readline())

# waits for received confirmation
print(ser.readline())

# starts communication
for sample in prbs_list:
	#print(ser.readline())
	#orientation.write(ser.readline() + ' ' + str(int(sample) * (pwm_limits_id[0] - pwm_limits_id[1]) + pwm_limits_id[1]))
	orientation.write(str(int(sample) * (pwm_limits_id[0] - pwm_limits_id[1]) + pwm_limits_id[1]) + ' ' + ser.readline())

ser.close()
orientation.close()
