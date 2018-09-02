writer = open('data.txt', 'w')

for sample in [1, 2, 3, 4, 5, 6, 17]:
	writer.write(str(sample) + '\n')
writer.close()