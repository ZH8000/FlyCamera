import time

oldValue = 0 # no signal
# Loop turning the LED on and off and reading the input state.
print 'Press Ctrl-C to quit.'
while True:
	time.sleep(2)
	file = open("gpio_in", "r+")
	print file.readline();
	file = open("gpio_in", "w")
	file.write("1")
	file.close()
	print "1"

	time.sleep(2)
	file = open("gpio_in", "r")
	print file.readline();
	file = open("gpio_in", "w")
	file.write("0")
	file.close()
	print "0"
