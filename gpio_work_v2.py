# Import standard Python time library.
import time

# Import GPIO and FT232H modules.
import Adafruit_GPIO as GPIO
import Adafruit_GPIO.FT232H as FT232H

# Temporarily disable the built-in FTDI serial driver on Mac &amp; Linux platforms.
FT232H.use_FT232H()

# Create an FT232H object that grabs the first available FT232H device found.
ft232h = FT232H.FT232H()

# Configure digital inputs and outputs using the setup function.
# Note that pin numbers 0 to 15 map to pins D0 to D7 then C0 to C7 on the board.
ft232h.setup(7, GPIO.IN)   # Make pin D7 a digital input.
ft232h.setup(8, GPIO.OUT)  # Make pin C0 a digital output.
ft232h.setup(11, GPIO.OUT)  # Make pin C3 a digital output. 
ft232h.setup(12, GPIO.OUT)  # Make pin C4 a digital output. (Bad product)
ft232h.setup(13, GPIO.OUT)  # Make pin C5 a digital output. (Good product)

oldValue = 0 # no signal

# init file
file = open("gpio_in_python", "w")
file.write("0")
file.close()

# Loop turning the LED on and off and reading the input state.
print 'Press Ctrl-C to quit.'
while True:
	file = open("gpio_result_cpp", "r");
	result = file.readline()
	if result == "0\n":
		print 'Bad product'
		ft232h.output(12, GPIO.HIGH);
		ft232h.output(13, GPIO.LOW);
	if result == "1\n":
		print 'Good product'
		ft232h.output(13, GPIO.HIGH);
		ft232h.output(12, GPIO.LOW);

	file = open("gpio_out_cpp", "r")
	cppState = file.readline()
	if cppState == "1\n": # move the machine
		print 'HIGH'
		ft232h.output(11, GPIO.HIGH)
	else:
		print 'LOW'       # stop the machine
		ft232h.output(11, GPIO.LOW)

	level = ft232h.input(7)
	if level == GPIO.LOW: # get low, because PLC pull high
		print 'Pin D7 is LOW'
		if oldValue == 1:  # if the old is 1, no action
			print "Get old signal 1"
		else:              # if the old isn't 1, refresh
			oldValue = 1
			print "Get new signal 1"
			file = open("gpio_in_python", "w")
			file.write("1")
			file.close()
			ft232h.output(8, GPIO.HIGH)
	else:                  # get high, because PLC pull low
		print 'Pin D7 is HIGH'
		if oldValue == 0:  # if the old is 0, no action
			print "Get old signal 0"
		else:
			oldValue = 0   # if the old isn't 0, refresh
			print "Get new signal 0"
			file = open("gpio_in_python", "w")
			file.write("0")
			file.close()
			ft232h.output(8, GPIO.LOW)
