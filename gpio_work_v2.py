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

oldValue = 0 # no signal
# Loop turning the LED on and off and reading the input state.
print 'Press Ctrl-C to quit.'
while True:
	# Set pin C0 to a high/low level so the LED turns on/off.
	#ft232h.output(8, GPIO.LOW)
	#time.sleep(1)
	#ft232h.output(8, GPIO.HIGH)
	#time.sleep(1)

	# Read the input on pin D7 and print out if it's high or low.
	#file = open("gpio_out_cpp", "r")
	#cppState = file.readline();
#	if data == "1\n":
#		print "gpio_out_cpp " + data
#	else:
#		print "gpio_out_cpp " + data
	#print "gpio_out_cpp " + cppState
	# get signal from PLC
	#if cppState == "1\n":

	file = open("gpio_out_cpp", "r")
	cppState = file.readline()
	if cppState == "1\n":
		print 'HIGHHIGHHIGHHIGHHIGHHIGHHIGHHIGHHIGHHIGHHIGH'
		ft232h.output(11, GPIO.HIGH)
	else:
		print 'LOWLOWLOWLOWLOWLOWLOWLOWLOWLOWLOWLOWLOWLOWLOW'
		ft232h.output(11, GPIO.LOW)

	level = ft232h.input(7)
	if level == GPIO.LOW: # get low, because PLC pull high
		print 'Pin D7 is LOW! ~~~~~~~~~~~~~~~~~~~~~~~~'
		if oldValue == 1:  # if the old is 1, no action
			print "Get old signal 1"
		else:              # if the old isn't 1, refresh
			oldValue = 1
			print "Get new signal 1"
			file = open("gpio_in_python", "w")
			file.write("1")
			file.close()
			ft232h.output(8, GPIO.HIGH)
	else:                  # get high, no PLC no signal
		print 'Pin D7 is HIGH! ~~~~~~~~~~~~~~~~~~~~~~~~'
		if oldValue == 0:  # if the old is 0, no action
			print "Get old signal 0"
		else:
			oldValue = 0   # if the old isn't 0, refresh
			print "Get new signal 0"
			file = open("gpio_in_python", "w")
			file.write("0")
			file.close()
			ft232h.output(8, GPIO.LOW)
