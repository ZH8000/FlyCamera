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
ft232h.setup(12, GPIO.OUT)  # Make pin C4 a digital output.
ft232h.setup(13, GPIO.OUT)  # Make pin C5 a digital output.

# Loop turning the LED on and off and reading the input state.
print 'Press Ctrl-C to quit.'
while True:
	# Set pin C0 to a high level so the LED turns on.
	ft232h.output(12, GPIO.HIGH)
	ft232h.output(13, GPIO.HIGH)
	# Sleep for 1 second.
	time.sleep(1)

	# Set pin C0 to a low level so the LED turns off.
	ft232h.output(12, GPIO.LOW)
	ft232h.output(13, GPIO.LOW)
	# Sleep for 1 second.
	time.sleep(1)
