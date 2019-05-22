import RPi.GPIO as gpio

gpio.setmode(gpio.BOARD)
gpio.setup(37, gpio.OUT)
gpio.cleanup()
