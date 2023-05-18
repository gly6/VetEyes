#use to check if the buttons are working 
import RPi.GPIO as GPIO 
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    time.sleep(0.2)
    if (not GPIO.input(4)):
        print("Button 4 has been pressed")
    if (not GPIO.input(17)):
        print("Button 17 has been pressed")
    if (not GPIO.input(18)):
        print("Button 18 has been pressed")
    if (not GPIO.input(27)):
        print("Button 27 has been pressed")
    if (not GPIO.input(22)):
        print("Button 22 has been pressed")

GPIO.cleanup()