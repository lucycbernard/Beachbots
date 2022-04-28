import RPi.GPIO as GPIO
import time


#Disable Warnings
GPIO.setwarnings(False)

#Set Pin numbering system
GPIO.setmode(GPIO.BOARD)

#Set control pins as outputs
#GPIO.setup(self.CLKPIN, GPIO.OUT)
#GPIO.setup(self.DIRPIN, GPIO.OUT)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)



while(True):
    print(GPIO.input(11))
    time.sleep(0.05)
    
