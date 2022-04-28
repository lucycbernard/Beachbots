import RPi.GPIO as GPIO
import time


#Disable Warnings
GPIO.setwarnings(False)

#Set Pin numbering system
GPIO.setmode(GPIO.BOARD)

#Set control pins as outputs
#GPIO.setup(self.CLKPIN, GPIO.OUT)
#GPIO.setup(self.DIRPIN, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(38, GPIO.HIGH) #Enable stepper
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    
t = 0.002

lastTime = time.time()
lastVal = False
count = 0
direction = 1
GPIO.output(40, direction) # Set Direction
while(True):
    if(time.time() - lastTime > t):
        lastVal = not lastVal
        GPIO.output(37, lastVal)
        lastTime = time.time()
#         print(lastVal)
        count = count + 1
        
        
#     if(count >= 500):
#         direction = not direction
#         GPIO.output(40,direction)
#         count = 0
