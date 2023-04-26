#!/usr/bin python
import RPi.GPIO as GPIO
from time import time as time


class StepperDriver:
    # To use this module, you should just need to initialize your stepper driver instance,
    # call the home function and wait for it to finish, and then call the setWantedStepsFromHome(wantedSteps) 
    # function to set the wanted position. The run() function must be called in a loop
    def __init__(self, CLKPIN, DIRPIN, ENPIN, HOMEPIN, HOMEDIR):
        self.CLKPIN = CLKPIN  # Pin for stepper clk/step
        self.DIRPIN = DIRPIN # Pin for stepper driver direction
        self.ENPIN = ENPIN # Pin for stepper driver enable
        self.HOMEPIN = HOMEPIN # Pin for the homing switch
        self.HOMEDIR = HOMEDIR
        self.currentStepsFromHome = 0
        self.wantedStepsFromHome = 0
        self.maxStepsPerSecond = 0 #500 has been tested and works
        self.wantedMaxStepsPerSecond = 0
        self.steppingDirection = False
        self.clkVal = False
        self.lastTimeStepped = time()
        self.debug = False
        
        self.arrivedAtPosition = False

        #Disable Warnings
        GPIO.setwarnings(False)

        #Set Pin numbering system
        GPIO.setmode(GPIO.BOARD)

        #Set control pins as outputs
        GPIO.setup(self.CLKPIN, GPIO.OUT)
        GPIO.setup(self.DIRPIN, GPIO.OUT)
        GPIO.setup(self.ENPIN, GPIO.OUT)
        
        self.enable()
        self.mode = "CONTINUOUS"

        if(self.HOMEPIN != 0):
            #Set homing pin as input
            GPIO.setup(self.HOMEPIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) #Setup homing pin as input pullup


    #Enables the stepper driver (sets en pin to high)
    def enable(self):
        GPIO.output(self.ENPIN, GPIO.HIGH)
        self.mode = "ENABLED"

        if(self.debug):
            print("Stepper Driver Enabled")
        

    #Disables the stepper driver (sets en pin to low)
    def disable(self):
        GPIO.output(self.ENPIN, GPIO.LOW)
        self.mode = "DISABLED"

        if(self.debug):
            print("Stepper Driver Disabled")


    def setSpeed(self, speed):
        if(speed < 0):
            self.setStepDirection(0)
        elif(speed > 0):
            self.setStepDirection(1)
        else:
            self.wantedMaxStepsPerSecond = 0
            return

        self.wantedMaxStepsPerSecond = abs(speed)



    def setStepDirection(self, increasing):
        if(increasing):
            GPIO.output(self.DIRPIN, GPIO.HIGH)
            self.steppingDirection = True
        else:
            GPIO.output(self.DIRPIN, GPIO.LOW)
            self.steppingDirection = False
    
    #Change value of clk pin
    def oscillateClk(self):
        self.clkVal = not self.clkVal
        GPIO.output(self.CLKPIN, self.clkVal)

        if(self.clkVal): #If it is the rising edge of the clk signal
            if(self.steppingDirection): ############################### Might need to make the steps increment only on rising edge depending on if driver steps on either edge or just rising
                self.currentStepsFromHome = self.currentStepsFromHome + 1
            else:
                self.currentStepsFromHome = self.currentStepsFromHome - 1

        if(self.debug):
            #print("Clock pin changed to: " + str(self.clkVal) + " at time " + str(time()))
            print(self.currentStepsFromHome)

    #To run in a loop (State Machine)
    def run(self):
        #If stepper is disabled, dont do anything
        if(self.mode == "DISABLED"):
            return

        # Otherwise, if the motor is set to continuous, run it at the speed
        try:
            if(self.maxStepsPerSecond < self.wantedMaxStepsPerSecond):
                self.maxStepsPerSecond = self.maxStepsPerSecond + 1
            elif(self.maxStepsPerSecond > self.wantedMaxStepsPerSecond):
                self.maxStepsPerSecond = self.maxStepsPerSecond - 1
                
                
            if(self.mode == "CONTINUOUS" and time() - (1/self.maxStepsPerSecond) > self.lastTimeStepped and self.maxStepsPerSecond != 0):
                self.arrivedAtPosition = False
                self.lastTimeStepped = time()
                self.oscillateClk()
                return
        except:
            #Catch divide by zero when the max steps per second is zero
            pass
        

if __name__ == "__main__":
    stepper = StepperDriver(13,16,18,0,0)
    stepper.setSpeed(1000)
    while True:
        stepper.run()
        #print(stepper.maxStepsPerSecond)
