#!/usr/bin python
import rospy
import RPi.GPIO as GPIO
from time import time as time



# Check that high direction is reverse, and low direction is forwards
# 
class StepperDriver:
    # To use this module, you should just need to initialize your stepper driver instance,
    # call the home function and wait for it to finish, and then call the setWantedStepsFromHome(wantedSteps) 
    # function to set the wanted position. The run() function must be called in a loop
    def __init__(self, CLKPIN, DIRPIN, ENPIN, HOMEPIN):
        self.ID = rospy.get_param("~ID")  # smallbot ID
        self.CLKPIN = rospy.get_param("~CLKPIN")  # clock pin for stepper driver
        self.DIRPIN = rospy.get_param("~DIRPIN")  # direction pin for stepper driver
        self.ENPIN = rospy.get_param("~ENPIN")  # Enable pin for stepper driver
        self.HOMEPIN = rospy.get_param("~HOMEPIN")  # homing switch pin for stepper
        self.currentStepsFromHome = 0
        self.wantedStepsFromHome = 0
        self.maxStepsPerSecond = 1000 #Assuming 1ms minimum step time
        self.steppingDirection = False
        self.clkVal = False
        self.lastTimeStepped = time()
        self.debug = False
        self.mode = "DISABLED"
        self.arrivedAtPosition = False

        #Disable Warnings
        GPIO.setwarnings(False)

        #Set Pin numbering system
        GPIO.setmode(GPIO.BCM)

        #Set control pins as outputs
        GPIO.setup(self.CLKPIN, GPIO.OUT)
        GPIO.setup(self.DIRPIN, GPIO.OUT)
        GPIO.setup(self.ENPIN, GPIO.OUT)

        #Set homing pin as input
        GPIO.setup(self.HOMEPIN, GPIO.IN)

# Need to keep track of steps
# Homing routine
# Step to position
# Enable disable functions

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

    #Runs homing routine and zeros stepping position
    def home(self):
        pass

    #Moves to given number of steps
    def setWantedStepsFromHome(self,wantedSteps):
        self.wantedStepsFromHome = wantedSteps

    def setStepDirection(self, increasing):
        if(increasing):
            GPIO.output(self.DIRPIN, GPIO.LOW)
            self.steppingDirection = True
        else:
            GPIO.output(self.DIRPIN, GPIO.HIGH)
            self.steppingDirection = False
    
    #Change value of clk pin
    def oscillateClk(self):
        self.clkVal = not self.clkVal
        GPIO.output(self.CLKPIN, self.clkVal)

        if(self.steppingDirection): ############################### Might need to make the steps increment only on rising edge depending on if driver steps on either edge or just rising
            self.currentStepsFromHome = self.currentStepsFromHome + 1
        else:
            self.currentStepsFromHome = self.currentStepsFromHome - 1

        if(self.debug):
            # print("Clock pin changed to: " + str(self.clkVal) + " at time " + str(time()))
            print(self.currentStepsFromHome)

    #To run in a loop (State Machine)
    def run(self):
        #If stepper is disabled, dont do anything
        if(self.mode == "DISABLED"):
            print("Running disabled")
            return
        if(self.mode == "HOMING"):
            print("Running homing")
            pass
            return

        #If stepper is enabled, and the wanted steps from home are not equal to the current steps from home, and enough time has elapsed
        elif(self.wantedStepsFromHome != self.currentStepsFromHome and self.wantedStepsFromHome != None and self.currentStepsFromHome != None and time() - (1/self.maxStepsPerSecond) > self.lastTimeStepped):
            self.arrivedAtPosition = False
            self.lastTimeStepped = time()
            #Set the direction for the stepper to drive
            self.setStepDirection(self.wantedStepsFromHome > self.currentStepsFromHome)

            self.oscillateClk()

        elif(self.wantedStepsFromHome == self.currentStepsFromHome):
            self.arrivedAtPosition = True
            if(self.debug):
                print("Stepper motor in position")

if __name__ == "__main__":
    stepper = StepperDriver()
    while not rospy.is_shutdown():
        if(stepper.arrivedAtPosition):
            rospy.sleep(0.1)
        else:
            stepper.run()