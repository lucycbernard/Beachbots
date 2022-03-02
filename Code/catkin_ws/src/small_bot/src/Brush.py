import queue
import rospy
import RPi.GPIO as GPIO
from time import time as time
from std_msgs.msg import Float32, Bool, String

class Brush:
    """
    Constructor
    """
    def __init__(self):
        rospy.init_node("Brush", anonymous=True)

        # Fetch parameters from ros parameter service
        self.ID = rospy.get_param("~ID") # smallbot ID
        self.CLKPIN = rospy.get_param("~CLKPIN_BRUSH") # clock pin for stepper driver
        self.DIRPIN = rospy.get_param("~DIRPIN_BRUSH") # direction pin for stepper driver
        self.ENPIN = rospy.get_param("~ENPIN_BRUSH") # Enable pin for stepper driver
        
        rospy.delete_param("~CLKPIN_BRUSH")
        rospy.delete_param("~DIRPIN_BRUSH")
        rospy.delete_param("~ENPIN_BRUSH")

        self.Stepper = StepperDriver(self.CLKPIN,self.DIRPIN,self.ENPIN,0, 0)

        rospy.Subscriber("Smallbot_" + self.ID + "/Brush/Speed", Float32, self.setSpeed)

        self.ChatterPublisher = rospy.Publisher("Smallbot_" + self.ID + "/Chatter", String, queue_size=10)

        self.speed = 0
        
        rospy.sleep(1)
        
        self.ChatterPublisher.publish("Brush Initialized")

    def setSpeed(self, data):
        """
        Updates the value of speed wanted for the brush
        Args:
            data (sts_msgs.msg.Float32): Wanted brush speed ~-500 to 500. Represents max steps per second *2
        """
        self.ChatterPublisher.publish("Setting Brush Speed")
        self.speed = data.data # Receive speed data
        self.Stepper.setSpeed(self.speed) # Set stepper speed

    
    # Main loop here
    def run(self):
        self.Stepper.run() #State machine for stepper

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
        self.steppingDirection = False
        self.clkVal = False
        self.lastTimeStepped = time()
        self.debug = False
        self.mode = "CONTINUOUS"
        self.arrivedAtPosition = False

        #Disable Warnings
        GPIO.setwarnings(False)

        #Set Pin numbering system
        GPIO.setmode(GPIO.BOARD)

        #Set control pins as outputs
        GPIO.setup(self.CLKPIN, GPIO.OUT)
        GPIO.setup(self.DIRPIN, GPIO.OUT)
        GPIO.setup(self.ENPIN, GPIO.OUT)

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
            self.maxStepsPerSecond = 0
            return

        self.maxStepsPerSecond = abs(speed)



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
            # print("Clock pin changed to: " + str(self.clkVal) + " at time " + str(time()))
            print(self.currentStepsFromHome)

    #To run in a loop (State Machine)
    def run(self):
        #If stepper is disabled, dont do anything
        if(self.mode == "DISABLED"):
            return

        # Otherwise, if the motor is set to continuous, run it at the speed
        if(self.mode == "CONTINUOUS" and time() - (1/self.maxStepsPerSecond) > self.lastTimeStepped and self.maxStepsPerSecond != 0):
            self.arrivedAtPosition = False
            self.lastTimeStepped = time()
            self.oscillateClk()
            return
        
        

if __name__ == "__main__":
    brush = Brush()
    while not rospy.is_shutdown():
        brush.run()