import queue
import rospy
import RPi.GPIO as GPIO
from time import time as time
from std_msgs.msg import Float32, Bool

class Sifter:
    """
    Constructor
    """
    def __init__(self):
        rospy.init_node("Sifter", anonymous=True)

        # Fetch parameters from ros parameter service
        self.ID = rospy.get_param("~ID") # smallbot ID
        self.CLKPIN = rospy.get_param("~CLKPIN") # clock pin for stepper driver
        self.DIRPIN = rospy.get_param("~DIRPIN") # direction pin for stepper driver
        self.ENPIN = rospy.get_param("~ENPIN") # Enable pin for stepper driver
        self.HOMEPIN = rospy.get_param("~HOMEPIN") # homing switch pin for stepper

        rospy.delete_param("~CLKPIN")
        rospy.delete_param("~DIRPIN")
        rospy.delete_param("~ENPIN")
        rospy.delete_param("~HOMEPIN")

        self.Stepper = StepperDriver(self.CLKPIN,self.DIRPIN,self.ENPIN,self.HOMEPIN)

        rospy.Subscriber("Smallbot_" + self.ID + "/Sifter/Depth", Float32, self.setDepth)
        self.sifterAtDepthPublisher = rospy.Publisher("Smallbot_" + self.ID + "/Sifter/AtDepth", Bool, queue_size=10)

        self.wantedDepth = 0
        self.wasAtDepth = False #If the sifter was at depth last iteration

        self.Stepper.enable()
        rospy.sleep(1)

    def setDepth(self, data):
        """
        Updates the value of depth wanted for the sifter
        Args:
            data (sts_msgs.msg.Float32): Wanted sifter height
        """
        
        self.wantedDepth = data.data
        #Convert from wanted depth in mm to steps
        steps = self.wantedDepth*100
        self.Stepper.setWantedStepsFromHome(steps)

    
    # Main loop here
    def run(self):
        self.Stepper.run() #State machine for stepper
        
        # If the stepper has arrived at its position, and it wasnt there on the last loop iteration
        if(self.Stepper.arrivedAtPosition):
            self.sifterAtDepthPublisher.publish(True)
        # If the stepper is not at the wanted position, and it was there on the last loop iteration
        elif(not self.Stepper.arrivedAtPosition):
            self.sifterAtDepthPublisher.publish(False)

        
        
        
class StepperDriver:
    # To use this module, you should just need to initialize your stepper driver instance,
    # call the home function and wait for it to finish, and then call the setWantedStepsFromHome(wantedSteps) 
    # function to set the wanted position. The run() function must be called in a loop
    def __init__(self, CLKPIN, DIRPIN, ENPIN, HOMEPIN):
        self.CLKPIN = CLKPIN  # Pin for stepper clk/step
        self.DIRPIN = DIRPIN # Pin for stepper driver direction
        self.ENPIN = ENPIN # Pin for stepper driver enable
        self.HOMEPIN = HOMEPIN # Pin for the homing switch
        self.currentStepsFromHome = 0
        self.wantedStepsFromHome = 0
        self.maxStepsPerSecond = 500 #Assuming 2ms minimum step time
        self.steppingDirection = False
        self.clkVal = False
        self.lastTimeStepped = time()
        self.debug = False
        self.mode = "ENABLED"
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
        #GPIO.setup(self.HOMEPIN, GPIO.IN)


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
    sifter = Sifter()
    while not rospy.is_shutdown():
        sifter.run()

        
