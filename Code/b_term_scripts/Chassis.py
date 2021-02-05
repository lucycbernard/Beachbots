import RPi.GPIO as GPIO
from AdafruitIMU import AdafruitIMU
from time import sleep

class Chassis:

    def __init__(self, RPWMF, RPWMB, LPWMF, LPWMB):
        self.RPWMF = RPWMF
        self.RPWMB = RPWMB
        self.LPWMF = LPWMF
        self.LPWMB = LPWMB
        self.IMU = AdafruitIMU()

        GPIO.setwarnings(False)  # disable warnings
        GPIO.setmode(GPIO.BOARD)  # set pin numbering system

        GPIO.setup(self.RPWMF, GPIO.OUT)
        GPIO.setup(self.RPWMB, GPIO.OUT)
        GPIO.setup(self.LPWMF, GPIO.OUT)
        GPIO.setup(self.LPWMB, GPIO.OUT)

        self.pi_rpwmf = GPIO.PWM(self.RPWMF, 1000)  # create PWM instance with frequency
        self.pi_rpwmb = GPIO.PWM(self.RPWMB, 1000)  # create PWM instance with frequency
        self.pi_lpwmf = GPIO.PWM(self.LPWMF, 1000)  # create PWM instance with frequency
        self.pi_lpwmb = GPIO.PWM(self.LPWMB, 1000)  # create PWM instance with frequency

        self.pi_rpwmf.start(0)  # start PWM of required Duty Cycle
        self.pi_rpwmb.start(0)  # start PWM of required Duty Cycle
        self.pi_lpwmf.start(0)  # start PWM of required Duty Cycle
        self.pi_lpwmb.start(0)  # start PWM of required Duty Cycle

    def drive(self, speed):
        if speed > 0:
            self.pi_rpwmf.ChangeDutyCycle(speed)
            self.pi_lpwmf.ChangeDutyCycle(speed)
        elif speed < 0:
            self.pi_rpwmb.ChangeDutyCycle(abs(speed))
            self.pi_lpwmb.ChangeDutyCycle(abs(speed))

    def estop(self):
        self.pi_rpwmf.ChangeDutyCycle(0)
        self.pi_lpwmf.ChangeDutyCycle(0)
        self.pi_rpwmb.ChangeDutyCycle(0)
        self.pi_lpwmb.ChangeDutyCycle(0)

    def turn_left(self, time, speed): #Turn is the angle at which one wants tu turn bot
        for x in range(time):
            self.pi_rpwmf.ChangeDutyCycle(speed)
            self.pi_lpwmb.ChangeDutyCycle(speed)

    def turn_right(self, time, speed): #Turn is the angle at which one wants tu turn bot
        for x in range(time):
            self.pi_rpwmb.ChangeDutyCycle(speed)
            self.pi_lpwmf.ChangeDutyCycle(speed)

    def turn(self, time, speed):
        if speed > 0: #Turn right
            for x in range(time):
                self.pi_rpwmb.ChangeDutyCycle(speed)
                self.pi_lpwmf.ChangeDutyCycle(speed)

        elif speed < 0: #Turn left
            for x in range(time):
                self.pi_rpwmf.ChangeDutyCycle(abs(speed))
                self.pi_lpwmb.ChangeDutyCycle(abs(speed))

        else:
            self.pi_rpwmf.ChangeDutyCycle(0)
            self.pi_lpwmf.ChangeDutyCycle(0)
            self.pi_rpwmb.ChangeDutyCycle(0)
            self.pi_lpwmb.ChangeDutyCycle(0)

    def set_motor_power(self, right, left):
        self.pi_rpwmf.ChangeDutyCycle(right)
        self.pi_lpwmf.ChangeDutyCycle(left)
        self.pi_rpwmb.ChangeDutyCycle(right)
        self.pi_lpwmb.ChangeDutyCycle(left)

    def limit(self, val, minVal, maxVal):
        return min(max(val, minVal), maxVal)

    def point_turn_IMU(self, currentAngle, wantedAngle, decelerationAngle, speed):
        relativePointAngle = self.IMU.angleWrap(wantedAngle - currentAngle)
        turn_speed = (relativePointAngle / decelerationAngle) * speed
        turn_speed = self.limit(turn_speed, -speed, speed)
        turn_speed = max(turn_speed, 50.0)
        self.set_motor_power(-turn_speed, turn_speed)

    def swing_turn_IMU(self, currentAngle, wantedAngle, decelerationAngle, speed):
        relativePointAngle = self.IMU.angleWrap(wantedAngle - currentAngle)
        turn_speed = (relativePointAngle / decelerationAngle) * speed
        turn_speed = self.limit(turn_speed, -speed, speed)
        turn_speed = max(turn_speed, 50.0)
        if relativePointAngle > 0:
            self.set_motor_power(0, turn_speed)
        else:
            self.set_motor_power(turn_speed, 0)


    def drive_IMU(self, currentAngle, wantedAngle, decelerationAngle, speed):
        relativePointAngle = self.angleWrap(wantedAngle - currentAngle)
        turn_speed = (relativePointAngle / decelerationAngle) * speed
        turn_speed = self.limit(turn_speed, -speed, speed)
        turn_speed = max(turn_speed, 50.0)
