# PIAPA Robot
# Grman Rodriguez, Edwin Acosta
# Obstacle avoidance
# Phase 2: OOP, Sensing and routing
# --------------------------------------------------
# Libraries
import numpy as np  # numpy will handle arrays
import math  # math library for operations
import RPi.GPIO as GPIO  # Raspberry GPIO pins
import time  # time library for delays
# --------------------------------------------------
# Classes


# The Scanner class will control the sensors and provide a map array
class Scanner:

    # scanMap will get a new map array, with distance measured from -45° to 135° with the precision allowed by the sensor
    # It is assumed that sensor 2 looks to the front, while sensors 1 and 3 are 90° to the left and 90° to the right respectively
    # The output is an array with 2 columns. The first corresponds to the angle and the second one the distance to the sensor in said direction
    def scanMap(self):
        newmap = np.empty((0, 2), float)
        for x in xrange(0, 45):
            self.setSensorAngle(x)
            dist = self.getDistance(2)
            newmap = np.vstack((newmap, [(x + 90) * math.pi / 180, dist]))
            dist = self.getDistance(1)
            newmap = np.vstack((newmap, [(x + 180) * math.pi / 180, dist]))
            dist = self.getDistance(3)
            newmap = np.vstack((newmap, [x * math.pi / 180, dist]))
        for x in xrange(-45, 0):
            self.setSensorAngle(x)
            dist = self.getDistance(2)
            newmap = np.vstack((newmap, [(x + 90) * math.pi / 180, dist]))
            dist = self.getDistance(1)
            newmap = np.vstack((newmap, [(x + 180) * math.pi / 180, dist]))
            dist = self.getDistance(3)
            newmap = np.vstack((newmap, [x * math.pi / 180, dist]))
        return newmap

    # getDistance starts the sensor and gets a reading of the distance in its current direction
    def getDistance(self, sensor):
        return 15.0

    # setSensorAngle moves the sensor array 'ang' degrees to the left
    def setSensorAngle(self, ang):
        angle = ang * math.pi / 180


# The Rover class will handle vehicle movement and create and update routes
class Rover:

    # The variables x and y set the target the Rover should move towards
    x = 0
    y = 0
    RLF = 19  # The forward-moving end of the rear left motor is connected to GPIO pin 19
    RLB = 21  # The backwards-moving end of the rear left motor is connected to GPIO pin 21
    RRF = 8  # The forward-moving end of the rear right motor is connected to GPIO pin 8
    RRB = 10  # The backwards-moving end of the rear right motor is connected to GPIO pin 10
    FLF = 11  # The forward-moving end of the front left motor is connected to GPIO pin 11
    FLB = 13  # The backwards-moving end of the front left motor is connected to GPIO pin 13
    FRF = 16  # The forward-moving end of the front right motor is connected to GPIO pin 16
    FRB = 18  # The backwards-moving end of the front right motor is connected to GPIO pin 13
    Turn = 5.8  # Time needed by the robot to make a 360 degree turn (TUNED)
    Straight = 1  # Time needed by the robot to advance 1 meter (TUNED)

    def __init__(self):
        # Setup pins as outputs
        GPIO.setmode(GPIO.BOARD)  # GPIO Configuration
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    # Calibrate the Turn parameter
    def calibrateTurn(self, Turn):
        self.Turn = Turn

    # Calibrate the Straight parameter
    def calibrateStraight(self, Straight):
        self.Straight = Straight

    # The following 8 functions change the pins used for each wheel if needs be
    def setRLF(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.RLF = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setRLB(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.RLB = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setRRF(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.RRF = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setRRB(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.RRB = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setFLF(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.FLF = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setFLB(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.FLB = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setFRB(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.FRB = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    def setFRL(self, pin):
        GPIO.cleanup()  # Sadly as far as I know you can't clean up a single pin, so they must all be erased and reconfigured
        self.FRL = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RLF, GPIO.OUT)
        GPIO.setup(self.RLB, GPIO.OUT)
        GPIO.setup(self.RRF, GPIO.OUT)
        GPIO.setup(self.RRB, GPIO.OUT)
        GPIO.setup(self.FLF, GPIO.OUT)
        GPIO.setup(self.FLB, GPIO.OUT)
        GPIO.setup(self.FRF, GPIO.OUT)
        GPIO.setup(self.FRB, GPIO.OUT)

    # Function to move forward
    def forw(self, a):
        # All wheels must move forward
        self.RL(1)
        self.RR(1)
        self.FL(1)
        self.FR(1)
        # Make the movement for "a" meters
        time.sleep(a * 1 / self.Straight)
        # Now stop
        self.RL(0)
        self.RR(0)
        self.FL(0)
        self.FR(0)

    # Function to move backwards
    def backw(self, a):
        # All wheel must move backwards
        self.RL(2)
        self.RR(2)
        self.FL(2)
        self.FR(2)
        # Make the movement for "a" meters
        time.sleep(a * 1 / self.Straight)
        # Now stop
        self.RL(0)
        self.RR(0)
        self.FL(0)
        self.FR(0)

    # Function to turn left
    def left(self, ang):
        # To turn right we maintain the front wheels off
        self.FL(0)
        self.FR(0)
        # The rear left wheel must move backwards
        self.RL(2)
        # The rear right wheel must move forward
        self.RR(1)
        # A delay is added, that defines the angle
        time.sleep(self.Turn * ang / 360)
        # The turn is stopped
        self.RL(0)
        self.RR(0)

    # Function to turn right
    def right(self, ang):
        # To turn right we maintain the front wheels off
        self.FL(0)
        self.FR(0)
        # The rear left wheel must move backwards
        self.RL(2)
        # The rear right wheel must move forward
        self.RR(1)
        # A delay is added, that defines the angle
        time.sleep(self.Turn * ang / 360)
        # The turn is stopped
        self.RL(0)
        self.RR(0)

    # Function to stop all wheels
    def noMove(self):
        # Stop all wheels
        self.FL(0)
        self.FR(0)
        self.RL(0)
        self.RR(0)

    # Rear left wheel control
    def RL(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            GPIO.output(self.RLF, GPIO.HIGH)
            GPIO.output(self.RLB, GPIO.LOW)
        elif A == 2:
            GPIO.output(self.RLF, GPIO.LOW)
            GPIO.output(self.RLB, GPIO.HIGH)
        elif A == 0:
            GPIO.output(self.RLF, GPIO.LOW)
            GPIO.output(self.RLB, GPIO.LOW)

    # Rear right wheel control
    def RR(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            GPIO.output(self.RRF, GPIO.HIGH)
            GPIO.output(self.RRB, GPIO.LOW)
        elif A == 2:
            GPIO.output(self.RRF, GPIO.LOW)
            GPIO.output(self.RRB, GPIO.HIGH)
        elif A == 0:
            GPIO.output(self.RRF, GPIO.LOW)
            GPIO.output(self.RRB, GPIO.LOW)

    # Front right wheel control
    def FR(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            GPIO.output(self.FRF, GPIO.HIGH)
            GPIO.output(self.FRB, GPIO.LOW)
        elif A == 2:
            GPIO.output(self.FRF, GPIO.LOW)
            GPIO.output(self.FRB, GPIO.HIGH)
        elif A == 0:
            GPIO.output(self.FRF, GPIO.LOW)
            GPIO.output(self.FRB, GPIO.LOW)

    # Front left wheel control
    def FL(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            GPIO.output(self.FLF, GPIO.HIGH)
            GPIO.output(self.FLB, GPIO.LOW)
        elif A == 2:
            GPIO.output(self.FLF, GPIO.LOW)
            GPIO.output(self.FLB, GPIO.HIGH)
        elif A == 0:
            GPIO.output(self.FLF, GPIO.LOW)
            GPIO.output(self.FLB, GPIO.LOW)

    # setTarget will update x and y to change the desired target
    def setTarget(self, x, y):
        # Distance will be limited to 4 meters as of now
        if (math.pow(x, 2) + math.pow(y, 2) > 16):
            ang = math.atan2(y, x)
            x = 4 * math.cos(ang)
            y = 4 * math.cos(ang)
        self.x = x
        self.y = y

    # Function to handle object destruction and general pin cleanup when needed
    def quit(self):
        GPIO.cleanup()

    # TO BE WRITTEN: goToTarget and route functions
