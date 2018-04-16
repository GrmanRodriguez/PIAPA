# PIAPA Robot
# Grman Rodriguez, Edwin Acosta
# Movement Manager
# General classes needed
# --------------------------------------------------
# Libraries
import RPi.GPIO as GPIO  # Raspberry GPIO pins
import time  # time library for delays
import serial  # library
# --------------------------------------------------


class MovementManager():
    gridSize = 0.3  # The size of the cells in meters
    RLF = 19  # The forward-moving end of the rear left motor is connected to GPIO pin 19
    RLB = 21  # The backwards-moving end of the rear left motor is connected to GPIO pin 21
    RRF = 22  # The forward-moving end of the rear right motor is connected to GPIO pin 8
    RRB = 24  # The backwards-moving end of the rear right motor is connected to GPIO pin 10
    FLF = 11  # The forward-moving end of the front left motor is connected to GPIO pin 11
    FLB = 13  # The backwards-moving end of the front left motor is connected to GPIO pin 13
    FRF = 16  # The forward-moving end of the front right motor is connected to GPIO pin 16
    FRB = 18  # The backwards-moving end of the front right motor is connected to GPIO pin 13
    turnClockw = 1.92  # Time needed by the robot to make a 360 degree clockwise turn (TUNED)
    turnCounterClockw = 2.1  # Time needed by the robot to make a 360 degree counterclockwise turn (TUNED)
    Straight = 0.9  # Time needed by the robot to advance 1 meter (TUNED)
    FTHC = 36 #The front HC-SR04 Trigger is connected to GPIO pin 36
    FEHC = 37 #The front HC-SR04 Echo is connected to GPIO pin 37
    RTHC = 40 #The right HC-SR04 Trigger is connected to GPIO pin 40 
    REHC = 35 #The right HC-SR04 Echo is connected to GPIO pin 35
    LTHC = 38 #The left HC-SR04 Trigger is connected to GPIO pin 38
    LEHC = 33 #The left HC-SR04 Echo is connected to GPIO pin 33

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
        GPIO.setup(self.FTHC, GPIO.OUT)
        GPIO.output(self.FTHC, False)
        GPIO.setup(self.FEHC, GPIO.IN)
        GPIO.setup(self.RTHC, GPIO.OUT)
        GPIO.output(self.RTHC, False)
        GPIO.setup(self.REHC, GPIO.IN)
        GPIO.setup(self.LTHC, GPIO.OUT)
        GPIO.output(self.LTHC, False)
        GPIO.setup(self.LEHC, GPIO.IN)

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

    # Function to turn
    def turn(self, ang):
        # A small pause is added to make sure there is no drift
        self.FR(0)
        self.FL(0)
        self.RL(0)
        self.RR(0)
        time.sleep(0.2)
        if ang > 0:  # if the angle is positive, turn left.
            # To turn left we move the left wheels backwards
            self.FL(2)
            self.RL(2)
            # To turn left we move the right wheels forwards
            self.FR(1)
            self.RR(1)
            # A delay is added, that defines the angle
            time.sleep(self.turnCounterClockw * abs(ang) / 360)
            # The turn is stopped
            self.FR(0)
            self.FL(0)
            self.RL(0)
            self.RR(0)
        if ang < 0:  # if the angle is negative, turn right.
            # To turn right we move the left wheels forwards
            self.FL(1)
            self.RL(1)
            # To turn right we move the right wheels backwards
            self.FR(2)
            self.RR(2)
            # A delay is added, that defines the angle
            time.sleep(self.turnClockw * abs(ang) / 360)
            # The turn is stopped
            self.FR(0)
            self.FL(0)
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

    # Function to handle object destruction and general pin cleanup when needed
    def quit(self):
        GPIO.cleanup()


class ArmManager(serial.Serial):
    def __init__(self):
        serial.Serial.__init__(self, '/dev/serial0', 115200)
        self.neutralArmPosition = '#0 p2000 s1050 #1 p2450 s300 #2 p3200 s300 #3 p0 '
        self.stretchedArmPosition = '#0 p2000 s1050 #1 p1450 s300 #2 p1800 s300 #3 p2250 s300 '
        self.closedHand = '#4 p2400\r\n'
        self.openedHand = '#4 p900\r\n'
        self.off = '#0 p0 #1 p0 #2 p0 #3 p0 #4 p0\r\n'

    def armOff(self):
        self.write(self.off)

    def no(self):
        self.write(self.neutralArmPosition + self.openedHand)

    def so(self):
        self.write(self.stretchedArmPosition + self.openedHand)

    def nc(self):
        self.write(self.neutralArmPosition + self.closedHand)

    def sc(self):
        self.write(self.stretchedArmPosition + self.closedHand)

    def release(self):
        self.write(self.openedHand)

    def grip(self):
        self.write(self.closedHand)
