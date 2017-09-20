# PIAPA Robot
# Grman Rodriguez
# Raspberry-Motors Connection
# Phase 2, TCP Connection
# --------------------------------------------------
# Libraries
import RPi.GPIO as GPIO  # Raspberry GPIO pins
import time  # time library for delays
import socket  # library for TCP connection
# --------------------------------------------------
# Definitions and Setup
GPIO.setmode(GPIO.BOARD)  # GPIO Configuration
RLF = 3  # The forward-moving end of the rear left motor is connected to GPIO pin 3
RLB = 5  # The backwards-moving end of the rear left motor is connected to GPIO pin 5
RRF = 8  # The forward-moving end of the rear right motor is connected to GPIO pin 8
RRB = 10  # The backwards-moving end of the rear right motor is connected to GPIO pin 10
FLF = 11  # The forward-moving end of the front left motor is connected to GPIO pin 11
FLB = 13  # The backwards-moving end of the front left motor is connected to GPIO pin 13
FRF = 16  # The forward-moving end of the front right motor is connected to GPIO pin 16
FRB = 18  # The backwards-moving end of the front right motor is connected to GPIO pin 13
Turn = 1  # Time needed by the robot to make a 360 degree turn (TO BE TUNED)
Lin = 0.02  # Time the rover moves on press of one key (TO BE TUNED)
minAng = 2  # Angle the rover rotates on press of one key (TO BE TUNED)
# Setup pins as outputs
GPIO.setup(RLF, GPIO.OUT)
GPIO.setup(RLB, GPIO.OUT)
GPIO.setup(RRF, GPIO.OUT)
GPIO.setup(RRB, GPIO.OUT)
GPIO.setup(FLF, GPIO.OUT)
GPIO.setup(FLB, GPIO.OUT)
GPIO.setup(FRF, GPIO.OUT)
GPIO.setup(FRB, GPIO.OUT)


# --------------------------------------------------
# Main function
def main():
    # Initialize socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Get own IP address
    IP = socket.gethostbyname(socket.gethostname())
    # Port 12345
    PORT = 12345
    # Connect to socket
    s.connect((IP, PORT))
    while 1:
        # Receive data and process it
        m = s.recv(1024)
        if m == 'w':
            Forw()
            time.delay(Lin)
            NoMove()
        elif m == 's':
            Backw()
            time.delay(Lin)
            NoMove()
        elif m == 'a':
            Left(minAng)
        elif m == 's':
            Right(minAng)
        elif m == 'q':
            NoMove()


# --------------------------------------------------
# Function to move forward
def Forw():
    # All wheels must move forward
    RL(1)
    RR(1)
    FL(1)
    FR(1)


# --------------------------------------------------
# Function to move backwards
def Backw():
    # All wheel must move backwards
    RL(2)
    RR(2)
    FL(2)
    FR(2)


# --------------------------------------------------
# Function to turn left
def Left(ang):
    # To turn right we maintain the front wheels off
    FL(0)
    FR(0)
    # The rear left wheel must move backwards
    RL(2)
    # The rear right wheel must move forward
    RR(1)
    # A delay is added, that defines the angle
    time.sleep(Turn * ang / 360)
    # The turn is stopped
    RL(0)
    RR(0)


# --------------------------------------------------
# Function to turn right
def Right(ang):
    # To turn right we maintain the front wheels off
    FL(0)
    FR(0)
    # The rear left wheel must move forward
    RL(1)
    # The rear right wheel must move backwards
    RR(2)
    # A delay is added, that defines the angle
    time.sleep(Turn * ang / 360)
    # The turn is stopped
    RL(0)
    RR(0)


# --------------------------------------------------
# Function to stop all wheel
def NoMove():
    # Stop all wheels
    FL(0)
    FR(0)
    RL(0)
    RR(0)


# --------------------------------------------------
# Rear left wheel control
def RL(A):
    # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
    if A == 1:
        GPIO.output(RLF, GPIO.HIGH)
        GPIO.output(RLB, GPIO.LOW)
    elif A == 2:
        GPIO.output(RLF, GPIO.LOW)
        GPIO.output(RLB, GPIO.HIGH)
    elif A == 0:
        GPIO.output(RLF, GPIO.LOW)
        GPIO.output(RLB, GPIO.LOW)


# --------------------------------------------------
# Rear right wheel control
def RR(A):
    # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
    if A == 1:
        GPIO.output(RRF, GPIO.HIGH)
        GPIO.output(RRB, GPIO.LOW)
    elif A == 2:
        GPIO.output(RRF, GPIO.LOW)
        GPIO.output(RRB, GPIO.HIGH)
    elif A == 0:
        GPIO.output(RRF, GPIO.LOW)
        GPIO.output(RRB, GPIO.LOW)


# --------------------------------------------------
# Front right wheel control
def FR(A):
    # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
    if A == 1:
        GPIO.output(FRF, GPIO.HIGH)
        GPIO.output(FRB, GPIO.LOW)
    elif A == 2:
        GPIO.output(FRF, GPIO.LOW)
        GPIO.output(FRB, GPIO.HIGH)
    elif A == 0:
        GPIO.output(FRF, GPIO.LOW)
        GPIO.output(FRB, GPIO.LOW)


# --------------------------------------------------
# Front left wheel control
def FL(A):
    # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
    if A == 1:
        GPIO.output(FLF, GPIO.HIGH)
        GPIO.output(FLB, GPIO.LOW)
    elif A == 2:
        GPIO.output(FLF, GPIO.LOW)
        GPIO.output(FLB, GPIO.HIGH)
    elif A == 0:
        GPIO.output(FLF, GPIO.LOW)
        GPIO.output(FLB, GPIO.LOW)


main()
