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
# The Rover class will handle vehicle movement and create and update routes


class Rover:

    state = [5, 5, 90]  # The state variable defines the y coordinate, x coordinate and angle. In that order.
    freq = 100  # The pwm frequency
    dutyCycle = 50  # The pwm duty cycle
    tasks = []  # This variable defines the tasks the vehicle must execute
    gridSize = 0.3  # The size of the cells in meters
    RLF = 19  # The forward-moving end of the rear left motor is connected to GPIO pin 19
    RLB = 21  # The backwards-moving end of the rear left motor is connected to GPIO pin 21
    RRF = 8  # The forward-moving end of the rear right motor is connected to GPIO pin 8
    RRB = 10  # The backwards-moving end of the rear right motor is connected to GPIO pin 10
    FLF = 11  # The forward-moving end of the front left motor is connected to GPIO pin 11
    FLB = 13  # The backwards-moving end of the front left motor is connected to GPIO pin 13
    FRF = 16  # The forward-moving end of the front right motor is connected to GPIO pin 16
    FRB = 18  # The backwards-moving end of the front right motor is connected to GPIO pin 13
    turnClockw = 9.15  # Time needed by the robot to make a 360 degree clockwise turn (TUNED)
    turnCounterClockw = 9.7  # Time needed by the robot to make a 360 degree counterclockwise turn (TUNED)
    Straight = 3.3  # Time needed by the robot to advance 1 meter (TUNED)

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
        self.rlf = GPIO.PWM(self.RLF, self.freq)
        self.rlb = GPIO.PWM(self.RLB, self.freq)
        self.rrf = GPIO.PWM(self.RRF, self.freq)
        self.rrb = GPIO.PWM(self.RRB, self.freq)
        self.flf = GPIO.PWM(self.FLF, self.freq)
        self.flb = GPIO.PWM(self.FLB, self.freq)
        self.frf = GPIO.PWM(self.FRF, self.freq)
        self.frb = GPIO.PWM(self.FRB, self.freq)

    # Calibrate the Turn parameter
    def calibrateTurn(self, Turn, which):
        if which == 'c':
            self.turnClockw = Turn
        elif which == 'cc':
            self.turnCounterClockw = Turn
        else:
            print('Invalid parameter(s)')

    # Calibrate the Straight parameter
    def calibrateStraight(self, Straight):
        self.Straight = Straight

    # The following 8 functions change the pins used for each wheel if needs be
    def setRLF(self, pin):
        self.rfl = GPIO.PWM(pin, self.freq)

    def setRLB(self, pin):
        self.rfb = GPIO.PWM(pin, self.freq)

    def setRRF(self, pin):
        self.rrf = GPIO.PWM(pin, self.freq)

    def setRRB(self, pin):
        self.rrb = GPIO.PWM(pin, self.freq)

    def setFLF(self, pin):
        self.flf = GPIO.PWM(pin, self.freq)

    def setFLB(self, pin):
        self.flb = GPIO.PWM(pin, self.freq)

    def setFRB(self, pin):
        self.frb = GPIO.PWM(pin, self.freq)

    def setFRL(self, pin):
        self.frl = GPIO.PWM(pin, self.freq)

    # Function to move forward
    def forw(self, a):
        # All wheels must move forward
        self.RL(1)
        self.RR(1)
        self.FL(1)
        self.FR(1)
        # Make the movement for "a" meters
        time.sleep(a * self.Straight)
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
        time.sleep(a * self.Straight)
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
            self.rlf.start(self.dutyCycle)
            self.rlb.stop()
        elif A == 2:
            self.rlf.stop()
            self.rlb.start(self.dutyCycle)
        elif A == 0:
            self.rlf.stop()
            self.rlb.stop()

    # Rear right wheel control
    def RR(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            self.rrf.start(self.dutyCycle)
            self.rrb.stop()
        elif A == 2:
            self.rrf.stop()
            self.rrb.start(self.dutyCycle)
        elif A == 0:
            self.rrf.stop()
            self.rrb.stop()

    # Front right wheel control
    def FR(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            self.frf.start(self.dutyCycle)
            self.frb.stop()
        elif A == 2:
            self.frf.stop()
            self.frb.start(self.dutyCycle)
        elif A == 0:
            self.frf.stop()
            self.frb.stop()

    # Front left wheel control
    def FL(self, A):
        # A defines movement, if 1 = move forward, if 2 = move backwards, if 0 = don't move
        if A == 1:
            self.flf.start(self.dutyCycle)
            self.flb.stop()
        elif A == 2:
            self.flf.stop()
            self.flb.start(self.dutyCycle)
        elif A == 0:
            self.flf.stop()
            self.flb.stop()

    # goToPoint will execute the necessary commands to go to the desired destination
    def goToPoint(self, y, x):
        ang = math.atan2(-(y - self.position[0]), x - self.position[1]) * 180 / math.pi  # We find the orientation the vehicle should have to go to the desired point
        toTurn = ang - self.angle
        if toTurn != 0:
            self.turn(toTurn)  # And make the turn
        self.angle = ang  # Update the state of the vehicle
        distance = ((y - self.position[0]) ** 2 + (x - self.position[1]) ** 2) ** 0.5 * self.gridSize
        self.forw(distance)  # Similarly, calculate the distance the vehicle should move and go forward
        self.position = [y, x]  # Finally, update the state of the vehicle

    # createTasks will use the points given from a Map object's dijkstra function to store the movements it must make
    def createTasks(self, points):
        for element in points:
            self.goToPoint(element[0], element[1])

    # stepTasks will create the generator object from the tasks list
    def stepTasks(self):
        while True:
            try:
                self.tasks[0]()
                del self.tasks[0]
                yield
            except Exception:
                break

    # executeTasks will iterate over the generator object, performing each task as it should
    def executeTasks(self):
        orders = self.stepTasks()
        while True:
            try:
                next(orders)
            except StopIteration:
                break

    # Function to handle object destruction and general pin cleanup when needed
    def quit(self):
        GPIO.cleanup()


class Map:
    def __init__(self, start=None, target=None):
        # The map is divided in a 7x7 cells arranged in matrix format from [0,0] to [6,6]
        # start and target are 2x1 lists
        self.nodeAmount = 7
        self.stepSize = 0.3  # Size of the cells in m
        if start is None:
            self.startY = 5
            self.startX = 5
        else:
            self.startY = start[0]
            self.startX = start[1]
        if target is None:
            self.targetY = 0
            self.targetX = 0
        else:
            self.targetY = target[0]
            self.targetX = target[1]
        self.adjMatrixCreate()

    def locateInAM(self, Y, X):
        return (self.nodeAmount * Y) + X

    def findNode(self, pos):
        return [pos // self.nodeAmount, pos % self.nodeAmount]

    def adjMatrixCreate(self):
        self.adjMatrix = np.zeros((self.nodeAmount ** 2, self.nodeAmount ** 2))
        for x in range(self.nodeAmount):
            for y in range(self.nodeAmount):
                if y + 1 < self.nodeAmount:
                    self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y + 1, x)] = 10
                    self.adjMatrix[self.locateInAM(y + 1, x)][self.locateInAM(y, x)] = 10
                if x + 1 < self.nodeAmount:
                    self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y, x + 1)] = 10
                    self.adjMatrix[self.locateInAM(y, x + 1)][self.locateInAM(y, x)] = 10
                if (x > 0) and (y + 1 < self.nodeAmount):
                    self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y + 1, x - 1)] = 14
                    self.adjMatrix[self.locateInAM(y + 1, x - 1)][self.locateInAM(y, x)] = 14
                if (x + 1 < self.nodeAmount) and (y + 1 < self.nodeAmount):
                    self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y + 1, x + 1)] = 14
                    self.adjMatrix[self.locateInAM(y + 1, x + 1)][self.locateInAM(y, x)] = 14
        self.adjMatrix[self.adjMatrix == 0] = -1

    def disableNode(self, y, x):
        pos = self.locateInAM(y, x)
        self.adjMatrix[pos] = -1
        self.adjMatrix[:, pos] = -1

    def dijkstra(self):
        for x in range(self.nodeAmount):
            for y in range(self.nodeAmount):
                if 'nodeList' in locals():
                    nodeList = np.append(nodeList, [[x, y, float("inf"), 0]], axis=0)
                else:
                    nodeList = np.array([[x, y, float("inf"), 0]])
        nodeList[self.locateInAM(self.startY, self.startX)][2] = 0
        routes = []
        for x in range(self.nodeAmount ** 2):
            routes.append([])
        routes[self.locateInAM(self.startY, self.startX)] = [[self.startY, self.startX]]

        while 1:
            newList = nodeList[nodeList[:, 3] == 0]
            newList = newList[newList[:, 2].argsort()]
            for node in newList:
                newMin = int(self.locateInAM(node[0], node[1]))
                [minY, minX] = self.findNode(newMin)
                for element in range(self.nodeAmount ** 2):
                    if self.adjMatrix[newMin][element] > 0 and nodeList[element][3] == 0:
                        if 'nears' in locals():
                            nears = np.append(nears, [self.findNode(element)], axis=0)
                        else:
                            nears = np.array([self.findNode(element)])

                if 'nears' in locals():
                    break
            if not 'nears' in locals():
                for node in newList:
                    newMin = int(self.locateInAM(node[0], node[1]))
                    [minY, minX] = self.findNode(newMin)
                    for element in range(self.nodeAmount ** 2):
                        if self.adjMatrix[newMin][element] > 0:
                            if 'nears' in locals():
                                nears = np.append(nears, [self.findNode(element)], axis=0)
                            else:
                                nears = np.array([self.findNode(element)])
                    if 'nears' in locals():
                        break
            for element in nears:
                value = nodeList[newMin][2] + self.adjMatrix[newMin][self.locateInAM(element[0], element[1])]
                if nodeList[self.locateInAM(element[0], element[1])][2] > value:
                    nodeList[self.locateInAM(element[0], element[1])][2] = value
                    routes[self.locateInAM(element[0], element[1])] = routes[newMin][:]
                    routes[self.locateInAM(element[0], element[1])].append(element.tolist())
            nodeList[newMin][3] = 1
            del(nears)
            if newMin == self.locateInAM(self.targetY, self.targetX):
                break
        return routes[self.locateInAM(self.targetY, self.targetX)][1:]


if __name__ == '__main__':
    r = Rover()
    m = Map()
    m.disableNode(4, 6)
    m.disableNode(4, 5)
    m.disableNode(4, 4)
    m.disableNode(4, 3)
    m.disableNode(4, 2)
    m.disableNode(4, 1)
    r.createTasks(m.dijkstra())
    r.quit()
