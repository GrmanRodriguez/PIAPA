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
import socket  # library needed for communication with UI through Wi-Fi
import json  # this library will allow us to send dictionaries through Wi-Fi as strings, amongst other things
from smbus import SMBus
import struct
from util import MovementManager, ArmManager
# --------------------------------------------------
# Classes
# The Rover class will handle vehicle movement and create and update routes


class Rover(MovementManager, ArmManager):

    # The variables angle and position define the current state of the robot
    angle = 270
    position = [0, 0]
    tasks = []  # This variable defines the tasks the vehicle must execute
    gridSize = 0.3  # The size of the cells in meters
    hasObject = False  # To know if it's in a picking phase or placing phase

    def __init__(self):
        MovementManager.__init__(self)
        ArmManager.__init__(self)
        self.imu = SMBus(1)
        self.no()
        self.armOff()

    # goToPoint will execute the necessary commands to go to the desired destination
    def goToPoint(self, y, x):
        ang = math.atan2(-(y - self.position[0]), x - self.position[1]) * 180 / math.pi  # We find the orientation the vehicle should have to go to the desired point
        if ang < 0:
            ang = 360 + ang
        toTurn = (ang - self.angle)
        if abs(toTurn) == 270 or abs(toTurn) == 360 or abs(toTurn) == 315 or abs(toTurn) == 225:
            toTurn = toTurn - 360 * np.sign(toTurn)
        self.turn(toTurn)
        self.angle = ang
        distance = ((y - self.position[0]) ** 2 + (x - self.position[1]) ** 2) ** 0.5 * self.gridSize
        self.forw(distance)  # Similarly, calculate the distance the vehicle should move and go forward
        self.position = [y, x]  # Finally, update the state of the vehicle

    def turnToPoint(self, y, x):
        ang = math.atan2(-(y - self.position[0]), x - self.position[1]) * 180 / math.pi  # We find the orientation the vehicle should have to go to the desired point
        if ang < 0:
            ang = 360 + ang
        toTurn = ang - self.angle
        if abs(toTurn) == 270 or abs(toTurn) == 360 or abs(toTurn) == 315 or abs(toTurn) == 225:
            toTurn = toTurn - 360 * np.sign(toTurn)
        self.turn(toTurn)
        self.angle = ang

    def pick(self):
        self.no()
        time.sleep(1)
        self.so()
        time.sleep(5)
        self.grip()
        time.sleep(2)
        self.nc()
        time.sleep(5)
        self.armOff()
        if not self.hasObject:
            self.hasObject = True

    def place(self):
        self.nc()
        time.sleep(1)
        self.sc()
        time.sleep(5)
        self.release()
        time.sleep(2)
        self.no()
        time.sleep(5)
        self.armOff()
        if self.hasObject:
            self.hasObject = False

    def readAngle(self):
        X = [self.imu.read_byte_data(0x0C, 0x04), self.imu.read_byte_data(0x0C, 0x03)]
        Y = [self.imu.read_byte_data(0x0C, 0x06), self.imu.read_byte_data(0x0C, 0x05)]
        readingX = X[0] * 256 + X[1]
        readingY = Y[0] * 256 + Y[1]
        reading = math.atan2(readingY, readingX) * 180 / math.pi
        print('The results for X were: {} in high byte, {} for low byte, total measurement is {}'.format(X[0], X[1], readingX))
        print('The results for Y were: {} in high byte, {} for low byte, total measurement is {}'.format(Y[0], Y[1], readingY))
        print('Angle is {}'.format(reading))


class Map(object):
    def __init__(self):
        # The map is divided in a 7x7 cells arranged in matrix format from [0,0] to [6,6]
        # start and target are 2x1 lists
        self.nodeAmount = 7
        self.stepSize = 0.3  # Size of the cells in m
        self._start = None
        self._target = None
        self.adjMatrixCreate()
        self.disabledNodes = []
        # the herald attribute will be the socket in charge of communicating with the main PC
        # It will use port 53626 for this function
        self.herald = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.herald.bind(('', 53626))
        self.herald.listen(0)
        self.conn, self.address = self.herald.accept()
        self.sendData()

    @property
    def target(self):
        return self._target

    @property
    def start(self):
        return self._start

    @target.setter
    def target(self, value):
        self._target = value
        self.sendData()

    @target.deleter
    def target(self):
        del self._target

    @start.setter
    def start(self, value):
        self._start = value
        self.sendData()

    @start.deleter
    def start(self):
        del self._start

    def sendData(self, pos=None, **kwargs):
        data = {'nodeAmount': self.nodeAmount,
                'stepSize': self.stepSize,
                'start': self.start,
                'target': self.target,
                'disabledNodes': self.disabledNodes}
        if 'type' in kwargs:
            if kwargs['type'] == 'route':
                data["route"] = self.route
            elif kwargs['type'] == 'pos':
                data['position'] = pos
            elif kwargs['type'] == 'pos_route':
                data["route"] = self.route
                data['position'] = pos
        self.conn.sendall(json.dumps(data).encode('utf-8'))

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
        self.disabledNodes.append([y, x])
        self.sendData()

    def enableNode(self, y, x):
        if x > 0:
            self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y, x - 1)] = 10
            self.adjMatrix[self.locateInAM(y, x - 1)][self.locateInAM(y, x)] = 10
        if y > 0:
            self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y - 1, x)] = 10
            self.adjMatrix[self.locateInAM(y - 1, x)][self.locateInAM(y, x)] = 10
        if (x > 0) and (y > 0):
            self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y - 1, x - 1)] = 14
            self.adjMatrix[self.locateInAM(y - 1, x - 1)][self.locateInAM(y, x)] = 14
        if (y > 0) and (x + 1 < self.nodeAmount):
            self.adjMatrix[self.locateInAM(y, x)][self.locateInAM(y - 1, x + 1)] = 14
            self.adjMatrix[self.locateInAM(y - 1, x + 1)][self.locateInAM(y, x)] = 14
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
        self.disabledNodes.remove([y, x])
        self.sendData()

    def enableAllNodes(self):
        dN_copy = list(self.disabledNodes)
        for element in dN_copy:
            try:
                self.enableNode(element[0], element[1])
            except Exception as e:
                pass
        self.sendData()

    def dijkstra(self):
        for x in range(self.nodeAmount):
            for y in range(self.nodeAmount):
                if 'nodeList' in locals():
                    nodeList = np.append(nodeList, [[x, y, float("inf"), 0]], axis=0)
                else:
                    nodeList = np.array([[x, y, float("inf"), 0]])
        nodeList[self.locateInAM(self.start[0], self.start[1])][2] = 0
        routes = []
        for x in range(self.nodeAmount ** 2):
            routes.append([])
        routes[self.locateInAM(self.start[0], self.start[1])] = [self.start]

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
            if newMin == self.locateInAM(self.target[0], self.target[1]):
                break
        self.route = routes[self.locateInAM(self.target[0], self.target[1])][1:]
        self.sendData({'type': 'route'})
        return self.route

# createTasks will use the points given from a Map object's dijkstra function to store the movements it must make


def createTasks(points, r, m):
    for element in points[:-1]:
        r.goToPoint(element[0], element[1])
        m.sendData(type='pos_route', pos=r.position)
    r.turnToPoint(points[-1][0], points[-1][1])
    if r.hasObject:
        r.place()
    else:
        r.pick()


# stepTasks will create the generator object from the tasks list


def stepTasks(tasks):
    while True:
        try:
            tasks[0]()
            del tasks[0]
            yield
        except Exception:
            break

# executeTasks will iterate over the generator object, performing each task as it should


def executeTasks(tasks):
    orders = stepTasks(tasks)
    while True:
        try:
            next(orders)
        except StopIteration:
            break


def test():
    r.position = [0, 0]
    r.angle = -90
    r.hasObject = False
    m.start = r.position
    m.target = [4, 3]
    createTasks(m.dijkstra(), r, m)
    m.start = r.position
    m.target = [6, 0]
    createTasks(m.dijkstra(), r, m)


if __name__ == '__main__':
    r = Rover()
    m = Map()
    m.disableNode(0, 1)
    m.disableNode(1, 1)
    m.disableNode(3, 0)
    m.disableNode(3, 2)
    m.disableNode(4, 2)
    m.disableNode(5, 2)
    m.disableNode(3, 3)
    m.disableNode(2, 3)
    m.disableNode(2, 4)
    test()
    r.quit()
    r.close()
    m.herald.close()
