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
import MySQLdb  # library needed for communication with UI through MySQL database
from smbus import SMBus
import threading
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
    magXMean = 0  # In order to calibrate the magnetometer
    magYMean = 0  # In order to calibrate the magnetometer

    def __init__(self):
        MovementManager.__init__(self)
        ArmManager.__init__(self)
        #self.imu = SMBus(1)
        #self.imu.write_byte_data(0x68, 0x6b, 0x00)
        #self.imu.write_byte_data(0x68, 0x37, 0x22)
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

    def readMags(self):
        self.imu.write_byte_data(0x0c, 0x0a, 0x00)
        time.sleep(0.01)
        self.imu.write_byte_data(0x0c, 0x0a, 0x06)
        time.sleep(0.01)
        X = [self.imu.read_byte_data(0x0c, 0x04), self.imu.read_byte_data(0x0C, 0x03)]
        Y = [self.imu.read_byte_data(0x0c, 0x06), self.imu.read_byte_data(0x0C, 0x05)]
        readingX = int((hex(X[0])[2:] + hex(X[1])[2:]), 16)
        if readingX > 0x7FFF:
            readingX -= 0x10000
        readingY = int((hex(Y[0])[2:] + hex(Y[1])[2:]), 16)
        if readingY > 0x7FFF:
            readingY -= 0x10000
        self.imu.write_byte_data(0x0c, 0x0a, 0x00)
        return readingX, readingY

    def readAngle(self):
        X, Y = self.readMags
        reading = math.atan2(Y, X) * 180 / math.pi
        if reading < 0:
            reading += 360
        print('The results for X were: {} in high byte, {} for low byte, total measurement is {}'.format(X[0], X[1], readingX))
        print('The results for Y were: {} in high byte, {} for low byte, total measurement is {}'.format(Y[0], Y[1], readingY))
        print('Angle is {}'.format(reading))

    def readSonic(self):
        GPIO.output(self.FTHC, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.FTHC, GPIO.LOW)
        while True:
            startTimeF = time.time()
            if GPIO.input(self.FEHC)==GPIO.HIGH:
                break
        while True:
            endTimeF = time.time()
            if GPIO.input(self.FEHC)==GPIO.LOW:
                break
        GPIO.output(self.RTHC, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.RTHC, GPIO.LOW)
        while True:
            startTime = time.time()
            if GPIO.input(self.REHC)==GPIO.HIGH:
                break
        while True:
            endTime = time.time()
            if GPIO.input(self.REHC)==GPIO.LOW:
                break
        GPIO.output(self.LTHC, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.LTHC, GPIO.LOW)
        while True:
            startTimeL = time.time()
            if GPIO.input(self.LEHC)==GPIO.HIGH:
                break
        while True:
            endTimeL = time.time()
            if GPIO.input(self.LEHC)==GPIO.LOW:
                break
        durationF=endTimeF-startTimeF
        distanceF=(durationF*34300)/2
        durationR=endTime-startTime
        distanceR=(durationR*34300)/2
        durationL=endTimeL-startTimeL
        distanceL=(durationL*34300)/2
        return distanceF
        #print('Distancia Lateral Derecha: {}'.format(distanceR))

    def createTasks(self, points):
        def Reduce(points):
            angle=math.atan2(points[1][0]-points[0][0],points[1][1]-points[0][1])
            reducedpoints=[points[0]]
            for x in range(1,len(points)-1):
                newangle=math.atan2(points[x+1][0]-points[x][0],points[x+1][1]-points[x][1])
                if newangle != angle:
                    reducedpoints.append(points[x])
                else:
                    if x == len(points)-2:
                        reducedpoints.append(points[x])
            reducedpoints.append(points[len(points)-1])
            return reducedpoints
        if len(points) > 4:
            points = Reduce(points)
        for element in points[:-1]:
            self.goToPoint(element[0], element[1])
            m.sendData(type='pos_route', pos=r.position)
        self.turnToPoint(points[-1][0], points[-1][1])
        if self.hasObject:
            self.place()
        else:
            self.pick()

    def createTasksComplete(self, points):
        def Reduce(points):
            angle=math.atan2(points[1][0]-points[0][0],points[1][1]-points[0][1])
            reducedpoints=[points[0]]
            for x in range(1,len(points)-1):
                newangle=math.atan2(points[x+1][0]-points[x][0],points[x+1][1]-points[x][1])
                if newangle != angle:
                    reducedpoints.append(points[x])
                else:
                    if x == len(points)-2:
                        reducedpoints.append(points[x])
            reducedpoints.append(points[len(points)-1])
            return reducedpoints
        if len(points) > 4:
            points = Reduce(points)
        for element in points[:-1]:
            self.turnToPoint(element[0], element[1])
            distance = ((element[0] - self.position[0]) ** 2 + (element[1] - self.position[1]) ** 2) ** 0.5 * self.gridSize
            interval = distance/(element[0] - self.position[0])
            begin = time.time()
            beginterv = time.time()
            self.RL(1); self.FL(1); self.FR(1); self.RR(1)
            while ((time.time() - begin) < (distance * 1 / self.Straight)):
                if ((time.time() - beginterv) > (interval * 1 / self.Straight)):
                    if element[0] > self.position[0]:
                        self.position = [self.position[0]+1, self.position[1]]
                    if element[1] > self.position[1]:
                        self.position = [self.position[0], self.position[1]+1]
                    beginterv = time.time()
                obstacle = self.readSonic()
                if obstacle < 2:
                    self.noMove()
                    self.createTasksComplete(m.dijkstra(interim_pos=self.position))
                    return
            self.noMove()
            self.position = element
            m.sendData(type='pos_route', pos=self.position)
        self.turnToPoint(points[-1][0], points[-1][1])
        if self.hasObject:
            self.place()
        else:
            self.pick()

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
        self.no_start_no_target = """INSERT INTO `piapa_db`.`status` (`stepSize`, `nodeAmount`, `disabledNodes`) VALUES ('{}', '{}', '{}');"""
        self.basic_send = """INSERT INTO `piapa_db`.`status` (`startY`, `startX`, `targetY`, `targetX`, `stepSize`, `nodeAmount`, `disabledNodes`) VALUES ('{}', '{}', '{}', '{}', '{}', '{}', '{}');"""
        self.route_send = """INSERT INTO `piapa_db`.`status` (`startY`, `startX`, `targetY`, `targetX`, `stepSize`, `nodeAmount`, `disabledNodes`, `route`) VALUES ('{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}');"""
        self.pos_route_send = """INSERT INTO `piapa_db`.`status` (`startY`, `startX`, `targetY`, `targetX`, `stepSize`, `nodeAmount`, `disabledNodes`, `route`, `positionY`, `positionX`) VALUES ('{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}');"""
        self.go_check = """SELECT * FROM begin ORDER BY id DESC LIMIT 1"""
        # the db and cur attributes will be the socket in charge of communicating with the main DB
        self.db = MySQLdb.connect(host="192.168.1.116",
                                  user="root",
                                  password="1234",
                                  db="piapa_db")
        self.cur = self.db.cursor()
        self.cur.execute("""SELECT * FROM orders ORDER BY id DESC LIMIT 1""")
        data = self.cur.fetchone()
        self.db.commit()
        self.lastOrder = int(data[0])
        self.cur.execute(self.go_check)
        comm = self.cur.fetchone()
        self.beginOrd = int(comm[0])
        self.db.commit()
        self.checkForOrders()
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

    def checkForStart(self):
        self.cur.execute(self.go_check)
        self.db.commit()
        comm = self.cur.fetchone()
        if int(comm[0]) > self.beginOrd:
            self.beginOrd = int(comm[0])
            return True
        else:
            return False

    def checkForOrders(self):
        self.cur.execute("""SELECT * FROM orders ORDER BY id DESC LIMIT 1""")
        data = self.cur.fetchone()
        if int(data[0]) > self.lastOrder:
            if data[1] == 'disable':
                if [int(data[2]), int(data[3])] not in self.disabledNodes:
                    self.disableNode(int(data[2]),int(data[3]))
            elif data[1] == 'enable':
                if [data[2], data[3]] in self.disabledNodes:
                    self.enableNode(int(data[2]),int(data[3]))
            elif data[1] == 'target':
                self.target = [int(data[2]), int(data[3])]
            self.lastOrder = int(data[0])
            return True
        else:
            return False


    def sendData(self, pos=None, **kwargs):
        if 'type' in kwargs:   
            if kwargs['type'] == 'route':
                self.cur.execute(self.route_send.format(self.start[0],self.start[1],self.target[0],self.target[1],self.stepSize,self.nodeAmount,self.disabledNodes,self.route))
            elif kwargs['type'] == 'pos_route':
                self.cur.execute(self.pos_route_send.format(self.start[0],self.start[1],self.target[0],self.target[1],self.stepSize,self.nodeAmount,self.disabledNodes,self.route,pos[0],pos[1]))
        else:
            if (self.start is not None) and (self.target is not None): 
                self.cur.execute(self.basic_send.format(self.start[0],self.start[1],self.target[0],self.target[1],self.stepSize,self.nodeAmount,self.disabledNodes))
            else:
                self.cur.execute(self.no_start_no_target.format(self.stepSize,self.nodeAmount,self.disabledNodes))
        self.db.commit()

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

    def dijkstra(self, interim_pos=None):
        if interim_pos is not None:
            final_pos = interim_pos
        else:
            final_pos = self.target
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
            if newMin == self.locateInAM(final_pos[0], final_pos[1]):
                break
        self.route = routes[self.locateInAM(final_pos[0], final_pos[1])][1:]
        self.sendData(type='route')
        return self.route

# createTasks will use the points given from a Map object's dijkstra function to store the movements it must make


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


def mapUpdate():
    m.checkForOrders()
    threading.Time(0.4, mapUpdate).start()

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
    r.position = [0,0]
    r.angle = -90
    m = Map()
    m.start = r.position
    m.target = [6,6]
    pickL = m.target
    placeL = m.start
    while not m.checkForStart():
        m.checkForOrders()
    while 1:
        try:
            isThereOrder = m.checkForOrders()
            if isThereOrder:
                if r.hasObject:
                    placeL = m.target
                else:
                    pickL = m.target
            r.createTasks(m.dijkstra())
            if r.hasObject:
                m.target = placeL
            else:
                m.target = pickL
            m.start = r.position
        except KeyboardInterrupt:
            break
    r.quit()
    r.close()
