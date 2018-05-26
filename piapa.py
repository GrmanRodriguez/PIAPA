# PIAPA Robot
# Grman Rodriguez, Edwin Acosta
# Obstacle avoidance
# --------------------------------------------------
# Libraries
import numpy as np  # numpy will handle arrays
import math  # math library for operations
import RPi.GPIO as GPIO  # Raspberry GPIO pins
import time  # time library for delays
import MySQLdb  # library needed for communication with UI through MySQL database
import atexit # To handle GPIO cleanup and port closure when exiting the code
import serial # To control different boards that handle actuators through serial commands
from util import MovementManager, ArmManager # refer to util.py for these libraries
# --------------------------------------------------
# Classes
# The Rover class will handle higher level vehicle movement and sensor data acquisition


class Rover(MovementManager, ArmManager):

    # The variables angle and position define the current state of the robot
    angle = 270
    position = [0, 0]
    tasks = []  # This variable defines the tasks the vehicle must execute
    hasObject = False  # To know if it's in a picking phase or placing phase

    def __init__(self):
        MovementManager.__init__(self)
        ArmManager.__init__(self)
        self.imu = serial.Serial('/dev/ttyACM0',baudrate=115200)
        anglelist = []
        for x in range(5):
            anglelist.append(self.readAngle())
        self.imuangle = sum(anglelist)/len(anglelist)        
        self.no()
        self.armOff()

    # goToPoint will execute the necessary commands to go to a desired destination in the work area
    def goToPoint(self, y, x):
        if self.position != [y,x]:
            ang = math.atan2(-(y - self.position[0]), x - self.position[1]) * 180 / math.pi  # We find the orientation the vehicle should have to go to the desired point
            if ang < 0: # We make sure to prevent negative angles
                ang = 360 + ang
            if ang != self.angle:
                self.turnToAngle(ang)
                self.angle = ang
            distance = ((y - self.position[0]) ** 2 + (x - self.position[1]) ** 2) ** 0.5 * self.gridSize
            self.forw(distance)  # Similarly, calculate the distance the vehicle should move and go forward
            self.position = [y, x]  # Finally, update the state of the vehicle

    # TurnToPoint offers the same functionality as goToPoint but without moving forward, just makes sure the robot is looking
    # in the direction of the supplied point 
    def turnToPoint(self, y, x):
        ang = math.atan2(-(y - self.position[0]), x - self.position[1]) * 180 / math.pi  # We find the orientation the vehicle should have to go to the desired point
        if ang < 0:
            ang = 360 + ang
        if ang != self.angle:
            self.turnToAngle(ang)
            self.angle = ang

    # The necessary commands to make the arm lean down, close the gripper, and go back up
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

    # The necessary commands to make the arm lean down, open the gripper, and go back up
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

    # The IMU used for angle correction requires calibration, for which it must take samples in every direction. For this, we make
    # the robot spin in its axis and let the IMU take the samples it needs
    def calibrateMag(self):
        self.imu.timeout = None
        self.imu.write('c')
        self.RL(1); self.FL(1); self.RR(2); self.FR(2)
        status = self.imu.readline()
        self.noMove()
        status = eval(status[:-1])
        if status == 1:
            print('Succesful Calibration')
        else:
            print('Something Happened')

    # readAngle() measures the magnetic field in the perpendicular axes and takes the data to calculate the robot's orientation
    # this works only if the robot is sufficiently far from any magnetic field inducing body such as magnets and coils
    # returns an angle in degrees
    def readAngle(self):
        self.imu.timeout = 3
        self.imu.write('s')
        angle = self.imu.readline()
        while len(angle) == 0:
            self.imu.write('s')
            angle = self.imu.readline()
        angle = eval(angle[:-1])
        return angle

    # avgAngle() measures the robot's orientation using readAngle() 8 times and returns the average, to reduce sensor variation error
    def avgAngle(self):
        angles = []
        for x in range(8):
            angle = self.readAngle()
            angles.append(angle)
        return sum(angles)/len(angles)

    # readSonic() takes a measurement from the HC-SR04 and returns it in meters
    def readSonic(self):
        GPIO.output(self.FTHC, GPIO.HIGH) # Set the trigger high to activate HC-SR04 sensor
        time.sleep(0.00001)
        GPIO.output(self.FTHC, GPIO.LOW)
        # Calculate echo pulse width
        while True:
            startTime = time.time()
            if GPIO.input(self.FEHC)==GPIO.HIGH:
                break
        while True:
            endTime = time.time()
            if GPIO.input(self.FEHC)==GPIO.LOW:
                break
        duration=endTime-startTime
        # Convert time into distance using the velocity of sound
        distance=(duration*343.0)/2.0
        return distance
        

    # createTasksComplete(points) is the main program for Pick-N-Plce functionality. It takes a list of points and goes to each of them
    # while sensing continuosly for obstacles and recursively re-planning when in presence of an obstacle. It also takes care
    # of arm functionality.
    # points should be a list of point duples or point lists with length 2, like so: [(1,1), (1,2)] or [[1,1], [1,2]]
    def createTasksComplete(self, points):
        # Reduce(points) removes makes sure consecutive points with the same orientation generate a continuous movement
        # rather than a boxy, cell-by-cell movement by removing redundant points from the list, returns the shortened list
        def Reduce(points):
            angle=math.atan2(points[1][0]-points[0][0],points[1][1]-points[0][1])
            reducedpoints=[]
            for x in range(len(points)-1):
                newangle=math.atan2(points[x+1][0]-points[x][0],points[x+1][1]-points[x][1])
                if newangle != angle:
                    reducedpoints.append(points[x])
                else:
                    if x == len(points)-2:
                        reducedpoints.append(points[x])
                angle = newangle
            reducedpoints.append(points[len(points)-1])
            return reducedpoints
        # Points are first reduced if necessary
        if len(points) > 2:
            reducedpoints = Reduce(points)
        else:
            reducedpoints = points
        # Main      
        for element in reducedpoints[:-1]:
            print('going to {}'.format(element))
            # Since we want to read the ultrasonic while the robot moves, we can't use Rover.forw(dist), we must use a makeshift
            # method that frees up the Rover for sensor reading
            # First we turn to the point we want to go
            self.turnToPoint(element[0], element[1])
            # Then calculate the distance to the next point
            distance = ((element[0] - self.position[0]) ** 2 + (element[1] - self.position[1]) ** 2) ** 0.5 * self.gridSize
            # This correction was needed when moving diagonally
            if self.angle in [45,135,225,315]:
                distance += 0.08
            # We neeed to update the robot's position in real time in order to properly place obstacles when detected, for this
            # we'll divide the distance in individual intervals that represent moving to each cell
            interval = distance/max([abs(element[0] - self.position[0]),abs(element[1]-self.position[1])])
            # Keep track of the time
            begin = time.time()
            beginterv = time.time()            
            while ((time.time() - begin) < (distance * 1 / self.Straight)):
                # Turn the wheels on
                self.RL(1); self.FL(1); self.FR(1); self.RR(1)
                # If the time interval has passed, the robot's position has changed by 1 cell in the direction
                # of movement. The robot's state must be updated and a new time interval must be measured.
                if ((time.time() - beginterv) > (interval * 1 / self.Straight)):
                    if element[0] > self.position[0]:
                        self.position = [self.position[0]+1, self.position[1]]
                    elif element[0] < self.position[0]:
                        self.position = [self.position[0]-1, self.position[1]]
                    if element[1] > self.position[1]:
                        self.position = [self.position[0], self.position[1]+1]
                    elif element[1] < self.position[1]:
                        self.position = [self.position[0], self.position[1]-1]
                    beginterv = time.time()
                # Reading the HC-SR04 for obstacles
                obstacle = self.readSonic()
                # 0.35m was a distance calculated empirically to provide good results for obstacle detection
                if obstacle < 0.35:
                    # Find the direction of the obstacle (same as direction of movement)
                    y=0
                    x=0
                    if element[0] > self.position[0]:
                        y+=1                        
                    elif element[0] < self.position[0]:
                        y-=1   
                    if element[1] > self.position[1]:
                        x+=1   
                    elif element[1] < self.position[1]:
                        x-=1
                    if [self.position[0]+y,self.position[1]+x] not in m.disabledNodes: # If this is a new obstacle
                        self.noMove() # Stop
                        obstacle = self.readSonic() # Measure again to prevent ghost readings
                        if obstacle < 0.35:
                            print('Obstacle found')
                            self.backw(0.03) # Move backwards a little to correct drift
                            m.disableNode(self.position[0]+y,self.position[1]+x)   
                            self.createTasksComplete(m.dijkstra(interim_pos=self.position)) # Recursively re-plan
                            return # If re-planning was done, there's no need to keep running the higher order method
                # A pseudo-PWM was implemented to make the robot move slower in straight line
                time.sleep(0.009)
                self.noMove()
                time.sleep(0.005)
            self.noMove()
            self.position = element
            m.sendData(type='pos_route', pos=self.position)
        # Turn to the last point, because the target is there and we must not reach it, but rather get to it
        self.turnToPoint(points[-1][0], points[-1][1])
        if self.hasObject:
            self.place()
        else:
            self.pick()

    # Function to handle object destruction and general pin cleanup when needed
    def quit(self):
        self.noMove()
        self.close()
        self.imu.close()
        GPIO.cleanup()

class Map(object):
    def __init__(self):
        # The map is divided in a 7x7 cells arranged in matrix format from [0,0] to [6,6]
        # start and target are 2x1 lists
        self.nodeAmount = 7
        self.stepSize = 0.35  # Size of the cells in m
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
            elif data[1] == 'forward':
                r.forw(self.stepSize)
            elif data[1] == 'clockw':
                r.turn(-180)
            elif data[1] == 'cclockw':
                r.turn(180)
            elif data[1] == 'calibrate':
                r.Straight = float(data[2])
                r.turnClockw = float(data[3])
                r.turnCounterClockw = float(data[4])
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
            starting_pos = interim_pos
        else:
            starting_pos = self.start
        for x in range(self.nodeAmount):
            for y in range(self.nodeAmount):
                if 'nodeList' in locals():
                    nodeList = np.append(nodeList, [[x, y, float("inf"), 0]], axis=0)
                else:
                    nodeList = np.array([[x, y, float("inf"), 0]])
        nodeList[self.locateInAM(starting_pos[0], starting_pos[1])][2] = 0
        routes = []
        for x in range(self.nodeAmount ** 2):
            routes.append([])
        routes[self.locateInAM(starting_pos[0], starting_pos[1])] = [starting_pos]

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
        self.route = routes[self.locateInAM(self.target[0], self.target[1])]
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

@atexit.register
def ctrlc():
    r.quit()


if __name__ == '__main__':
    r = Rover()    
    r.position = [4,3]
    r.angle = 225
    r.hasObject = True
    print('5 seconds for calibration, put Rover in position...')
    time.sleep(3)
    r.calibrateMag()
    time.sleep(2)
    r.grip()
    m = Map()
    m.start = r.position
    m.target = [6,6]
    pickL = m.target
    placeL = m.start
    while not m.checkForStart():
        m.checkForOrders()
        pickL = m.target
    while 1:
        try:
            isThereOrder = m.checkForOrders()
            if isThereOrder:
                if r.hasObject:
                    placeL = m.target
                else:
                    pickL = m.target
            r.createTasksComplete(m.dijkstra())
            if r.hasObject:
                m.target = placeL
            else:
                m.target = pickL
            m.start = r.position
        except KeyboardInterrupt:
            break
 