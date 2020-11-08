import sys
from croblink import *
import math
import xml.etree.ElementTree as ET

import numpy as np

from astar import *

CELLROWS=7
CELLCOLS=14
offset_rotation = 0

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # sensors IDs
        self.center_id = 0
        self.left_id = 1
        self.right_id = 2
        self.back_id = 3
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        while True:
            self.readSensors()
            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                self.returning = False
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                    self.driveMotors(0.0,0.0)
                else:
                    if challenge == '1':
                        print("Challenge 1")
                        path = astar(maze, start, end)
                        self.readSensors()
                        self.startPosition = (self.measures.y, self.measures.x)
                        self.followPath(path)
                    elif challenge == '2':
                        print("Challenge 2")
                        self.exploreMap()
                        # path = astar(self.maze, self.start, self.end)
                        # self.followPath(path)

            elif state == 'wait':
                self.returning = False
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state == 'return':
                self.returning = True
                if challenge == '1':
                    pathR = astar(maze, end, start)
                elif challenge == '2':
                    self.maze = np.flip(self.maze,0)
                    self.getStartEnd(self.maze)
                    
                    self.printMapV2()

                    # print(self.maze[self.end[0]][self.end[1]], self.end)
                    # print(self.maze[self.start[0]][self.start[1]], self.start)
                    print(self.end, self.start)
                    pathR = astar(self.maze, self.end, self.start)

                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.followPath(pathR)
                return
            # elif state == 'explore':
            #     self.exploreMap()
            #     state == 'wait'
            
    def printMapV2(self):
        with np.printoptions(threshold=np.inf, linewidth=np.inf):
            print(self.maze)
        # print(self.maze)

    def moveToGPSPoint(self, point):
        northVec = [0,1]
        while self.measures.y > point[0] + 0.3 or self.measures.y < point[0] - 0.3  or self.measures.x > point[1] + 0.3 or self.measures.x < point[1] - 0.3:
            print(self.measures.y)
            print(self.measures.x)
            print("MOVE TO POINT:" + str(point))
            dirVec = [point[0] - self.measures.y, point[1] - self.measures.x]
            #PODE NÂO RESULTAR POIS CALCULA O ANGULO SEMPRE POSITIVO E NUNCA NEGATIVO
            unit_vector_1 = northVec / np.linalg.norm(northVec)
            unit_vector_2 = dirVec / np.linalg.norm(dirVec)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)  
            print(math.degrees(angle))
            #if(point[0] < self.startPosition[0]):
            #    angle = -angle
            self.rotate(math.degrees(angle))
            self.driveMotors(0.03,0.03)
            self.readSensors()
        

    def cellToGPSPoint(self, cell):
        return [self.startPosition[0] + cell[0] - startCELL[0]*2 , self.startPosition[1] + cell[1] - startCELL[1]*2]
    
    def moveToCell(self, cell):
        print("MOVE TO CELL:" + str(cell))
        self.moveToGPSPoint(self.cellToGPSPoint(cell))
        
    def followPath2(self):
        print(path)
        path.pop(0)
        while len(path)> 1:
            self.moveToCell(path.pop(0))
            

    def followPath(self, path):
        print(path)
        while len(path)> 1:
            currentPos = path.pop(0)
            dirToMove = (( path[0][0]) - currentPos[0], (path[0][1] - currentPos[1] ))
            print("Move: ", dirToMove)
            if(dirToMove[0] > 0):
                print("Move down")
                self.rotate(-90)
                self.moveFrontOne(-90)
            elif(dirToMove[0] < 0):
                self.rotate(90)
                self.moveFrontOne(90)
            if(dirToMove[1] > 0):
                self.rotate(0)
                self.moveFrontOne(0)
            elif(dirToMove[1] < 0):
                self.rotate(180)
                self.moveFrontOne(180)
            

    def moveFrontOne(self, direction):
        posInit = [self.measures.x, self.measures.y]
        #direction = 0 #elf.measures.compass #0 90 180 -90
        currentPos = [self.measures.x, self.measures.y]
        distance = math.hypot(currentPos[0] - posInit[0], currentPos[1] - posInit[1])

        while distance < 2:
            self.readSensors()

            if (round(currentPos[1]-self.dY), round(currentPos[0]-self.dX)) == self.start and self.returning:
                print("REACHED START POSITION")
                self.driveMotors(0.0,0.0)
                break

            if self.measures.ground == 0 and not self.returning:
                print("REACHED TARGET POSITION")
                self.driveMotors(0.0,0.0)
                break

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
                break

            # if abs(self.measures.compass) < direction - 1:
            #     print(self.measures.compass)
            #     if self.measures.compass < 0:
            #         # print("Correct LEFT")
            #         self.driveMotors(+0.1,0.09)
            #         # print("Corrected: ", self.measures.compass)
            #     else:
            #         # print("Correct RIGHT")
            #         self.driveMotors(0.09,+0.1)
            # # if(direction == 180):
            # #     if(self.measures.compass > -179):
            # #         self.driveMotors(+0.1,0.09)
            # #     elif(self.measures.compass < 179):
            # #         self.driveMotors(0.09,+0.1)
            # elif(self.measures.compass > direction + 1):
            #     self.driveMotors(+0.1,0.09)
            # elif(self.measures.compass < direction -1):
            #     self.driveMotors(0.09,+0.1)
            # else:
            #     self.driveMotors(+0.1,+0.1)

            if self.measures.irSensor[self.left_id] > 2:
                self.driveMotors(0.1,0.09)
            if self.measures.irSensor[self.right_id] > 2:
                self.driveMotors(0.09,0.1)

            if self.measures.irSensor[self.right_id] < 2.2 and self.measures.irSensor[self.left_id] < 2.2:
                if abs(self.measures.compass) < direction - 1:
                    if self.measures.compass < 0:
                        self.driveMotors(+0.1,0.09)
                    else:
                        self.driveMotors(0.09,+0.1)
                elif(self.measures.compass > direction + 1):
                    self.driveMotors(+0.1,0.09)
                elif(self.measures.compass < direction -1):
                    self.driveMotors(0.09,+0.1)
                else:
                    self.driveMotors(0.1,0.1)

            currentPos = [self.measures.x, self.measures.y]
            distance = math.hypot(currentPos[0] - posInit[0], currentPos[1] - posInit[1])
            
            # print(distance, currentPos, posInit)

        self.driveMotors(0.00,-0.00)    

    # Sem considerar o noise
    def previewMove(self, direction, speed, pos):
        lin = (speed[0], speed[1])/2
        x = pos[0] + lin*math.cos(math.radians(direction))
        y = pos[1] + lin*math.sin(math.radians(direction))
        return (x,y)

    def angConverter(self, ang):
        if(ang < 0):
            return (360 - ang)
        return ang

    # def rotate(self, ang):
    #     current_ang = self.angConverter(self.measures.compass)
    #     angle = self.angConverter(ang)
    #     print(current_ang)
    #     print(ang)
    #     if ang == 0:
    #         if(self.measures.compass > 0):
    #             while self.measures.compass > 0:
    #                 self.driveMotors(+0.1,-0.1)
    #                 self.readSensors()
    #         else:
    #             while self.measures.compass < 0:
    #                 self.driveMotors(-0.1,+0.1)
    #                 self.readSensors()
    #     elif current_ang < angle:
    #         while self.angConverter(self.measures.compass) < self.angConverter(ang):
    #             self.driveMotors(-0.1,+0.1)
    #             self.readSensors()
    #     else:
    #         while self.angConverter(self.measures.compass) > self.angConverter(ang):
    #             self.driveMotors(+0.1,-0.1)
    #             self.readSensors()
    #     self.driveMotors(0.00,-0.00)

    def rotate(self, ang):
        print("Rotating to ", ang)
        if(ang == 90):
            if abs(self.measures.compass) > 90:    
                while abs(self.measures.compass) > 90:
                    # print("Rotating... ", abs(self.measures.compass))
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
            else:
                while self.measures.compass < 90:
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
        elif(ang == 180):
            if self.measures.compass > 0:
                while self.measures.compass < 178 and self.measures.compass > 0:
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else:
                while self.measures.compass < 178 and self.measures.compass < 0:
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        elif(ang == -90):
            if abs(self.measures.compass) > 90: 
                while abs(self.measures.compass) > 90:    
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else:
                while abs(self.measures.compass) < 90:    
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        elif(ang == 0):
            if self.measures.compass < 0:
                while self.measures.compass < 0:
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else: 
                while self.measures.compass > 0:
                    # print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        self.driveMotors(0.00,-0.00)

    def exploreMap(self): 
        self.maze = [[8] * (CELLCOLS*4-1) for i in range(CELLROWS*4-1)]
        self.start = (round((len(self.maze)-1)/2),round((len(self.maze[0])-1)/2))
        # self.start = tuple(currentCoord)
        # start location
        self.maze[self.start[0]][self.start[1]] = 5
        
        # Minimum distance at which we assume a wall exists
        minD = 0.5
        minDSides = 0.525
        minDSidesMove = 0.54
        # min60 = (minD+0.5)/math.sin(math.radians(60))-0.5
        
        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        thresholdSides = 1/minDSides    # threshold side sensors
        thresholdSidesMove = 1/minDSidesMove

        self.dX, self.dY = (self.measures.x - self.start[1], self.measures.y - self.start[0])

        direction = 0

        while True:    
            self.readSensors()
            coord = (round(self.measures.y-self.dY), round(self.measures.x-self.dX))
            
            # Assign 0 to all visited cells except target and start
            if coord != self.start:  
                self.maze[coord[0]][coord[1]] = 0   
                
            print(self.measures.irSensor, coord)
            
            # print(self.maze[coord[0]+5])
            # print(self.maze[coord[0]+4])
            # print(self.maze[coord[0]+3])
            # print(self.maze[coord[0]+2])
            # print(self.maze[coord[0]+1])
            # print(self.maze[coord[0]])
            # print(self.maze[coord[0]-1])
            # print(self.maze[coord[0]-2])
            # print(self.maze[coord[0]-3])
            # print(self.maze[coord[0]-4])
            # print(self.maze[coord[0]-5])
            
            # check for walls
            if self.measures.irSensor[self.center_id] > threshold:           # front wall
                self.driveMotors(0.0,0.0)
                if direction == 0:
                    self.maze[coord[0]][coord[1]+1] = 1
                    print("ADDED FRONT WALL-> ", coord)
                if direction == 90:
                    self.maze[coord[0]+1][coord[1]] = 1
                    print("ADDED FRONT WALL -> ", coord)
                if direction == -90:
                    self.maze[coord[0]-1][coord[1]] = 1
                    print("ADDED FRONT WALL -> ", coord)
                if direction == 180:
                    self.maze[coord[0]][coord[1]-1] = 1
                    print("ADDED FRONT WALL -> ", coord)

                if self.measures.irSensor[self.right_id] > thresholdSidesMove:        # right wall
                    if self.measures.irSensor[self.left_id] > thresholdSidesMove:     # left wall     CASE: FRONT, RIGHT, LEFT 
                        print("front right left wall")
                        direction = self.rotateBack(direction)                        
                    else:                                               #               CASE: FRONT, RIGHT
                        print("front right wall")
                        direction = self.rotateLeft(direction)
                elif self.measures.irSensor[self.left_id] > thresholdSidesMove:       # left wall     CASE: FRONT, LEFT 
                    print("front left wall")
                    direction = self.rotateRight(direction)
                else:
                    # direction = self.rotateLeft(direction)
                    choice = np.random.choice(2,1)
                    if choice == 0:
                        direction = self.rotateRight(direction)
                    else:
                        direction = self.rotateLeft(direction) 

            if self.measures.irSensor[self.left_id] > thresholdSides:           # left wall     CASE: LEFT
                if direction == 0:
                    self.maze[coord[0]+1][coord[1]] = 1
                    print("ADDED LEFT WALL -> ", coord)
                if direction == 90:
                    self.maze[coord[0]][coord[1]-1] = 1
                    print("ADDED LEFT WALL -> ", coord)
                if direction == -90:
                    self.maze[coord[0]][coord[1]+1] = 1
                    print("ADDED LEFT WALL -> ", coord)
                if direction == 180:
                    self.maze[coord[0]-1][coord[1]] = 1
                    print("ADDED LEFT WALL -> ", coord)
            
            if self.measures.irSensor[self.right_id] > thresholdSides:          # right wall    CASE: RIGHT
                if direction == 0:
                    self.maze[coord[0]-1][coord[1]] = 1
                    print("ADDED RIGHT WALL -> ", coord)
                if direction == 90:
                    self.maze[coord[0]][coord[1]+1] = 1
                    print("ADDED RIGHT WALL -> ", coord)
                if direction == -90:
                    self.maze[coord[0]][coord[1]-1] = 1
                    print("ADDED RIGHT WALL -> ", coord)
                if direction == 180:
                    self.maze[coord[0]+1][coord[1]] = 1
                    print("ADDED RIGHT WALL -> ", coord)

            # NOT WORKING 100%
            if direction == 0:
                if self.maze[coord[0]][coord[1]+1] == 0:
                    print("MOVE NORTH")
                    # self.moveFrontOne(0)
                    if self.measures.irSensor[self.left_id] < 1/0.6 and self.maze[coord[0]+1][coord[1]] == 8:
                        direction = self.rotateLeft(direction)
                    elif self.measures.irSensor[self.right_id] < 1/0.6 and self.maze[coord[0]-1][coord[1]] == 8:
                        direction = self.rotateRight(direction)

            if direction == 90:
                if self.maze[coord[0]+1][coord[1]] == 0:
                    print("MOVE WEST")
                    # self.moveFrontOne(90)
                    if self.measures.irSensor[self.left_id] < 1/0.6 and self.maze[coord[0]][coord[1]-1] == 8:
                        direction = self.rotateLeft(direction)
                    elif self.measures.irSensor[self.right_id] < 1/0.6 and self.maze[coord[0]][coord[1]+1] == 8:
                        direction = self.rotateRight(direction)

            if direction == -90:
                if self.maze[coord[0]-1][coord[1]] == 0:
                    print("MOVE EAST")
                    # self.moveFrontOne(-90)
                    if self.measures.irSensor[self.left_id] < 1/0.6 and self.maze[coord[0]][coord[1]-1] == 8:
                        direction = self.rotateLeft(direction)
                    elif self.measures.irSensor[self.right_id] < 1/0.6 and self.maze[coord[0]][coord[1]-1] == 8:
                        direction = self.rotateRight(direction)

            if direction == 180:
                if self.maze[coord[0]][coord[1]-1] == 0:
                    print("MOVE SOUTH")
                    # self.moveFrontOne(180)
                    if self.measures.irSensor[self.left_id] < 1/0.6 and self.maze[coord[0]-1][coord[1]] == 8:
                        direction = self.rotateLeft(direction)
                    elif self.measures.irSensor[self.right_id] < 1/0.6 and self.maze[coord[0]+1][coord[1]] == 8:
                        direction = self.rotateRight(direction)

            # move away from walls
            if self.measures.irSensor[self.left_id] > 2.1:
                self.driveMotors(0.1,0.09)
            if self.measures.irSensor[self.right_id] > 2.1:
                self.driveMotors(0.09,0.1)

            # extreme turn, almost hitting wall
            if self.measures.irSensor[self.left_id] > 6:
                self.driveMotors(0.1,0)
            if self.measures.irSensor[self.right_id] > 6:
                self.driveMotors(0,0.1)
            
            # keep straight direction 
            if self.measures.irSensor[self.right_id] < 2.2 and self.measures.irSensor[self.left_id] < 2.2: 
                if abs(self.measures.compass) < direction - 1:
                    if self.measures.compass < 0:
                        self.driveMotors(0.1,0.09)
                    else:
                        self.driveMotors(0.09,0.1)
                elif(self.measures.compass > direction + 1):
                    self.driveMotors(0.1,0.09)
                elif(self.measures.compass < direction -1):
                    self.driveMotors(0.09,0.1)
                else:
                    self.driveMotors(0.1,0.1)
    
            # check for target
            if self.measures.ground == 0:
                end = (round(self.measures.y-self.dY), round(self.measures.x-self.dX))
                # Assign 2 to goal   
                self.maze[end[0]][end[1]] = 2   
                return 

    def rotateBack(self, direction):
        if direction == 0:
            self.rotate(180)
            direction = 180
        elif direction == 90:
            self.rotate(-90)
            direction = -90
        elif direction == -90:
            self.rotate(90)
            direction = 90
        else:
            self.rotate(0)
            direction = 0
        return direction   

    def rotateLeft(self, direction):
        if direction == 0:
            self.rotate(90)
            direction = 90
        elif direction == 90:
            self.rotate(180)
            direction = 180
        elif direction == 180:
            self.rotate(-90)
            direction = -90
        else:
            self.rotate(0)
            direction = 0
        return direction

    def rotateRight(self, direction):
        if direction == 0:
            self.rotate(-90)
            direction = -90
        elif direction == 90:
            self.rotate(0)
            direction = 0
        elif direction == 180:
            self.rotate(90)
            direction = 90
        else:
            self.rotate(180)
            direction = 180
        return direction

    def getStartEnd(self, maze):
        for x in range(len(maze)):
            for y in range(len(maze[x])):
                if maze[x][y] == 5:
                    self.start = (x,y)
                elif maze[x][y] == 2:
                    self.end = (x,y)

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        self.obstacleGrid = [[0] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        x = 0
        i=1
        for child in root.iter('Row'):
            line=child.attrib['Pattern']
            row =int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c+1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][int((c+1)/3*2-1)]='|'
                            self.obstacleGrid[row][int(c/1.5)] = 1
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][int(c/3*2)]='-'
                            self.obstacleGrid[row][int(c/3*2)]=1
                            if (line[c-1] == '-' or line[c-1] == '·') and int(c/3*2) < len(self.obstacleGrid[row])-1:
                                self.obstacleGrid[row][int(c/3*2)+1]=1
                            if (line[c-1] == '-' or line[c-1] == '·') and int(c/3*2) > 0: 
                                self.obstacleGrid[row][int(c/3*2)-1]=1  
                        else:
                            None
               
            i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
startCELL = (0,0)
targetCELL = (0,0)
challenge = 1
path = []
maze = []
start = ()
end = ()

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-n") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--start" or sys.argv[i] == "-s") and i != len(sys.argv) - 1:
        startCELL = [int(i) for i in sys.argv[i + 1].split(",")]
    elif (sys.argv[i] == "--target" or sys.argv[i] == "-t") and i != len(sys.argv) - 1:
        targetCELL = [int(i) for i in sys.argv[i + 1].split(",")]
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
        # mapc.obstacleGrid[startCELL[0]*2][startCELL[1]*2] = 5
        # mapc.obstacleGrid[targetCELL[0]*2][targetCELL[1]*2] = 2
        
        maze = np.flip(mapc.obstacleGrid,0)
        maze[startCELL[0]*2][startCELL[1]*2] = 5
        maze[targetCELL[0]*2][targetCELL[1]*2] = 2

        #print(mapc.obstacleGrid)
        for x in maze:
            print(x)
        start = (startCELL[0]*2 , startCELL[1]*2) # (y,x)
        end = (targetCELL[0]*2 , targetCELL[1]*2)  

        

    rob.run()
