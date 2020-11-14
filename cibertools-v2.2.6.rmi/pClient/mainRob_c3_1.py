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

        path = astar(maze, start, end)
        print(path)
        state = 'stop'
        stopped_state = 'run'
        self.readSensors()
        self.startPosition = (self.measures.x, self.measures.y)
        while True:
            self.readSensors()
            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                    self.driveMotors(0.0,0.0)
                else:
                    self.followPath(path)
                    self.driveMotors(0.00,-0.00)

            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                print("RETURN")
                pathR = astar(maze, end, start)
                print(pathR)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.followPath(pathR)
                self.finish()
            
        


    def cellToGPSPoint(self, cell):
        print("cell: ", cell, " startCell: ", startCELL)
        return [self.startPosition[0] + cell[0] - startCELL[1]*2 , self.startPosition[1] + cell[1] - startCELL[0]*2]



    def followPath(self, path):
        while len(path)> 1:
            current = path.pop(0)
            currentPos = [current[1], current[0]] 
            dirToMove = (( path[0][1]) - currentPos[0], (path[0][0] - currentPos[1] ))
            print(dirToMove, path[0],  currentPos)
            if(dirToMove[0] > 0):
                self.rotate(0)
                self.moveFrontOne(0, currentPos)
            elif(dirToMove[0] < 0):
                self.rotate(180)
                self.moveFrontOne(180, currentPos)
            if(dirToMove[1] > 0):
                self.rotate(90)
                self.moveFrontOne(90, currentPos)
            elif(dirToMove[1] < 0):
                self.rotate(-90)
                self.moveFrontOne(-90, currentPos)
            self.driveMotors(0.00,-0.00)
            
    def moveFrontOne(self, direction, pos):
        print("MOVE TO " + str(direction))
        prevTime = self.measures.time
        while self.measures.time - prevTime < 20:
            #print(self.measures.irSensor[self.left_id], self.measures.irSensor[self.right_id] )
            err = self.errorCorrection(direction)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()
        print("CHEGOU A CELULA")
        self.driveMotors(0.00,-0.00)
    
    def errorCorrection(self, direction):

        errorDir = direction - self.measures.compass  # directional error
        if(self.measures.compass < 0 and direction == 180):
            errorDir =  - (direction - abs(self.measures.compass))  # directional error
        errorGeo = 0
        if self.measures.irSensor[self.left_id] > 2.2:
            errorGeo =  2.2 - self.measures.irSensor[self.left_id]
            print("COORECT CLOSE TO WALL LEFT")
        elif self.measures.irSensor[self.right_id] > 2.2:
            errorGeo = self.measures.irSensor[self.right_id] - 2.2
            print("COORECT CLOSE TO WALL RIGHT")
        return errorDir*0.01 + errorGeo*0.1
 
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


    def rotate(self, ang):
        print("ROTARING to ", ang)
        if(ang == 90):
            if abs(self.measures.compass) > 90:    
                while abs(self.measures.compass) > 90:
                    print("Rotating... ", abs(self.measures.compass))
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
            else:
                while self.measures.compass < 90:
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
        elif(ang == 180):
            if self.measures.compass >= 0:
                while self.measures.compass < 178 and self.measures.compass >= 0:
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else:
                while self.measures.compass < 178 and self.measures.compass <= 0:
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        elif(ang == -90):
            if abs(self.measures.compass) > 90: 
                while abs(self.measures.compass) > 90:    
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else:
                while abs(self.measures.compass) < 90:    
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        elif(ang == 0):
            if self.measures.compass < 0:
                while self.measures.compass < 0:
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(-0.05,+0.05)
                    self.readSensors()
            else: 
                while self.measures.compass > 0:
                    print("Rotating... ", self.measures.compass)
                    self.driveMotors(+0.05,-0.05)
                    self.readSensors()
        self.driveMotors(0.00,-0.00)

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
        mapc.obstacleGrid[startCELL[0]*2][startCELL[1]*2] = 5
        mapc.obstacleGrid[targetCELL[0]*2][targetCELL[1]*2] = 2
        
        #maze = np.flip(mapc.obstacleGrid,0)
        maze = mapc.obstacleGrid
        #maze[startCELL[1]*2][startCELL[0]*2] = 5
        #maze[targetCELL[1]*2][targetCELL[0]*2] = 2

        for x in range(len(maze)):
            for y in range(len(maze[x])):
                if maze[x][y] == 5:
                    start = (x,y)
                elif maze[x][y] == 2:
                    end = (x,y)


        #print(mapc.obstacleGrid)
        for x in maze:
            print(x)
        #start = (startCELL[1]*2 , startCELL[0]*2) # (y,x)
        #end = (targetCELL[1]*2 , targetCELL[0]*2)  

        

    rob.run()
