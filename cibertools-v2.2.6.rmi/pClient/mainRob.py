

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
        self.readSensors()
        self.startPosition = (self.measures.y, self.measures.x)
        while True:
            self.readSensors()
            #print(self.measures.ground)
            #print(self.measures.groundReady)
            #print(self.measures.compass)
            #print(self.measures.x)
            #print(self.measures.y)
            if self.measures.endLed:
                print(self.rob_name + " exiting")
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
                    return
                #self.wander()
                #self.moveFrontOne()
                #self.rotate(90)
                #self.moveFrontOne()

                self.followPath()

                #self.followPath2()
                return
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                #self.wander()
            

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
            


    def followPath(self):
        while len(path)> 1:
            currentPos = path.pop(0)
            dirToMove = (( path[0][0]) - currentPos[0], (path[0][1] - currentPos[1] ))
            print(dirToMove)
            if(dirToMove[0] > 0):
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
        while distance < 2 :
            if(self.measures.compass > direction + 1):
                self.driveMotors(+0.1,0.07)
            elif(self.measures.compass < direction -1):
                self.driveMotors(0.07,+0.1)
            else:
                self.driveMotors(+0.1,+0.1)
            self.readSensors()
            currentPos = [self.measures.x, self.measures.y]
            distance = math.hypot(currentPos[0] - posInit[0], currentPos[1] - posInit[1])
            #print(distance)
        self.driveMotors(0.00,-0.00)
    

    def angConverter(self, ang):
        if(ang < 0):
            return (360- ang)
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
        if(ang > 0):
            while self.measures.compass < ang-1:
                self.driveMotors(-0.1,+0.1)
                self.readSensors()
        elif(ang < 0):
            while self.measures.compass > ang+1:
                self.driveMotors(0.1,-0.1)
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
        
        maze = np.flip(mapc.obstacleGrid,0)

        #print(mapc.obstacleGrid)
        for x in maze:
            print(x)
        start = (startCELL[0]*2 , startCELL[1]*2) # (y,x)
        end = (targetCELL[0]*2 , targetCELL[1]*2)  
        print(targetCELL*2)
        path = astar(maze, start, end)
        print(path)

    rob.run()
