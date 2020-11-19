import sys
from croblink import *
import math
import xml.etree.ElementTree as ET

import numpy as np

from astar import *

from astar2 import *

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
        self.startCELL = (0,0)
        self.ring = 2
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
                    if challenge == '1':
                        print("Challenge 1")
                        path = astar(maze, start, end)
                        self.followPath(path)
                    elif challenge == '2':
                        print("Challenge 2")
                        self.exploreMap()

            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state == 'return':
                if challenge == '1':
                    pathR = astar(maze, end, start)
                elif challenge == '2':
                    self.getStartEnd(self.maze)
                    
                    for p in self.maze:
                        print(p)

                    print(self.end, self.startCELL, self.maze[self.startCELL[0]][self.startCELL[1]], self.maze[self.end[0]][self.end[1]])
                    print("startCell", self.startCELL)
                    pathR = astar(self.maze, self.end, self.startCELL)
                    print("pathR:", pathR)
                    savedPath = pathR.copy()

                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                if challenge == '1':
                    self.followPath(pathR)
                elif challenge == '2':
                    self.followPath(pathR, False)
                    print(savedPath)
                    self.saveMap(savedPath)
                self.finish()
            
    def cellToGPSPoint(self, cell):
        # print("cell: ", cell, " startCell: ", self.startCELL)
        return [self.startPosition[0] + cell[1] - self.startCELL[1] , self.startPosition[1] + cell[0] - self.startCELL[0]]

    def followPath(self, path, double=True):
        print(path)
        while len(path) > 1:
            current = path.pop(0)
            currentPos = [current[1], current[0]]
            dirToMove = (path[0][1]-currentPos[0], path[0][0] - currentPos[1])
            print("Move: ", dirToMove)
            if(dirToMove[0] > 0):
                self.rotate(0)
                self.moveFrontOne(0, current, double)
            elif(dirToMove[0] < 0):
                self.rotate(180)
                self.moveFrontOne(180, current, double)
            if(dirToMove[1] > 0):
                self.rotate(90)
                self.moveFrontOne(90, current, double)
            elif(dirToMove[1] < 0):
                self.rotate(-90)
                self.moveFrontOne(-90, current, double)
            self.driveMotors(0.0,0.0)
            
    def moveFirstHalf(self, direction, pos, double=True):
        # print("MOVE FRONT")
        posInit = [self.measures.x, self.measures.y]
        currentPos = [self.measures.x, self.measures.y]
        gpsPos = self.cellToGPSPoint(pos)

        # print(posInit, gpsPos, pos)

        if(direction == 0):
            posDest = [pos[0], pos[1]+2]
            gpsDest = [gpsPos[0]+1 , gpsPos[1]]
        elif(direction == 180):
            gpsDest = [gpsPos[0]-1, gpsPos[1]]
            posDest = [pos[0], pos[1]-2]
        elif(direction == 90):
            gpsDest = [gpsPos[0], gpsPos[1]+1]
            posDest = [pos[0]+2, pos[1]]
        elif(direction == -90):
            gpsDest = [gpsPos[0] , gpsPos[1]-1]
            posDest = [pos[0]-2, pos[1]]
        move = True

        while move:
            err = self.errorCorrection(direction, gpsPos)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()

            currentPos = [self.measures.x, self.measures.y]

            print(self.measures.irSensor)

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
                print("STOPPPPPP", self.measures.irSensor)
                move = False

            if(direction == 0):
                if(currentPos[0] > gpsDest[0] -0.1):
                    move  = False
            elif(direction == 180):
                if(currentPos[0] < gpsDest[0] + 0.1):
                    move  = False
            elif(direction == 90):
                if(currentPos[1] > gpsDest[1] - 0.1):
                    move  = False
            elif(direction == -90):
                if(currentPos[1] < gpsDest[1] + 0.1):
                    move  = False          



        self.driveMotors(0.00,-0.00) 
        return posDest


    def moveSecondHalf(self, direction, pos, double=True):
        # print("MOVE FRONT")
        posInit = [self.measures.x, self.measures.y]
        currentPos = [self.measures.x, self.measures.y]
        gpsPos = self.cellToGPSPoint(pos)

        print(posInit, gpsPos, pos)

        if(direction == 0):
            gpsDest = [gpsPos[0]+2 , gpsPos[1]]
        elif(direction == 180):
            gpsDest = [gpsPos[0]-2, gpsPos[1]]
        elif(direction == 90):
            gpsDest = [gpsPos[0], gpsPos[1]+2]
        elif(direction == -90):
            gpsDest = [gpsPos[0] , gpsPos[1]-2]
        move = True

        while move:
            err = self.errorCorrection(direction, gpsPos)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()

            currentPos = [self.measures.x, self.measures.y]

            print(self.measures.irSensor)

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
                print("STOPPPPPP")
                move = False

            if(direction == 0):
                posDest = [pos[0], pos[1]+2]
                if(currentPos[0] > gpsDest[0] - 0.1):
                    move  = False
            elif(direction == 180):
                posDest = [pos[0], pos[1]-2]
                if(currentPos[0] < gpsDest[0] + 0.1):
                    move  = False
            elif(direction == 90):
                posDest = [ pos[0]+2, pos[1]]
                if(currentPos[1] > gpsDest[1] - 0.1):
                    move  = False
            elif(direction == -90):
                posDest = [ pos[0]-2, pos[1]]
                if(currentPos[1] < gpsDest[1] + 0.1):
                    move  = False          

        if(direction == 0):
            if(currentPos[0] - gpsDest[0] > 1.5):
                posDest = [pos[0], pos[1]+2]
        elif(direction == 180):
            if(gpsDest[0] - currentPos[0] > 1.5):
                posDest = [pos[0], pos[1]-2]
        elif(direction == 90):
            if(currentPos[1] - gpsDest[1] > 1.5):
                posDest = [pos[0]+2, pos[1]]
        elif(direction == -90):
            if(gpsDest[1] - currentPos[1] > 1.5):
                posDest = [pos[0]-2, pos[1]]	


        self.driveMotors(0.00,-0.00) 
        return posDest

    def moveFrontOne(self, direction, pos, double=True):
        # print("MOVE FRONT")
        posInit = [self.measures.x, self.measures.y]
        currentPos = [self.measures.x, self.measures.y]
        gpsPos = self.cellToGPSPoint(pos)

        # print(posInit, gpsPos, pos)

        if(direction == 0):
            gpsDest = [gpsPos[0]+2 , gpsPos[1]]
        elif(direction == 180):
            gpsDest = [gpsPos[0]-2, gpsPos[1]]
        elif(direction == 90):
            gpsDest = [gpsPos[0], gpsPos[1]+2]
        elif(direction == -90):
            gpsDest = [gpsPos[0] , gpsPos[1]-2]
        move = True

        while move:
            err = self.errorCorrection(direction, gpsPos)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()

            currentPos = [self.measures.x, self.measures.y]

            print(self.measures.irSensor)

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
                print("STOPPPPPP")
                move = False

            if(direction == 0):
                posDest = [pos[0], pos[1]+2]
                if(currentPos[0] > gpsDest[0] - 0.1):
                    move  = False
            elif(direction == 180):
                posDest = [pos[0], pos[1]-2]
                if(currentPos[0] < gpsDest[0] + 0.1):
                    move  = False
            elif(direction == 90):
                posDest = [ pos[0]+2, pos[1]]
                if(currentPos[1] > gpsDest[1] - 0.1):
                    move  = False
            elif(direction == -90):
                posDest = [ pos[0]-2, pos[1]]
                if(currentPos[1] < gpsDest[1] + 0.1):
                    move  = False          

        if(direction == 0):
            if(currentPos[0] - gpsDest[0] > 1.5):
                posDest = [pos[0], pos[1]+2]
        elif(direction == 180):
            if(gpsDest[0] - currentPos[0] > 1.5):
                posDest = [pos[0], pos[1]-2]
        elif(direction == 90):
            if(currentPos[1] - gpsDest[1] > 1.5):
                posDest = [pos[0]+2, pos[1]]
        elif(direction == -90):
            if(gpsDest[1] - currentPos[1] > 1.5):
                posDest = [pos[0]-2, pos[1]]	


        self.driveMotors(0.00,-0.00) 
        return posDest

        # dif = (round(currentPos[0]-posInit[0]), round(currentPos[1]-posInit[1]))
        # print(dif)
        # if dif == (2,0):
        #     return [pos[1], pos[0]+2]
        # elif dif == (-2.0):
        #     return [pos[1], pos[0]-2]
        # elif dif == (0,2):
        #     return [pos[1]+2, pos[0]]
        # elif dif == (0,-2):
        #     return [pos[1]-2, pos[0]]
        # else:
        #     return [pos[1], pos[0]]

    def errorCorrection(self, direction, pos):
        errorDir = direction - self.measures.compass  # directional error
        if(self.measures.compass < 0 and direction == 180):
            errorDir =  - (direction - abs(self.measures.compass))  # directional error
        if(direction == 0):
            errorGeo =  pos[1] - self.measures.y #Geographical error
        elif(direction == 180):
            errorGeo =  self.measures.y - pos[1]
        elif(direction == -90):
            errorGeo =  pos[0] - self.measures.x #Geographical error
        else:
            errorGeo =  self.measures.x - pos[0]
        #print(errorGeo)
        # if errorGeo > 0.2:
        #     quit()
        return errorDir*0.01 + errorGeo*0.5

    def angConverter(self, ang):
        if(ang < 0):
            return (360 - ang)
        return ang

    def rotate(self, ang):
        print("Rotating to ", ang)
        if(ang == 90):
            if abs(self.measures.compass) > 90:    
                while abs(self.measures.compass) > 90:
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
                # if self.measures.compass != 90:
                #     self.rotate(90)
            else:
                while self.measures.compass < 90:
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
                # if self.measures.compass != 90:
                #     self.rotate(90)
        elif(ang == 180):
            if self.measures.compass > 0:
                while self.measures.compass < 178 and self.measures.compass >= 0:
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
            else:
                while self.measures.compass < 178 and self.measures.compass <= 0:
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
        elif(ang == -90):
            if self.measures.compass > 45 or self.measures.compass < -90: 
                while self.measures.compass > 45 or self.measures.compass < -90:    
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
                # if self.measures.compass != -90:
                #     self.rotate(-90)
            else:
                while self.measures.compass  < 45 and self.measures.compass > -90:    
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
                # if self.measures.compass != -90:
                #     self.rotate(-90)
        elif(ang == 0):
            if self.measures.compass < 0:
                while self.measures.compass < 0:
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
                # if self.measures.compass != 0:
                #     self.rotate(0)
            else: 
                while self.measures.compass > 0:
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
                # if self.measures.compass != 0:
                #     self.rotate(0)

        self.driveMotors(0.00,-0.00)

    def exploreMap(self): 
        self.maze = [[8] * (CELLCOLS*4-1) for i in range(CELLROWS*4-1)]
        #self.maze = [[8] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1)]
        self.startCELL = (round((len(self.maze)-1)/2) + 1,round((len(self.maze[0])-1)/2) +1)
        #self.startCELL = (14,28)
        # start location
        self.maze[self.startCELL[0]][self.startCELL[1]] = 5

        coord = self.startCELL

        direction = 0

        print("coord:", coord)
        #self.checkWalls(direction, coord)

        self.checkWalls(direction, coord)
        while True:    
            self.readSensors()
            
            print(self.measures.irSensor)


            # print(self.maze[coord[0]-4])
            # print(self.maze[coord[0]-3])
            # print(self.maze[coord[0]-2])
            # print(self.maze[coord[0]-1])
            # print(self.maze[coord[0]])
            # print(self.maze[coord[0]+1])
            # print(self.maze[coord[0]+2])
            # print(self.maze[coord[0]+3])
            # print(self.maze[coord[0]+4])
            self.ring = 2
            unknownCell = self.getunknownCell(coord)

            #unknownCell = (18,44)
            print("coord:", coord)
            print("unknownCell: ", unknownCell)
            
            #path = astar2(self.maze, coord, (self.startCELL[0] + 10, self.startCELL[1]))
            path = astar2(self.maze, coord, unknownCell)
            if path == None:
                print("NO PATH")
                self.maze[unknownCell[0]][unknownCell[1]] = 0
                continue

            #FOLLOW PATH
            print("PATH: ", path)
            while len(path)> 1:
                current = path.pop(0)
                currentPos = [current[1], current[0]] 
                dirToMove = (( path[0][1]) - currentPos[0], (path[0][0] - currentPos[1] ))
                
                #self.checkWalls(direction, coord)
                
                print("coord:", coord)
                print("dirToMove", dirToMove)
                
                if(dirToMove[0] > 0 and self.maze[coord[0]][coord[1]+1] != 1):
                    self.rotate(0)
                    direction = 0
                elif(dirToMove[0] < 0 and self.maze[coord[0]][coord[1]-1] != 1):
                    self.rotate(180)
                    direction = 180
                elif(dirToMove[1] > 0 and self.maze[coord[0] +1][coord[1]] != 1):
                    self.rotate(90)
                    direction = 90
                elif(dirToMove[1] < 0 and self.maze[coord[0] -1][coord[1]] != 1):
                    self.rotate(-90)
                    direction = -90
                else:
                    print("BREAK")
                    break

                #coord = self.moveFrontOne(direction, coord)
                prevCoord = coord
                print("START FIRST HALF")
                coord = self.moveFirstHalf(direction, coord)
                print("FINISH FIRST HALF")
                self.checkWallsSide(direction, coord)
                print("START SECOND HALF", prevCoord, coord)
                self.moveSecondHalf(direction, prevCoord)
                print("FINISH SECOND HALF")
                self.checkWallsFront(direction, coord)
                #Assign 0 to all visited cells except target and start
                if coord != self.startCELL:  
                    self.maze[coord[0]][coord[1]] = 0
                    
                    if direction == 0:
                        self.maze[coord[0]][coord[1]-1] = 0
                    elif direction == 180:
                        self.maze[coord[0]][coord[1]+1] = 0
                    elif direction == 90:
                        self.maze[coord[0]-1][coord[1]] = 0
                    else:
                        self.maze[coord[0]+1][coord[1]] = 0

                self.driveMotors(0.00,-0.00)

            # check for target
            if self.measures.ground == 0:
                # Assign 2 to goal   
                self.maze[coord[0]][coord[1]] = 2  
                return 

    def isWallinFront(self):
        if self.measures.irSensor[self.center_id] > 1/0.7:
                return True
        return False

    def getunknownCell2(self):
        for x in range(len(self.maze)):
            for y in range(len(self.maze[x])):
                if self.maze[x][y] == 8 and x%2 == 0 and y%2 == 0:
                    return (x,y)
        return -1

    def getunknownCell(self, center):
        while True:
            adjacent_cells = ((0, -self.ring), (0, self.ring), (-self.ring, 0), (self.ring, 0), (-self.ring, -self.ring), (-self.ring, self.ring), (self.ring, -self.ring), (self.ring, self.ring))
            for new_position in adjacent_cells: # Adjacent cells
                node_position = [center[0] + new_position[0], center[1] + new_position[1]]
                print("node", node_position[0])
                #is in range
                if not (node_position[0] > (len(self.maze) - 1) or node_position[0] < 0 or node_position[1] > (len(self.maze[len(self.maze)-1]) -1) or node_position[1] < 0):
                    # print("HERE")
                    #is unknown
                    if self.maze[node_position[0]][ node_position[1] ] == 8 :
                        return (center[0] + new_position[0], center[1] + new_position[1])
            self.ring = self.ring + 2 #next ring
            if(self.ring > len(self.maze) / 2):
                break
        return -1


    def checkWallsSide(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minD = 0.7
        #minDSides = 0.59
        minDSides = 0.625
        # minDSidesMove = 0.6
        # min60 = (minD+0.5)/math.sin(math.radians(60))-0.5
        # print("coord (checkWals): ", coord)
        #print(maze)
        #mazeReturn = maze.copy()
        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        thresholdSides = 1/minDSides    # threshold side sensors
        # thresholdSidesMove = 1/minDSidesMove

        # check for walls
        if self.measures.irSensor[self.left_id] >= thresholdSides:           # left wall     CASE: LEFT
            print("ADDED LEFT WALL -> ", coord)
            if direction == 0:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
        else:
            if direction == 0:
                self.maze[coord[0]+1][coord[1]] = 0
            elif direction == 90:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == -90:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]] = 0

        if self.measures.irSensor[self.right_id] >= thresholdSides:          # right wall    CASE: RIGHT
            print("ADDED RIGHT WALL -> ", coord)
            if direction == 0:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
        else:
            if direction == 0:
                self.maze[coord[0]-1][coord[1]] = 0
            elif direction == 90:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == -90:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]] = 0


    def checkWallsFront(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minD = 0.7
        minDSides = 0.59
        # minDSidesMove = 0.6
        # min60 = (minD+0.5)/math.sin(math.radians(60))-0.5
        # print("coord (checkWals): ", coord)
        #print(maze)
        #mazeReturn = maze.copy()
        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        thresholdSides = 1/minDSides    # threshold side sensors
        # thresholdSidesMove = 1/minDSidesMove

        if self.measures.irSensor[self.center_id] >= threshold:           # front wall
            self.driveMotors(0.0,0.0)
            print("ADDED FRONT WALL-> ", coord) 
            if direction == 0:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1



    def checkWalls(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minD = 0.7
        minDSides = 0.59
        # minDSidesMove = 0.6
        # min60 = (minD+0.5)/math.sin(math.radians(60))-0.5
        # print("coord (checkWals): ", coord)
        #print(maze)
        #mazeReturn = maze.copy()
        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        thresholdSides = 1/minDSides    # threshold side sensors
        # thresholdSidesMove = 1/minDSidesMove

        # check back wall
        if coord == self.startCELL:
            if self.measures.irSensor[self.back_id] >= threshold:
                if direction == 0:
                    self.maze[coord[0]-1][coord[1]-1] = 1
                    self.maze[coord[0]][coord[1]-1] = 1
                    self.maze[coord[0]+1][coord[1]-1] = 1
                elif direction == 90:
                    self.maze[coord[0]-1][coord[1]-1] = 1
                    self.maze[coord[0]-1][coord[1]] = 1
                    self.maze[coord[0]-1][coord[1]+1] = 1
                elif direction == -90:
                    self.maze[coord[0]+1][coord[1]-1] = 1
                    self.maze[coord[0]+1][coord[1]] = 1
                    self.maze[coord[0]+1][coord[1]+1] = 1
                elif direction == 180:
                    self.maze[coord[0]-1][coord[1]+1] = 1
                    self.maze[coord[0]][coord[1]+1] = 1
                    self.maze[coord[0]+1][coord[1]+1] = 1
            else:
                if direction == 0:
                    self.maze[coord[0]][coord[1]-1] = 0
                elif direction == 90:
                    self.maze[coord[0]-1][coord[1]] = 0
                elif direction == -90:
                    self.maze[coord[0]+1][coord[1]] = 0
                elif direction == 180:
                    self.maze[coord[0]][coord[1]+1] = 0

        # check for walls
        if self.measures.irSensor[self.left_id] >= thresholdSides:           # left wall     CASE: LEFT
            print("ADDED LEFT WALL -> ", coord)
            if direction == 0:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
        else:
            if direction == 0:
                self.maze[coord[0]+1][coord[1]] = 0
            elif direction == 90:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == -90:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]] = 0

        if self.measures.irSensor[self.right_id] >= thresholdSides:          # right wall    CASE: RIGHT
            print("ADDED RIGHT WALL -> ", coord)
            if direction == 0:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
        else:
            if direction == 0:
                self.maze[coord[0]-1][coord[1]] = 0
            elif direction == 90:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == -90:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]] = 0

        if self.measures.irSensor[self.center_id] >= threshold:           # front wall
            self.driveMotors(0.0,0.0)
            print("ADDED FRONT WALL-> ", coord) 
            if direction == 0:
                self.maze[coord[0]-1][coord[1]+1] = 1
                self.maze[coord[0]][coord[1]+1] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]+1][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]] = 1
                self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]-1][coord[1]] = 1
                self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]-1] = 1
                self.maze[coord[0]][coord[1]-1] = 1
                self.maze[coord[0]+1][coord[1]-1] = 1
        #return mazeReturn

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
                    self.startCELL = (x,y)
                elif maze[x][y] == 2:
                    self.end = (x,y)

    def saveMap(self, path):
        f = open("map.txt", "w+")

        for r in range(len(self.maze)):
            s = ""
            for c in range(len(self.maze[r])):
                if self.maze[r][c] == 8:
                    s += " "
                elif self.maze[r][c] == 1:
                    if r%2 == 0:
                        s += "-"
                    else:
                        s += "|"
                elif self.maze[r][c] == 0:
                    # print((r,c), path)
                    if (r,c) in path or (r+1,c) in path or (r-1,c) in path or (r,c+1) in path or (r,c-1) in path:
                        s += "p"
                    else:
                        s += "v"
                elif self.maze[r][c] == 5:
                    s += "S"
                elif self.maze[r][c] == 2:
                    s += "E"
            s += "\n"
            f.write(s)
        f.close()

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
        maze = mapc.obstacleGrid
        rob.startCELL = startCELL

        for x in maze:
            print(x)
        start = (startCELL[0]*2 , startCELL[1]*2) # (y,x)
        end = (targetCELL[0]*2 , targetCELL[1]*2)  

        

    rob.run()
