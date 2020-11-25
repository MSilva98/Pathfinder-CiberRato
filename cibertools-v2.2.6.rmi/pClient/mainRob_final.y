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
        self.endCELL = (0,0)
        self.startPosition = (0,0)
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
        if challenge != '3':
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
                        path = astar(self.maze, self.startCELL, self.endCELL)
                        self.followPath(path, challenge)
                    else:
                        print("Challenge ", challenge)
                        self.exploreMap(challenge)

            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)

            elif state == 'return':
                if challenge == '1':
                    pathR = astar(self.maze, self.endCELL, self.startCELL)
                    self.followPath(pathR, challenge)
                else:
                    self.getStartEnd(self.maze)
                    
                    for p in reversed(self.maze):
                        print(p)

                    pathR = astar(self.maze, self.endCELL, self.startCELL)
                    print("pathR:", pathR)
                    savedPath = pathR.copy()
                    self.followPath(pathR, challenge)
                    self.saveMap(savedPath)

                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.finish()
            
    def cellToGPSPoint(self, cell):
        return [self.startPosition[0] + cell[1] - self.startCELL[1] , self.startPosition[1] + cell[0] - self.startCELL[0]]

    def followPath(self, path, challenge):
        while len(path) > 1:
            current = path.pop(0)
            currentPos = [current[1], current[0]]
            dirToMove = (path[0][1]-currentPos[0], path[0][0] - currentPos[1])
            if(dirToMove[0] > 0):
                self.rotate(0)
                print("Move to North")
                if challenge != '3':
                    self.moveFront(0, current, 2)
                else:
                    self.moveFrontOneNOGPS(0)
            elif(dirToMove[0] < 0):
                self.rotate(180)
                print("Move to South")
                if challenge != '3':
                    self.moveFront(180, current, 2)
                else:
                    self.moveFrontOneNOGPS(180)
            if(dirToMove[1] > 0):
                self.rotate(90)
                print("Move to West")
                if challenge != '3':
                    self.moveFront(90, current, 2)
                else:
                    self.moveFrontOneNOGPS(90)
            elif(dirToMove[1] < 0):
                self.rotate(-90)
                print("Move to East")
                if challenge != '3': 
                    self.moveFront(-90, current, 2)
                else:
                    self.moveFrontOneNOGPS(-90)
            self.driveMotors(0.0,0.0)
            
    def moveFrontOneNOGPS(self, direction):
        prevTime = self.measures.time
        while self.measures.time - prevTime < 20:
            err = self.errorCorrectionSensors(direction)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
            
        self.driveMotors(0.00,-0.00)


    def out_t(self, in_motors, prev_out):
        noise = 1
        return 2*((in_motors * 0.5 + prev_out*0.5)* noise)

    def lin(self, out_left, out_right):
        return (out_left + out_right )/ 2

    def xt (self, xt_prev, ang, lin):
        return xt_prev + lin*math.cos(math.radians(ang))
    
    def yt (self, yt_prev, ang, lin):
        return yt_prev + lin*math.sin(math.radians(ang))

    def moveHalfTimed(self, direction, pos, distance = 0.99):
        if(direction == 0):
            posDest = [pos[0], pos[1]+2]
        elif(direction == 180):
            posDest = [pos[0], pos[1]-2]
        elif(direction == 90):
            posDest = [pos[0]+2, pos[1]]
        elif(direction == -90):
            posDest = [pos[0]-2, pos[1]]
        translation_prev = 0
        out_l = 0
        out_r = 0
        translation = 0
        prev_ang = 0
        prevTime = self.measures.time        
        while abs(translation) < distance:
            # check for target
            if self.measures.ground == 0:
                return posDest

            #---- prev translation
            prev_out_l = out_l
            prev_out_r = out_r
            translation_prev = translation
            prev_ang = self.measures.compass
            err = self.errorCorrectionSensors(direction)

            self.driveMotors(0.1 - err , 0.1 + err)

            #----translation
            out_t_l = self.out_t(0.1 - err, prev_out_l)
            out_t_r = self.out_t(0.1 + err, prev_out_r)
            lin = self.lin(out_t_l, out_t_r)
            if(direction == 0 or direction == 180):
                translation = self.xt(translation_prev, prev_ang,lin )
            else:
                translation = self.yt(translation_prev, prev_ang,lin )

            self.readSensors()
            if self.measures.irSensor[self.center_id] > 1.9:
                self.driveMotors(0.0,0.0)
                if self.measures.time - prevTime < 3:
                    return pos
                else:
                    return posDest

        self.driveMotors(0.00,-0.00) 
        return posDest

    def moveFront(self, direction, pos, dist):
        posInit = [self.measures.x, self.measures.y]
        currentPos = [self.measures.x, self.measures.y]
        gpsPos = self.cellToGPSPoint(pos)

        if(direction == 0):
            gpsDest = [gpsPos[0]+dist, gpsPos[1]]
        elif(direction == 180):
            gpsDest = [gpsPos[0]-dist, gpsPos[1]]
        elif(direction == 90):
            gpsDest = [gpsPos[0], gpsPos[1]+dist]
        elif(direction == -90):
            gpsDest = [gpsPos[0], gpsPos[1]-dist]
        move = True

        
        while move:
            err = self.errorCorrection(direction, gpsPos)
            self.driveMotors(0.1 - err , 0.1 + err)
            self.readSensors()

            currentPos = [self.measures.x, self.measures.y]

            if self.measures.irSensor[self.center_id] > 2:
                self.driveMotors(0.0,0.0)
                move = False
                if(direction == 0):
                    if(currentPos[0] < gpsDest[0] - 1.5):
                        return pos
                elif(direction == 180):
                    if(currentPos[0] > gpsDest[0] + 1.5):
                        return pos
                elif(direction == 90):
                    if(currentPos[1] < gpsDest[1] - 1.5):
                        return pos
                elif(direction == -90):
                    if(currentPos[1] > gpsDest[1] + 1.5):
                        return pos

            if(direction == 0):
                posDest = [pos[0], pos[1]+2]
                if(currentPos[0] > gpsDest[0] - 0.1):
                    move  = False
            elif(direction == 180):
                posDest = [pos[0], pos[1]-2]
                if(currentPos[0] < gpsDest[0] + 0.1):
                    move  = False
            elif(direction == 90):
                posDest = [pos[0]+2, pos[1]]
                if(currentPos[1] > gpsDest[1] - 0.1):
                    move  = False
            elif(direction == -90):
                posDest = [pos[0]-2, pos[1]]
                if(currentPos[1] < gpsDest[1] + 0.1):
                    move  = False          

        self.driveMotors(0.00,-0.00) 
        return posDest

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
        return errorDir*0.01 + errorGeo*0.5

    def errorCorrectionSensors(self, direction):
        errorDir = direction - self.measures.compass  # directional error
        if(self.measures.compass < 0 and direction == 180):
            errorDir =  - (direction - abs(self.measures.compass))  # directional error
        errorGeo = 0
        if self.measures.irSensor[self.left_id] > 2.1:
            errorGeo =  2.1 - self.measures.irSensor[self.left_id]
        elif self.measures.irSensor[self.right_id] > 2.1:
            errorGeo = self.measures.irSensor[self.right_id] - 2.1
        return errorDir*0.01 + errorGeo*0.15

    def rotate(self, ang):
        print("Rotate to ", ang)
        if(ang == 90):
            if abs(self.measures.compass) > 90:    
                while abs(self.measures.compass) > 90:
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
            else:
                while self.measures.compass < 90:
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
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
            else:
                while self.measures.compass  < 45 and self.measures.compass > -90:    
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
        elif(ang == 0):
            if self.measures.compass < 0:
                while self.measures.compass > 150 or self.measures.compass < 0:
                    self.driveMotors(-0.03,+0.03)
                    self.readSensors()
            else: 
                while self.measures.compass > 0:
                    self.driveMotors(+0.03,-0.03)
                    self.readSensors()
        self.driveMotors(0.00,-0.00)


    def exploreMap(self, challenge):
        self.maze = [[8] * (CELLCOLS*4) for i in range(CELLROWS*4)]
        self.startCELL = (round((len(self.maze)-1)/2),round((len(self.maze[0])-1)/2))
        # start location
        self.maze[self.startCELL[0]][self.startCELL[1]] = 5

        coord = self.startCELL
        direction = 0
        
        self.checkWalls(direction, coord)

        while True:    
            self.readSensors()
            
            self.ring = 2
            unknownCell = self.getunknownCell(coord, direction)
            
            path = astar2(self.maze, coord, unknownCell)
            if path == None:
                if unknownCell != self.startCELL:    
                    self.maze[unknownCell[0]][unknownCell[1]] = 0
                continue
            
            while len(path)> 1:
                current = path.pop(0)
                currentPos = [current[1], current[0]] 
                dirToMove = (( path[0][1]) - currentPos[0], (path[0][0] - currentPos[1] ))
                                                
                if(dirToMove[0] > 0 and self.maze[coord[0]][coord[1]+1] != 1):
                    self.rotate(0)
                    print("Move to North")
                    direction = 0
                elif(dirToMove[0] < 0 and self.maze[coord[0]][coord[1]-1] != 1):
                    self.rotate(180)
                    print("Move to South")
                    direction = 180
                elif(dirToMove[1] > 0 and self.maze[coord[0] +1][coord[1]] != 1):
                    self.rotate(90)
                    print("Move to West")
                    direction = 90
                elif(dirToMove[1] < 0 and self.maze[coord[0] -1][coord[1]] != 1):
                    self.rotate(-90)
                    print("Move to East")
                    direction = -90
                else:
                    break

                prevCoord = coord
                if challenge == '3':    
                    coord = self.moveHalfTimed(direction, coord, 1.1)
                    self.checkWallsSide(direction, coord)
                    self.moveHalfTimed(direction, prevCoord, 0.7)
                    self.checkWallsFront(direction, coord)
                else:
                    coord = self.moveFront(direction, coord, 1)
                    self.checkWallsSide(direction, coord)
                    self.moveFront(direction, prevCoord, 2)
                    self.checkWallsFront(direction, coord)

                #Assign 0 to all visited cells except target and start
                if tuple(coord) != self.startCELL:  
                    self.maze[coord[0]][coord[1]] = 0
                    
                    if direction == 0 and (coord[0],coord[1]-1) != self.startCELL:
                        self.maze[coord[0]][coord[1]-1] = 0
                    elif direction == 180 and (coord[0],coord[1]+1) != self.startCELL:
                        self.maze[coord[0]][coord[1]+1] = 0
                    elif direction == 90 and (coord[0]-1,coord[1]) != self.startCELL:
                        self.maze[coord[0]-1][coord[1]] = 0
                    elif direction == -90  and (coord[0]+1,coord[1]) != self.startCELL:
                        self.maze[coord[0]+1][coord[1]] = 0

                self.driveMotors(0.00,-0.00)

            # check for target
            if self.measures.ground == 0:
                # Assign 2 to goal   
                self.maze[coord[0]][coord[1]] = 2  
                return

    def getunknownCell(self, center, direction):
        vecs = self.neighbors(self.ring, center[0], center[1])
        vecs = self.sortRing(direction,vecs)
        while True:
            for new_position in vecs:
                node_position = [center[0] + new_position[0], center[1] + new_position[1]]
                if (node_position[0] < (len(self.maze) - 1) and node_position[0] > 0 and node_position[1] < (len(self.maze[0]) -1) and node_position[1] > 0):
                    if self.maze[node_position[0]][ node_position[1] ] == 8 and node_position[0]%2 == 0 and node_position[1]%2 == 0:
                        return (center[0] + new_position[0], center[1] + new_position[1])
            self.ring = self.ring + 2 #next ring
            vecs = self.neighbors(self.ring, center[0], center[1])
            vecs = self.sortRing(direction, vecs)
            if(self.ring > len(self.maze[0]) / 2):
                break
        return -1

    def neighbors(self, radius, r, c):
        cells = list()
        for i in range(r-radius, r+radius+1):
            cells.append(self.maze[i][c-radius:c+radius+1])

        center = (round((len(cells)-1)/2),round((len(cells[0])-1)/2))
        vecs = list()
        for c in range(len(cells)):
            if (c-center[0])%2 == 0:
                for c1 in range(len(cells[0])):
                    if (c1-center[1])%2 == 0:
                        vecs.append((c-center[0], c1-center[1]))
        return vecs


    def sortRing(self, direction, vecs):
        if direction == 0:
            vecs.sort(key=lambda x: x[1], reverse=True)
        elif direction == 180:
            vecs.sort(key=lambda x: x[1])        
        elif direction == 90:
            vecs.sort(key=lambda x: x[0], reverse=True)
            vecs.reverse()
        elif direction == -90:
            vecs.sort(key=lambda x: x[0])
        return vecs

    def checkWallsSide(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minDSides = 0.65
        
        # Sensor value to detect wall
        thresholdSides = 1/minDSides    # threshold side sensors
        
        # check for walls
        if self.measures.irSensor[self.left_id] >= thresholdSides:           # left wall     CASE: LEFT
            if direction == 0:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == -90:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
        else:
            if direction == 0 and (coord[0]+1,coord[1]) != self.startCELL:
                self.maze[coord[0]+1][coord[1]] = 0
            elif direction == 90 and (coord[0],coord[1]-1) != self.startCELL:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == -90 and (coord[0],coord[1]+1) != self.startCELL:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == 180 and (coord[0]-1,coord[1]) != self.startCELL:
                self.maze[coord[0]-1][coord[1]] = 0

        if self.measures.irSensor[self.right_id] >= thresholdSides:          # right wall    CASE: RIGHT
            if direction == 0:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0:
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1:
                    self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1:
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
        else:
            if direction == 0 and (coord[0]-1,coord[1]) != self.startCELL:
                self.maze[coord[0]-1][coord[1]] = 0
            elif direction == 90 and (coord[0],coord[1]+1) != self.startCELL:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == -90 and (coord[0],coord[1]-1) != self.startCELL:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == 180 and (coord[0]+1,coord[1]) != self.startCELL:
                self.maze[coord[0]+1][coord[1]] = 0


    def checkWallsFront(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minD = 0.6

        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        
        if self.measures.irSensor[self.center_id] >= threshold:           # front wall
            self.driveMotors(0.0,0.0)
            if direction == 0:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1



    def checkWalls(self, direction, coord):
        self.readSensors()
        # Minimum distance at which we assume a wall exists
        minD = 0.7
        minDSides = 0.62
        # Sensor value to detect wall
        threshold = 1/minD              # threshold front sensor
        thresholdSides = 1/minDSides    # threshold side sensors
        
        # check back wall
        if coord == self.startCELL:
            if self.measures.irSensor[self.back_id] >= threshold:
                if direction == 0:
                    self.maze[coord[0]][coord[1]-1] = 1 
                    if coord[0] > 0 and coord[1] > 0: 
                        self.maze[coord[0]-1][coord[1]-1] = 1
                    if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                        self.maze[coord[0]+1][coord[1]-1] = 1
                elif direction == 90:
                    self.maze[coord[0]-1][coord[1]] = 1
                    if coord[0] > 0 and coord[1] > 0: 
                        self.maze[coord[0]-1][coord[1]-1] = 1
                    if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                        self.maze[coord[0]-1][coord[1]+1] = 1
                elif direction == -90:
                    self.maze[coord[0]+1][coord[1]] = 1
                    if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                        self.maze[coord[0]+1][coord[1]-1] = 1
                    if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                        self.maze[coord[0]+1][coord[1]+1] = 1
                elif direction == 180:
                    self.maze[coord[0]][coord[1]+1] = 1
                    if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                        self.maze[coord[0]-1][coord[1]+1] = 1
                    if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                        self.maze[coord[0]+1][coord[1]+1] = 1
            else:
                if direction == 0 and (coord[0],coord[1]-1) != self.startCELL:
                    self.maze[coord[0]][coord[1]-1] = 0
                elif direction == 90 and (coord[0]-1,coord[1]) != self.startCELL:
                    self.maze[coord[0]-1][coord[1]] = 0
                elif direction == -90 and (coord[0]+1,coord[1]) != self.startCELL:
                    self.maze[coord[0]+1][coord[1]] = 0
                elif direction == 180 and (coord[0],coord[1]+1) != self.startCELL:
                    self.maze[coord[0]][coord[1]+1] = 0


        # check for walls
        if self.measures.irSensor[self.left_id] >= thresholdSides:           # left wall     CASE: LEFT
            if direction == 0:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == -90:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
        else:
            if direction == 0 and (coord[0]+1,coord[1]) != self.startCELL:
                self.maze[coord[0]+1][coord[1]] = 0
            elif direction == 90 and (coord[0],coord[1]-1) != self.startCELL:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == -90 and (coord[0],coord[1]+1) != self.startCELL:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == 180 and (coord[0]-1,coord[1]) != self.startCELL:
                self.maze[coord[0]-1][coord[1]] = 0

        if self.measures.irSensor[self.right_id] >= thresholdSides:          # right wall    CASE: RIGHT
            if direction == 0:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0:
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1:
                    self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1:
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
            elif direction == 180:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
        else:
            if direction == 0 and (coord[0]-1,coord[1]) != self.startCELL:
                self.maze[coord[0]-1][coord[1]] = 0
            elif direction == 90 and (coord[0],coord[1]+1) != self.startCELL:
                self.maze[coord[0]][coord[1]+1] = 0
            elif direction == -90 and (coord[0],coord[1]-1) != self.startCELL:
                self.maze[coord[0]][coord[1]-1] = 0
            elif direction == 180 and (coord[0]+1,coord[1]) != self.startCELL:
                self.maze[coord[0]+1][coord[1]] = 0

        if self.measures.irSensor[self.center_id] >= threshold:           # front wall
            self.driveMotors(0.0,0.0)
            if direction == 0:
                self.maze[coord[0]][coord[1]+1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == 90:
                self.maze[coord[0]+1][coord[1]] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]+1][coord[1]+1] = 1
            elif direction == -90:
                self.maze[coord[0]-1][coord[1]] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] < len(self.maze[0])-1: 
                    self.maze[coord[0]-1][coord[1]+1] = 1
            elif direction == 180:
                self.maze[coord[0]][coord[1]-1] = 1
                if coord[0] > 0 and coord[1] > 0: 
                    self.maze[coord[0]-1][coord[1]-1] = 1
                if coord[0] < len(self.maze)-1 and coord[1] > 0: 
                    self.maze[coord[0]+1][coord[1]-1] = 1

    def getStartEnd(self, maze):
        for x in range(len(maze)):
            for y in range(len(maze[x])):
                if maze[x][y] == 5:
                    self.startCELL = (x,y)
                elif maze[x][y] == 2:
                    self.endCELL = (x,y)

    def saveMap(self, path):
        f = open("map.txt", "w+")

        txt = list()
        for r in range(len(self.maze)):
            s = ""
            for c in range(len(self.maze[r])):
                if self.maze[r][c] == 8:
                    s += " "
                elif self.maze[r][c] == 1:
                    if r%2 == 0:
                        s += "|"
                    else:
                        s += "-"
                elif self.maze[r][c] == 0:
                    if (r,c) in path:
                        s += "p"
                    elif (r,c-1) in path and (r,c+1) in path:
                        s += "p"
                    elif (r-1,c) in path and (r+1,c) in path:
                        s += "p"
                    else:
                        s += "v"
                elif self.maze[r][c] == 5:
                    s += "S"
                elif self.maze[r][c] == 2:
                    s += "E"
            s += "\n"
            txt.append(s)

        for t in reversed(txt):
            f.write(t)
        f.close()

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
    rob=MyRob(rob_name,pos,[0.0,55.0,-55.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
        mapc.obstacleGrid[startCELL[0]*2][startCELL[1]*2] = 5
        mapc.obstacleGrid[targetCELL[0]*2][targetCELL[1]*2] = 2
        rob.maze = mapc.obstacleGrid

        for x in rob.maze:
            print(x)

        rob.startCELL = (startCELL[0]*2 , startCELL[1]*2) # (y,x)
        rob.endCELL = (targetCELL[0]*2 , targetCELL[1]*2)  

        

    rob.run()
