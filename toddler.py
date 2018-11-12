    #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 11 11:25:29 2018

@author: s1873447
"""

#!/usr/bin/env python

import time
import numpy as np
from copy import copy, deepcopy
import cv2

initalTurnTimeDiff = 0.56
initialForwardTimeDiff = 0.266

# UPDATE THESE BASED ON EXPERIMENTS
#motorFullTurnTime = 7.0
#motorFullTurnTime = 5.4
odometerAngularSpeed = 360 / 15.0
#motorSpeed = 0.19
#motorSpeed = 0.25
odometerSpeed = 3.0 / 59.2

#motorTurnNoise = 70


robotWidth = 0.25
robotLength = 0.32

leftIRSensorLocalPosition = np.array([-0.13, 0.16])
leftIRSensorAngleOffset = -45
rightIRSensorLocalPosition = np.array([0.13, 0.16])
rightIRSensorAngleOffset = 45

sonarSensorLocalPosition = np.array([0.0, 0.15])
sonarConeAngle = 25

class SensorLocations:

    def __init__(self):
        self.sensor_locations = {"lights": [7], "ir": {"left": 4, "right": 5}, "sonar": 3}
        self.input_locations = {"left_whisker":0, "right_whisker":1, "odometer":7}
        
    def getLightSensors(self):
        return self.sensor_locations["lights"]
    
    def getIRSensorLeft(self):
        return self.sensor_locations["ir"]["left"]
    
    def getIRSensorRight(self):
        return self.sensor_locations["ir"]["right"]
    
    def getSonar(self):
        return self.sensor_locations["sonar"]
    
    def getWhiskerLeft(self):
        return self.input_locations["left_whisker"]
    
    def getWhiskerRight(self):
        return self.input_locations["right_whisker"]
    
    def getOdometer(self):
        return self.input_locations["odometer"]
    
class MotorBoardLocations:
    
    def __init__(self):
        self.motor_board_locations = {"light": 3, "left": 2, "right": 4}
    
    def getLeftMotor(self):
        return self.motor_board_locations["left"]

    def getRightMotor(self):
        return self.motor_board_locations["right"]
    
    def getLight(self):
        return self.motor_board_locations["light"]


class SensorManager:
    
    def __init__(self, getSensors, getInputs):
        self.sensorLocations = SensorLocations()
        self.getSensors = getSensors
        self.getInputs = getInputs
        self.sensorReadings = self.getSensors()
        self.inputReadings = self.getInputs()
        self.odometerCount = 0
        self.oldOdometer = self.getOdometer()
        
        self.lastUpdateTime = None
        self.forwardTimeDifferences = [initialForwardTimeDiff]
        self.turnTimeDifferences = [initalTurnTimeDiff]
        
        self.onPOI = False
        
    
    def updateOdometer(self, command, onPOI, onStart):   
        if onStart or onPOI:
            return
        
        if self.getOdometer() and not self.oldOdometer:
            if self.lastUpdateTime != None:
                
                timeDifference = time.time() - self.lastUpdateTime
                
                if command == 'forward':
                    self.forwardTimeDifferences += [timeDifference]
                elif command == 'turn_left' or command == 'turn_right':
                    self.turnTimeDifferences += [timeDifference]
            self.odometerCount += 1
            self.lastUpdateTime = time.time()
            
        self.oldOdometer = self.getOdometer()
        
    
    def printRawReadings(self):
        print("Sensors: " + str(self.sensorReadings) + ", Inputs: " + str(self.inputReadings))
        #print("Inputs: " + str(self.inputReadings))
        
    def displaySensorReadings(self):
        irSensorLeft = self.getIRSensorLeft()
        irSensorRight = self.getIRSensorRight()
        sonar = self.getSonar()
        lightSensors = self.getLightSensors()
                
        print("Left IR = " + str(irSensorLeft))
        print("Right IR = " + str(irSensorRight))
        print("Sonar = " + str(sonar))
        print("Light Sensor Readings" + str(lightSensors))
    
    def getLightSensors(self):
        locations = self.sensorLocations.getLightSensors()
        readings = []
        for i in locations:
            readings += [self.sensorReadings[i]]
            time.sleep(0.05)
        return readings
    
    def getIRSensorLeft(self):
        location = self.sensorLocations.getIRSensorLeft()
        return self.sensorReadings[location]
    
    def getIRSensorRight(self):
        location = self.sensorLocations.getIRSensorRight()
        return self.sensorReadings[location]
    
    def getSonar(self):
        location = self.sensorLocations.getSonar()
        return self.sensorReadings[location]
    
    def getWhiskerLeft(self):
        location = self.sensorLocations.getWhiskerLeft()
        return bool(self.inputReadings[location])
    
    def getWhiskerRight(self):
        location = self.sensorLocations.getWhiskerRight()
        return bool(self.inputReadings[location])
    
    def getOdometer(self):
        location = self.sensorLocations.getOdometer()
        return bool(self.inputReadings[location])
    
    def getOdometerCount(self):
        return self.odometerCount

    def resetOdometerCount(self):
        self.odometerCount = 0
        self.lastUpdateTime = None
    
    def getAverageForwardTimeDifference(self):
        #print("Forward", self.forwardTimeDifferences)
        return np.median(self.forwardTimeDifferences)
    
    def getAverageTurnTimeDifference(self):
        print("Turning", self.turnTimeDifferences)
        return np.median(self.turnTimeDifferences)
    
class MotorBoardManager:
    
    def __init__(self, motor_control):
        self.motorBoardLocations = MotorBoardLocations()
        self.motorControl = motor_control
        self.motorsEngaged = False
        self.motorStartTime = -1
        self.currentCommand = None
        
    def turnLightOn(self):
        lightLocation = self.motorBoardLocations.getLight()
        self.motorControl.setMotor(lightLocation, 70)
        
    def turnLightOff(self):
        lightLocation = self.motorBoardLocations.getLight()
        self.motorControl.setMotor(lightLocation, 0)
        
    def turnLeft(self, speed=100):
        self.currentCommand = 'turn_left'
        self.turn(speed, isLeft=True)
    
    def turnRight(self, speed=100):
        self.currentCommand = 'turn_right'
        self.turn(speed, isLeft=False)
    
    '''Use turnLeft and turnRight instead'''
    def turn(self, speed, isLeft):
        if self.motorsEngaged:
            print("Attempted to move turn but the motor was already engaged")
            return
        self.setEngaged(True)
        leftMotor = self.motorBoardLocations.getLeftMotor()
        rightMotor = self.motorBoardLocations.getRightMotor()
        self.motorControl.setMotor(leftMotor, (1 if not isLeft else -1) * speed)
        self.motorControl.setMotor(rightMotor, (1 if isLeft else -1) * speed)
        
    def moveForward(self, speed=100, backwards=False):
        self.currentCommand = 'backwards' if backwards else 'forward'
        if self.motorsEngaged:
            #print("Attempted to move forward but the motor was already engaged")
            return
        self.setEngaged(True)
        leftMotor = self.motorBoardLocations.getLeftMotor()
        rightMotor = self.motorBoardLocations.getRightMotor()
        self.motorControl.setMotor(leftMotor, (-1 if not backwards else 1) * speed)
        self.motorControl.setMotor(rightMotor, (-1 if not backwards else 1) * speed)
    
    def stopMoving(self):
        duration = self.getDuration()
        self.setEngaged(False)
        leftMotor = self.motorBoardLocations.getLeftMotor()
        rightMotor = self.motorBoardLocations.getRightMotor()
        self.motorControl.stopMotor(leftMotor)
        self.motorControl.stopMotor(rightMotor)
        command = self.currentCommand
        self.currentCommand = 'stopped'
        return duration, command
        
    def stopAll(self):
        self.setEngaged(False)
        self.motorControl.stopMotors()
        
    def setEngaged(self, engaged):
        self.motorsEngaged = engaged
        if engaged:
            self.motorStartTime = time.time()
        else:
            self.motorStartTime = -1
    
    def getDuration(self):
        if not self.motorsEngaged:
            return 0
        elif self.motorStartTime == -1:
            print("ERROR: Motor start time set to -1 while engaged!")
            return 0
        return time.time() - self.motorStartTime

class ServoManager():
    
    def __init__(self, servoControl):
        self.servoControl = servoControl
        self.setPosition(0)
        
    def setPosition(self, angle):
        self.servoControl.setPosition(angle)
        self.currentAngle = angle
        
    def getPosition(self):
        return self.currentAngle
    
    
class ArenaMap:
    
    def __init__(self):
        '''outer_wall = [(0, 1.05),
                      (1.05, 1.05),
                      (1.05, 0),
                      (2.9, 0),
                      (3.2, 0.3),
                      (3.2, 4.25),
                      (0.7, 4.25),
                      (0, 3.65)]
        
        tri_obstacle = [(1, 3.2),
                        (1.6495, 2.825),
                        (1, 2.45)]
        
        rect_obstacle = [(2.29, 3.2),
                         (2.45, 3.2),
                         (2.45, 1.05),
                         (2.29, 1.05)]'''
        outer_wall = [(0, 1.08),       
                      (1.08, 1.08),     
                      (1.08, 0),        
                      (2.89, 0),        
                      (3.22, 0.34),
                      (3.21, 4.27),
                      (0.62, 4.27),
                      (0, 3.57)]
        
        tri_obstacle = [(0.99, 3.18),
                        (1.66, 2.79),
                        (0.99, 2.41)]
        
        rect_obstacle = [(2.24, 3.22),
                         (2.42, 3.22),
                         (2.42, 1.08),
                         (2.24, 1.08)]
        

        self.bounds_rect = [(0, 0), (3.2, 4.25)]
        self.obstacles = [outer_wall, tri_obstacle, rect_obstacle]
        
        self.startPos = np.array([1.4, 0.41])
        self.satellitePos = np.array([0.69, -0.03])
        self.satelliteHeight = 2.95
        
        self.POILocations = []
        
        #self.waypoints = [(1.43, 0.4), (1.43, 1.5), (0.3, 1.5), (0.35, 3.4),
        #                  (0.65, 3.4), (0.65, 1.66), (1.95, 1.66), (1.29, 2.12),
        #                  (2, 2.5), (2, 3), (0.75, 3.95), (2.9, 3.95), (2.9, 3.75),
        #                  (0.75, 3.75), (0.75, 3.6), (2.9, 3.6), (2.65, 3.18),
        #                  (3, 2.5), (2.66, 1.99), (3, 1.5), (2.66, 1.09), (3, 0.5),
        #                  (2.51, 0.73), (2.49, 0.2), (2, 0.77), (2, 0.23)]
        self.waypoints = [(1.43,0.45),(2.23,0.46),(2.89,0.47),(2.96,1.22),(2.93,2.54),(2.89,3.96),(0.67,4),(0.45,3.63),(1.94,3.7),(1.93,2.56),(1.91,0.59),(2.7,0.67),(2.69,2.31),(2.56,3.64),(1.32,3.47),(0.31,3.36),(0.33,2.44),(0.32,1.29),(0.9,1.33),(0.87,2.14),(1.48,2.28),(1.5,1.5),(1.43,0.45)]
        
    def checkCollision(self, start, end, returnPoint=False):
        closest = None
        closest_dist = np.inf 
        for obstacle in self.obstacles:
            for i in range(len(obstacle)):
                wallp1 = obstacle[i]
                wallp2 = obstacle[(i+1)%len(obstacle)]
                collisionLocation = lineSegmentCollision(wallp1, wallp2, start, end)
                if collisionLocation == None:
                    continue
                distance = np.linalg.norm(start - collisionLocation)
                if distance < closest_dist:
                    closest = collisionLocation
                    closest_dist = distance
        
        #print(closest)
        if closest != None:
            if returnPoint:
                return closest
            else:
                return True
        else:
            return False
    
    def checkInsideMap(self, p):
        #inBounds = p[0] < self.bounds_rect[1][0] and p[0] > self.bounds_rect[0][0] and p[1] < self.bounds_rect[1][1] and p[1] > self.bounds_rect[1][0]
        #inCorner = p0
        #self.bounds_rect = [(0, 0), (3.2, 4.25)]
        #self.obstacles = [outer_wall, tri_obstacle, rect_obstacle] 
        x=p[0]         
        y=p[1]
        insideList=[]
        insideArena=False         
        for obstacle in self.obstacles:
            poly=obstacle          
            n = len(poly)
            inside =False   
            p1x,p1y = poly[0]
            for i in range(n+1):
                p2x,p2y = poly[i % n]
                if y > min(p1y,p2y):
                    if y <= max(p1y,p2y):
                        if x <= max(p1x,p2x):
                            if p1y != p2y:
                                xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside                               
                p1x,p1y = p2x,p2y
            insideList.append(inside)
        if(insideList[0] and not insideList[1] and not insideList[2]):
            insideArena= True
        return insideArena
        #return True
    
    def checkRobotCollision(self, pos, orient):
        forward = computeOrientationVector(orient) * robotLength / 2.0
        right = computeOrientationVector((orient + 90) % 360) * robotWidth / 2.0
        
        #print(forward)
        #print(right)
        
        frontLeft = np.array(pos) + forward - right
        frontRight = np.array(pos) + forward + right
        backLeft = np.array(pos) - forward - right
        backRight = np.array(pos) - forward + right
        
        #print(frontLeft, frontRight, backLeft, backRight)
        
        
        frontCollide =  self.checkCollision(frontLeft, frontRight)
        rightCollide = self.checkCollision(frontRight, backRight)
        backCollide = self.checkCollision(backRight, backLeft)
        leftCollide = self.checkCollision(backLeft, frontLeft)
        
        #print([frontCollide, rightCollide, backCollide, leftCollide])
        
        return frontCollide or rightCollide or backCollide or leftCollide
    
    
    def addPOILocation(self, pos):
        isNew = self.newPOILocation(pos)
        if isNew:
            self.POILocations += [pos]
            
    def newPOILocation(self, pos):
        isNew = True
        for poi in self.POILocations:
            if np.linalg.norm(np.array(pos) - np.array(poi)) < 0.4:
                isNew = False
        return isNew
                
        
        
class Particle:
    
    def __init__(self, arenaMap, pos, orient, particleFilter, toddler):
        self.arenaMap = arenaMap
        self.pos = pos
        self.orient = orient
        self.particleFilter = particleFilter
        self.toddler = toddler
        
        self.weight = 0

    def moveForward(self, duration, backwards=False):
        orientVec = computeOrientationVector(self.orient)
        
        #motorForwardNoise = self.particleFilter.motorForwardNoise * 10 if self.particleFilter.explore else self.particleFilter.motorForwardNoise
        motorForwardNoise = self.particleFilter.motorForwardNoise * self.particleFilter.exploreFactor
        #motorDriftNoise = self.particleFilter.motorDriftNoise * 10 if self.particleFilter.explore else self.particleFilter.motorDriftNoise
        motorDriftNoise = self.particleFilter.motorDriftNoise * self.particleFilter.exploreFactor
        
        distanceMoved = self.toddler.motorSpeed * duration
        forwardNoise =  distanceMoved * orientVec * np.random.normal(0, motorForwardNoise)
        positionChangeVector = (-1 if backwards else 1) * distanceMoved * orientVec + forwardNoise
        

        
        driftNoise = distanceMoved * np.random.normal(0, motorDriftNoise)
        newOrient = (self.orient + driftNoise) % 360
        
        factor = 1.0
        
        newPositionChangeVector = positionChangeVector
        
        loops = 4
        # Binary search 4 times
        for i in range(loops):
            newPositionChangeVector = positionChangeVector * factor
            #forwardNoise =  distanceMoved * orientVec * np.random.normal(0, motorForwardNoise)
        
            #print(noise)
            #print("Prev: "+ str(self.pos))
            newX = self.pos[0] + newPositionChangeVector[0]
            newY = self.pos[1] + newPositionChangeVector[1]

            newPos = (newX, newY)
            
            if self.arenaMap.checkRobotCollision(newPos, newOrient):
                factor /= 2.0
                if i == loops - 1:
                    newPos = self.pos
            else:
                break
        
        self.pos = newPos
        self.orient = newOrient
        #print(self.orient)
        #print("After: " + str(self.pos))

    def moveBackwards(self, duration):
        self.moveForward(duration, backwards=True)

    def turn(self, duration, isLeft):
        #print("BEFORE: " + str(self.orient))
        
        #print(self.toddler.angularSpeed)
        turns = self.toddler.getAngularSpeed() * duration / 360.0
        
        exploreFactor = self.particleFilter.exploreFactor#10.0 if self.particleFilter.explore else 1.0
        onPoiFactor = 2.0 if self.toddler.sensors.onPOI else 1.0
        
        motorTurnNoise = self.particleFilter.motorTurnNoise * exploreFactor * onPoiFactor
        #motorTurnPositionNoise = self.particleFilter.motorTurnPositionNoise * 10 if self.particleFilter.explore else self.particleFilter.motorTurnPositionNoise
        motorTurnPositionNoise = self.particleFilter.motorTurnPositionNoise * self.particleFilter.exploreFactor
        
        #self.orient += (-1 if isLeft else 1) * duration / motorFullTurnTime * 360 + turns * np.random.normal(0, motorTurnNoise)
        newOrient = (self.orient + (-1 if isLeft else 1) * turns * 360 + turns * np.random.normal(0, motorTurnNoise)) % 360
        
        newX = self.pos[0] + turns * np.random.normal(0, motorTurnPositionNoise)
        newY = self.pos[1] + turns * np.random.normal(0, motorTurnPositionNoise)
        newPos = (newX, newY)
        
        
        self.orient = newOrient
        self.pos = newPos
        #print("AFTER: " + str(self.orient))

    def turnLeft(self, duration):
        self.turn(duration, isLeft=True)

    def turnRight(self, duration):
        self.turn(duration, isLeft=False)
        
    def senseLeftIR(self):

        leftIRGlobalPosition, leftIROrientVector = computeLeftIRPositionAndOrientationVector(self)
        
        irRange = 5
        
        collisionLocation = self.arenaMap.checkCollision(leftIRGlobalPosition, leftIRGlobalPosition + irRange * leftIROrientVector, returnPoint=True)
        #print("LeftIRGlobalPosition: " + str(leftIRGlobalPosition))
        #assert(collisionLocation, "Particle's Left IR sensor didn't collide")

        distance = np.linalg.norm(collisionLocation - leftIRGlobalPosition)
        
        return distance
        
    def senseRightIR(self):

        rightIRGlobalPosition = self.pos + computeOrientationVector(self.orient, referenceVec=rightIRSensorLocalPosition)
        rightIROrientVector = computeOrientationVector((self.orient + rightIRSensorAngleOffset) % 360)
        
        irRange = 5
        
        collisionLocation = self.arenaMap.checkCollision(rightIRGlobalPosition, rightIRGlobalPosition + irRange * rightIROrientVector, returnPoint=True)
        #print("RightIRGlobalPosition: " + str(rightIRGlobalPosition))
        #assert(collisionLocation, "Particle's Right IR sensor didn't collide")

        distance = np.linalg.norm(collisionLocation - rightIRGlobalPosition)
        
        return distance

    def senseSonar(self):
        
        sonarGlobalPosition, sonarOrientVector, sonarLeftOrientVector, sonarRightOrientVector = computeSonarPositionAndOrientVectors(self)
        
        sonarRange = 10
        
        forwardCollisionLocation = self.arenaMap.checkCollision(sonarGlobalPosition, sonarGlobalPosition + sonarRange * sonarOrientVector, returnPoint=True)
        leftCollisionLocation = self.arenaMap.checkCollision(sonarGlobalPosition, sonarGlobalPosition + sonarRange * sonarLeftOrientVector, returnPoint=True)
        rightCollisionLocation = self.arenaMap.checkCollision(sonarGlobalPosition, sonarGlobalPosition + sonarRange * sonarRightOrientVector, returnPoint=True)
        
        #assert(collisionLocation, "Sonar didn't collide")
        
        forwardDistance = np.linalg.norm(forwardCollisionLocation - sonarGlobalPosition)
        leftDistance =  np.linalg.norm(leftCollisionLocation - sonarGlobalPosition)
        rightDistance = np.linalg.norm(rightCollisionLocation - sonarGlobalPosition)
        
        return min([forwardDistance, leftDistance, rightDistance])

class ParticleFilter:

    def __init__(self, arenaMap, num_particles, POIGrid, toddler):
        
        self.arenaMap = arenaMap 
        self.POIGrid = POIGrid
        self.particles = []
        
        self.motorTurnNoise = 15
        self.motorTurnPositionNoise = 0.08
        self.motorForwardNoise = 0.02
        self.motorDriftNoise = 6
        
        self.exploreFactor = 1.0
        
        while len(self.particles) < num_particles:
            
            #x = np.random.uniform(arenaMap.bounds_rect[0][0], arenaMap.bounds_rect[1][0])
            #y = np.random.uniform(arenaMap.bounds_rect[0][1], arenaMap.bounds_rect[1][1])
            
            x, y, orient = self.generateStartPoint(1.43, 0.40, 0.1, 0.1, 90, 3)
            
            #print("test")
            #print(self.arenaMap.checkRobotCollision((x, y), orient))
            while not (self.arenaMap.checkInsideMap((x ,y)) and (not self.arenaMap.checkRobotCollision((x, y), orient))):
                #print("test")
                x, y, orient = self.generateStartPoint(1.43, 0.40, 0.1, 0.1, 0, 4)
                #x = np.random.uniform(arenaMap.bounds_rect[0][0], arenaMap.bounds_rect[1][0])
                #y = np.random.uniform(arenaMap.bounds_rect[0][1], arenaMap.bounds_rect[1][1])


            if arenaMap.checkInsideMap((x, y)):
                self.particles += [Particle(arenaMap, (x, y), orient, particleFilter=self, toddler=toddler)]
        
        self.bestParticlePos = self.particles[0].pos
        self.bestParticleOrient = self.particles[0].orient
        #self.bestParticle = self.particles[0]
                
    def generateStartPoint(self, xPos, yPos, xRange, yRange, orientation, orientRange):
        x = xPos + np.random.uniform(-xRange/2.0, xRange/2.0)
        y = yPos + np.random.uniform(-yRange/2.0, yRange/2.0)
        
        orient = (orientation + np.random.uniform(-orientRange/2.0, orientRange/2.0)) % 360
        return x, y, orient
                
    def updateParticles(self, command, duration):
        if not (command in ['forward', 'backwards', 'turn_left', 'turn_right']):
            return
        for particle in self.particles:
            if command == 'forward':
                particle.moveForward(duration)
            elif command == 'backwards':
                particle.moveBackwards(duration)
            elif command == 'turn_left':
                #print("turning left")
                particle.turnLeft(duration)
            elif command == 'turn_right':
                particle.turnRight(duration)

    def measurementProb(self, particle, measurements):
        leftIRReading = measurements['leftIR']
        rightIRReading = measurements['rightIR']
        sonarReading = measurements['sonar']

        particleLeftIRDistance = particle.senseLeftIR()
        particleLeftIRReading = convertToLeftIRReading(particleLeftIRDistance)

        leftIRNoiseStd = self.computeIRNoise(particleLeftIRDistance)
        leftProb = gaussian(particleLeftIRReading, leftIRNoiseStd, leftIRReading)


        particleRightIRDistance = particle.senseRightIR()
        particleRightIRReading = convertToRightIRReading(particleRightIRDistance)
        
        rightIRNoiseStd = self.computeIRNoise(particleRightIRDistance)
        rightProb = gaussian(particleRightIRReading, rightIRNoiseStd, rightIRReading)
        
        particleSonarDistance = particle.senseSonar()
        particleSonarReading = convertToSonarReading(particleSonarDistance)
        
        sonarNoiseStd = 50
        sonarProb = gaussian(particleSonarReading, sonarNoiseStd, sonarReading)
        
        #collided = self.arenaMap.checkRobotCollision(particle.pos, particle.orient)
        #validPos = 0 if collided else 1
        #print("Left prob: " + str(particleLeftIRDistance) + ", Right prob: " + str(particleRightIRDistance))
        
        #print("LeftIR = " + str(leftIRReading) + ", Prediction = " + str(particleLeftIRReading) + " - Distance = " + str(particleLeftIRDistance))
        #print("RightIR = " + str(rightIRReading) + ", Prediction = " + str(particleRightIRReading) + " - Distance = " + str(particleRightIRDistance))
        #print("Sonar = " + str(sonarReading) + ", Predi
        
        prob = leftProb * rightProb * sonarProb#* validPos
        
        return prob
    
    def computeIRNoise(self, distance):
        closeIRNoiseStd = 200
        normalIRNoiseStd = 20
        maxIRNoiseStd = 45

        maxAccurateRange = 1.0
        
        if distance < 0.08:
            noiseStd = closeIRNoiseStd
        elif distance < maxAccurateRange:
            noiseStd = normalIRNoiseStd
        else:
            noiseStd = maxIRNoiseStd  
            
        return noiseStd
    
    def resampleParticles(self, measurements):
        
        # Compute Importance weights
        weights = []
        for particle in self.particles:
            likelihood = self.measurementProb(particle, measurements)
            particle.weight = likelihood
        weights = [particle.weight for particle in self.particles]
        total = sum(weights)
        print(total)
        
        if total < 1e-4:
            self.exploreFactor = int(min(-np.log10(total), 15))
            print("Starting to explore...: ", self.exploreFactor)
            #self.explore = True
        else:
            #self.explore = False
            self.exploreFactor = 1.0
        
        for particle in self.particles:
            particle.weight /= total
            
            # Update grid map
            #self.POIGrid.update(particle.pos, particle.weight, measurements['light'])
        
        weights = [particle.weight for particle in self.particles]
        # Resample Particles
        newParticles = []
        beta = 0
        index = 0
        maxWeight = max(weights)
        #bestParticle = self.particles[np.argmax(weights)]
        #print("Best particle: " + str(bestParticle.pos[0]) + ", " + str(bestParticle.pos[1]) + ", " + str(bestParticle.orient))
        #print("Best particle readings: " + str(convertToIRReading(bestParticle.senseLeftIR())) + ", " + str(convertToIRReading(bestParticle.senseRightIR())))
        
        for i in range(len(self.particles)):
            beta = beta + 2.0 * maxWeight * np.random.rand()
            while beta > weights[index]:
                beta = beta - weights[index]
                index = (index + 1) % len(self.particles)
            selectedParticle = copy(self.particles[index])
            newParticles.append(selectedParticle)
        
        self.particles = newParticles
        
        totalPos = np.array([0.0, 0.0])
        totalOrientVec = np.array([0.0, 0.0])
        
        for particle in self.particles:
            totalPos[0] += particle.pos[0] * particle.weight
            totalPos[1] += particle.pos[1] * particle.weight
            #print("Orient", particle.orient)
            totalOrientVec += computeOrientationVector(particle.orient) * particle.weight
        
        
        total = np.sum([p.weight for p in self.particles])
        
        averagePos = (totalPos[0] / total, totalPos[1] / total)
        averageOrientVec = totalOrientVec / np.linalg.norm(totalOrientVec)
        averageOrient = (np.arctan2(averageOrientVec[0], averageOrientVec[1]) / (2.0 * np.pi) * 360) % 360
        
        #print("Average Pos", averagePos)
        #print("Average Orient", averageOrient)
        #print("Average Orient Vec", averageOrientVec)
        
        
        self.bestParticlePos = averagePos
        self.bestParticleOrient = averageOrient
        #self.bestParticle = self.particles[np.argmax([particle.weight for particle in self.particles])]
        
    def getBestEstimate(self):
        return self.bestParticlePos, self.bestParticleOrient
        #return self.bestParticle.pos, self.bestParticle.orient


class POIGrid:
    
    def __init__(self, gridWidth, gridHeight, arenaMap):
        self.gridHeight = gridHeight
        self.gridWidth = gridWidth
        
        self.cellWidth = 3.22 / self.gridWidth
        self.cellHeight = 4.27 / self.gridHeight
        
        self.arenaMap = arenaMap
        
        self.grid = np.zeros((self.gridWidth, self.gridHeight))
        self.initGrid()
        
    def initGrid(self):
        for gridX in range(self.gridWidth):
            for gridY in range(self.gridHeight):
                centerPos = self.getCenterPos(gridX, gridY)
                if self.arenaMap.checkInsideMap(centerPos) and not self.arenaMap.checkRobotCollision(centerPos, 0):
                    self.grid[gridX][gridY] = 1.0
                else:
                    self.grid[gridX][gridY] = 0.0
        
    def getCenterPos(self, gridX, gridY):
        x = self.cellWidth * (gridX + 0.5)
        y = self.cellHeight * (gridY + 0.5)
        return x, y
                
    def update(self, pos, likelihood, POIdetected):
        gridX = int(pos[0] / self.cellWidth)
        gridY = int(pos[1] / self.cellHeight)
        if POIdetected:
            self.grid[gridX][gridY] += likelihood
        else:
            self.grid[gridX][gridY] -= likelihood

    def getCellWidth(self):
        return self.cellWidth
        
    def getCellHeight(self):
        return self.cellHeight
        
    def getLikelihood(self, gridX, gridY):
        return self.grid[gridX, gridY]
        

class Toddler:
    __version = '2018a'
    
    
    def __init__(self, IO):
        print('[Toddler] I am toddler {} playing in a sandbox'.format(Toddler.__version))

        self.camera = IO.camera.initCamera('pi', 'low')
        self.getInputs = IO.interface_kit.getInputs
        self.getSensors = IO.interface_kit.getSensors
        self.mc = IO.motor_control
        self.sc = IO.servo_control
        
        self.sc.engage()
        
        self.sensors = SensorManager(self.getSensors, self.getInputs)
        self.motorBoard = MotorBoardManager(self.mc)
        self.servo = ServoManager(self.sc)
        
        self.motorBoard.turnLightOn()
        
        self.servoHeight = 0.2

        self.facingSatellite = False
        
        '''if len(sys.argv) > 2:
            currentX = sys.argv[1]
            currentY = sys.argv[2]
            self.currentPos = np.array([float(currentX), float(currentY)])
            self.currentOrient = float(sys.argv[3])
        else:
            self.currentPos = self.POIpos1
            self.currentOrient = 0
        '''
        self.arenaMap = ArenaMap()
        self.POIGrid = POIGrid(25, 33, self.arenaMap)
        self.particleFilter = ParticleFilter(self.arenaMap, 500, self.POIGrid, toddler=self)
        self.mapRenderer = MapRenderer(self.arenaMap, self.particleFilter, self.POIGrid)
        
        self.counter = 0
        
        self.motorSpeed = 0
        self.angularSpeed = 0
        
        self.pathIndex = 0

        self.mapRenderer.drawMap()
        self.mapRenderer.drawParticles(withWeights=False)
        self.mapRenderer.showMap()
        
        self.onStart = True
        
        
    
    def computeHeadingAndServoRotation(self, currentPos, currentOrient):
        targetDirection = currentPos - self.arenaMap.satellitePos
        
        theta_radians = np.arctan2(targetDirection[0], targetDirection[1])
        rotationAngle = (theta_radians) / (2.0 * np.pi) * 360
        
        epsilon = (currentOrient - rotationAngle) % 360
        
        if epsilon < 90:
            turnDirection = "left"
            turnAngle = epsilon
            servoInverted = True
        elif epsilon < 180:
            turnDirection = "right"
            turnAngle = 180 - epsilon
            servoInverted = False
        elif epsilon < 270:
            turnDirection = "left"
            turnAngle = epsilon - 180 
            servoInverted = False
        else:
            turnDirection = "right"
            turnAngle = 360 - epsilon
            servoInverted = True
        
        horizontalDistance = np.linalg.norm(currentPos - self.arenaMap.satellitePos)
        verticalDistance = self.arenaMap.satelliteHeight - self.servoHeight
        
        servoRotationRadians = np.arctan(verticalDistance / horizontalDistance)
        print(servoRotationRadians)
        if servoInverted:
            servoRotationAngle = servoRotationRadians / np.pi * 180;
        else:
            servoRotationAngle = 180 - servoRotationRadians / np.pi * 180;
        
        return turnAngle, turnDirection, servoRotationAngle
        
        
    
    def control(self):
        
        
        self.pathWithCollisionAvoidanceControl()
        #self.followPathControl()
        #self.collisionAvoidanceControl()
        #self.particleFilterControl()

        #self.forwardUntilPOIControl()
        #self.servoPointControl()
        
        #self.moveForwardControl()
        #self.fullTurnControl()
        #self.collisionAvoidanceControl()
        #self.printSensorControl()
        #self.printOdometerAndMoveControl()
        
        time.sleep(0.05)
        
    def printOdometerAndMoveControl(self):
        self.motorBoard.turnLeft()
        odometerCount = self.sensors.getOdometerCount()
        #while odometerCount < 58:
        while odometerCount < 12:
            self.sensors.updateOdometer()
            odometerCount = self.sensors.getOdometerCount()
            print(odometerCount)
        #time.sleep(0.05)
        self.motorBoard.stopMoving()
        time.sleep(5)
        
        
    def printSensorControl(self):
        leftIR = self.sensors.getIRSensorLeft()
        rightIR = self.sensors.getIRSensorRight()
        sonar = self.sensors.getSonar()
        
        particle = self.particleFilter.particles[0]
        
        particleLeftIRDistance = particle.senseLeftIR()
        particleLeftIRReading = convertToLeftIRReading(particleLeftIRDistance)
        
        particleRightIRDistance = particle.senseRightIR()
        particleRightIRReading = convertToRightIRReading(particleRightIRDistance)
        
        particleSonarDistance = particle.senseSonar()
        particleSonarReading = convertToSonarReading(particleSonarDistance)
        
        print('Left Distance = ', particleLeftIRDistance, 'Predicted IR = ', particleLeftIRReading, 'Actual Reading = ', leftIR)
        print('Right Distance = ', particleRightIRDistance, ', Predicted IR = ', particleRightIRReading, 'Actual Reading = ', rightIR)
        print('Sonar Distance = ', particleSonarDistance, 'Predicted = ', particleSonarReading, 'Actual Reading = ', sonar)
    
    def printSonarControl(self):
        readings = []
        for i in range(1):
            reading = self.sensors.getSonar()
            readings += [reading]
            time.sleep(0.05)
        print(np.mean(readings))
    
    
    def moveForwardControl(self):
        duration = 6
        self.motorBoard.moveForward()
        time.sleep(duration)
        self.motorBoard.stopMoving()
        
        averageTimeDifference = self.sensors.getAverageForwardTimeDifference()
        
        speedEstimate = odometerSpeed / averageTimeDifference
        #angularSpeedEstimate = odometerAngularSpeed / averageTimeDifference
        
        distanceEstimate = speedEstimate * duration
        #angleEstimate = angularSpeedEstimate * duration
        
        print(self.sensors.getOdometerCount())
        print("Average Time Diff", averageTimeDifference)
        print("Speed", speedEstimate)
        print("Distance", distanceEstimate)
        #print("Angular Speed", angularSpeedEstimate)
        #print("Angle", angleEstimate)
        
        time.sleep(10)
    
    def fullTurnControl(self):
        #while self.sensors.getOdometerCount() <= 36:
        #    time.sleep(0.05)
        #duration = time.time() - startTime
        #self.sensors.resetOdometerCount()
        #self.sleepUpdate(motorFullTurnTime)
        #self.motorBoard.stopMoving()
        #time.sleep(3)
        self.motorBoard.turnRight()
        self.sleepUpdate(8.25)
        self.motorBoard.stopMoving()
        
    def getAngularSpeed(self):
        if self.sensors.onPOI:
            return self.angularSpeed * 0.75
        elif self.onStart:
            return self.angularSpeed * 0.5
        else:
            return self.angularSpeed
    
    def executeCommand(self, command, duration):
        if command == 'stopped':
            return
        leftIRReadings = []
        rightIRReadings = []
        
        for i in range(5):
            time.sleep(0.07)
            leftIRReadings += [self.sensors.getIRSensorLeft()]
            time.sleep(0.07) 
            rightIRReadings += [self.sensors.getIRSensorRight()]
           
        time.sleep(0.07)
        sonarReading = self.sensors.getSonar()
        time.sleep(0.07)
        lightReading = self.sensors.getLightSensors()[0]
        time.sleep(0.07)
        
        measurements = {'leftIR': np.mean(leftIRReadings), 'rightIR': np.mean(rightIRReadings), 'sonar': sonarReading, 'light': lightReading > 62}    
        time.sleep(0.05)
        self.particleFilter.updateParticles(command, duration)
        time.sleep(0.05)
        self.particleFilter.resampleParticles(measurements)
        time.sleep(0.05)
        self.sensors.resetOdometerCount()
        time.sleep(0.05)
        self.mapRenderer.drawMap()
        self.mapRenderer.drawParticles()
        self.mapRenderer.showMap()
        
    def particleFilterControl(self):        
        #self.motorBoard.turnLeft()
        #time.sleep(motorFullTurnTime / 10)
        self.motorBoard.turnLeft()
        self.sleepUpdate(5)
        duration = self.motorBoard.getDuration()
        duration, command = self.motorBoard.stopMoving()
        self.executeCommand(command, duration)
        #print("IR Readings: " + str(measurements['leftIR']) + ", " + str(measurements['rightIR']))

        
        self.mapRenderer.drawMap()
        self.mapRenderer.drawParticles()
        self.mapRenderer.showMap()
        
        time.sleep(1)
        
    def forwardUntilPOIControl(self):
        lightSensors = self.sensors.getLightSensors()
        light = lightSensors[0]
        
        lightUpperThreshold = 550
        lightLowerThreshold = 390
        
        print(light)
        
        if light > lightUpperThreshold:# or light < lightLowerThreshold:
            print("POI Detected")
            time.sleep(0.5)
            self.motorBoard.stopMoving()
            time.sleep(5)
        if light < lightLowerThreshold:
            print("Start position detected")
            self.motorBoard.stopMoving()
            time.sleep(5)
        else:
            self.motorBoard.moveForward()

        time.sleep(0.05)

    def servoPointControl(self, currentPos, currentOrient):        
        rotationAmountAngle, turnDirection, servoRotation = self.computeHeadingAndServoRotation(currentPos, currentOrient)
        
        if turnDirection == "left":
            self.motorBoard.turnLeft()
            print("turning left")
        else:
            self.motorBoard.turnRight()
            print("turning right")
        
        self.servo.setPosition(servoRotation)
        
        angularSpeed = self.getAngularSpeed()
        
        rotationTime = rotationAmountAngle / angularSpeed
        while(self.motorBoard.getDuration() < rotationTime):
            time.sleep(0.01)
        self.motorBoard.stopMoving()
        time.sleep(5)

    def sleepUpdate(self, duration):
        start = time.time()
        while time.time() - start < duration:
            time.sleep(0.05)
            #self.sensors.update()
        time.sleep(0.05)
    
    def avoidLeft(self, rnd):
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)
        self.sleepUpdate(0.02)
        self.motorBoard.turnLeft()
        self.sleepUpdate(rnd)
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)

    def avoidRight(self, rnd):
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)
        self.sleepUpdate(0.02)
        self.motorBoard.turnRight()
        self.sleepUpdate(rnd)
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)

    def avoidBack(self, rnd):
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)
        self.sleepUpdate(0.02)
        self.motorBoard.moveForward(backwards=True)
        self.sleepUpdate(0.8)
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)
        #self.sleepUpdate(0.25)
        #self.motorBoard.turnLeft()
        #self.sleepUpdate(rnd)
        #duration, command = self.motorBoard.stopMoving()
        #self.sensors.resetOdometerCount()
        #self.executeCommand(command, duration)

    def updateSpeedEstimates(self):
        
        averageForwardTimeDifference = self.sensors.getAverageForwardTimeDifference()
        averageTurnTimeDifference = self.sensors.getAverageTurnTimeDifference()
        
        self.motorSpeed = odometerSpeed / averageForwardTimeDifference
        self.angularSpeed = odometerAngularSpeed / averageTurnTimeDifference
        
        #print("Motor Speed", self.motorSpeed)
        #print("Angular Speed", self.angularSpeed)
        #print("Speed ratio", self.motorSpeed / self.angularSpeed)
        


    def followPathControl(self):
        
        self.updateSpeedEstimates()
        
        currentPos, currentOrient = self.particleFilter.getBestEstimate()
        #print("Current Pos", currentPos)
        #print("Current Orient", currentOrient)
        desiredPos = self.arenaMap.waypoints[self.pathIndex]
        #print("Desired pos", desiredPos)
        moveVector = np.array(desiredPos) - np.array(currentPos)
        
        if np.linalg.norm(moveVector) < 0.2:
            print("Waypoint Found!!!!")
            self.pathIndex += 1
            print(self.pathIndex)
            #desiredPos = self.arenaMap.waypoints[self.pathIndex]
            #moveVector = np.array(currentPos) - np.array(desiredPos)
            return
        
        #print(moveVector)
        thetaRadians = np.arctan2(moveVector[0], moveVector[1])
        desiredOrient = ((thetaRadians) / (2.0 * np.pi) * 360) % 360
        #print("Current angle", currentOrient, "Desired", desiredOrient)
        
        rotationAngle = (desiredOrient - currentOrient) % 360
        #print("Rotation Angle", rotationAngle)
        
        if rotationAngle < 8.0 or rotationAngle > 352.0:
            moveDistance = min(np.linalg.norm(moveVector), 1.0)
            moveTime = moveDistance / self.motorSpeed
            self.motorBoard.moveForward()
            time.sleep(moveTime)
            duration, command = self.motorBoard.stopMoving()
            self.executeCommand(command, duration)
        else:
            if rotationAngle < 180:
                rotationTime = rotationAngle / self.getAngularSpeed()
                self.motorBoard.turnRight()
            else:
                rotationAngle = 360 - rotationAngle
                rotationTime = rotationAngle / self.getAngularSpeed()
                self.motorBoard.turnLeft()
            time.sleep(rotationTime)
            duration, command = self.motorBoard.stopMoving()
            self.executeCommand(command, duration)
        

    def collisionAvoidanceControl(self, stopOnPOI=True):
        
        self.updateSpeedEstimates()
        
        self.POIGrid.grid[5][5] = 0.5
        
        irThreshold = 400
        
        time.sleep(0.05)
        leftIR = self.sensors.getIRSensorLeft()
        time.sleep(0.05)
        rightIR = self.sensors.getIRSensorRight() 
        time.sleep(0.05)
        whiskerLeft = self.sensors.getWhiskerLeft()
        time.sleep(0.05)
        whiskerRight = self.sensors.getWhiskerRight()
        time.sleep(0.05)
        sonar = self.sensors.getSonar()
        time.sleep(0.05)
        #self.sensors.update()
        
        lightUpperThreshold = 56.5
        lightLowerThreshold = 19
        
        lightSensors = self.sensors.getLightSensors()
        time.sleep(0.05)
        
        light = lightSensors[0]
        rnd = 1.5 + 0.5 * np.random.rand()
        
        self.sensors.onPOI = False
        if light > lightUpperThreshold:# or light < lightLowerThreshold:
            print("POI Detected")
            #self.motorBoard.stopMoving()
            self.sensors.onPOI = True
        #    time.sleep(5)
        
        
        if whiskerLeft and not whiskerRight:
            self.avoidRight(rnd)
        elif whiskerRight and not whiskerLeft:
            self.avoidLeft(rnd)
        elif whiskerLeft and whiskerRight:
            self.avoidBack(rnd)
        elif leftIR > irThreshold and rightIR < irThreshold:
            self.avoidRight(rnd)
        elif rightIR > irThreshold and leftIR < irThreshold:
            self.avoidLeft(rnd)
        elif leftIR > irThreshold and rightIR > irThreshold:
            self.avoidBack(rnd)
        elif sonar < 23:
            self.avoidBack(rnd)
        elif self.motorBoard.getDuration() > 2:
            duration, command = self.motorBoard.stopMoving()
            self.sensors.resetOdometerCount()
            self.executeCommand(command, duration)
            #self.motorBoard.turnLeft()
            #self.sleepUpdate(1.5)
            #duration = self.motorBoard.stopMoving()
            #self.executeCommand('turn_left', duration)
            #self.motorBoard.turnRight()
            #self.sleepUpdate(1.5)
            #duration = self.motorBoard.stopMoving()
            #self.executeCommand('turn_right', duration)
        else:
            self.motorBoard.moveForward()
            
        time.sleep(0.05)
    
    def pathWithCollisionAvoidanceControl(self):
        self.updateSpeedEstimates()
        
        if not self.motorBoard.motorsEngaged:
            currentPos, currentOrient = self.particleFilter.getBestEstimate()
            if self.onStart:
                #print(abs(currentPos[0] - self.arenaMap.startPos[0]), abs(currentPos[1] - self.arenaMap.startPos[1]))
                if abs(currentPos[0] - self.arenaMap.startPos[0]) > 0.16 or abs(currentPos[1] - self.arenaMap.startPos[1]) > 0.23:
                    self.onStart = False
                    
            desiredPos = self.arenaMap.waypoints[self.pathIndex]
            moveVector = np.array(desiredPos) - np.array(currentPos)
            
            if np.linalg.norm(moveVector) < 0.2:
                print("Waypoint Found!!!!")
                self.pathIndex += 1
                print(self.pathIndex)
                return
        
        
        irThreshold = 400
        lightUpperThreshold = 530
        lightLowerThreshold = 390
        sonarThreshold = 20
        
        time.sleep(0.05)
        leftIR = self.sensors.getIRSensorLeft()
        time.sleep(0.05)
        rightIR = self.sensors.getIRSensorRight() 
        time.sleep(0.05)
        whiskerLeft = self.sensors.getWhiskerLeft()
        time.sleep(0.05)
        whiskerRight = self.sensors.getWhiskerRight()
        time.sleep(0.05)
        sonar = self.sensors.getSonar()
        time.sleep(0.05)
        lightSensor = self.sensors.getLightSensors()[0]
        time.sleep(0.05)
        
        print(lightSensor)
        
        if lightSensor > lightUpperThreshold:
            if self.sensors.onPOI == False:
                self.sensors.onPOI = True
                duration, command = self.motorBoard.stopMoving()
                self.executeCommand(command, duration)
                currentPos, currentOrient = self.particleFilter.getBestEstimate()            
                if self.arenaMap.newPOILocation(currentPos):
                    self.arenaMap.addPOILocation(currentPos)

                    self.servoPointControl(currentPos, currentOrient)
                    time.sleep(30)
        else:
            self.sensors.onPOI = False
            #self.arenaMap.POILocations += []
        
        rnd = 0.5 + 0.75 * np.random.rand()
        if whiskerLeft and not whiskerRight:
            self.avoidRight(rnd)
        elif whiskerRight and not whiskerLeft:
            self.avoidLeft(rnd)
        elif whiskerLeft and whiskerRight:
            self.avoidBack(rnd)
        elif leftIR > irThreshold and rightIR < irThreshold:
            self.avoidRight(rnd)
        elif rightIR > irThreshold and leftIR < irThreshold:
            self.avoidLeft(rnd)
        elif leftIR > irThreshold and rightIR > irThreshold:
            self.avoidBack(rnd)
        elif sonar < sonarThreshold:
            self.avoidBack(rnd + 0.1)
        elif self.motorBoard.motorsEngaged and self.motorBoard.getDuration() > self.pathDuration:
            duration, command = self.motorBoard.stopMoving()
            self.sensors.resetOdometerCount()
            self.executeCommand(command, duration)
        elif not self.motorBoard.motorsEngaged:            
            thetaRadians = np.arctan2(moveVector[0], moveVector[1])
            desiredOrient = ((thetaRadians) / (2.0 * np.pi) * 360) % 360
            
            rotationAngle = (desiredOrient - currentOrient) % 360
            
            if rotationAngle < 12.0 or rotationAngle > 348.0:
                moveDistance = min(np.linalg.norm(moveVector), 0.6)
                moveTime = moveDistance / self.motorSpeed
                self.motorBoard.moveForward()
                self.pathDuration = moveTime
            else:
                if rotationAngle < 180:
                    rotationTime = rotationAngle / self.getAngularSpeed()
                    self.motorBoard.turnRight()
                else:
                    rotationAngle = 360 - rotationAngle
                    rotationTime = rotationAngle / self.getAngularSpeed()
                    self.motorBoard.turnLeft()
                self.pathDuration = rotationTime
    
    def vision(self):
        
        if self.motorBoard.motorsEngaged:
            onPOI = self.sensors.onPOI
            onStart = self.onStart
            self.sensors.updateOdometer(self.motorBoard.currentCommand, onPOI, onStart)
        time.sleep(0.005)
        #image = self.camera.getFrame()
        #self.camera.imshow('Camera', image)



def computeOrientationVector(orient, referenceVec=np.array([0,1])):
    assert(orient >= 0.0 and orient <= 360.0)
    
    radOrient = np.deg2rad(orient)
    rotationMat = np.array([[np.cos(radOrient), np.sin(radOrient)],[-np.sin(radOrient), np.cos(radOrient)]])
    orientVec = np.dot(rotationMat, referenceVec)
    return orientVec

def lineSegmentCollision(p1, p2, p3, p4):
    t_a_num = (p3[1] - p4[1])*(p1[0] - p3[0]) + (p4[0] - p3[0])*(p1[1] - p3[1])
    t_b_num = (p1[1] - p2[1])*(p1[0] - p3[0]) + (p2[0] - p1[0])*(p1[1] - p3[1])
    
    t_den = (p4[0] - p3[0])*(p1[1] - p2[1]) - (p1[0] - p2[0])*(p4[1] - p3[1])
    
    if t_den == 0:
        return None
    
    t_a = t_a_num / t_den
    t_b = t_b_num / t_den
    
    if t_a < 0 or t_a > 1 or t_b < 0 or t_b > 1:
        return None
    
    intersection_point = (p1[0] + t_a * (p2[0] - p1[0]), p1[1] + t_a * (p2[1] - p1[1]))
    return intersection_point

def convertToLeftIRReading(distance):
    linearSlope = 514 / 0.08
    if distance < 0.08:
        return linearSlope*distance
    else:
        leftCoefficients = [1.36004736e+01,  6.61258519e+01, -7.25741092e+00,  1.16883544e+00, -6.04365223e-02]
        distanceVec = np.array([np.power(distance, -i) for i in range(5)])
        return np.dot(leftCoefficients, distanceVec)
    
def convertToRightIRReading(distance):
    linearSlope = 544 / 0.08
    if distance < 0.08:
        return linearSlope*distance
    else:
        rightCoefficients = [2.17272201e+01,  4.95704114e+01, -6.12409129e-01, 2.08434128e-01, -1.67611785e-02]
        distanceVec = np.array([np.power(distance, -i) for i in range(5)])
        return np.dot(rightCoefficients, distanceVec)
    
    
def convertToSonarReading(distance):
    if distance < 0.35:
        return 20
    else:
        coefficients = [-5.5181781, 73.862337]
        distanceVec = np.array([1, distance])
        return np.dot(coefficients, distanceVec)
    
        
    
def computeLeftIRPositionAndOrientationVector(particle):
    
    leftIRGlobalPosition = particle.pos + computeOrientationVector(particle.orient, referenceVec=leftIRSensorLocalPosition)
    leftIROrientVector = computeOrientationVector((particle.orient + leftIRSensorAngleOffset) % 360)
    
    return leftIRGlobalPosition, leftIROrientVector

def computeRightIRPositionAndOrientationVector(particle):
    
    rightIRGlobalPosition = particle.pos + computeOrientationVector(particle.orient, referenceVec=rightIRSensorLocalPosition)
    rightIROrientVector = computeOrientationVector((particle.orient + rightIRSensorAngleOffset) % 360)
    
    return rightIRGlobalPosition, rightIROrientVector

def computeSonarPositionAndOrientVectors(particle):
    
    sonarGlobalPosition = particle.pos + computeOrientationVector(particle.orient, referenceVec=sonarSensorLocalPosition)
    sonarOrientVector = computeOrientationVector(particle.orient)
    sonarLeftOrientVector = computeOrientationVector((particle.orient - sonarConeAngle) % 360)
    sonarRightOrientVector = computeOrientationVector((particle.orient + sonarConeAngle) % 360)
    
    return sonarGlobalPosition, sonarOrientVector, sonarLeftOrientVector, sonarRightOrientVector

def gaussian(mu, sigma, x):        
    # Given a x position, mean mu and std sigma of Gaussian, calculates the probability
    # Note
    # mu: estimated distance by each particle's position, (map and landmarks are known)
    # x:  measured distance by the robot
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

class MapRenderer:

    def __init__(self, arenaMap, particleFilter, POIGrid):
        self.arenaMap = ArenaMap()
        self.particleFilter = particleFilter
        self.POIGrid = POIGrid

        self.scale = 200
        self.padding = 0.2

        width = 3.2
        height = 4.25

        self.imgWidth = int(self.scale * (width + self.padding * 2))
        self.imgHeight = int(self.scale * (height + self.padding * 2))
        self.img = np.zeros((self.imgHeight, self.imgWidth, 3), np.uint8)
        self.img[:] = (255,255,255)


    def convertToImgCoords(self, x, y):
        imgX = int(self.scale * (x + self.padding))
        imgY = self.imgHeight - int(self.scale * (y + self.padding))
        
        return imgX, imgY

    def drawMap(self):
        self.img[:] = (255,255,255)
        #self.drawPOIGrid()
        for obstacle in self.arenaMap.obstacles:
            for i in range(len(obstacle)):
                p1 = obstacle[i]
                p2 = obstacle[(i+1)%len(obstacle)]
                #p1x = int(self.scale * (p1[0] + self.padding))
                #p1y = self.imgHeight - int(self.scale * (p1[1] + self.padding))
                p1x, p1y = self.convertToImgCoords(p1[0], p1[1])
                #p2x = int(self.scale * (p2[0] + self.padding))
                #p2y = self.imgHeight - int(self.scale * (p2[1] + self.padding))
                p2x, p2y = self.convertToImgCoords(p2[0], p2[1])
                
                cv2.line(self.img, (p1x,p1y), (p2x, p2y), (0,0,0), 2)
                #plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='k', linestyle='-', linewidth=2)
        
        self.drawPath()
        
    def drawPath(self):
        for i in range(len(self.arenaMap.waypoints) - 1):
            currentWaypoint = self.arenaMap.waypoints[i]
            nextWaypoint = self.arenaMap.waypoints[(i+1) % len(self.arenaMap.waypoints)]
            
            currImgX, currImgY = self.convertToImgCoords(currentWaypoint[0], currentWaypoint[1])
            nextImgX, nextImgY = self.convertToImgCoords(nextWaypoint[0], nextWaypoint[1])
            
            cv2.circle(self.img, (currImgX, currImgY), 7, (255, 0, ), thickness=-1)          
            cv2.line(self.img, (currImgX, currImgY), (nextImgX, nextImgY), (255, 0, 0), 2)

    def drawPOIGrid(self):
        cellWidth = self.POIGrid.getCellWidth()
        cellHeight = self.POIGrid.getCellHeight()
        for gridX in range(self.POIGrid.gridWidth):
            for gridY in range(self.POIGrid.gridHeight):
                likelihood = self.POIGrid.getLikelihood(gridX, gridY)
                
                x = gridX * cellWidth
                y = gridY * cellHeight
                
                imgXLower, imgYLower = self.convertToImgCoords(x, y)
                imgXUpper, imgYUpper = self.convertToImgCoords(x+cellWidth, y+cellHeight)
                
                cv2.rectangle(self.img, (imgXLower, imgYLower), (imgXUpper, imgYUpper), (0,100*(1-likelihood),255), -1)

                
                
                
                

    def drawParticles(self, withWeights=True):
        
        if withWeights:
            weights = [p.weight for p in self.particleFilter.particles]
            maxWeight = max(weights)
            
            
        '''particleLocations = [(p.pos[0], p.pos[1], p.orient) for p in self.particleFilter.particles]
        k = 3
        
        data = np.array(particleLocations)
        print(data.shape)
            
        
        n = data.shape[0]
        c = data.shape[1]
            
        mean = np.mean(data, axis = 0)
        std = np.std(data, axis = 0)
        centroids = np.random.randn(k,c)*std + mean
        
        centroidsOld = np.zeros(centroids.shape)
        centroidsNew = deepcopy(centroids)
        
        def computeDifferenceWithCentroid(data, centroid):
            #print(data.shape)
            #print(centroid.shape)
            posDiff = (data[:, :2] - centroid[:2]) / np.array([3.22, 4.27])
            orientDiff = np.min([data[:, 2] - centroid[2], data[:, 2] - centroid[2] + 360, data[:, 2] - centroid[2] - 360], axis=0) / 360.0
            return np.concatenate([posDiff, orientDiff.reshape(-1, 1)], axis=1)
        
        clusters = np.zeros(n)
        distances = np.zeros((n,k))
        
        error = np.linalg.norm(centroidsNew - centroidsOld)
        
        maxIterations = 50
        i = 0
        while error != 0:
            i =+ 1
            if i > maxIterations:
                break
            # Measure the distance to every centroids
            for i in range(k):
                distances[:,i] = np.linalg.norm(computeDifferenceWithCentroid(data, centroids[i]), axis=1)
                
                # Assign all training data to closest centroids
                clusters = np.argmin(distances, axis = 1)
    
                centroidsOld = deepcopy(centroidsNew)
                # Calculate mean for every cluster and update the centroids
            print(distances)
            for i in range(k):
                if data[clusters == i].shape[0] == 0:
                    centroidsNew[i] = centroidsOld[i]
                else:    
                    centroidsNew[i] = np.mean(data[clusters == i], axis=0)
                error = np.linalg.norm(centroidsNew - centroidsOld)
            
        #clusterWeightedPosition = np.zeros(n, k)
        clusterWeightTotals = np.zeros(k)

        print(clusters)    
            
        for i in range(n):
            assignment = clusters[i]
            weight = self.particleFilter.particles[i].weight
            clusterWeightTotals[assignment] += weight
            
        bestCluster = np.argmax(clusterWeightTotals)
        bestClusterPosition = centroidsNew[bestCluster]
        '''
            
        #print(weights)
        #for particle, assignment in zip(self.particleFilter.particles, clusters):
        for particle in self.particleFilter.particles:
            
            particleImgX = int(self.scale * (particle.pos[0] + self.padding))
            particleImgY = self.imgHeight - int(self.scale * (particle.pos[1] + self.padding))


            colour = (255,0, 255*particle.weight/maxWeight) if withWeights else (255, 0, 125)
            #colour = (255*(1-int(assignment)/float(k)),255*int(assignment)/float(k), 255*int(assignment)/float(k))
            #colour = (255, 255, 0)
            
            cv2.circle(self.img, (particleImgX, particleImgY), 4, colour, thickness=-1)
            orientVec = computeOrientationVector(particle.orient) * 15
            orientVec[1] = -orientVec[1]

            cv2.line(self.img, (particleImgX, particleImgY), (particleImgX + int(orientVec[0]), particleImgY + int(orientVec[1])), (14,180,40), 2)
            
            
            '''leftIRPosition, leftIROrientationVector = computeLeftIRPositionAndOrientationVector(particle)
            leftIRImgX = int(self.scale * (leftIRPosition[0] + self.padding))
            leftIRImgY = self.imgHeight - int(self.scale * (leftIRPosition[1] + self.padding))
            cv2.circle(self.img, (leftIRImgX, leftIRImgY), 4, (0, 255, 0), thickness=-1)
            
            rightIRPosition, rightIROrientationVector = computeRightIRPositionAndOrientationVector(particle)
            rightIRImgX = int(self.scale * (rightIRPosition[0] + self.padding))
            rightIRImgY = self.imgHeight - int(self.scale * (rightIRPosition[1] + self.padding))
            cv2.circle(self.img, (rightIRImgX, rightIRImgY), 4, (0, 255, 0), thickness=-1)'''
        
        if withWeights:
            bestParticlePos, bestParticleOrient = self.particleFilter.getBestEstimate()#particles[np.argmax(weights)]
            
            bestParticleImgX = int(self.scale * (bestParticlePos[0] + self.padding))
            bestParticleImgY = self.imgHeight - int(self.scale * (bestParticlePos[1] + self.padding))
            
            
            cv2.circle(self.img, (bestParticleImgX, bestParticleImgY), 4, (0,0,0), thickness=-1)
            orientVec = computeOrientationVector(bestParticleOrient) * 30
            orientVec[1] = -orientVec[1]
    
            cv2.line(self.img, (bestParticleImgX, bestParticleImgY), (bestParticleImgX + int(orientVec[0]), bestParticleImgY + int(orientVec[1])), (0,0,0), 2)            
            

    def showMap(self):
        cv2.imshow("test", self.img)
        cv2.waitKey(0)