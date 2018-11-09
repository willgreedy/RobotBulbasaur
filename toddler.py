    #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 11 11:25:29 2018

@author: s1873447
"""

#!/usr/bin/env python

import time
import numpy as np
from copy import copy
import cv2

angularSpeed = 0
motorSpeed = 0

initalTurnTimeDiff = 0.441819
initialForwardTimeDiff = 0.22779955863952636

# UPDATE THESE BASED ON EXPERIMENTS
#motorFullTurnTime = 7.0
#motorFullTurnTime = 5.4
odometerAngularSpeed = 360 / 15.0
#motorSpeed = 0.19
#motorSpeed = 0.25
odometerSpeed = 3.0 / 59.2

#motorTurnNoise = 70
motorTurnNoise = 10
motorTurnPositionNoise = 0.05
motorForwardNoise = 0.01
motorDriftNoise = 6

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
        
    
    def updateOdometer(self, command):   
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
        return [self.sensorReadings[i] for i in locations]
    
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
        print("Forward", self.forwardTimeDifferences)
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
        self.motorControl.setMotor(lightLocation, 100)
        
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
        return duration, self.currentCommand
        
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
        self.satellitePos = np.array([0.69, -0.1])
        self.satelliteHeight = 2.95
        self.POIpos1 = np.array([1.755, 3.725])
        self.POIpos2 = np.array([0.47, 2.37])
        
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
        return True
    
    def checkRobotCollision(self, pos, orient):
        forward = computeOrientationVector(orient) * robotLength / 2
        right = computeOrientationVector((orient + 90) % 360) * robotWidth / 2
        
        #print(forward)
        #print(right)
        
        frontLeft = np.array(pos) + forward - right
        frontRight = np.array(pos) + forward + right
        backLeft = np.array(pos) - forward - right
        backRight = np.array(pos) - forward + right
        
        frontCollide = self.checkCollision(frontLeft, frontRight)
        rightCollide = self.checkCollision(frontRight, backRight)
        backCollide = self.checkCollision(backRight, backLeft)
        leftCollide = self.checkCollision(backLeft, frontLeft)
        
        #print([frontCollide, rightCollide, backCollide, leftCollide])
        
        return frontCollide or backCollide or leftCollide or rightCollide

class Particle:
    
    def __init__(self, arenaMap, pos, orient, toddler):
        self.arenaMap = arenaMap
        self.pos = pos
        self.orient = orient
        self.toddler = toddler
        
        self.weight = 0

    def moveForward(self, duration, backwards=False):
        orientVec = computeOrientationVector(self.orient)
        
        distanceMoved = self.toddler.motorSpeed * duration

        forwardNoise =  distanceMoved * orientVec * np.random.normal(0, motorForwardNoise)
        
        #print(noise)
        #print("Prev: "+ str(self.pos))
        newX = self.pos[0] + (-1 if backwards else 1) * orientVec[0] * distanceMoved + forwardNoise[0]
        newY = self.pos[1] + (-1 if backwards else 1) * orientVec[1] * distanceMoved + forwardNoise[1]
        self.pos = (newX, newY)
        
        #print(self.orient)
        driftNoise = distanceMoved * np.random.normal(0, motorDriftNoise)
        self.orient += driftNoise
        self.orient %= 360
        #print(self.orient)
        #print("After: " + str(self.pos))

    def moveBackwards(self, duration):
        self.moveForward(duration, backwards=True)

    def turn(self, duration, isLeft):
        #print("BEFORE: " + str(self.orient))
        
        print(self.toddler.angularSpeed)
        turns = self.toddler.angularSpeed * duration / 360.0     
        
        #self.orient += (-1 if isLeft else 1) * duration / motorFullTurnTime * 360 + turns * np.random.normal(0, motorTurnNoise)
        self.orient += (-1 if isLeft else 1) * turns * 360 + turns * np.random.normal(0, motorTurnNoise)
        self.orient %= 360
        
        newX = self.pos[0] + turns * np.random.normal(0, motorTurnPositionNoise)
        newY = self.pos[1] + turns * np.random.normal(0, motorTurnPositionNoise)
        self.pos = (newX, newY)
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

    def __init__(self, arenaMap, num_particles, toddler):
        
        self.arenaMap = arenaMap 
        self.particles = []
        while len(self.particles) < num_particles:
            x = np.random.uniform(arenaMap.bounds_rect[0][0], arenaMap.bounds_rect[1][0])
            y = np.random.uniform(arenaMap.bounds_rect[0][1], arenaMap.bounds_rect[1][1])
            
            #x = np.random.uniform(1.275, 1.525)
            #y = np.random.uniform(0.285, 0.535)
        
            #x = 3.06 + np.random.uniform(0.05, 0.05)
            #y = 0.44 + np.random.uniform(-0.05, 0.05)
            
            #x = 1.43 + np.random.uniform(-0.05, 0.05)
            #y = 0.40 + np.random.uniform(-0.05, 0.05)
            
            orient = (0 + np.random.uniform(0, 360)) % 360

            if arenaMap.checkInsideMap((x, y)):
                self.particles += [Particle(arenaMap, (x, y), orient, toddler=toddler)]

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

        leftIRNoiseStd = self.computeIRNoise(particleLeftIRDistance) * 2
        leftProb = gaussian(particleLeftIRReading, leftIRNoiseStd, leftIRReading)


        particleRightIRDistance = particle.senseRightIR()
        particleRightIRReading = convertToRightIRReading(particleRightIRDistance)
        
        rightIRNoiseStd = self.computeIRNoise(particleRightIRDistance) * 2
        rightProb = gaussian(particleRightIRReading, rightIRNoiseStd, rightIRReading)
        
        particleSonarDistance = particle.senseSonar()
        particleSonarReading = convertToSonarReading(particleSonarDistance)
        
        sonarNoiseStd = 30 * 2
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
        closeIRNoiseStd = 100
        normalIRNoiseStd = 20
        maxIRNoiseStd = 50

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
        for particle in self.particles:
            particle.weight /= total
        
        
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
        
        #self.motorBoard.turnLightOn()
        
        
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
        self.particleFilter = ParticleFilter(self.arenaMap, 1000, toddler=self)
        self.mapRenderer = MapRenderer(self.arenaMap, self.particleFilter)
        self.counter = 0
        
        self.motorSpeed = 0
        self.angularSpeed = 0
    
    def computeHeadingAndServoRotation(self):
        targetDirection = self.currentPos - self.satellitePos
        
        theta_radians = np.arctan2(targetDirection[0], targetDirection[1])
        rotationAngle = (theta_radians) / (2.0 * np.pi) * 360;
        
        epsilon = (self.currentOrient - rotationAngle) % 360
        
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
        
        horizontalDistance = np.linalg.norm(self.currentPos - self.satellitePos)
        verticalDistance = self.satelliteHeight - self.servoHeight
        
        servoRotationRadians = np.arctan(verticalDistance / horizontalDistance)
        print(servoRotationRadians)
        if servoInverted:
            servoRotationAngle = servoRotationRadians / np.pi * 180;
        else:
            servoRotationAngle = 180 - servoRotationRadians / np.pi * 180;
        
        return turnAngle, turnDirection, servoRotationAngle
        
        
    
    def control(self):
        
        
        self.collisionAvoidanceControl()
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
        self.motorBoard.moveForward()
        startTime = time.time()
        while self.sensors.getOdometerCount() <= 36:
            time.sleep(0.05)
        duration = time.time() - startTime
        self.sensors.resetOdometerCount()
        #self.sleepUpdate(motorFullTurnTime)
        self.motorBoard.stopMoving()
        time.sleep(3)
        self.motorBoard.turnRight()
        self.sleepUpdate(duration / 3.0)
        self.motorBoard.stopMoving()
        
    
    
    def executeCommand(self, command, duration):
        
        leftIRReadings = []
        rightIRReadings = []
        
        for i in range(5):
            leftIRReadings += [self.sensors.getIRSensorLeft()]
            rightIRReadings += [self.sensors.getIRSensorRight()]
            time.sleep(0.05)
        
        sonarReading = self.sensors.getSonar()
        
        measurements = {'leftIR': np.mean(leftIRReadings), 'rightIR': np.mean(rightIRReadings), 'sonar': sonarReading}    
        self.particleFilter.resampleParticles(measurements)
        
        self.particleFilter.updateParticles(command, duration)
        self.sensors.resetOdometerCount()
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
        
        lightUpperThreshold = 62
        
        if light > lightUpperThreshold:# or light < lightLowerThreshold:
            print("POI Detected")
            time.sleep(0.5)
            self.motorBoard.stopMoving()
            time.sleep(5)
        else:
            self.motorBoard.moveForward()

        time.sleep(0.05)

    def servoPointControl(self):
        if self.facingSatellite:
            return
        self.facingSatellite = True
        
        time.sleep(1)

        rotationAmountAngle, turnDirection, servoRotation = self.computeHeadingAndServoRotation()
        
        if turnDirection == "left":
            self.motorBoard.turnLeft()
            print("turning left")
        else:
            self.motorBoard.turnRight()
            print("turning right")
        
        self.servo.setPosition(servoRotation)
        while(self.motorBoard.getDuration() < (motorFullTurnTime * rotationAmountAngle / 360)):
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
        self.sleepUpdate(0.25)
        self.motorBoard.turnLeft()
        self.sleepUpdate(rnd)
        duration, command = self.motorBoard.stopMoving()
        self.sensors.resetOdometerCount()
        self.executeCommand(command, duration)

    def updateSpeedEstimates(self):
        
        averageForwardTimeDifference = self.sensors.getAverageForwardTimeDifference()
        averageTurnTimeDifference = self.sensors.getAverageTurnTimeDifference()
        
        self.motorSpeed = odometerSpeed / averageForwardTimeDifference
        self.angularSpeed = odometerAngularSpeed / averageTurnTimeDifference
        
        print("Motor Speed", self.motorSpeed)
        print("Angular Speed", self.angularSpeed)

    def collisionAvoidanceControl(self, stopOnPOI=True):
        
        self.updateSpeedEstimates()
        
        irThreshold = 400
        
        leftIR = self.sensors.getIRSensorLeft() 
        rightIR = self.sensors.getIRSensorRight() 
        whiskerLeft = self.sensors.getWhiskerLeft()
        whiskerRight = self.sensors.getWhiskerRight()
        #self.sensors.update()
        
        lightUpperThreshold = 56.5
        lightLowerThreshold = 19
        
        lightSensors = self.sensors.getLightSensors()
        
        light = lightSensors[0]
        rnd = 0.5 + 1.5 * np.random.rand()
        
        #if light > lightUpperThreshold:# or light < lightLowerThreshold:
        #    print("POI Detected")
        #    self.motorBoard.stopMoving()
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
        elif self.sensors.getSonar() < 23:
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
            
    
    def vision(self):
    
        self.sensors.updateOdometer(self.motorBoard.currentCommand)
        time.sleep(0.002)
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
    if distance < 0.3:
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

    def __init__(self, arenaMap, particleFilter):
        self.arenaMap = ArenaMap()
        self.particleFilter = particleFilter

        self.scale = 200
        self.padding = 0.2

        width = 3.2
        height = 4.25

        self.imgWidth = int(self.scale * (width + self.padding * 2))
        self.imgHeight = int(self.scale * (height + self.padding * 2))
        self.img = np.zeros((self.imgHeight, self.imgWidth, 3), np.uint8)
        self.img[:] = (255,255,255)


    def drawMap(self):
        self.img[:] = (255,255,255)
        for obstacle in self.arenaMap.obstacles:
            for i in range(len(obstacle)):
                p1 = obstacle[i]
                p2 = obstacle[(i+1)%len(obstacle)]
                p1x = int(self.scale * (p1[0] + self.padding))
                p1y = self.imgHeight - int(self.scale * (p1[1] + self.padding))
                p2x = int(self.scale * (p2[0] + self.padding))
                p2y = self.imgHeight - int(self.scale * (p2[1] + self.padding))
                
                cv2.line(self.img, (p1x,p1y), (p2x, p2y), (0,0,0), 2)
                #plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='k', linestyle='-', linewidth=2)

    def drawParticles(self):
        weights = [p.weight for p in self.particleFilter.particles]
        maxWeight = max(weights)
        #print(weights)
        for particle in self.particleFilter.particles:
            
            particleImgX = int(self.scale * (particle.pos[0] + self.padding))
            particleImgY = self.imgHeight - int(self.scale * (particle.pos[1] + self.padding))


            cv2.circle(self.img, (particleImgX, particleImgY), 4, (255,0, 255*particle.weight/maxWeight), thickness=-1)
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
        
        bestParticle = self.particleFilter.particles[np.argmax(weights)]
        
        bestParticleImgX = int(self.scale * (particle.pos[0] + self.padding))
        bestParticleImgY = self.imgHeight - int(self.scale * (particle.pos[1] + self.padding))
        
        
        cv2.circle(self.img, (bestParticleImgX, bestParticleImgY), 4, (0,0,0), thickness=-1)
        orientVec = computeOrientationVector(bestParticle.orient) * 30
        orientVec[1] = -orientVec[1]

        cv2.line(self.img, (bestParticleImgX, bestParticleImgY), (bestParticleImgX + int(orientVec[0]), bestParticleImgY + int(orientVec[1])), (0,0,0), 2)            
            

    def showMap(self):
        cv2.imshow("test", self.img)
        cv2.waitKey(0)