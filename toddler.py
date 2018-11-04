    #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 11 11:25:29 2018

@author: s1873447
"""

#!/usr/bin/env python

import time
import sys        
import numpy as np
from copy import copy

# UPDATE THESE BASED ON EXPERIMENTS
motorFullTurnTime = 5.2
motorSpeed = 0.425

motorTurnNoise = 20
motorForwardNoise = 10


leftIRSensorLocalPosition = np.array([-0.13, 0.15])
leftIRSensorAngleOffset = -45
rightIRSensorLocalPosition = np.array([0.13, 0.15])
rightIRSensorAngleOffset = 45


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
    
    def update(self):        
        if self.getOdometer() and not self.oldOdometer:
            self.odometerCount += 1
            print(self.odometerCount)
            
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
    
class MotorBoardManager:
    
    def __init__(self, motor_control):
        self.motorBoardLocations = MotorBoardLocations()
        self.motorControl = motor_control
        self.motorsEngaged = False
        self.motorStartTime = -1
        
    def turnLightOn(self):
        lightLocation = self.motorBoardLocations.getLight()
        self.motorControl.setMotor(lightLocation, 100)
        
    def turnLightOff(self):
        lightLocation = self.motorBoardLocations.getLight()
        self.motorControl.setMotor(lightLocation, 0)
        
    def turnLeft(self, speed=100):
        self.turn(speed, isLeft=True)
    
    def turnRight(self, speed=100):
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
        if self.motorsEngaged:
            #print("Attempted to move forward but the motor was already engaged")
            return
        self.setEngaged(True)
        leftMotor = self.motorBoardLocations.getLeftMotor()
        rightMotor = self.motorBoardLocations.getRightMotor()
        self.motorControl.setMotor(leftMotor, (-1 if not backwards else 1) * speed)
        self.motorControl.setMotor(rightMotor, (-1 if not backwards else 1) * speed)
    
    def stopMoving(self):
        self.setEngaged(False)
        leftMotor = self.motorBoardLocations.getLeftMotor()
        rightMotor = self.motorBoardLocations.getRightMotor()
        self.motorControl.stopMotor(leftMotor)
        self.motorControl.stopMotor(rightMotor)
        
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
        outer_wall = [(0, 1.05),
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
                         (2.29, 1.05)]


        self.bounds_rect = [(0, 0), (3.2, 4.25)]
        self.obstacles = [outer_wall, tri_obstacle, rect_obstacle]
        
    def checkClosestCollision(self, start, end):
        closest = None
        closest_dist = np.inf 
        for obstacle in self.obstacles:
            for i in range(len(obstacle)):
                wallp1 = obstacle[i]
                wallp2 = obstacle[(i+1)%len(obstacle)]
                collisionLocation = lineSegmentCollision(wallp1, wallp2, start, end)
                distance = np.linalg.norm(start - collisionLocation)
                if distance < closest_dist:
                    closest = collisionLocation
                    closest_dist = distance
        if closest != None:
            return closest
        else:
            return False
    
    def checkInsideMap(self, p):
        #inBounds = p[0] < self.bounds_rect[1][0] and p[0] > self.bounds_rect[0][0] and p[1] < self.bounds_rect[1][1] and p[1] > self.bounds_rect[1][0]
        #inCorner = p0
        return True


class Particle:

    def __init__(self, arenaMap, pos, orient):
        self.arenaMap = arenaMap
        self.pos = pos
        self.orient = orient

    def moveForward(self, duration, backwards=False):
        orientVec = computeOrientationVector(self.orient)
        self.pos += (-1 if backwards else 1) * orientVec * motorSpeed * duration + np.random.normal(0, motorForwardNoise)

    def moveBackwards(self, duration):
        self.moveForward(duration, backwards=True)

    def turn(self, duration, isLeft):
        print("BEFORE: " + str(self.orient))
        self.orient += (-1 if isLeft else 1) * duration / motorFullTurnTime * 360 + np.random.normal(0, motorTurnNoise)
        self.orient %= 360
        print("AFTER: " + str(self.orient))

    def turnLeft(self, duration):
        self.turn(duration, isLeft=True)

    def turnRight(self, duration):
        self.turn(duration, isLeft=False)
        
    def senseLeftIR(self):

        leftIRGlobalPosition = computeOrientationVector(self.orient, referenceVec=leftIRSensorLocalPosition)
        leftIROrientVector = computeOrientationVector((self.orient + leftIRSensorAngleOffset) % 360)
        
        irRange = 5
        
        collisionLocation = self.arenaMap.checkClosestCollision(leftIRGlobalPosition, leftIRGlobalPosition + irRange * leftIROrientVector)
        assert(collisionLocation, "Particle's Left IR sensor didn't collide")

        distance = np.linalg.norm(collisionLocation - leftIRGlobalPosition)
        
        return distance
        
    def senseRightIR(self):

        rightIRGlobalPosition = computeOrientationVector(self.orient, referenceVec=rightIRSensorLocalPosition)
        rightIROrientVector = computeOrientationVector((self.orient + rightIRSensorAngleOffset) % 360)
        
        irRange = 5
        
        collisionLocation = self.arenaMap.checkClosestCollision(rightIRGlobalPosition, rightIRGlobalPosition + irRange * rightIROrientVector)
        assert(collisionLocation, "Particle's Right IR sensor didn't collide")

        distance = np.linalg.norm(collisionLocation - rightIRGlobalPosition)
        
        return distance

class ParticleFilter:

    def __init__(self, arenaMap, num_particles):
        
        self.arenaMap = arenaMap 
        self.particles = []
        while len(self.particles) < num_particles:
            #x = np.random.uniform(arenaMap.bounds_rect[0][0], arenaMap.bounds_rect[1][0])
            #y = np.random.uniform(arenaMap.bounds_rect[0][1], arenaMap.bounds_rect[1][1])
            
            x = np.random.uniform(1.275, 1.525)
            y = np.random.uniform(0.285, 0.535)
            orient = np.random.uniform(0, 360)

            if arenaMap.checkInsideMap((x, y)):
                self.particles += [Particle(arenaMap, (x, y), orient)]

    def updateParticles(self, command, duration):
        if not (command in ['forward', 'backwards', 'turn_left', 'turn_right']):
            return
        for particle in self.particles:
            if command == 'forward':
                particle.moveForward(duration)
            elif command == 'backwards':
                particle.moveBackwards(duration)
            elif command == 'turn_left':
                particle.turnLeft(duration)
            elif command == 'turn_right':
                particle.turn_right(duration)

    def measurementProb(self, particle, measurements):
        leftIRReading = measurements['leftIR']
        rightIRReading = measurements['rightIR']
        #sonarReading = measurements['sonar']
        
        
        particleLeftIRDistance = particle.senseLeftIR()
        particleRightIRDistance = particle.senseRightIR()
        
        
        particleLeftIRReading = convertToIRReading(particleLeftIRDistance)
        particleRightIRReading = convertToIRReading(particleRightIRDistance)
        
        irNoiseStd = 15
        
        leftProb = gaussian(particleLeftIRReading, irNoiseStd, leftIRReading)
        rightProb = gaussian(particleRightIRReading, irNoiseStd, rightIRReading)
        
        prob = leftProb * rightProb
        
        return prob

    def resampleParticles(self, measurements):
        
        # Compute Importance weights
        weights = []
        for particle in self.particles:
            likelihood = self.measurementProb(particle, measurements)
            weights.append(likelihood)
        total = sum(weights)
        weights = [w / total for w in weights]

        # Resample Particles
        newParticles = []
        beta = 0
        index = 0
        maxWeight = max(weights)
        bestParticle = self.particles[np.argmax(weights)]
        print("Best particle: " + str(bestParticle.pos[0]) + ", " + str(bestParticle.pos[1]) + ", " + str(bestParticle.orient))
        print("Best particle readings: " + str(convertToIRReading(bestParticle.senseLeftIR())) + ", " + str(convertToIRReading(bestParticle.senseRightIR())))
        
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
        
        self.motorBoard.turnLightOn()
        
        self.startPos = np.array([1.4, 0.41])
        self.satellitePos = np.array([0.69, -0.1])
        self.satelliteHeight = 2.95
        
        self.servoHeight = 0.2
        
        self.POIpos1 = np.array([1.755, 3.725])
        self.POIpos2 = np.array([0.47, 2.37])

        self.facingSatellite = False
        
        if len(sys.argv) > 2:
            currentX = sys.argv[1]
            currentY = sys.argv[2]
            self.currentPos = np.array([float(currentX), float(currentY)])
            self.currentOrient = float(sys.argv[3])
        else:
            self.currentPos = self.POIpos1
            self.currentOrient = 0
        
        self.arenaMap = ArenaMap()
        self.particleFilter = ParticleFilter(self.arenaMap, 500)
        self.counter = 0
            
    
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
        
        #self.collisionAvoidanceControl()

        #self.forwardUntilPOIControl()
        #self.servoPointControl()
        self.counter += 1
        if self.counter < 20:
            self.fullTurnControl()
        
        time.sleep(0.1)
        
    def fullTurnControl(self):        
        self.motorBoard.turnLeft()
        time.sleep(motorFullTurnTime / 10)
        duration = self.motorBoard.getDuration()
        self.motorBoard.stopMoving()
        measurements = {'leftIR': self.sensors.getIRSensorLeft(), 'rightIR': self.sensors.getIRSensorLeft()}
        print("IR Readings: " + str(measurements['leftIR']) + ", " + str(measurements['rightIR']))
        self.particleFilter.resampleParticles(measurements)
        self.particleFilter.updateParticles('turn_left', duration)
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


    def avoidLeft(self, rnd):
        self.motorBoard.stopMoving()
        time.sleep(0.02)
        self.motorBoard.turnLeft()
        time.sleep(rnd)
        self.motorBoard.stopMoving()

    def avoidRight(self, rnd):
        self.motorBoard.stopMoving()
        time.sleep(0.02)
        self.motorBoard.turnRight()
        time.sleep(rnd)
        self.motorBoard.stopMoving()

    def avoidBack(self, rnd):
        self.motorBoard.stopMoving()
        time.sleep(0.02)
        self.motorBoard.moveForward(backwards=True)
        time.sleep(0.8)
        self.motorBoard.stopMoving()
        time.sleep(0.25)
        self.motorBoard.turnLeft()
        time.sleep(rnd)
        self.motorBoard.stopMoving()

    def collisionAvoidanceControl(self, stopOnPOI=True):
        
        irThreshold = 400
        
        leftIR = self.sensors.getIRSensorLeft() 
        rightIR = self.sensors.getIRSensorRight() 
        whiskerLeft = self.sensors.getWhiskerLeft()
        whiskerRight = self.sensors.getWhiskerRight()
        
        
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
        elif self.sensors.getSonar() < 20:
            self.avoidBack(rnd)
        else:
            self.motorBoard.moveForward()
            
        time.sleep(0.05)
            
    
    def vision(self):
        time.sleep(0.1)
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
        return False
    
    t_a = t_a_num / t_den
    t_b = t_b_num / t_den
    
    if t_a < 0 or t_a > 1 or t_b < 0 or t_b > 1:
        return False
    
    intersection_point = (p1[0] + t_a * (p2[0] - p1[0]), p1[1] + t_a * (p2[1] - p1[1]))
    return intersection_point

def convertToIRReading(distance):
    if distance < 0.1:
        return 4700*distance
    else:
        return 64.053 / (distance + 0.03623)
    
def gaussian(mu, sigma, x):        
    # Given a x position, mean mu and std sigma of Gaussian, calculates the probability
    # Note
    # mu: estimated distance by each particle's position, (map and landmarks are known)
    # x:  measured distance by the robot
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))