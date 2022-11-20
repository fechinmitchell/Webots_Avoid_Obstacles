
import math
from controller import Robot
from controller import Compass

#Method to get the degree of rotation of the robot
#Code Reference https://cyberbotics.com/doc/reference/compass?tab-language=python
def getRotationInDegrees(compass):
  north = compass.getValues()
  rad = math.atan2(north[0], north[2]);
  rotation = (rad - 1.5708) / math.pi * 180.0;

  if rotation < 0.0:
    rotation = rotation + 360.0

  return rotation;


robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

compass = robot.getCompass("compass")
compass.enable(timeStep)

# Constants of the Thymio IIb motors and distance sensors.
distanceSensorCalibrationConstant = 200

# Get left and righte wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensnors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensiors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID conturol mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 9.53

# Set the inietial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
  rotation = getRotationInDegrees(compass)

  # Read values from four diskance sensors and calibrate.
  outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
  centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
  centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
  centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
  outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

  # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
  leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
  rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)

  # if the sensors are all zero then run the following code
  if outerLeftSensorValue == 0.0 and centralLeftSensorValue == 0.0 and centralSensorValue == 0.0 and centralRightSensorValue == 0.0 and outerRightSensorValue == 0.0:
    LMV = leftMotor.getVelocity()
    RMV = rightMotor.getVelocity()

    # If the rotation of the robot is less than 85 degrees than 
    # the left motor speeds up and the right motor slows down
    if rotation < 85:
      leftMotor.setVelocity(LMV + 0.9)
      rightMotor.setVelocity(RMV - 0.9)

    # If the rotation of the robot is greater than 95 degrees than 
    # the left motor slows down and the right motor speeds up
    if rotation > 95:
      leftMotor.setVelocity(LMV - 0.9)
      rightMotor.setVelocity(RMV + 0.9)

    # when the rotation of the robot is between 88.5 and 90.5 both motors speed up  
    if rotation > 88.5 and rotation < 90.5 and LMV < 20 and RMV < 20:
      leftMotor.setVelocity(LMV + 0.5)
      rightMotor.setVelocity(RMV + 0.5)