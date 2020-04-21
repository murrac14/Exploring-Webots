"""six_joint_robot_arm controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math


def get_2d_direction(unit_vector):
    radians_west_of_north = math.atan2(unit_vector[0], unit_vector[2])
    degrees = math.degrees(radians_west_of_north)
    if degrees < 0:
        degrees = (degrees + 360)
    return degrees


def find_desired_orientation(orientation, turn, orientation_flipped):
    if (turn == "right" and not orientation_flipped) or (turn == "left" and orientation_flipped):
        next_orientation = orientation + 90
    elif (turn == "left" and not orientation_flipped) or (turn == "right" and orientation_flipped):
        next_orientation = orientation - 90
    if next_orientation > 360:
        next_orientation = next_orientation - 360
    elif next_orientation < 0:
        next_orientation = next_orientation + 360
    return next_orientation


# constants (UPPER) and variables (lower)
TIMESTEP = 64
US_SENSOR_COUNT = 2
MIN_OBJECT_DISTANCE = 800.0
ORIENTATION_BUFFER = 2.0
WHEEL_RADIUS = 0.08
robot_speed = 2.0
achieved_orientation = True
turn_left = False
turn_right = False
robot_flipped = False

# create the Robot instance.
robot = Robot()

# make a list of the wheels
wheels = []
wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
for i in range(4):
    wheels.append(robot.getMotor(wheel_names[i]))
    wheels[i].setPosition(float("inf"))
    wheels[i].setVelocity(0.0)
# make a list of the ultrasonic sensors
US_sensors = []
US_distances = []
US_sensor_names = ["distance sensor front right", "distance sensor front left"]
for i in range(US_SENSOR_COUNT):
    US_sensors.append(robot.getDistanceSensor(US_sensor_names[i]))
    US_sensors[i].enable(TIMESTEP)
    US_distances.append(US_sensors[i].getValue())
# initialise accelerometer
accelerometer = robot.getAccelerometer("accelerometer")
accelerometer.enable(TIMESTEP)
# initialise compass
compass = robot.getCompass("compass")
compass.enable(TIMESTEP)
desired_orientation = get_2d_direction(compass.getValues())

# Main loop
while robot.step(TIMESTEP) != -1:
    # read the sensors
    acceleration_vector = accelerometer.getValues()
    degrees_east_of_north = get_2d_direction(compass.getValues())

    if acceleration_vector[1] > 0 and robot_flipped:
        robot_flipped = False
        robot_speed = -robot_speed
    elif acceleration_vector[1] < 0 and not robot_flipped:
        robot_flipped = True
    print("Flipped: " + str(robot_flipped))

    for i in range(US_SENSOR_COUNT):
        US_distances[i] = US_sensors[i].getValue()
        # print(US_distances[i])

    for i in range(US_SENSOR_COUNT):
        if US_distances[i] < MIN_OBJECT_DISTANCE and achieved_orientation:
            if ((US_sensor_names[i] == "distance sensor front right" and not robot_flipped)
                    or (US_sensor_names[i] == "distance sensor front left" and robot_flipped)):
                desired_orientation = find_desired_orientation(degrees_east_of_north, "left", robot_flipped)
                turn_left = True
                print("Turn LEFT")
            elif ((US_sensor_names[i] == "distance sensor front left" and not robot_flipped)
                    or (US_sensor_names[i] == "distance sensor front right" and robot_flipped)):
                desired_orientation = find_desired_orientation(degrees_east_of_north, "right", robot_flipped)
                turn_right = True

            achieved_orientation = False
        print(US_distances[i])
    print("Desired orientation")
    print(desired_orientation)
    print("degrees_east_of_north")
    print(degrees_east_of_north)

    if not achieved_orientation:
        # if robots upright turn left, if upside-down turn right and vice versa
        # Wheels: left front = 0, right front = 1, left rear = 2, right rear = 3
        if turn_left:
            # and not robot_flipped) or (turn_right and robot_flipped):
            wheels[0].setVelocity(-robot_speed)
            wheels[2].setVelocity(-robot_speed)
            wheels[1].setVelocity(robot_speed)
            wheels[3].setVelocity(robot_speed)
            print("TURNING TO THE LEFT")

        # elif (turn_right and not robot_flipped) or (turn_left and robot_flipped):
        else:
            wheels[0].setVelocity(robot_speed)
            wheels[2].setVelocity(robot_speed)
            wheels[1].setVelocity(-robot_speed)
            wheels[3].setVelocity(-robot_speed)
            print("TURNING TO THE RIGHT")

    elif robot_flipped:
        for i in range(4):
            wheels[i].setVelocity(-robot_speed)

    else:
        for i in range(4):
            wheels[i].setVelocity(robot_speed)

    # if the orientation of the robot is within the buffer zone then we have reached our desired orientation
    if (desired_orientation - ORIENTATION_BUFFER) < degrees_east_of_north < (desired_orientation + ORIENTATION_BUFFER):
        turn_left = False
        turn_right = False
        achieved_orientation = True
