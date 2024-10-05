from controller import Robot, Motor, DistanceSensor 

# Time step of the simulation
TIME_STEP = 64

# Create the Robot instance
robot = Robot()

# Initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Initialize proximity sensors
sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
ir_sensors = [robot.getDevice(name) for name in sensor_names]
for sensor in ir_sensors:
    sensor.enable(TIME_STEP)

# Function to adjust motor speed based on sensor values
def adjust_motor_speed(sensor_values):
    left_speed = 3.0  # base speed
    right_speed = 3.0  # base speed

    if sensor_values[0] > 100 or sensor_values[1] > 100:  # Obstacle to the left
        left_speed -= 2.0  # turn right
        right_speed += 2.0
    elif sensor_values[6] > 100 or sensor_values[7] > 100:  # Obstacle to the right
        left_speed += 2.0  # turn left
        right_speed -= 2.0

    return left_speed, right_speed

# Main control loop
while robot.step(TIME_STEP) != -1:
    sensor_values = [sensor.getValue() for sensor in ir_sensors]
    left_speed, right_speed = adjust_motor_speed(sensor_values)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
