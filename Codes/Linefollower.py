from controller import Robot, Motor, DistanceSensor

# Time step of the simulation
TIME_STEP = 32

# Create the Robot instance
robot = Robot()

# Initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Initialize infrared sensors
sensors = ["ir0", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7"]
ir_sensors = [robot.getDevice(sensor_name) for sensor_name in sensors]
for sensor in ir_sensors:
    sensor.enable(TIME_STEP)

# Function to read sensor values
def read_sensors():
    return [sensor.getValue() for sensor in ir_sensors]

# PID control function
def compute_control(sensor_values):
    set_point = 700
    left_sensor = sensor_values[2]
    right_sensor = sensor_values[5]
    error = left_sensor - right_sensor
    
    # PID coefficients
    Kp = 0.005
    Ki = 0.0
    Kd = 0.001
    
    # Proportional term
    P = error
    
    # Integral term
    I = 0
    
    # Derivative term
    D = error - compute_control.prev_error if hasattr(compute_control, 'prev_error') else 0
    
    # PID output
    pid_output = Kp * P + Ki * I + Kd * D
    compute_control.prev_error = error
    
    base_speed = 2.0
    left_speed = base_speed - pid_output
    right_speed = base_speed + pid_output
    
    return left_speed, right_speed

# Main control loop
while robot.step(TIME_STEP) != -1:
    sensor_values = read_sensors()
    left_speed, right_speed = compute_control(sensor_values)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
