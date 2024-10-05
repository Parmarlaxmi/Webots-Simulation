Webots Robotics Projects


Overview


This README provides an explanation of two robotic projects implemented using the Webots robotics simulator: an obstacle avoidance robot and a line follower robot. Both projects demonstrate basic navigation techniques using different sensor setups and control algorithms.



Project 1: Obstacle Avoidance Robot


Objective


The goal of this project is to design and implement a robot capable of navigating an environment while avoiding obstacles. The robot uses proximity sensors to detect obstacles and adjusts the speeds of its wheels to steer away from them.



Components


1. Robot: The main robot instance.
2. Motors: Two motors controlling the left and right wheels of the robot.
3. Proximity Sensors: A set of proximity sensors placed around the robot to detect obstacles.


Implementation Details


Initialization


1. Robot Instance: An instance of the Robot class is created to represent the robot in the simulation.
2. Motors: The left and right motors are initialized and set to operate in velocity control mode with infinite rotation. Their initial velocities are set to zero.
3. Proximity Sensors: Eight proximity sensors (ps0 to ps7) are initialized and enabled with the specified time step to allow continuous reading of sensor values.
4. Reading Sensor Values A function read_sensors() reads the current values from the proximity sensors. These values indicate the distance to the nearest obstacle detected by each sensor.
5. Adjusting Motor Speed The function adjust_motor_speed(sensor_values) calculates the appropriate speeds for the left and right motors based on the readings from the proximity sensors:

6. Base Speed: Both motors are set to a base speed of 3.0.



Obstacle Detection:


If an obstacle is detected by the left sensors (ps0 or ps1), the robot will turn right by decreasing the speed of the left motor and increasing the speed of the right motor.


If an obstacle is detected by the right sensors (ps6 or ps7), the robot will turn left by increasing the speed of the left motor and decreasing the speed of the right motor.


Main Control Loop


The main control loop continuously executes the following steps:

1. Read Sensor Values: The current values from the proximity sensors are read.
2. Compute Motor Speeds: The motor speeds are adjusted based on the sensor values using the adjust_motor_speed() function.
3. Set Motor Velocities: The computed speeds are applied to the left and right motors to control the robot's movement.
4. This loop runs repeatedly, allowing the robot to continuously navigate and avoid obstacles in real-time.









Project 2: Line Follower Robot


Objective


The goal of this project is to design and implement a robot capable of following a line on the ground. The robot uses infrared (IR) sensors to detect the line and adjusts its movement to stay on the line.



Components

1. Robot: The main robot instance.
2. Motors: Two motors controlling the left and right wheels of the robot.
3. Infrared Sensors: A set of IR sensors to detect the line on the ground.


Implementation Details


Initialization


1. Robot Instance: An instance of the Robot class is created to represent the robot in the simulation.
2. Motors: The left and right motors are initialized and set to operate in velocity control mode with infinite rotation. Their initial velocities are set to zero.
3. Infrared Sensors: A set of IR sensors is initialized and enabled with the specified time step to allow continuous reading of sensor values.
4. Reading Sensor Values
A function read_sensors() reads the current values from the IR sensors. These values indicate the presence of the line detected by each sensor.

5. PID Control
The function compute_control(sensor_values) uses a PID controller to adjust the motor speeds based on the sensor readings:

6. Error Calculation: The difference between the readings of the left and right sensors is calculated.
7. PID Coefficients: Proportional (P), Integral (I), and Derivative (D) coefficients are defined.
8. Control Output: The PID output is calculated using the error and the PID coefficients.
9. Motor Speed Adjustment: The motor speeds are adjusted based on the PID output to correct the robot's path.



Main Control Loop
The main control loop continuously executes the following steps:



1. Read Sensor Values: The current values from the IR sensors are read.
2. Compute Motor Speeds: The motor speeds are adjusted based on the sensor values using the compute_control() function.
3. Set Motor Velocities: The computed speeds are applied to the left and right motors to control the robot's movement.
4. This loop runs repeatedly, allowing the robot to continuously follow the line in real-time.



Conclusion


This document provides a comprehensive explanation of two robotic projects implemented using the Webots simulator: an obstacle avoidance robot and a line follower robot. Each project demonstrates basic navigation techniques using different sensor setups and control algorithms. Webots' powerful simulation capabilities make it an ideal tool for developing and testing robotic systems in a controlled virtual environment.





