# Hexapod Robot

<img src="https://github.com/user-attachments/assets/1888492b-be02-4d7f-9e52-b3b36c682b3e" width="500" height="450" />


## Overview

This project is dedicated to building a **hexapod robot** a six-legged robot that can walk, turn, and navigate in any direction. The robot is designed to be controlled remotely, while also being capable of autonomous navigation.

The goal of this project is to develop a versatile, stable, and efficient walking robot that can be used for research and exploration purposes.

## Body Architecture
The body architecture of any hexapod robot falls into either of the two categories; ones that have a rectangular base with three legs on either side or ones that have a circular/hexagonal base with the legs placed in a radially symmetrical position.

In this project, we have decided to use the latter configuration having a circular base as it has better stability and turning ability and is generally more versatile when it comes to implementing various gaits.

<img src="https://github.com/user-attachments/assets/59514e4d-28b3-4a90-b09a-b06dc587a80f" width="400" height="200" />

The link lengths of the leg were assigned to be l1, l2, and l3 and with the use of inverse kinematics, we obtained equations to calculate the joint angles needed to achieve a desired end effector position and orientation.

The trajectory followed by each leg follows a fixed path that goes through a series of phases in a repeating sequence: lift-off, swing, touchdown and stance. Various types of gaits can be implemented on an hexapod owing to the stability that the six legs can provide. We decided to implement a tripod gait as it provides a perfect balance between speed and stability

## Hardware Components

- **Microcontroller**: Raspberry Pi 4 for controlling motors and sensors
- **Servomotors**: 18 servo motors to drive the legs
- **Power Supply**: SMPS 5V,24A
- **Chassis**: Laser cut acrylic sheets
- **Sensors**:
  - Ultrasonic for obstacle detection.
  - IMU (Inertial Measurement Unit) for balance and orientation.

## Software Requirements

- **Raspberry Pi OS**: For programming the rpi4
- **Python**: Language for writing control algorithms
- **ROS (Robot Operating System)**: For advanced control and simulation.

## Results
We were able to achieve basic locomotion in four directions and the code was also written so as to make it easier to upgrade into omni-directional locomotion. We also tested our robot on how effectively it can climb over obstacles and were able to achieve a maximum climbing height of about 10cm.

- **Walking**
<img src="https://github.com/user-attachments/assets/c1d49b16-61b9-49f3-b9e6-71adc0f41e7a" width="700" height="450" />

- **Obstacle Climbing**
<img src="https://github.com/user-attachments/assets/a215c731-96ce-4ea3-92b6-bd626a3c5730" width="700" height="450" />
