# Autonomous Robot Navigation Firmware

This project is designed to control an autonomous robot, enabling it to navigate towards a target position. It utilizes PID control for precise movement and implements various algorithms for path planning and drive control.

## Features

- **[PID Controller](#PID)**: Utilizes Proportional-Integral-Derivative (PID) control to manage the robot's speed and direction accurately.
- **[Serial Communication](#comm)**: Communicates with external devices or computers through serial communication, allowing for dynamic target position updates.
- **[Odometry Calculations](#odo)**: Estimates the robot's position and orientation based on encoder feedback, enabling accurate movement control.
- **[Vehicle Control](#vehicle)**: Controls the robot's motors to achieve the desired velocities and directions for navigation.
- **[Circle Tracking](#circle)**: Includes an algorithm for circle tracking and path planning, enabling the robot to calculate the desired turning radius and wheel speeds to get from the current position to the target position.


## Components

- **vec Struct**: Represents a 2D vector with `x` and `y` components, used for positions and velocities.
- **Setup Function**: Initializes serial communication, motor, and encoder pins, and attaches interrupt routines for encoders.
- **Main Loop**: Continuously reads serial data for new target positions, calculates desired velocities using PID control, and sends commands to the motors.
- **Interrupt Service Routines (ISRs)**: Reads encoder positions to provide feedback for the control system.
- **Utility Functions**:
  - `recvWithStartEndMarkers()`: Reads incoming serial data with start and end markers.
  - `parseDataPID()`: Parses the received data into target positions, modes, and reset commands.
  - `lowPassFilter()`: Applies a low-pass filter to smooth out the control signals.
  - `boundAngle()`: Normalizes angles to the range [0, 2π) for consistent calculations.

## Setup

1. **Hardware Setup**: Connect motors, encoders, and any other necessary hardware to the microcontroller according to the defined pin assignments in the code.
2. **Software Setup**: Upload the provided code to the microcontroller. Ensure any external device intended to communicate with the robot is set up to send data in the correct format.

## Usage

- To command the robot, send data in the format `<target_x, target_y, mode, resetEnc>` through serial communication.
  - `target_x` and `target_y` are the coordinates of the target position in centimeters.
  - `mode` can be `0` for normal operation, `1` for off, or `2` for brake.
  - `resetEnc` resets the encoder positions when changed from `0` to `1`.

## Note

This project was developed by Odyssey for the 2024 QUT DRC. It provides a basic framework for autonomous robot navigation but may require adjustments and improvements for specific applications or more complex environments.