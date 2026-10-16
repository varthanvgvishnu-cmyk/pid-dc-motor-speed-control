# PID-Based Closed-Loop DC Motor Speed Control Using Arduino

## Project Overview

This project implements a closed-loop DC motor speed control system using a Proportional–Integral–Derivative (PID) controller. The controller compares the desired motor speed with the measured rotational speed and adjusts the motor driver's PWM output to minimize the speed error.

The project demonstrates fundamental control-system concepts, embedded programming, PWM motor control, and real-time feedback.

## Objectives

- Implement PID-based closed-loop motor speed control.
- Set a desired motor speed using a potentiometer or another suitable input.
- Measure actual motor speed using a rotational speed sensor.
- Calculate the error between the reference speed and measured speed.
- Adjust PWM duty cycle to regulate motor speed.
- Evaluate the response to changes in the setpoint and motor load.

## Hardware Components

- Arduino Uno or Arduino Nano
- DC geared motor
- L298N or compatible motor driver
- Rotational speed sensor, such as a Hall-effect sensor with a magnet or a motor encoder
- Potentiometer for speed reference
- External motor power supply
- Jumper wires and connecting components

*The final component list and wiring depend on the hardware used in the prototype.*

## Working Principle

1. **Reference input:** The desired motor speed is selected using a potentiometer or another input method.
2. **Speed measurement:** A sensor measures the motor's rotational speed.
3. **Error calculation:** The difference between the target speed and measured speed is calculated.
4. **PID control:** The controller processes the error using proportional, integral, and derivative terms.
5. **PWM adjustment:** The controller changes the PWM duty cycle sent to the motor driver.
6. **Feedback:** The measured speed is repeatedly compared with the target speed to reduce the error.

## Control System

The closed-loop system follows this general signal path:

Reference Speed → Error Calculation → PID Controller → PWM Motor Driver → DC Motor → Speed Sensor → Feedback

## PID Controller

The control output is calculated using:

u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt

Where:

- **Kp:** Proportional gain
- **Ki:** Integral gain
- **Kd:** Derivative gain
- **e(t):** Difference between target and measured speed
- **u(t):** Controller output used to determine the motor drive command

The PID gains must be tuned and tested on the actual hardware.

## Software and Tools

- Arduino IDE
- Embedded C/C++
- PWM motor control
- PID control algorithm
- Serial Monitor for debugging and performance observation

## Testing and Results

Record the following after implementing and testing the system:

- Target motor speed (RPM)
- Measured motor speed (RPM)
- PWM duty cycle
- Steady-state speed error
- Response to changes in the target speed
- Response to changes in motor load

Add genuine measurements, graphs, and photographs from the completed prototype.

## Applications

- DC motor speed regulation
- Educational control-system experiments
- Small automation systems
- Embedded motor-control applications
- Demonstrations of feedback control and PID tuning

## Future Improvements

- Automatic PID gain tuning
- Live RPM monitoring dashboard
- Graphical analysis of speed response
- Improved filtering of sensor measurements
- Testing under different load conditions

## Author

**Vishnu Varthan VG**

Electrical and Electronics Engineering (EEE)
