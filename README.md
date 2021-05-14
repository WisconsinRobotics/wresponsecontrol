# WResponseControl
## Purpose
This package provides a ROS package that, given a setpoint stream, output channel, and feedback mechanism, controls the output to generate feedback matching the setpoint using the PID control scheme.
  
---
## Usage
To use WResponseControl, place this project in your ROS package directory and make a launch file like `launch/demo.launch` and use a YAML ROS parameters file like `demo/demo.yaml` to configure what controllers you want and how you want them to work.
  
---
## Future Work
### Control Loop Tuning
It would be useful to include a built-in program to help tune for the correct PID values.
### Other Control Loop Types
It might be helpful to have non-PID options for a control loop, but we couldn't name any off the top of our heads.