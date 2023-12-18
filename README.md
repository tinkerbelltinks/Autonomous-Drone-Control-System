# Autonomous Drone Control System

## Overview
The autonomous drone control system is designed to control the movement and behavior of a drone using ROS (Robot Operating System). It consists of modules for Trajectory Generation, Finite State Machine (FSM), Telemetry, Ground Station, and Behavior Planner.

## File Structure
- `scripts/trajectory_generator.py`: Module for generating drone trajectories
- `scripts/fsm.py`: Module for implementing the Finite State Machine logic
- `scripts/.vscode/c_cpp_properties.json`: Configuration file for C/C++ properties
- `scripts/.vscode/settings.json`: Configuration file for settings
- `scripts/ground_station/ground_station.py`: Module for handling user input and mission commands from the ground station
- `scripts/telemetry.py`: Module for telemetry data collection from the drone
- `scripts/behavior_planner.py`: Module for planning and executing drone behaviors based on telemetry and mission commands
- `msg/Status.msg`: Message file defining the status data structure
- `CMakeLists.txt`: CMake configuration file for building and installing the package

## Error Handling and Solutions
During the development process, the following errors were encountered and addressed:

### Error 1: Connectivity Issues
- **Cause**: Connection problems between the drone and the ground control system.
- **Solution**: Implemented retry mechanisms and verified connection status through the Telemetry module.

### Error 2: Mission Invalidation
- **Cause**: Invalid or conflicting mission commands being received by the drone.
- **Solution**: Updated the FSM module to handle and validate mission commands before execution.

### Error 3: Input Permission Synchronization
- **Cause**: Inconsistencies in input permission coordination between the Ground Station and Behavior Planner.
- **Solution**: Synchronized input permissions using message acknowledgement and rate management.

### Error 4: ROS Message Parsing
- **Cause**: Inaccurate parsing of ROS messages leading to data inconsistency.
- **Solution**: Implemented robust message parsing and validation mechanisms in the Telemetry and Ground Station modules.

### Error 5: Finite State Machine Logic
- **Cause**: Improper transitions and logical flaws in the Finite State Machine.
- **Solution**: Refactored FSM logic to ensure correct state transitions and mission execution.

## Conclusion
The development of the autonomous drone control system involved addressing various errors and challenges, ultimately leading to a robust and reliable system for controlling the drone's behavior and movements.

