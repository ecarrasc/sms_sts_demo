# SMS/STS Servo ROS 2 Demo Overview

This demo is split into two repositories because two different computers are involved:

- `sms-sts-driver`
  - Runs on the ESP32.
  - Talks directly to the Waveshare SMS/STS servo bus.
  - Uses Arduino, PlatformIO, and the `SCServo` library.
  - Owns low-level servo commands such as position, speed, voltage, temperature, and bus communication.

- `sms_sts_demo`
  - Runs on the ROS 2 computer.
  - Talks to the ESP32 over USB serial.
  - Publishes ROS joint state data for RViz.
  - Subscribes to a ROS command topic and converts radians into raw servo ticks.

## System Data Flow

- ROS 2 command path:
  - User publishes a command:
    - Topic: `/servo/command`
    - Type: `std_msgs/msg/Float64`
    - Unit: radians
  - `sms_sts_one_joint_node` receives the command.
  - The node converts radians to raw servo position ticks.
  - The node sends a serial command to the ESP32:
    - `MOVE <id> <position> <velocity> <acceleration>`
  - The ESP32 receives `MOVE`.
  - The ESP32 calls `SmsStsDriver`.
  - `SmsStsDriver` sends the command to the servo bus.

- ROS 2 feedback path:
  - `sms_sts_one_joint_node` sends a serial read request:
    - `READ <id>`
  - The ESP32 reads the servo state.
  - The ESP32 replies with one serial line:
    - `STATE <id> position=<raw> speed=<raw> voltage=<v> temperature=<c> moving=<0|1> valid=<0|1> reachable=<0|1> last_update_ms=<ms>`
  - The ROS 2 node parses the `STATE` line.
  - The node publishes:
    - Topic: `/joint_states`
    - Type: `sensor_msgs/msg/JointState`
  - `robot_state_publisher` uses `/joint_states` and the URDF to publish TF.
  - RViz displays the moving servo horn.

## ESP32 Firmware Repository

- Location:
  - `/home/ecarrasc/workspaces/c-cpp_ws/sms-sts-driver`

- Important files:
  - `platformio.ini`
    - Defines the ESP32 PlatformIO project.
    - Uses the Arduino framework.
    - Depends on `SCServo@1.0.1`.
    - Uses C++17.
  - `include/SmsStsDriver.hpp`
    - Declares the low-level SMS/STS driver API.
    - Defines:
      - `SmsStsDriver`
      - `ServoState`
      - `ServoCommand`
      - `ControlMode`
      - `DriverState`
  - `src/SmsStsDriver.cpp`
    - Implements direct communication with the servo bus.
    - Uses:
      - `Arduino.h`
      - `SCServo.h`
      - ESP32 UART pins
    - Starts the servo bus with:
      - RX pin: `18`
      - TX pin: `19`
      - Servo bus baudrate: `1000000`
  - `src/main.cpp`
    - Implements the USB serial command interface used by ROS 2.
    - Registers servo IDs `1` and `2`.
    - Parses commands from the PC.

## ESP32 Serial Commands

- Position command:
  - Format:
    - `MOVE <id> <position> <velocity> <acceleration>`
  - Example:
    - `MOVE 1 4000 1500 50`
  - Meaning:
    - Servo ID: `1`
    - Raw target position: `4000`
    - Raw motion velocity: `1500`
    - Acceleration: `50`
  - Firmware behavior:
    - Sets servo to position mode.
    - Stores a position command in the driver.
    - Writes the command to the servo.

- Velocity command:
  - Format:
    - `SPEED <id> <speed> <acceleration>`
  - Example:
    - `SPEED 2 1500 50`
    - `SPEED 1 -1500 50`
  - Firmware behavior:
    - Sets servo to velocity mode.
    - Sends signed speed command.

- Position mode command:
  - Format:
    - `POSITION_MODE <id>`
  - Example:
    - `POSITION_MODE 1`

- Read command:
  - Format:
    - `READ <id>`
  - Example:
    - `READ 1`
  - Example response:
    - `STATE 1 position=4093 speed=0 voltage=11.90 temperature=39.00 moving=0 valid=1 reachable=1 last_update_ms=29746`

## ROS 2 Repository

- Location:
  - `/home/ecarrasc/workspaces/ros2_ws/src/sms_sts_demo`

- Important files:
  - `urdf/one_joint_servo.urdf.xacro`
    - Describes the one-servo robot model.
    - Links:
      - `base_link`
      - `servo_case`
      - `servo_horn`
    - Joint:
      - `servo_output_joint`
  - `rviz/sms_sts.rviz`
    - RViz configuration.
    - Displays the robot model and TF.
  - `launch/display.launch.py`
    - Starts the visual demo.
    - Can run either simulated GUI mode or real hardware mode.
  - `src/sms_sts_one_joint_node.cpp`
    - ROS 2 C++ hardware bridge node.
    - Opens the ESP32 serial port.
    - Sends `MOVE` and `READ` commands.
    - Parses `STATE` responses.
    - Publishes `/joint_states`.

## ROS 2 Node Responsibilities

- `sms_sts_one_joint_node` does not talk directly to the SMS/STS servo bus.
- It does not include:
  - `Arduino.h`
  - `SCServo.h`
  - `SmsStsDriver.hpp`
- It only talks to the ESP32 over Linux serial.
- It converts:
  - ROS command in radians
  - to raw servo ticks
- It converts:
  - raw servo feedback ticks
  - to ROS joint position in radians

## Important ROS 2 Topics

- Command topic:
  - Name: `/servo/command`
  - Type: `std_msgs/msg/Float64`
  - Unit: radians
  - Example:
    - `ros2 topic pub /servo/command std_msgs/msg/Float64 "{data: 1.0}" --once`

- Joint state topic:
  - Name: `/joint_states`
  - Type: `sensor_msgs/msg/JointState`
  - Published by:
    - `sms_sts_one_joint_node` in hardware mode
    - `joint_state_publisher_gui` in GUI mode

- Robot description:
  - Name: `/robot_description`
  - Used by RViz RobotModel display.

## Launch Modes

- GUI-only mode:
  - Does not use hardware.
  - Uses `joint_state_publisher_gui`.
  - Good for checking URDF and RViz.
  - Command:
    - `ros2 launch sms_sts_demo display.launch.py`

- Hardware mode:
  - Uses the ESP32 and real servo.
  - Starts `sms_sts_one_joint_node`.
  - Does not start the GUI slider.
  - Command:
    - `ros2 launch sms_sts_demo display.launch.py use_hardware:=true`

- Hardware mode with explicit serial port:
  - Example:
    - `ros2 launch sms_sts_demo display.launch.py use_hardware:=true serial_port:=/dev/ttyUSB0`

- Hardware mode with slower polling:
  - Useful if the ESP32 serial parser is overloaded.
  - Example:
    - `ros2 launch sms_sts_demo display.launch.py use_hardware:=true update_rate_hz:=50.0 state_request_timeout_ms:=150`

## Build Steps

- Build the ROS 2 package:
  - `cd /home/ecarrasc/workspaces/ros2_ws`
  - `source /opt/ros/jazzy/setup.zsh`
  - `colcon build --packages-select sms_sts_demo`
  - `source install/setup.zsh`

- Launch RViz with hardware:
  - `ros2 launch sms_sts_demo display.launch.py use_hardware:=true`

- Send a command:
  - `ros2 topic pub /servo/command std_msgs/msg/Float64 "{data: 1.0}" --once`

- Check feedback:
  - `ros2 topic echo /joint_states`

## Unit Conversion

- The ROS 2 node assumes:
  - Raw center position: `2048`
  - Raw range: `0` to `4095`
  - One full turn: approximately `4095` ticks
  - Default scale:
    - `-4095 / (2*pi)` ticks per radian
    - The negative sign maps positive ROS rotation to the physical servo's
      positive rotor direction.

- Raw to radians:
  - `position_rad = (raw_position - raw_center) / raw_ticks_per_rad`

- Radians to raw:
  - `raw_position = raw_center + position_rad * raw_ticks_per_rad`

- If the commanded physical direction is opposite to the desired ROS direction:
  - Reverse the sign of `raw_ticks_per_rad`.

- If RViz zero does not match the real horn zero:
  - Adjust `raw_center`.

## Common Problems

- RViz says no transform from `servo_horn` to `base_link`:
  - `/joint_states` is probably missing.
  - Check:
    - `ros2 topic echo /joint_states`
  - The ESP32 must answer `READ <id>` with a valid `STATE ...` line.

- RViz shows the servo but it does not move:
  - The read path works, but the command path may be wrong.
  - Confirm the ROS node sends:
    - `MOVE <id> <position> <velocity> <acceleration>`
  - Confirm the ESP32 serial monitor accepts the same command.

- The servo moves but RViz is delayed:
  - Increase `update_rate_hz`.
  - Increase RViz frame rate.
  - Do not flood the ESP32 with too many `READ` requests.

- The servo stops reacting after increasing update rate:
  - The ESP32 may be overloaded by repeated `READ` commands.
  - Lower the polling rate:
    - `update_rate_hz:=50.0`
  - Increase request timeout:
    - `state_request_timeout_ms:=150`

- RViz movement does not match physical movement:
  - Calibrate:
    - `raw_center`
    - `raw_ticks_per_rad`
  - Direction can be inverted by making `raw_ticks_per_rad` negative.

## Teaching Summary

- The ESP32 is the hardware controller.
- The ROS 2 computer is the robot software layer.
- The serial protocol is the boundary between them.
- The firmware should stay responsible for direct servo bus communication.
- The ROS 2 node should stay responsible for ROS topics, units, URDF state, and RViz integration.
- Keeping this separation makes it easier to extend the demo later to:
  - two servos
  - a differential drive robot
  - velocity control
  - odometry
  - `ros2_control`
