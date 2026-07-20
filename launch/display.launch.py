from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sms_sts_demo")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gui = LaunchConfiguration("use_gui")
    use_hardware = LaunchConfiguration("use_hardware")
    servo_id = LaunchConfiguration("servo_id")
    joint_name = LaunchConfiguration("joint_name")
    command_topic = LaunchConfiguration("command_topic")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    read_command = LaunchConfiguration("read_command")
    position_command = LaunchConfiguration("position_command")
    update_rate_hz = LaunchConfiguration("update_rate_hz")
    state_request_timeout_ms = LaunchConfiguration("state_request_timeout_ms")
    use_gui_without_hardware = PythonExpression([
        "'",
        use_gui,
        "' == 'true' and '",
        use_hardware,
        "' != 'true'",
    ])
    use_joint_state_publisher_without_hardware = PythonExpression([
        "'",
        use_gui,
        "' != 'true' and '",
        use_hardware,
        "' != 'true'",
    ])

    robot_description = {
        "robot_description": Command(["xacro ", model])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([
                package_share,
                "urdf",
                "one_joint_servo.urdf.xacro",
            ]),
            description="Absolute path to the one-joint servo xacro model.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                package_share,
                "rviz",
                "sms_sts.rviz",
            ]),
            description="Absolute path to the RViz config file.",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for interactive joint motion.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Start the SMS/STS hardware node instead of the joint-state GUI.",
        ),
        DeclareLaunchArgument(
            "servo_id",
            default_value="1",
            description="SMS/STS servo bus ID.",
        ),
        DeclareLaunchArgument(
            "joint_name",
            default_value="servo_output_joint",
            description="URDF joint controlled by the SMS/STS servo.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="servo/command",
            description="Float64 position command topic in radians.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial device connected to the ESP32 firmware bridge.",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial baud rate used by the ESP32 firmware bridge.",
        ),
        DeclareLaunchArgument(
            "read_command",
            default_value="READ",
            description="Command word sent to request servo state from the ESP32.",
        ),
        DeclareLaunchArgument(
            "position_command",
            default_value="MOVE",
            description="Command word sent to write a target position to the ESP32.",
        ),
        DeclareLaunchArgument(
            "update_rate_hz",
            default_value="100.0",
            description="Rate in Hz for requesting state from the ESP32.",
        ),
        DeclareLaunchArgument(
            "state_request_timeout_ms",
            default_value="100",
            description="Timeout before retrying an unanswered state request.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(use_gui_without_hardware),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(use_joint_state_publisher_without_hardware),
        ),
        Node(
            package="sms_sts_demo",
            executable="sms_sts_one_joint_node",
            name="sms_sts_one_joint_node",
            output="screen",
            condition=IfCondition(use_hardware),
            parameters=[{
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "servo_id": servo_id,
                "joint_name": joint_name,
                "command_topic": command_topic,
                "read_command": read_command,
                "position_command": position_command,
                "update_rate_hz": update_rate_hz,
                "state_request_timeout_ms": state_request_timeout_ms,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
