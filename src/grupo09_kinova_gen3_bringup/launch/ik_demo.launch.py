from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('kortex_description'), 'robots', 'kinova.urdf.xacro'
        ]), ' ',
        'robot_ip:=xxx.yyy.zzz.www ',
        'name:=kinova ',
        'arm:=gen3 ',
        'gripper:="" ',
        'dof:=6 ',
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('kortex_description'), 'rviz', 'view_robot.rviz'
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),

        Node(
            package='grupo09_kinova_gen3_kinematics',
            executable='ik_node',
            output='screen',
        ),

        Node(
            package='grupo09_kinova_gen3_kinematics',
            executable='fk_node',
            output='screen',
        ),
    ])
