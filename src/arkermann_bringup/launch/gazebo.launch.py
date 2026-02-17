from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory("arkermann_bringup")

    # World file path
    world_path = os.path.join(
        pkg_path,
        "gazebo",
        "maze.world"
    )

    # Robot URDF
    urdf_path = os.path.join(
        get_package_share_directory("arkermann_description"),
        "urdf",
        "arkermann_car.urdf"
    )

    # ⭐ Tell Gazebo where models are
    gazebo_model_path = os.path.join(pkg_path, "gazebo")

    # Set environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    return LaunchDescription([

        set_gazebo_model_path,

        gazebo,

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'arkermann_car',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '0.2'
            ],
            output='screen'
        )
    ])

