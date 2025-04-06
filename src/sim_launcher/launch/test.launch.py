from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    sdf_path = "/home/vvoxel/ros2_ws_mgr/gazebo/building_robot_camera.sdf"
    
    apriltag_config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )

    return LaunchDescription([
    
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':/home/vvoxel/ros2_ws_mgr/gazebo/Local_Gazebo_Resources'
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.environ.get('GZ_MODEL_PATH', '') + ':/home/vvoxel/ros2_ws_mgr/gazebo/Local_Gazebo_Resources/models'
        ),

        # GAZEBO - symulacja
        ExecuteProcess(
            cmd=['gz', 'sim', sdf_path],
            output='screen'
        ),

        # ROS-GZ Bridges
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/model/MTracker/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        # Apriltag node
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'apriltag_ros', 'apriltag_node',
                '--ros-args',
                '-r', 'image_rect:=/kamera/image',
                '-r', 'camera_info:=/kamera/camera_info',
                '--params-file', apriltag_config
            ],
            output='screen'
        ),

        # Dodanie opóźnienia, aby uniknąć crasha bridge’a
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                        '/kamera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                    ],
                    output='screen'
                )
            ]
        )
    ])

