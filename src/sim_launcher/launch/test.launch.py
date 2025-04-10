from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    sdf_path = "/home/vvoxel/ros2_ws_mgr/gazebo/building_robot_camera.sdf"
    
    apriltag_config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )

    return LaunchDescription([

        # GAZEBO - symulacja
        ExecuteProcess(
            cmd=['gz', 'sim', sdf_path],
            output='screen'
        ),

        # ROS-GZ Bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/MTracker/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            remappings=[
                ('/model/MTracker/cmd_vel', '/cmd_vel')
            ]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/kamera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/kamera/camera_info', '/kamera/camera_info')
            ]
        ),
        
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/kamera/image']
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/MTracker/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            remappings=[
                ('/model/MTracker/tf', '/tf')
            ]
        ),

        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])

