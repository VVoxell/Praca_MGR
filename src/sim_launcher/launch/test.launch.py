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
    
    rviz_config = os.path.join(os.getcwd(), 'rviz_config', 'rviz_cfg.rviz')

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
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/MTracker/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            remappings=[
                ('/model/MTracker/odometry', '/odom')
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera',
            output='screen',
            arguments=[
                '0.025', '0', '0.06',    
                '0', '-0.2', '0',         
                'MTracker/chassis',      
                'MTracker/camera/rgbd_camera' 
            ]
        ),
        
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[apriltag_config],
            remappings=[
                ('image_rect', '/kamera/image'),
                ('camera_info', '/kamera/camera_info')
            ]
        ),


        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])

