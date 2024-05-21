from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    video_device = LaunchConfiguration('video_device')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    video_device_cmd = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='path to camera file'
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'video_device': video_device,
            'image': '[640,480]',
            'camera_frame_id': 'camera_link_optical'
        }],
        # remappings=[('/image_raw', '/camera/image_raw')]
    )

    ld = LaunchDescription()
    ld.add_action(camera_node)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(video_device_cmd)

    return ld
