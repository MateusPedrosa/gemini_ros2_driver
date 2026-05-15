from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('live_mode',      default_value='True'),
        DeclareLaunchArgument('log_file_path',  default_value='/workspace/build/sim_test/sample.glf'),
        DeclareLaunchArgument('record_path',    default_value=''),
        DeclareLaunchArgument('frame_id',       default_value='sonar_link'),

        Node(
            package='gemini_ros2',
            executable='gemini_sonar_node',
            name='gemini_sonar_node',
            output='screen',
            parameters=[{
                # Environment Settings
                'live_mode':      LaunchConfiguration('live_mode'),
                'log_file_path':  LaunchConfiguration('log_file_path'),
                'record_path':    LaunchConfiguration('record_path'),
                'frame_id':       LaunchConfiguration('frame_id'),

                # Core Acoustic Controls
                'range': 20.0,
                'gain': 50,
                'ping_mode': 0,

                # Gemini 1200iK Specific Controls
                'frequency_mode': 0,
                'range_threshold': 20.0,
                'high_resolution': True,
                'chirp_mode': 2,

                # Speed of Sound
                'use_manual_sos': False,
                'manual_sos': 1500.0,

                # Image & Hardware
                'sonar_inverted': False,
                'aperture': 120.0,
                'image_quality': 3,

                # Playback
                'loop_playback': False,
                'playback_speed': 1,
            }]
        )
    ])
