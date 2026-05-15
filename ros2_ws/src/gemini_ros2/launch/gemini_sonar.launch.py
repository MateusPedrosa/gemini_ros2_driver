from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gemini_ros2',
            executable='gemini_sonar_node',
            name='gemini_sonar_node',
            output='screen',
            parameters=[{
                # Environment Settings
                'live_mode': False, # Set to True when connected to the physical 1200iK
                'log_file_path': '/workspace/build/sim_test/sample.glf',

                # Core Acoustic Controls
                'range': 20.0, # Max distance in meters
                'gain': 50,    # Receiver gain (1-100%)
                'ping_mode': 0, # 0: Free Run, 1: Fixed Interval, 2: Ext TTL, 3: Manual

                # Gemini 1200iK Specific Controls
                'frequency_mode': 0, # 0: Auto, 1: Low (720kHz), 2: High (1.2MHz)
                'range_threshold': 20.0, # Range in meters to trigger Auto-Frequency switch
                'high_resolution': True, # Doubles the range lines
                'chirp_mode': 2, # 0: Disabled, 1: Enabled, 2: Auto
                
                # Speed of Sound
                'use_manual_sos': False,
                'manual_sos': 1500.0, # m/s
            }]
        )
    ])