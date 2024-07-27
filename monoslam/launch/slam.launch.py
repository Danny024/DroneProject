from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # video_pub = Node(
    #     package="monoslam",
    #     executable="vod_img_pub",  # Ensure this matches the entry point defined in your setup.py or CMakeLists.txt
    #     name='vod_img_pub',  # Optional, but useful for debugging
    #     arguments=['src/slam/resource/videos/test_ohio.mp4'],
    #     output='screen',
    #     emulate_tty=True,  # Ensures proper handling of console output
    # )

    slam = Node(
            package='monoslam',
            executable='slam',
            name='slam',
            output='screen',
            emulate_tty=True,  # Optional, useful for debugging
        )
    

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",  # Ensure this matches the entry point defined in your setup.py or CMakeLists.txt
    #     output='screen',
    #     arguments=['-d' + 'src/slam/rviz/py_slam_ros2_v4.rviz']
    # )


    # ld.add_action(video_pub)
    ld.add_action(slam)
    # ld.add_action(rviz)

    return ld
