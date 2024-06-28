from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    #  pkg path
    pkg_tb4_viz = get_package_share_directory('turtlebot4_viz') 
    pkg_tb4_navigation = get_package_share_directory('turtlebot4_navigation') 

    wp_name = LaunchConfiguration('wp_name')
    wp_name_arg = DeclareLaunchArgument(
        'wp_name',
        default_value='wp',
        description='Name of the waypoint'
    )
    
    wp_move_node = Node(
        package='aaqr_ros2_py',
        executable='waypoints_move_node',
        name='wp_move_node',
        emulate_tty=True,
        parameters=[{'wp_name': wp_name}],
        output="screen",
        prefix=["xterm -e"]
    )

    pose_pub_node = Node(
            package='aaqr_ros2_py',
            executable='framelistener',
            name='framelistener',
            output='log'
        )    
    
    map_arg = LaunchConfiguration('map')

    turtlebot4_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_tb4_navigation, 'launch', 'localization.launch.py'])]
        ),
        launch_arguments={'map': map_arg}.items(),
        
    )
    
    turtlebot4_viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_tb4_viz, 'launch', 'view_robot.launch.py'])]
        )
    )
    # Nav2
    turtlebot4_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_tb4_navigation, 'launch', 'nav2.launch.py'])]
        )
    )
    
    return LaunchDescription([
        turtlebot4_viz,
        turtlebot4_nav2,
        pose_pub_node,
        turtlebot4_localization,
        wp_name_arg,
        wp_move_node
    ])
