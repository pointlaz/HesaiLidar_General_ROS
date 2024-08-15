from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcap_file', default_value=''),
        DeclareLaunchArgument('server_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('lidar_recv_port', default_value='2368'),
        DeclareLaunchArgument('gps_port', default_value='10110'),
        DeclareLaunchArgument('start_angle', default_value='0.0'),
        DeclareLaunchArgument('lidar_type', default_value='PandarXT-32'),
        DeclareLaunchArgument('frame_id', default_value='PandarXT-32'),
        DeclareLaunchArgument('pcldata_type', default_value='0'),
        DeclareLaunchArgument('publish_type', default_value='points'),
        DeclareLaunchArgument('timestamp_type', default_value=''),
        DeclareLaunchArgument('data_type', default_value=''),
        DeclareLaunchArgument('namespace', default_value='hesai'),
        DeclareLaunchArgument('lidar_correction_file', default_value=[ThisLaunchFileDir(), '/config/PandarXT-32.csv']),
        DeclareLaunchArgument('multicast_ip', default_value=''),
        DeclareLaunchArgument('coordinate_correction_flag', default_value='false'),
        DeclareLaunchArgument('fixed_frame', default_value=''),
        DeclareLaunchArgument('target_frame', default_value=''),

        Node(
            package='hesai_lidar',
            executable='hesai_lidar_node',
            name='standby_hesai_lidar',
            output='screen',
            parameters=[{
                'pcap_file': LaunchConfiguration('pcap_file'),
                'server_ip': LaunchConfiguration('server_ip'),
                'lidar_recv_port': LaunchConfiguration('lidar_recv_port'),
                'gps_port': LaunchConfiguration('gps_port'),
                'start_angle': LaunchConfiguration('start_angle'),
                'lidar_type': LaunchConfiguration('lidar_type'),
                'frame_id': LaunchConfiguration('frame_id'),
                'pcldata_type': LaunchConfiguration('pcldata_type'),
                'publish_type': LaunchConfiguration('publish_type'),
                'timestamp_type': LaunchConfiguration('timestamp_type'),
                'data_type': LaunchConfiguration('data_type'),
                'lidar_correction_file': LaunchConfiguration('lidar_correction_file'),
                'multicast_ip': LaunchConfiguration('multicast_ip'),
                'coordinate_correction_flag': LaunchConfiguration('coordinate_correction_flag'),
                'fixed_frame': LaunchConfiguration('fixed_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
            }]
        ),
    ])
