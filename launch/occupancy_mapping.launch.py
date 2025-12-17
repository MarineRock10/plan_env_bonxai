from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 集中式参数配置
    occupancy_map_params = {
        # 网格参数
        'resolution': 0.05,
        'inflation_radius': 0.15,
        'prob_hit': 0.7,
        'prob_miss': 0.4,
        'clamp_min': 0.12,
        'clamp_max': 0.97,
        'occupancy_threshold': 0.5,
        
        # 传感器参数
        'max_range': 30.0,
        'min_range': 0.1,
        'global_frame': 'odom',
        'sensor_frame': 'base_link',
        
        # 发布参数
        'publish_rate': 10.0,
        'publish_occupied': True,
        
        # 性能参数
        'max_queue_size': 10,
        'odom_timeout': 0.1,
    }

    # 节点定义
    occupancy_map_node = Node(
        package='plan_env',
        executable='occupancy_map_node',
        name='occupancy_map_node',
        output='screen',
        parameters=[occupancy_map_params],
    )

    return LaunchDescription([
        occupancy_map_node
    ])