# plan_env_bonxai - Bonxai-based 2D Occupancy Mapping for ROS 2

A lightweight ROS 2 package for real-time 2D occupancy mapping using the Bonxai sparse voxel grid library. Designed for robotic navigation with minimal dependencies and efficient memory usage.

## Features

- **2D Occupancy Mapping**: Maintains occupancy probability at z=0 plane
- **Sparse Storage**: Uses Bonxai sparse voxel grid for memory efficiency
- **Real-time Updates**: Processes LiDAR point clouds with low latency
- **Probability-based**: Log-odds probability updates
- **Obstacle Inflation**: Configurable safety margins
- **No TF Dependencies**: Simplified architecture
- **Thread-safe Queries**: Safe for concurrent access

## Installation

### Prerequisites
- ROS 2 (Humble or newer)
- C++17 compiler
- PCL and Eigen3 libraries

### Build from Source
```bash
# Clone this repository
mkdir -p ~/plan_env_ws/src
cd ~/plan_env_ws/src
git clone <repository-url> plan_env

# Build
cd ~/plan_env_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch the Node
```bash
ros2 launch plan_env occupancy_mapping.launch.py
```

## Configuration

### Input Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/car_pc2` | `sensor_msgs/msg/PointCloud2` | LiDAR point cloud (global frame) |
| `/car_odom` | `nav_msgs/msg/Odometry` | Robot odometry for sensor position |

### Output Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/occupancy_map/occupied` | `sensor_msgs/msg/PointCloud2` | Occupied cells (red points) |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | 0.05 m | Voxel resolution |
| `inflation_radius` | 0.1 m | Obstacle inflation radius |
| `max_range` | 30.0 m | Maximum sensor range |
| `min_range` | 0.1 m | Minimum sensor range |
| `publish_rate` | 10.0 Hz | Visualization publish rate |
| `prob_hit` | 0.7 | Probability update for hits |
| `prob_miss` | 0.4 | Probability update for misses |

## Architecture Overview

### Core Components
1. **OccupancyMap**: Bonxai-based sparse voxel grid with probability updates
2. **BonxaiRosConverter**: Data conversion between ROS and Bonxai formats
3. **OccupancyMapNode**: ROS 2 node with sensor data processing

## File Structure
```
plan_env/
├── CMakeLists.txt
├── include/
│   ├── bonxai_core/          # Bonxai library headers
│   ├── bonxai_map/           # Core mapping classes
│   └── plan_env/             # ROS integration
├── launch/
│   └── occupancy_mapping.launch.py
├── package.xml
└── src/
    ├── bonxai_ros_converter.cpp
    ├── occupancy_map_node.cpp
    └── main.cpp
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Bonxai Library](https://github.com/felipe-tovar-henao/Bonxai) for sparse voxel grid
- ROS 2 community for the robotics framework



