# 项目基础设置
cmake_minimum_required(VERSION 3.12)
project(plan_env)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

# 创建共享库
add_library(${PROJECT_NAME} SHARED
  src/occupancy_map_node.cpp
  src/bonxai_ros_converter.cpp
)

# 为库设置Ament依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# 链接第三方库
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# 设置包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 创建可执行文件
add_executable(occupancy_map_node
  src/main.cpp
)

# 为可执行文件设置Ament依赖
ament_target_dependencies(occupancy_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# 链接到本项目的库
target_link_libraries(occupancy_map_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# 设置RPATH
set_target_properties(occupancy_map_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
)

# --- 安装部分 ---
# 1. 安装库文件（到标准lib目录）
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. 安装可执行文件
install(TARGETS occupancy_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# 3. 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 4. 安装launch文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch/")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# 5. 安装package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- 导出配置 ---
# 安装targets导出文件
install(EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# 导出包含目录和依赖
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  Eigen3
  PCL
)

# 必须调用
ament_package()