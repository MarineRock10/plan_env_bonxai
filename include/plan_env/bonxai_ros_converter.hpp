/*
    MIT License

    Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "bonxai_map/occupancy_map.hpp"

namespace plan_env {

class BonxaiRosConverter {
public:
    // 使用Bonxai的实际类型
    using VoxelGrid = Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>;
    using Accessor = VoxelGrid::Accessor;
    using CoordT = Bonxai::CoordT;
    
    // Bonxai使用简单的结构体
    struct BonxaiPoint {
        float x, y, z;
        BonxaiPoint() : x(0), y(0), z(0) {}
        BonxaiPoint(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
        explicit BonxaiPoint(const pcl::PointXYZ& p) : x(p.x), y(p.y), z(p.z) {}
    };
    
    // 用于ROS转换的Eigen点
    using Point3D = Eigen::Vector3d;
    
    /**
     * @brief 从ROS点云提取为Eigen点
     */
    static std::vector<Point3D> extractPointCloud(
        const sensor_msgs::msg::PointCloud2& msg);
    
    /**
     * @brief 从ROS点云提取为Bonxai坐标
     */
    static std::vector<CoordT> extractPointCloudAsCoords(
        const sensor_msgs::msg::PointCloud2& msg,
        const VoxelGrid& grid);
    
    /**
     * @brief 从Eigen点转换为Bonxai点
     */
    static std::vector<BonxaiPoint> convertPointsToBonxaiPoints(
        const std::vector<Point3D>& points);
    
    /**
     * @brief 从Eigen点转换为Bonxai坐标
     */
    static std::vector<CoordT> convertPointsToCoords(
        const std::vector<Point3D>& points,
        const VoxelGrid& grid);
    
    /**
     * @brief 插入点云到占用地图
     */
    static void insertPointCloudToMap(
        Bonxai::OccupancyMap& map,
        const std::vector<BonxaiPoint>& points,
        const BonxaiPoint& sensor_origin,
        float max_range);
    
    /**
     * @brief 从里程计提取传感器位置
     */
    static BonxaiPoint extractSensorPosition(
        const nav_msgs::msg::Odometry& odom);
    
    /**
     * @brief 从里程计提取传感器坐标
     */
    static CoordT extractSensorCoord(
        const nav_msgs::msg::Odometry& odom,
        const VoxelGrid& grid);
    
    /**
     * @brief 创建占用网格的点云可视化
     */
    static sensor_msgs::msg::PointCloud2::SharedPtr
    occupancyMapToPointCloud(
        const Bonxai::OccupancyMap& map,
        const std::string& frame_id = "map",
        const rclcpp::Time& stamp = rclcpp::Time(),
        bool occupied_only = true);
    
    /**
     * @brief 获取地图统计信息
     */
    static void getMapStatistics(
        const Bonxai::OccupancyMap& map,
        size_t& occupied_count,
        size_t& free_count,
        size_t& unknown_count);
    
    /**
     * @brief 检查体素是否被占用
     */
    static bool isOccupied(const Bonxai::MapUtils::CellOcc& cell, 
                          int32_t threshold);
    
    /**
     * @brief 检查体素是否空闲
     */
    static bool isFree(const Bonxai::MapUtils::CellOcc& cell,
                      int32_t threshold);
    
    /**
     * @brief 检查体素是否未知
     */
    static bool isUnknown(const Bonxai::MapUtils::CellOcc& cell);
    
private:
    // 转换辅助函数
    static BonxaiPoint toBonxaiPoint(const Point3D& point);
    static Point3D toEigenPoint(const BonxaiPoint& point);
};

} // namespace plan_env