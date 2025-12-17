#include "plan_env/bonxai_ros_converter.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>

namespace plan_env {

// 转换Eigen Vector3d到BonxaiPoint
BonxaiRosConverter::BonxaiPoint 
BonxaiRosConverter::toBonxaiPoint(const Point3D& point) {
    return BonxaiPoint(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z())
    );
}

// 转换BonxaiPoint到Eigen Vector3d
BonxaiRosConverter::Point3D 
BonxaiRosConverter::toEigenPoint(const BonxaiPoint& point) {
    return Point3D(point.x, point.y, point.z);
}

std::vector<BonxaiRosConverter::Point3D> 
BonxaiRosConverter::extractPointCloud(
    const sensor_msgs::msg::PointCloud2& msg)
{
    std::vector<Point3D> points;
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    
    points.reserve(pcl_cloud.size());
    for (const auto& point : pcl_cloud) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            points.emplace_back(point.x, point.y, point.z);
        }
    }
    
    return points;
}

std::vector<Bonxai::CoordT> 
BonxaiRosConverter::extractPointCloudAsCoords(
    const sensor_msgs::msg::PointCloud2& msg,
    const VoxelGrid& grid)
{
    std::vector<CoordT> coords;
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    
    coords.reserve(pcl_cloud.size());
    for (const auto& point : pcl_cloud) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            BonxaiPoint bonxai_point(point);
            coords.push_back(grid.posToCoord(bonxai_point.x, bonxai_point.y, bonxai_point.z));
        }
    }
    
    return coords;
}

std::vector<BonxaiRosConverter::BonxaiPoint> 
BonxaiRosConverter::convertPointsToBonxaiPoints(
    const std::vector<Point3D>& points)
{
    std::vector<BonxaiPoint> bonxai_points;
    bonxai_points.reserve(points.size());
    
    for (const auto& point : points) {
        bonxai_points.push_back(toBonxaiPoint(point));
    }
    
    return bonxai_points;
}

std::vector<Bonxai::CoordT> 
BonxaiRosConverter::convertPointsToCoords(
    const std::vector<Point3D>& points,
    const VoxelGrid& grid)
{
    std::vector<CoordT> coords;
    coords.reserve(points.size());
    
    for (const auto& point : points) {
        BonxaiPoint bonxai_point = toBonxaiPoint(point);
        coords.push_back(grid.posToCoord(bonxai_point.x, bonxai_point.y, bonxai_point.z));
    }
    
    return coords;
}

void BonxaiRosConverter::insertPointCloudToMap(
    Bonxai::OccupancyMap& map,
    const std::vector<BonxaiPoint>& points,
    const BonxaiPoint& sensor_origin,
    float max_range)
{
    if (points.empty()) return;
    
    // 使用insertPointCloud API
    std::vector<Bonxai::OccupancyMap::Vector3D> bonxai_points;
    bonxai_points.reserve(points.size());
    
    for (const auto& point : points) {
        bonxai_points.emplace_back(point.x, point.y, point.z);
    }
    
    // 传感器位置
    Bonxai::OccupancyMap::Vector3D sensor_position(
        sensor_origin.x, sensor_origin.y, sensor_origin.z);
    
    // 使用原生API插入点云
    map.insertPointCloud(bonxai_points, sensor_position, static_cast<double>(max_range));
    
    // 增加更新计数
    map.incrementUpdateCount();
}

BonxaiRosConverter::BonxaiPoint 
BonxaiRosConverter::extractSensorPosition(
    const nav_msgs::msg::Odometry& odom)
{
    return BonxaiPoint(
        static_cast<float>(odom.pose.pose.position.x),
        static_cast<float>(odom.pose.pose.position.y),
        static_cast<float>(odom.pose.pose.position.z)
    );
}

Bonxai::CoordT BonxaiRosConverter::extractSensorCoord(
    const nav_msgs::msg::Odometry& odom,
    const VoxelGrid& grid)
{
    BonxaiPoint position = extractSensorPosition(odom);
    return grid.posToCoord(position.x, position.y, position.z);
}

sensor_msgs::msg::PointCloud2::SharedPtr
BonxaiRosConverter::occupancyMapToPointCloud(
    const Bonxai::OccupancyMap& map,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    bool occupied_only)
{
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    
    std::vector<Bonxai::OccupancyMap::Vector3D> points;
    
    if (occupied_only) {
        // 获取占用体素
        map.getOccupiedVoxels(points);
    } else {
        // 获取空闲体素
        map.getFreeVoxels(points);
    }
    
    if (points.empty()) {
        return cloud;
    }
    
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl_cloud.reserve(points.size());
    
    for (const auto& point : points) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        
        if (occupied_only) {
            pcl_point.r = 255;  // 红色 - 占用
            pcl_point.g = 0;
            pcl_point.b = 0;
        } else {
            pcl_point.r = 0;    // 绿色 - 空闲
            pcl_point.g = 255;
            pcl_point.b = 0;
        }
        
        pcl_cloud.push_back(pcl_point);
    }
    
    pcl::toROSMsg(pcl_cloud, *cloud);
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = stamp;
    
    return cloud;
}

void BonxaiRosConverter::getMapStatistics(
    const Bonxai::OccupancyMap& map,
    size_t& occupied_count,
    size_t& free_count,
    size_t& unknown_count)
{
    occupied_count = 0;
    free_count = 0;
    unknown_count = 0;
    
    // 获取占用和空闲体素数量
    std::vector<Bonxai::OccupancyMap::Vector3D> occupied_points, free_points;
    map.getOccupiedVoxels(occupied_points);
    map.getFreeVoxels(free_points);
    
    occupied_count = occupied_points.size();
    free_count = free_points.size();
    
    // 未知体素数量 = 总激活体素 - 占用 - 空闲
    size_t total_cells = map.getActiveCellCount();
    unknown_count = total_cells - occupied_count - free_count;
}

bool BonxaiRosConverter::isOccupied(const Bonxai::MapUtils::CellOcc& cell, 
                                   int32_t threshold)
{
    return cell.probability_log > threshold;
}

bool BonxaiRosConverter::isFree(const Bonxai::MapUtils::CellOcc& cell,
                               int32_t threshold)
{
    return cell.probability_log < threshold;
}

bool BonxaiRosConverter::isUnknown(const Bonxai::MapUtils::CellOcc& cell)
{
    return cell.probability_log == Bonxai::MapUtils::UnknownProbability;
}

} // namespace plan_env