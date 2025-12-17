#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bonxai_map/occupancy_map.hpp"
#include "plan_env/bonxai_ros_converter.hpp"

namespace plan_env {

/**
 * @brief 简化版占用地图节点
 * 移除TF依赖，只保留点云可视化
 */
class OccupancyMapNode : public rclcpp::Node {
public:
    explicit OccupancyMapNode(const std::string& node_name, 
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~OccupancyMapNode() override;
    
    // 禁用拷贝
    OccupancyMapNode(const OccupancyMapNode&) = delete;
    OccupancyMapNode& operator=(const OccupancyMapNode&) = delete;

    bool isOccupied(const Eigen::Vector2d& pos, 
                                     double z_height = 0.0) const noexcept;
    
private:
    // 参数结构
    struct Parameters {
        // 网格参数
        double resolution{0.05};
        double inflation_radius{0.1};
        float prob_hit{0.7f};
        float prob_miss{0.4f};
        float clamp_min{0.12f};
        float clamp_max{0.97f};
        float occupancy_threshold{0.5f};
        
        // 传感器参数
        float max_range{30.0f};
        float min_range{0.1f};
        std::string global_frame{"map"};  // 保留用于输出
        std::string sensor_frame{"base_link"};  // 保留但不使用
        
        // 发布参数
        float publish_rate{1.0f};
        bool publish_occupied{true};
        
        // 性能参数
        int max_queue_size{10};
        float odom_timeout{0.1f};
    };
    
    // 数据队列项
    struct SensorData {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud;
        nav_msgs::msg::Odometry::SharedPtr odom;
        rclcpp::Time timestamp;
    };
    
    // 统计信息
    struct Statistics {
        std::atomic<size_t> total_points{0};
        std::atomic<size_t> total_updates{0};
        std::atomic<size_t> queue_size{0};
        std::atomic<double> avg_process_time{0.0};
        size_t occupied_count{0};
        size_t free_count{0};
        size_t unknown_count{0};
    };
    
    // 初始化
    void initialize();
    void loadParameters();
    void setupInterfaces();
    
    // 数据回调
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    
    // 处理线程
    void processingThread();
    void processSensorData(SensorData&& data);
    
    // 地图更新
    void updateOccupancyMap(
        const std::vector<Bonxai::CoordT>& points,
        const Bonxai::CoordT& sensor_coord);
    
    // 发布
    void publishVisualization();
    void publishStatistics();

    // 查询
    bool queryVoxel(
        const Bonxai::CoordT& coord,
        Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>::Accessor& accessor) const noexcept;
    
    // 工具函数
    std::vector<Bonxai::CoordT> transformAndFilterPoints(
        const sensor_msgs::msg::PointCloud2& cloud,
        const nav_msgs::msg::Odometry& odom);
    
    // 成员变量
    Parameters params_;
    std::unique_ptr<Bonxai::OccupancyMap> occupancy_map_;
    
    // ROS接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // 数据队列
    std::queue<SensorData> data_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // 处理线程
    std::thread processing_thread_;
    std::atomic<bool> running_{true};
    
    // 统计
    Statistics stats_;
};

} // namespace plan_env