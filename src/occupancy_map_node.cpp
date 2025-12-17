#include "plan_env/occupancy_map_node.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace plan_env {

OccupancyMapNode::OccupancyMapNode(const std::string& node_name, 
                                   const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options) {
    initialize();
}

OccupancyMapNode::~OccupancyMapNode() {
    running_ = false;
    queue_cv_.notify_all();
    
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    RCLCPP_INFO(get_logger(), "OccupancyMapNode shutdown");
}

void OccupancyMapNode::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing OccupancyMapNode...");
    
    // 1. 加载参数
    loadParameters();
    
    // 2. 创建占用地图
    Bonxai::MapUtils::OccupancyOptions map_options;
    map_options.resolution = params_.resolution;
    map_options.inflation_radius = params_.inflation_radius;
    map_options.prob_hit_log = Bonxai::MapUtils::logods(params_.prob_hit);
    map_options.prob_miss_log = Bonxai::MapUtils::logods(params_.prob_miss);
    map_options.clamp_min_log = Bonxai::MapUtils::logods(params_.clamp_min);
    map_options.clamp_max_log = Bonxai::MapUtils::logods(params_.clamp_max);
    map_options.occupancy_threshold_log = Bonxai::MapUtils::logods(params_.occupancy_threshold);
    
    occupancy_map_ = std::make_unique<Bonxai::OccupancyMap>(
        static_cast<double>(params_.resolution), map_options);
    
    RCLCPP_INFO(get_logger(), "Created occupancy map with resolution: %.3fm", 
                params_.resolution);
    
    // 3. 设置ROS接口
    setupInterfaces();
    
    // 4. 启动处理线程
    processing_thread_ = std::thread(&OccupancyMapNode::processingThread, this);
    
    RCLCPP_INFO(get_logger(), "OccupancyMapNode initialized");
}

void OccupancyMapNode::loadParameters() {
    RCLCPP_INFO(get_logger(), "Loading parameters...");
    
    // 方法1：直接声明并获取
    this->declare_parameter("resolution", 0.05);
    params_.resolution = this->get_parameter("resolution").as_double();
    
    this->declare_parameter("inflation_radius", 0.1);
    params_.inflation_radius = this->get_parameter("inflation_radius").as_double();
    
    this->declare_parameter("prob_hit", 0.7);
    params_.prob_hit = static_cast<float>(this->get_parameter("prob_hit").as_double());
    
    this->declare_parameter("prob_miss", 0.4);
    params_.prob_miss = static_cast<float>(this->get_parameter("prob_miss").as_double());
    
    this->declare_parameter("clamp_min", 0.12);
    params_.clamp_min = static_cast<float>(this->get_parameter("clamp_min").as_double());
    
    this->declare_parameter("clamp_max", 0.97);
    params_.clamp_max = static_cast<float>(this->get_parameter("clamp_max").as_double());
    
    this->declare_parameter("occupancy_threshold", 0.5);
    params_.occupancy_threshold = static_cast<float>(this->get_parameter("occupancy_threshold").as_double());
    
    this->declare_parameter("max_range", 30.0);
    params_.max_range = static_cast<float>(this->get_parameter("max_range").as_double());
    
    this->declare_parameter("min_range", 0.1);
    params_.min_range = static_cast<float>(this->get_parameter("min_range").as_double());
    
    this->declare_parameter("global_frame", "odom");
    params_.global_frame = this->get_parameter("global_frame").as_string();
    
    this->declare_parameter("sensor_frame", "base_link");
    params_.sensor_frame = this->get_parameter("sensor_frame").as_string();
    
    this->declare_parameter("publish_rate", 10.0);
    params_.publish_rate = static_cast<float>(this->get_parameter("publish_rate").as_double());
    
    this->declare_parameter("publish_occupied", true);
    params_.publish_occupied = this->get_parameter("publish_occupied").as_bool();
    
    this->declare_parameter("max_queue_size", 10);
    params_.max_queue_size = this->get_parameter("max_queue_size").as_int();
    
    this->declare_parameter("odom_timeout", 0.1);
    params_.odom_timeout = static_cast<float>(this->get_parameter("odom_timeout").as_double());
    
    RCLCPP_INFO(get_logger(), "✅ Parameters loaded successfully");
    
    RCLCPP_INFO(get_logger(), "Parameters loaded:");
    RCLCPP_INFO(get_logger(), "  [Grid]");
    RCLCPP_INFO(get_logger(), "    Resolution: %.3f m", params_.resolution);
    RCLCPP_INFO(get_logger(), "    Inflation radius: %.3f m", params_.inflation_radius);
    RCLCPP_INFO(get_logger(), "    Prob hit: %.2f", params_.prob_hit);
    RCLCPP_INFO(get_logger(), "    Prob miss: %.2f", params_.prob_miss);
    RCLCPP_INFO(get_logger(), "    Clamp min: %.2f", params_.clamp_min);
    RCLCPP_INFO(get_logger(), "    Clamp max: %.2f", params_.clamp_max);
    RCLCPP_INFO(get_logger(), "    Occupancy threshold: %.2f", params_.occupancy_threshold);
    
    RCLCPP_INFO(get_logger(), "  [Sensor]");
    RCLCPP_INFO(get_logger(), "    Max range: %.1f m", params_.max_range);
    RCLCPP_INFO(get_logger(), "    Min range: %.1f m", params_.min_range);
    RCLCPP_INFO(get_logger(), "    Global frame: %s", params_.global_frame.c_str());
    RCLCPP_INFO(get_logger(), "    Sensor frame: %s", params_.sensor_frame.c_str());
    
    RCLCPP_INFO(get_logger(), "  [Publish]");
    RCLCPP_INFO(get_logger(), "    Publish rate: %.1f Hz", params_.publish_rate);
    RCLCPP_INFO(get_logger(), "    Publish occupied: %s", 
                params_.publish_occupied ? "true" : "false");
    
    RCLCPP_INFO(get_logger(), "  [Performance]");
    RCLCPP_INFO(get_logger(), "    Max queue size: %d", params_.max_queue_size);
    RCLCPP_INFO(get_logger(), "    Odom timeout: %.3f s", params_.odom_timeout);
}



void OccupancyMapNode::setupInterfaces() {
    // 订阅器
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "car_pc2", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pointCloudCallback(std::move(msg));
        });
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "car_odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            odomCallback(std::move(msg));
        });
    
    // 发布器 - 只发布点云
    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "occupancy_map/occupied", 1);
    
    // 定时器
    if (params_.publish_rate > 0) {
        auto period = std::chrono::duration<double>(1.0 / params_.publish_rate);
        publish_timer_ = create_wall_timer(period, [this]() {
            publishVisualization();
        });
    }
    
    // 统计定时器
    // stats_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    //     publishStatistics();
    // });
}

void OccupancyMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (data_queue_.size() >= static_cast<size_t>(params_.max_queue_size)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Queue full (%zu), dropping point cloud",
                           data_queue_.size());
        data_queue_.pop();
    }
    
    SensorData data;
    data.cloud = std::move(msg);
    data.timestamp = data.cloud->header.stamp;
    data.odom = nullptr;
    
    data_queue_.push(std::move(data));
    stats_.queue_size = data_queue_.size();
    
    queue_cv_.notify_one();
}

void OccupancyMapNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (data_queue_.empty()) return;
    
    rclcpp::Time odom_time = msg->header.stamp;
    std::queue<SensorData> temp_queue;
    bool matched = false;
    
    while (!data_queue_.empty()) {
        auto data = std::move(data_queue_.front());
        data_queue_.pop();
        
        double time_diff = std::abs((odom_time - data.timestamp).seconds());
        
        if (time_diff <= params_.odom_timeout && !matched) {
            data.odom = msg;
            matched = true;
            temp_queue.push(std::move(data));
        } else if (odom_time > data.timestamp) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Dropping old point cloud (time diff: %.3fs)", 
                                time_diff);
        } else {
            temp_queue.push(std::move(data));
        }
    }
    
    data_queue_ = std::move(temp_queue);
    stats_.queue_size = data_queue_.size();
    
    if (matched) {
        queue_cv_.notify_one();
    }
}

void OccupancyMapNode::processingThread() {
    RCLCPP_INFO(get_logger(), "Processing thread started");
    
    while (rclcpp::ok() && running_) {
        SensorData data;
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                if (!running_) return true;
                if (data_queue_.empty()) return false;
                
                // 检查是否有完整的传感器数据
                std::queue<SensorData> temp = data_queue_;
                while (!temp.empty()) {
                    if (temp.front().odom != nullptr) return true;
                    temp.pop();
                }
                return false;
            });
            
            if (!running_) break;
            
            // 获取第一个完整的数据
            while (!data_queue_.empty()) {
                if (data_queue_.front().odom != nullptr) {
                    data = std::move(data_queue_.front());
                    data_queue_.pop();
                    break;
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                       "Dropping point cloud without odometry");
                    data_queue_.pop();
                }
            }
            
            stats_.queue_size = data_queue_.size();
        }
        
        if (data.cloud && data.odom) {
            processSensorData(std::move(data));
        }
    }
    
    RCLCPP_INFO(get_logger(), "Processing thread stopped");
}

void OccupancyMapNode::processSensorData(SensorData&& data) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 1. 转换和过滤点
        auto points = transformAndFilterPoints(*data.cloud, *data.odom);
        if (points.empty()) return;
        
        // 2. 提取传感器坐标
        auto sensor_coord = BonxaiRosConverter::extractSensorCoord(
            *data.odom, occupancy_map_->getGrid());
        
        // 3. 更新占用地图
        updateOccupancyMap(points, sensor_coord);
        
        // 4. 更新统计
        auto end_time = std::chrono::high_resolution_clock::now();
        double process_time = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
        
        stats_.total_updates++;
        stats_.total_points += points.size();
        stats_.avg_process_time = (stats_.avg_process_time * (stats_.total_updates - 1) + 
                                   process_time) / stats_.total_updates;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error processing sensor data: %s", e.what());
    }
}

std::vector<Bonxai::CoordT> 
OccupancyMapNode::transformAndFilterPoints(
    const sensor_msgs::msg::PointCloud2& cloud,
    const nav_msgs::msg::Odometry& odom)
{
    // 假设点云已经在全局坐标系中，直接提取坐标
    auto raw_coords = BonxaiRosConverter::extractPointCloudAsCoords(
        cloud, occupancy_map_->getGrid());
    
    if (raw_coords.empty()) {
        return {};
    }
    
    // 提取传感器坐标
    auto sensor_coord = BonxaiRosConverter::extractSensorCoord(
        odom, occupancy_map_->getGrid());
    
    // 过滤：距离检查
    std::vector<Bonxai::CoordT> filtered_coords;
    filtered_coords.reserve(raw_coords.size());
    
    const auto& grid = occupancy_map_->getGrid();
    auto sensor_pos = grid.coordToPos(sensor_coord);
    
    for (const auto& coord_3d : raw_coords) {
        auto pos_3d = occupancy_map_->getGrid().coordToPos(coord_3d);
        
        // 计算2D距离（忽略z轴）
        double distance_2d = std::sqrt(
            std::pow(pos_3d.x - sensor_pos.x, 2) +
            std::pow(pos_3d.y - sensor_pos.y, 2));
        
        if (distance_2d >= params_.min_range && distance_2d <= params_.max_range) {
            // 关键：将3D坐标投影到z=0平面
            Bonxai::CoordT coord_2d = occupancy_map_->getGrid().posToCoord(
                pos_3d.x, pos_3d.y, 0.0);
            filtered_coords.push_back(coord_2d);
        }
    }
    
    return filtered_coords;
}

void OccupancyMapNode::updateOccupancyMap(
    const std::vector<Bonxai::CoordT>& points,
    const Bonxai::CoordT& sensor_coord)
{
    if (points.empty()) return;
    
    // 转换为Bonxai点
    auto& grid = occupancy_map_->getGrid();
    std::vector<BonxaiRosConverter::BonxaiPoint> bonxai_points;
    bonxai_points.reserve(points.size());
    
    for (const auto& coord : points) {
        auto pos = grid.coordToPos(coord);
        bonxai_points.emplace_back(pos.x, pos.y, pos.z);
    }
    
    // 传感器位置
    auto sensor_pos = grid.coordToPos(sensor_coord);
    BonxaiRosConverter::BonxaiPoint sensor_position(sensor_pos.x, sensor_pos.y, sensor_pos.z);
    
    // 插入点云
    BonxaiRosConverter::insertPointCloudToMap(
        *occupancy_map_, bonxai_points, sensor_position, params_.max_range);
}

void OccupancyMapNode::publishVisualization() {
    if (!occupancy_map_) return;
    
    rclcpp::Time now = this->now();
    
    // 只发布点云
    auto cloud = BonxaiRosConverter::occupancyMapToPointCloud(
        *occupancy_map_, params_.global_frame, now, true);
    if (cloud && !cloud->data.empty()) {
        occupied_pub_->publish(*cloud);
        
        // 可选：输出调试信息
        static size_t publish_count = 0;
        if (++publish_count % 10 == 0) {
            RCLCPP_DEBUG(get_logger(), "Published occupancy cloud with %zu points", 
                         cloud->data.size() / cloud->point_step);
        }
    }
}

void OccupancyMapNode::publishStatistics() {
    if (!occupancy_map_) return;
    
    // 获取地图统计
    BonxaiRosConverter::getMapStatistics(
        *occupancy_map_,
        stats_.occupied_count,
        stats_.free_count,
        stats_.unknown_count);
    
    // 计算处理频率
    double update_hz = 0.0;
    if (stats_.total_updates > 0) {
        update_hz = stats_.total_updates / 5.0;  // 5秒窗口
    }
    
    // 创建统计消息
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "\n=== Occupancy Map Statistics ==="
       << "\nProcessing Rate: " << update_hz << " Hz"
       << "\nTotal updates: " << stats_.total_updates
       << "\nTotal points processed: " << stats_.total_points
       << "\nAvg process time: " << stats_.avg_process_time << " ms"
       << "\nCurrent queue: " << stats_.queue_size
       << "\n--- Map Cells ---"
       << "\nOccupied: " << stats_.occupied_count
       << "\nFree: " << stats_.free_count
       << "\nUnknown: " << stats_.unknown_count
       << "\nTotal active: " << occupancy_map_->getActiveCellCount()
       << "\nMemory usage: " << occupancy_map_->getMemoryUsage() / 1024.0 / 1024.0 << " MB"
       << "\n===============================";
    
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}

bool OccupancyMapNode::isOccupied(
    const Eigen::Vector2d& pos, double z_height) const noexcept {
    
    // 检查地图是否初始化
    if (!occupancy_map_) {
        return false;  // 地图未初始化 = 未占据
    }
    
    // 线程局部访问器（无锁）
    thread_local auto accessor = occupancy_map_->getGrid().createAccessor();
    
    // 直接转换坐标
    auto coord = occupancy_map_->getGrid().posToCoord(
        pos.x(), pos.y(), z_height);
    
    // 直接查询，不创建新体素
    return queryVoxel(coord, accessor);
}

bool OccupancyMapNode::queryVoxel(
    const Bonxai::CoordT& coord,
    Bonxai::VoxelGrid<Bonxai::MapUtils::CellOcc>::Accessor& accessor) const noexcept {
    
    if (auto* cell = accessor.value(coord, false)) {  // false: 不创建
        return cell->probability_log > occupancy_map_->getOptions().occupancy_threshold_log;
    }
    return false;  // 体素不存在 = 未占据
}

} // namespace plan_env