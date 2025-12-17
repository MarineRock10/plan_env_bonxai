#include "plan_env/occupancy_map_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;

    const std::string occupancy_node_name = "occupancy_map_node";
        
    auto node = std::make_shared<plan_env::OccupancyMapNode>(occupancy_node_name, options);
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}