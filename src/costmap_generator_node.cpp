#include "gn10_navigation/costmap_generator_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

CostmapGeneratorNode::CostmapGeneratorNode() : Node("costmap_generator_node")
{
    declare_parameter("map_source_type", "json");
    declare_parameter("map_file_path", "");
    declare_parameter("map_objects", std::vector<std::string>{});

    generateCostmap();
}

void CostmapGeneratorNode::generateCostmap()
{
    std::string map_source = this->get_parameter("map_source_type").as_string();
    std::vector<FieldObject> map_objects;

    if (map_source == "json") {
        std::string json_path = this->get_parameter("map_file_path").as_string();
        if (json_path.empty()) {
            json_path =
                ament_index_cpp::get_package_share_directory("gn10_navigation") +
                "/config/nhk2026_map.json";
        }
        map_objects = MapLoader::loadFromJSON(json_path);
    } else if (map_source == "ros2_param") {
        auto param_list = this->get_parameter("map_objects").as_string_array();
        map_objects     = MapLoader::loadFromParams(param_list);
    }

    if (map_objects.empty()) {
        RCLCPP_WARN(
            this->get_logger(), "Map is empty or failed to load. Falling back to default map."
        );
        map_objects = MapLoader::createNHK2026FieldMap();
    }

    nav_msgs::msg::OccupancyGrid costmap_msg;
    costmap_msg.header.frame_id = "map";
    costmap_msg.info.resolution  = 0.05;  // 5cm per cell
    costmap_msg.info.width       = 400;   // 20m x 20m area
    costmap_msg.info.height      = 400;
    costmap_msg.info.origin.position.x = -10.0;
    costmap_msg.info.origin.position.y = -10.0;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;

    // Initialize the occupancy grid with free space
    costmap_msg.data.resize(costmap_msg.info.width * costmap_msg.info.height, 0);

    // Fill in the occupancy grid based on the field objects
    for (const auto& obj : map_objects) {
        int min_x_idx = static_cast<int>((obj.center_x - obj.param1) / costmap_msg.info.resolution);
        int max_x_idx = static_cast<int>((obj.center_x + obj.param1) / costmap_msg.info.resolution);
        int min_y_idx = static_cast<int>((obj.center_y - obj.param2) / costmap_msg.info.resolution);
        int max_y_idx = static_cast<int>((obj.center_y + obj.param2) / costmap_msg.info.resolution);
        for (int x = min_x_idx; x <= max_x_idx; ++x) {
            for (int y = min_y_idx; y <= max_y_idx; ++y) {
                if (x >= 0 && x < static_cast<int>(costmap_msg.info.width) &&
                    y >= 0 && y < static_cast<int>(costmap_msg.info.height)) {
                    costmap_msg.data[y * costmap_msg.info.width + x] = 100;  // Occupied
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}