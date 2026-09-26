#include "gn10_navigation/costmap_generator_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>

CostmapGeneratorNode::CostmapGeneratorNode() : Node("costmap_generator_node")
{
    declareAndGetParameters();

    // 静的マップの配信用に Transient Local (ラッチ機能) を有効化
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", map_qos);

    generateAndPublishCostmap();
}

void CostmapGeneratorNode::declareAndGetParameters()
{
    // チームカラー指定 ("red" または "blue")
    this->declare_parameter("team_color", "red");
    this->declare_parameter("map_source_type", "json");
    this->declare_parameter("map_file_path", "");
    this->declare_parameter("map_objects", std::vector<std::string>{});

    // コストマップの範囲・解像度設定
    this->declare_parameter("resolution", 0.05);        // 5cm/cell
    this->declare_parameter("map_bounds.min_x", -6.0);
    this->declare_parameter("map_bounds.max_x", 6.0);
    this->declare_parameter("map_bounds.min_y", -6.5);
    this->declare_parameter("map_bounds.max_y", 6.5);
    this->declare_parameter("robot_clearance.z_min", 0.03); // ロボットが乗り越えられる高さ閾値
    this->declare_parameter("robot_clearance.z_max", 1.00);

    team_color_       = this->get_parameter("team_color").as_string();
    map_source_type_  = this->get_parameter("map_source_type").as_string();
    map_file_path_    = this->get_parameter("map_file_path").as_string();
    map_objects_params_ = this->get_parameter("map_objects").as_string_array();

    resolution_  = this->get_parameter("resolution").as_double();
    map_min_x_   = this->get_parameter("map_bounds.min_x").as_double();
    map_max_x_   = this->get_parameter("map_bounds.max_x").as_double();
    map_min_y_   = this->get_parameter("map_bounds.min_y").as_double();
    map_max_y_   = this->get_parameter("map_bounds.max_y").as_double();
    robot_z_min_ = this->get_parameter("robot_clearance.z_min").as_double();
    robot_z_max_ = this->get_parameter("robot_clearance.z_max").as_double();

    // 小文字化
    std::transform(team_color_.begin(), team_color_.end(), team_color_.begin(), ::tolower);
    RCLCPP_INFO(this->get_logger(), "Team Color set to: %s", team_color_.c_str());
}

int CostmapGeneratorNode::getObjectCost(const std::string& comment, ObjectType /*type*/) const
{
    // コスト定義（0: 自由領域, 120: 外壁(接触許容), 200: 中央仕切り, 254: 障害物・絶対接触NG）
    if (comment.find("外壁") != std::string::npos) {
        return 120;
    } else if (comment.find("仕切り板") != std::string::npos || comment.find("教壇") != std::string::npos) {
        return 200;
    } else if (comment.find("バケツ") != std::string::npos || 
               comment.find("机") != std::string::npos || 
               comment.find("椅子") != std::string::npos ||
               comment.find("旗") != std::string::npos) {
        return 254; // 絶対接触NG
    }
    return 254; // デフォルトは最高リスク
}

void CostmapGeneratorNode::generateAndPublishCostmap()
{
    std::vector<FieldObject> map_objects;

    if (map_source_type_ == "json") {
        std::string json_path = map_file_path_;
        if (json_path.empty()) {
            json_path = ament_index_cpp::get_package_share_directory("gn10_navigation") +
                        "/config/nhk2026_map.json";
        }
        map_objects = MapLoader::loadFromJSON(json_path);
    }

    if (map_objects.empty()) {
        RCLCPP_WARN(this->get_logger(), "Map is empty. Falling back to default map.");
        return;
    }

    // チームゾーンに応じたマップフィルタリング/展開範囲の設定
    // NHK2026等の配置で領域A(Y>0) / 領域B(Y<0) などが分かれている場合に最適化
    std::string target_zone_comment = (team_color_ == "red") ? "領域A" : "領域B";

    nav_msgs::msg::OccupancyGrid costmap_msg;
    costmap_msg.header.stamp    = this->now();
    costmap_msg.header.frame_id = "map";

    costmap_msg.info.resolution = static_cast<float>(resolution_);
    costmap_msg.info.width      = static_cast<uint32_t>(std::ceil((map_max_x_ - map_min_x_) / resolution_));
    costmap_msg.info.height     = static_cast<uint32_t>(std::ceil((map_max_y_ - map_min_y_) / resolution_));

    costmap_msg.info.origin.position.x = map_min_x_;
    costmap_msg.info.origin.position.y = map_min_y_;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;

    // 初期化 (全て自由領域 0)
    costmap_msg.data.resize(costmap_msg.info.width * costmap_msg.info.height, 0);

    const double origin_x = map_min_x_;
    const double origin_y = map_min_y_;
    const int width       = static_cast<int>(costmap_msg.info.width);
    const int height      = static_cast<int>(costmap_msg.info.height);

    for (const auto& obj : map_objects) {
        // ロボットの高さ判定 (z軸の範囲外なら読み飛ばす)
        if (obj.z_max < robot_z_min_ || obj.z_min > robot_z_max_) {
            continue;
        }

        // チームゾーン判定: 自チーム以外の領域オブジェクトを除外する場合などのフィルタ
        // (全領域を共通で表示しつつ、自陣固有オブジェクトのみ判定したい場合に使用)
        /*
        if (obj.comment.find("領域") != std::string::npos && obj.comment.find(target_zone_comment) == std::string::npos) {
            continue; // 相手チーム専用オブジェクトを判定から外す場合はコメント解除
        }
        */

        int cost = getObjectCost(obj.comment, obj.type);

        if (obj.type == BOX || obj.type == VISUAL_BOX) {
            // param1, param2 は中心からの X/Y 半幅 (extents)
            double min_x = obj.center_x - obj.param1;
            double max_x = obj.center_x + obj.param1;
            double min_y = obj.center_y - obj.param2;
            double max_y = obj.center_y + obj.param2;

            int min_x_idx = std::max(0, static_cast<int>((min_x - origin_x) / resolution_));
            int max_x_idx = std::min(width - 1, static_cast<int>((max_x - origin_x) / resolution_));
            int min_y_idx = std::max(0, static_cast<int>((min_y - origin_y) / resolution_));
            int max_y_idx = std::min(height - 1, static_cast<int>((max_y - origin_y) / resolution_));

            for (int x = min_x_idx; x <= max_x_idx; ++x) {
                for (int y = min_y_idx; y <= max_y_idx; ++y) {
                    int idx = y * width + x;
                    costmap_msg.data[idx] = static_cast<int8_t>(std::max(static_cast<int>(costmap_msg.data[idx]), cost));
                }
            }
        } 
        else if (obj.type == CYLINDER) {
            double radius = obj.param1;
            int min_x_idx = std::max(0, static_cast<int>((obj.center_x - radius - origin_x) / resolution_));
            int max_x_idx = std::min(width - 1, static_cast<int>((obj.center_x + radius - origin_x) / resolution_));
            int min_y_idx = std::max(0, static_cast<int>((obj.center_y - radius - origin_y) / resolution_));
            int max_y_idx = std::min(height - 1, static_cast<int>((obj.center_y + radius - origin_y) / resolution_));

            for (int x = min_x_idx; x <= max_x_idx; ++x) {
                for (int y = min_y_idx; y <= max_y_idx; ++y) {
                    double cell_x = origin_x + (x + 0.5) * resolution_;
                    double cell_y = origin_y + (y + 0.5) * resolution_;
                    double dist_sq = (cell_x - obj.center_x) * (cell_x - obj.center_x) +
                                     (cell_y - obj.center_y) * (cell_y - obj.center_y);

                    if (dist_sq <= radius * radius) {
                        int idx = y * width + x;
                        costmap_msg.data[idx] = static_cast<int8_t>(std::max(static_cast<int>(costmap_msg.data[idx]), cost));
                    }
                }
            }
        }
    }

    costmap_pub_->publish(costmap_msg);
    RCLCPP_INFO(this->get_logger(), "Published Costmap for team: %s (%dx%d)", team_color_.c_str(), width, height);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}