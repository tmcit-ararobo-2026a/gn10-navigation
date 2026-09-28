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
    this->declare_parameter("map_source_type", "json");
    this->declare_parameter("map_file_path", "");

    // コストマップの範囲・解像度設定
    this->declare_parameter("resolution", 0.05);        // 5cm/cell
    this->declare_parameter("map_bounds.min_x", -6.0);
    this->declare_parameter("map_bounds.max_x", 6.0);
    this->declare_parameter("map_bounds.min_y", -6.5);
    this->declare_parameter("map_bounds.max_y", 6.5);
    this->declare_parameter("robot_clearance.z_min", 0.00); // ロボットが乗り越えられる高さ閾値
    this->declare_parameter("robot_clearance.z_max", 1.40); // ロボットの高さ

    // 膨張コスト（インフレーション）のパラメータ設定
    this->declare_parameter("inflation.robot_radius", 0.35);      // ロボット物理半径 [m] (254エリア)
    this->declare_parameter("inflation.margin", 0.50);            // グラデーションの膨張余白 [m]
    this->declare_parameter("inflation.max_cost", 200);           // 障害物直近の膨張コスト上限

    // オブジェクト別コスト定義のパラメータ化
    this->declare_parameter("costs.wall", 120);         // 外壁
    this->declare_parameter("costs.partition", 200);    // 中央仕切り・教壇
    this->declare_parameter("costs.obstacle", 254);     // バケツ・机・椅子・旗など（絶対接触NG）
    this->declare_parameter("costs.default_cost", 254); // デフォルト

    map_source_type_ = this->get_parameter("map_source_type").as_string();
    map_file_path_   = this->get_parameter("map_file_path").as_string();

    resolution_  = this->get_parameter("resolution").as_double();
    map_min_x_   = this->get_parameter("map_bounds.min_x").as_double();
    map_max_x_   = this->get_parameter("map_bounds.max_x").as_double();
    map_min_y_   = this->get_parameter("map_bounds.min_y").as_double();
    map_max_y_   = this->get_parameter("map_bounds.max_y").as_double();
    robot_z_min_ = this->get_parameter("robot_clearance.z_min").as_double();
    robot_z_max_ = this->get_parameter("robot_clearance.z_max").as_double();

    // 膨張パラメータの取得
    robot_radius_       = this->get_parameter("inflation.robot_radius").as_double();
    inflation_margin_   = this->get_parameter("inflation.margin").as_double();
    max_inflation_cost_ = static_cast<int>(this->get_parameter("inflation.max_cost").as_int());

    // コストパラメータの取得
    cost_wall_      = static_cast<int>(this->get_parameter("costs.wall").as_int());
    cost_partition_ = static_cast<int>(this->get_parameter("costs.partition").as_int());
    cost_obstacle_  = static_cast<int>(this->get_parameter("costs.obstacle").as_int());
    cost_default_   = static_cast<int>(this->get_parameter("costs.default_cost").as_int());
}

int CostmapGeneratorNode::getObjectCost(const std::string& comment, ObjectType /*type*/) const
{
    if (comment.find("外壁") != std::string::npos) {
        return cost_wall_;
    } else if (comment.find("仕切り板") != std::string::npos || comment.find("教壇") != std::string::npos) {
        return cost_partition_;
    } else if (comment.find("バケツ") != std::string::npos || 
               comment.find("机") != std::string::npos || 
               comment.find("椅子") != std::string::npos ||
               comment.find("旗") != std::string::npos) {
        return cost_obstacle_;
    }
    return cost_default_;
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
        RCLCPP_WARN(this->get_logger(), "Map is empty or failed to load.");
        return;
    }

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

    // 原理障害物（254などの静的オブジェクト）のレイアウト配置
    for (const auto& obj : map_objects) {
        // ロボットの高さ判定 (z軸の範囲外なら読み飛ばす)
        if (obj.z_max < robot_z_min_ || obj.z_min > robot_z_max_) {
            continue;
        }

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

    // 全ての障害物セルから連続的な膨張コスト（Cost Gradient）を生成
    applyInflation(costmap_msg);

    costmap_pub_->publish(costmap_msg);
    RCLCPP_INFO(this->get_logger(), "Published Costmap (%dx%d)", width, height);
}

void CostmapGeneratorNode::applyInflation(nav_msgs::msg::OccupancyGrid& costmap_msg)
{
    if (robot_radius_ <= 0.0 && inflation_margin_ <= 0.0) {
        return;
    }

    const int width = static_cast<int>(costmap_msg.info.width);
    const int height = static_cast<int>(costmap_msg.info.height);
    const double total_radius = robot_radius_ + inflation_margin_;
    const int radius_cells = static_cast<int>(std::ceil(total_radius / resolution_));

    // 膨張前の障害物セル（254）のインデックスを抽出
    std::vector<std::pair<int, int>> obstacle_cells;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (static_cast<uint8_t>(costmap_msg.data[y * width + x]) >= 254) {
                obstacle_cells.emplace_back(x, y);
            }
        }
    }

    // 各障害物セルから近傍セルへ距離に応じたコストを膨張
    for (const auto& [ox, oy] : obstacle_cells) {
        int min_x = std::max(0, ox - radius_cells);
        int max_x = std::min(width - 1, ox + radius_cells);
        int min_y = std::max(0, oy - radius_cells);
        int max_y = std::min(height - 1, oy + radius_cells);

        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                int dx = x - ox;
                int dy = y - oy;
                double dist = std::hypot(dx, dy) * resolution_;

                int cost = 0;
                if (dist <= robot_radius_) {
                    cost = 254; // ロボットの物理接触領域（絶対不可侵）
                } else if (dist <= total_radius && inflation_margin_ > 0.0) {
                    // 障害物に近づくほど滑らかにコストが高くなる線形勾配
                    double ratio = (total_radius - dist) / inflation_margin_; // 1.0 〜 0.0
                    cost = static_cast<int>(ratio * max_inflation_cost_);
                } else {
                    continue;
                }

                int idx = y * width + x;
                int current_cost = static_cast<uint8_t>(costmap_msg.data[idx]);
                
                // より高いコストを採用して保持する
                if (cost > current_cost) {
                    costmap_msg.data[idx] = static_cast<int8_t>(cost);
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