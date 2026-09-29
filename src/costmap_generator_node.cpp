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
    this->declare_parameter("inflation.robot_radius", 0.35);      // ロボット物理半径 [m] (絶対侵入不可領域)
    this->declare_parameter("inflation.margin", 0.50);            // 基準となる最大の膨張余白 [m]
    this->declare_parameter("inflation.cost_scale_factor", 1.0);  // コストによるマージン変化の度合い (0.0〜2.0程度)

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
    robot_radius_      = this->get_parameter("inflation.robot_radius").as_double();
    inflation_margin_  = this->get_parameter("inflation.margin").as_double();
    cost_scale_factor_ = this->get_parameter("inflation.cost_scale_factor").as_double();

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

    // ① 原理障害物のレイアウト配置 (各オブジェクトの基準コストを設定)
    for (const auto& obj : map_objects) {
        if (obj.z_max < robot_z_min_ || obj.z_min > robot_z_max_) {
            continue;
        }

        int cost = getObjectCost(obj.comment, obj.type);

        if (obj.type == BOX || obj.type == VISUAL_BOX) {
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

    // ② コストおよびスケーリングファクターに基づく可変マージン付きインフレーションの適用
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

    struct SourceCell {
        int x;
        int y;
        int cost;
    };
    std::vector<SourceCell> obstacle_cells;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int c = static_cast<uint8_t>(costmap_msg.data[y * width + x]);
            if (c > 0) {
                obstacle_cells.push_back({x, y, c});
            }
        }
    }

    std::vector<uint8_t> inflated_data(costmap_msg.data.begin(), costmap_msg.data.end());

    for (const auto& source : obstacle_cells) {
        // コスト(0〜254)に応じたマージン率の計算 (0.0〜1.0)
        double cost_ratio = static_cast<double>(source.cost) / 254.0;
        
        // cost_scale_factor_ に応じて高コストほどマージンが広くなるよう可変スケール
        // (例: factor=1.0 のとき、コスト254なら100%マージン、コスト120なら約47%マージン)
        double scaled_margin = inflation_margin_ * (1.0 - cost_scale_factor_ * (1.0 - cost_ratio));
        scaled_margin = std::max(0.0, scaled_margin); // 負の値を防ぐ

        double effective_total_radius = robot_radius_ + scaled_margin;
        int radius_cells = static_cast<int>(std::ceil(effective_total_radius / resolution_));

        int min_x = std::max(0, source.x - radius_cells);
        int max_x = std::min(width - 1, source.x + radius_cells);
        int min_y = std::max(0, source.y - radius_cells);
        int max_y = std::min(height - 1, source.y + radius_cells);

        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                int dx = x - source.x;
                int dy = y - source.y;
                double dist = std::hypot(dx, dy) * resolution_;

                int calculated_cost = 0;
                if (dist <= robot_radius_) {
                    calculated_cost = 254; // ロボット物理半径は絶対不可侵
                } else if (dist <= effective_total_radius && scaled_margin > 0.0) {
                    // 個別の scaled_margin に基づいてグラデーションコストを計算
                    double ratio = (effective_total_radius - dist) / scaled_margin;
                    calculated_cost = static_cast<int>(ratio * source.cost);
                } else {
                    continue;
                }

                int idx = y * width + x;
                if (calculated_cost > inflated_data[idx]) {
                    inflated_data[idx] = static_cast<uint8_t>(calculated_cost);
                }
            }
        }
    }

    for (size_t i = 0; i < costmap_msg.data.size(); ++i) {
        costmap_msg.data[i] = static_cast<int8_t>(inflated_data[i]);
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