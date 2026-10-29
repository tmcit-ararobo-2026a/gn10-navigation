#include "gn10_navigation/costmap_generator_node.hpp"
#include "gn10_navigation/robot_geometry.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace gn10_navigation {

CostmapGeneratorNode::CostmapGeneratorNode(const rclcpp::NodeOptions& options)
    : Node("costmap_generator_node", options)
{
    declareAndGetParameters();

    // 静的マップの配信用に Transient Local (ラッチ機能) を有効化
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", map_qos);

    // 動的パラメータ変更コールバックの登録
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CostmapGeneratorNode::onSetParameters, this, std::placeholders::_1)
    );

    // 初回生成＆キャッシュ
    cached_costmap_msg_ = generateCostmap();

    // 1Hzで送信するタイマー
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&CostmapGeneratorNode::publishCachedCostmap, this)
    );

    RCLCPP_INFO(this->get_logger(), "CostmapGeneratorNode initialized.");
}

void CostmapGeneratorNode::declareAndGetParameters()
{
    this->declare_parameter("map_source_type", "json");
    this->declare_parameter("map_file_path", "");

    // コストマップの範囲・解像度設定
    this->declare_parameter("resolution", 0.05);  // 0.05 m/cell
    this->declare_parameter("map_bounds.min_x", -6.0);
    this->declare_parameter("map_bounds.max_x", 6.0);
    this->declare_parameter("map_bounds.min_y", -6.5);
    this->declare_parameter("map_bounds.max_y", 6.5);
    this->declare_parameter("robot_clearance.z_min", 0.00);
    this->declare_parameter("robot_clearance.z_max", 1.40);

    // 膨張コスト（インフレーション）のパラメータ設定
    this->declare_parameter(
        "inflation.robot_radius", kDefaultRobotRadius
    );  // 中心点の不可侵半径 [m] (絶対不可侵: 254)
    this->declare_parameter("inflation.margin", kDefaultInflationMargin);  // コスト波及マージン幅 [m]

    // オブジェクト別コスト定義
    this->declare_parameter("costs.wall", 120);          // 外壁: 120
    this->declare_parameter("costs.partition", 200);     // 仕切り板: 200
    this->declare_parameter("costs.obstacle", 254);      // 絶対不可侵障害物: 254
    this->declare_parameter("costs.default_cost", 254);  // デフォルト: 254

    map_source_type_ = this->get_parameter("map_source_type").as_string();
    map_file_path_   = this->get_parameter("map_file_path").as_string();

    resolution_  = this->get_parameter("resolution").as_double();
    map_min_x_   = this->get_parameter("map_bounds.min_x").as_double();
    map_max_x_   = this->get_parameter("map_bounds.max_x").as_double();
    map_min_y_   = this->get_parameter("map_bounds.min_y").as_double();
    map_max_y_   = this->get_parameter("map_bounds.max_y").as_double();
    robot_z_min_ = this->get_parameter("robot_clearance.z_min").as_double();
    robot_z_max_ = this->get_parameter("robot_clearance.z_max").as_double();

    robot_radius_     = this->get_parameter("inflation.robot_radius").as_double();
    inflation_margin_ = this->get_parameter("inflation.margin").as_double();

    cost_wall_      = static_cast<int>(this->get_parameter("costs.wall").as_int());
    cost_partition_ = static_cast<int>(this->get_parameter("costs.partition").as_int());
    cost_obstacle_  = static_cast<int>(this->get_parameter("costs.obstacle").as_int());
    cost_default_   = static_cast<int>(this->get_parameter("costs.default_cost").as_int());

    for (const auto cost : {cost_wall_, cost_partition_, cost_obstacle_, cost_default_}) {
        if (cost < 0 || cost > 254) {
            throw std::invalid_argument("Custom cost must be within [0, 254]");
        }
    }
    if (!std::isfinite(resolution_) || resolution_ <= 0.0 || !std::isfinite(map_min_x_) ||
        !std::isfinite(map_max_x_) || !std::isfinite(map_min_y_) || !std::isfinite(map_max_y_) ||
        map_min_x_ >= map_max_x_ || map_min_y_ >= map_max_y_) {
        throw std::invalid_argument("Invalid map bounds/resolution");
    }
}

rcl_interfaces::msg::SetParametersResult CostmapGeneratorNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful    = true;
    bool need_regenerate = false;

    for (const auto& param : parameters) {
        const auto& name = param.get_name();

        if (name == "inflation.robot_radius") {
            robot_radius_   = param.as_double();
            need_regenerate = true;
        } else if (name == "inflation.margin") {
            inflation_margin_ = param.as_double();
            need_regenerate   = true;
        } else if (name == "costs.wall") {
            cost_wall_      = static_cast<int>(param.as_int());
            need_regenerate = true;
        } else if (name == "costs.partition") {
            cost_partition_ = static_cast<int>(param.as_int());
            need_regenerate = true;
        } else if (name == "costs.obstacle") {
            cost_obstacle_  = static_cast<int>(param.as_int());
            need_regenerate = true;
        } else if (name == "costs.default_cost") {
            cost_default_   = static_cast<int>(param.as_int());
            need_regenerate = true;
        }
    }

    if (need_regenerate) {
        cached_costmap_msg_ = generateCostmap();
        RCLCPP_INFO(this->get_logger(), "Costmap regenerated due to parameter update.");
    }

    return result;
}

int CostmapGeneratorNode::getObjectCost(const FieldObject& obj) const
{
    // JSON で明示的に cost が指定されている場合はそれを最優先
    if (obj.cost >= 0 && obj.cost <= 254) {
        return obj.cost;
    }

    // 後方互換: comment 文字列による自動判定
    const auto& comment = obj.comment;
    if (comment.find("外壁") != std::string::npos) {
        return cost_wall_;
    } else if (comment.find("仕切り板") != std::string::npos || comment.find("教壇") != std::string::npos) {
        return cost_partition_;
    } else if (comment.find("バケツ") != std::string::npos || comment.find("机") != std::string::npos || comment.find("椅子") != std::string::npos || comment.find("旗") != std::string::npos) {
        return cost_obstacle_;
    }
    return cost_default_;
}

nav_msgs::msg::OccupancyGrid CostmapGeneratorNode::generateCostmap()
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

    nav_msgs::msg::OccupancyGrid costmap_msg;

    if (map_objects.empty()) {
        RCLCPP_WARN(this->get_logger(), "Map is empty or failed to load.");
        return costmap_msg;
    }

    costmap_msg.header.frame_id = "map";

    costmap_msg.info.resolution = static_cast<float>(resolution_);
    costmap_msg.info.width =
        static_cast<uint32_t>(std::ceil((map_max_x_ - map_min_x_) / resolution_));
    costmap_msg.info.height =
        static_cast<uint32_t>(std::ceil((map_max_y_ - map_min_y_) / resolution_));

    costmap_msg.info.origin.position.x    = map_min_x_;
    costmap_msg.info.origin.position.y    = map_min_y_;
    costmap_msg.info.origin.position.z    = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;

    const double origin_x = map_min_x_;
    const double origin_y = map_min_y_;
    const int width       = static_cast<int>(costmap_msg.info.width);
    const int height      = static_cast<int>(costmap_msg.info.height);

    std::vector<uint8_t> obstacle_costs(width * height, 0);
    std::vector<uint8_t> penalty_costs(width * height, 0);

    // ① 原理障害物および通過可能ペナルティ領域のレイアウト配置
    for (const auto& obj : map_objects) {
        if (obj.z_max < robot_z_min_ || obj.z_min > robot_z_max_) {
            continue;
        }

        int cost = getObjectCost(obj);
        bool is_passable = obj.passable;
        auto& target_grid = is_passable ? penalty_costs : obstacle_costs;

        if (obj.type == BOX || obj.type == VISUAL_BOX || obj.type == ZONE) {
            double half_w = obj.half_x();
            double half_d = obj.half_y();
            double min_x = obj.center_x - half_w;
            double max_x = obj.center_x + half_w;
            double min_y = obj.center_y - half_d;
            double max_y = obj.center_y + half_d;

            int min_x_idx = std::max(0, static_cast<int>((min_x - origin_x) / resolution_));
            int max_x_idx = std::min(width - 1, static_cast<int>((max_x - origin_x) / resolution_));
            int min_y_idx = std::max(0, static_cast<int>((min_y - origin_y) / resolution_));
            int max_y_idx =
                std::min(height - 1, static_cast<int>((max_y - origin_y) / resolution_));

            for (int x = min_x_idx; x <= max_x_idx; ++x) {
                for (int y = min_y_idx; y <= max_y_idx; ++y) {
                    int idx           = y * width + x;
                    target_grid[idx] = std::max(target_grid[idx], static_cast<uint8_t>(cost));
                }
            }
        } else if (obj.type == CYLINDER) {
            double radius = obj.radius();
            int min_x_idx =
                std::max(0, static_cast<int>((obj.center_x - radius - origin_x) / resolution_));
            int max_x_idx = std::min(
                width - 1, static_cast<int>((obj.center_x + radius - origin_x) / resolution_)
            );
            int min_y_idx =
                std::max(0, static_cast<int>((obj.center_y - radius - origin_y) / resolution_));
            int max_y_idx = std::min(
                height - 1, static_cast<int>((obj.center_y + radius - origin_y) / resolution_)
            );

            for (int x = min_x_idx; x <= max_x_idx; ++x) {
                for (int y = min_y_idx; y <= max_y_idx; ++y) {
                    double cell_x  = origin_x + (x + 0.5) * resolution_;
                    double cell_y  = origin_y + (y + 0.5) * resolution_;
                    double dist_sq = (cell_x - obj.center_x) * (cell_x - obj.center_x) +
                                     (cell_y - obj.center_y) * (cell_y - obj.center_y);

                    if (dist_sq <= radius * radius) {
                        int idx           = y * width + x;
                        target_grid[idx] = std::max(target_grid[idx], static_cast<uint8_t>(cost));
                    }
                }
            }
        }
    }

    // ② Multi-Source BFSによる波及コスト計算およびペナルティ領域のマージ
    costmap_msg.data.resize(width * height, 0);
    applyMultiSourceBfsInflation(obstacle_costs, penalty_costs, width, height, costmap_msg.data);

    RCLCPP_INFO(this->get_logger(), "Generated Costmap cache (%dx%d)", width, height);
    return costmap_msg;
}

void CostmapGeneratorNode::applyMultiSourceBfsInflation(
    const std::vector<uint8_t>& obstacle_costs,
    const std::vector<uint8_t>& penalty_costs,
    int width,
    int height,
    std::vector<int8_t>& output_grid
)
{
    const double max_dist = robot_radius_ + inflation_margin_;
    std::vector<uint8_t> computed_cost(width * height, 0);

    if (max_dist > 0.0) {
        struct BfsItem {
            int x;
            int y;
            int src_x;
            int src_y;
            uint8_t base_cost;
        };

        std::queue<BfsItem> q;
        std::vector<float> min_dist_sq(width * height, std::numeric_limits<float>::infinity());

        // 初期シード: 不可侵障害物セルをキューに登録
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (obstacle_costs[idx] > 0) {
                    computed_cost[idx] = 254;  // 障害物セル自身は不可侵(254)
                    min_dist_sq[idx]   = 0.0f;
                    q.push({x, y, x, y, obstacle_costs[idx]});
                }
            }
        }

        const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
        const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!q.empty()) {
            BfsItem item = q.front();
            q.pop();

            for (int dir = 0; dir < 8; ++dir) {
                int nx = item.x + dx[dir];
                int ny = item.y + dy[dir];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }

                int nidx      = ny * width + nx;
                int diff_x    = nx - item.src_x;
                int diff_y    = ny - item.src_y;
                float dist_sq = static_cast<float>(
                    (diff_x * diff_x + diff_y * diff_y) * (resolution_ * resolution_)
                );

                if (dist_sq > static_cast<float>(max_dist * max_dist)) {
                    continue;
                }

                double dist  = std::sqrt(dist_sq);
                uint8_t cost = 0;
                if (dist <= robot_radius_) {
                    cost = 254;
                } else if (dist <= max_dist && inflation_margin_ > 0.0) {
                    double factor = 1.0 - (dist - robot_radius_) / inflation_margin_;
                    cost = static_cast<uint8_t>(
                        std::clamp(std::round(item.base_cost * factor), 0.0, 253.0)
                    );
                }

                if (cost > computed_cost[nidx]) {
                    computed_cost[nidx] = cost;
                    min_dist_sq[nidx]   = dist_sq;
                    q.push({nx, ny, item.src_x, item.src_y, item.base_cost});
                } else if (cost == computed_cost[nidx] && dist_sq < min_dist_sq[nidx]) {
                    min_dist_sq[nidx] = dist_sq;
                    q.push({nx, ny, item.src_x, item.src_y, item.base_cost});
                }
            }
        }
    } else {
        for (size_t i = 0; i < obstacle_costs.size(); ++i) {
            if (obstacle_costs[i] > 0) {
                computed_cost[i] = 254;
            }
        }
    }

    // ③ 通過可能ペナルティ領域（避けたい場所）のマージ
    // 障害物（254）のセル以外に、ペナルティコストを重ね合わせる
    for (size_t i = 0; i < computed_cost.size(); ++i) {
        if (penalty_costs[i] > 0 && computed_cost[i] < 254) {
            computed_cost[i] = std::max(computed_cost[i], penalty_costs[i]);
        }
    }

    // ④ OccupancyGridスケーリング
    // 仕様: 254のセルを100(不可侵)、それ以外の波及・ペナルティコスト(0〜253)を0〜99の範囲にスケール変換
    for (size_t i = 0; i < computed_cost.size(); ++i) {
        uint8_t c = computed_cost[i];
        if (c >= 254) {
            output_grid[i] = 100;
        } else if (c == 0) {
            output_grid[i] = 0;
        } else {
            output_grid[i] = static_cast<int8_t>(
                std::clamp(static_cast<int>(std::round(c * 99.0 / 253.0)), 1, 99)
            );
        }
    }
}

void CostmapGeneratorNode::publishCachedCostmap()
{
    if (cached_costmap_msg_.data.empty()) {
        return;
    }

    cached_costmap_msg_.header.stamp = this->now();
    costmap_pub_->publish(cached_costmap_msg_);
}

}  // namespace gn10_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::CostmapGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
