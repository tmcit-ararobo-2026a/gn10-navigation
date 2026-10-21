#include "gn10_navigation/path_planner_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gn10_navigation {

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions& options)
    : Node("path_planner_node", options)
{
    declareAndGetParameters();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&PathPlannerNode::onCostmap, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::onGoalPose, this, std::placeholders::_1)
    );

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "PathPlannerNode initialized.");
}

void PathPlannerNode::declareAndGetParameters()
{
    this->declare_parameter("map_configs.map_frame", "map");
    this->declare_parameter("map_configs.base_frame", "base_link");
    this->declare_parameter("a_star.planning_cost", 3.0);

    map_frame_            = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_           = this->get_parameter("map_configs.base_frame").as_string();
    planning_cost_factor_ = this->get_parameter("a_star.planning_cost").as_double();
}

void PathPlannerNode::onCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    latest_costmap_ = *msg;
    has_costmap_    = true;
}

std::pair<int, int> PathPlannerNode::worldToGrid(double wx, double wy) const
{
    double res = latest_costmap_.info.resolution;
    int gx     = static_cast<int>(std::floor((wx - latest_costmap_.info.origin.position.x) / res));
    int gy     = static_cast<int>(std::floor((wy - latest_costmap_.info.origin.position.y) / res));
    return {gx, gy};
}

std::pair<double, double> PathPlannerNode::gridToWorld(int gx, int gy) const
{
    double res = latest_costmap_.info.resolution;
    double wx  = latest_costmap_.info.origin.position.x + (gx + 0.5) * res;
    double wy  = latest_costmap_.info.origin.position.y + (gy + 0.5) * res;
    return {wx, wy};
}

bool PathPlannerNode::isInsideGrid(int gx, int gy) const
{
    return gx >= 0 && gx < static_cast<int>(latest_costmap_.info.width) && gy >= 0 &&
           gy < static_cast<int>(latest_costmap_.info.height);
}

bool PathPlannerNode::isPassable(int gx, int gy) const
{
    if (!isInsideGrid(gx, gy)) {
        return false;
    }
    int idx     = gy * static_cast<int>(latest_costmap_.info.width) + gx;
    int8_t cost = latest_costmap_.data[idx];
    // 0〜99 が通行可能領域、100は不可侵、負数は未知領域
    return cost >= 0 && cost < 100;
}

std::optional<std::pair<int, int>> PathPlannerNode::findNearestPassable(
    int gx, int gy, int max_radius_cells
) const
{
    if (isPassable(gx, gy)) {
        return std::make_pair(gx, gy);
    }

    double best_dist_sq                          = std::numeric_limits<double>::infinity();
    std::optional<std::pair<int, int>> best_cell = std::nullopt;

    for (int r = 1; r <= max_radius_cells; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::abs(dx) != r && std::abs(dy) != r) {
                    continue;
                }
                int nx = gx + dx;
                int ny = gy + dy;
                if (isPassable(nx, ny)) {
                    double d_sq = dx * dx + dy * dy;
                    if (d_sq < best_dist_sq) {
                        best_dist_sq = d_sq;
                        best_cell    = std::make_pair(nx, ny);
                    }
                }
            }
        }
        if (best_cell.has_value()) {
            return best_cell;
        }
    }
    return std::nullopt;
}

std::vector<std::pair<int, int>> PathPlannerNode::aStarSearch(
    int start_x, int start_y, int goal_x, int goal_y
)
{
    const int width     = static_cast<int>(latest_costmap_.info.width);
    const int height    = static_cast<int>(latest_costmap_.info.height);
    const int num_cells = width * height;

    struct AStarElement {
        int x;
        int y;
        double f;

        bool operator>(const AStarElement& other) const
        {
            return f > other.f;
        }
    };

    std::priority_queue<AStarElement, std::vector<AStarElement>, std::greater<AStarElement>>
        open_set;
    std::vector<double> g_score(num_cells, std::numeric_limits<double>::infinity());
    std::vector<int> parent_index(num_cells, -1);
    std::vector<bool> closed_set(num_cells, false);

    auto getIndex = [width](int x, int y) { return y * width + x; };

    auto heuristic = [](int x1, int y1, int x2, int y2) -> double {
        int dx = std::abs(x1 - x2);
        int dy = std::abs(y1 - y2);
        // Octile distance
        int min_d = std::min(dx, dy);
        int max_d = std::max(dx, dy);
        return (max_d - min_d) + 1.41421356237 * min_d;
    };

    int start_idx      = getIndex(start_x, start_y);
    g_score[start_idx] = 0.0;
    open_set.push({start_x, start_y, heuristic(start_x, start_y, goal_x, goal_y)});

    const int dx[8]            = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8]            = {0, 0, -1, 1, -1, 1, -1, 1};
    const double step_costs[8] = {
        1.0, 1.0, 1.0, 1.0, 1.41421356237, 1.41421356237, 1.41421356237, 1.41421356237};

    bool found = false;

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        int curr_idx = getIndex(current.x, current.y);
        if (closed_set[curr_idx]) {
            continue;
        }
        closed_set[curr_idx] = true;

        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isPassable(nx, ny)) {
                continue;
            }

            // 斜め移動時の角抜け防止
            if (dx[i] != 0 && dy[i] != 0) {
                if (!isPassable(current.x + dx[i], current.y) &&
                    !isPassable(current.x, current.y + dy[i])) {
                    continue;
                }
            }

            int next_idx = getIndex(nx, ny);
            if (closed_set[next_idx]) {
                continue;
            }

            // Cost_step = StepLength + CostmapValue * k_penalty
            int8_t costmap_val = latest_costmap_.data[next_idx];
            double edge_cost =
                step_costs[i] + static_cast<double>(costmap_val) * planning_cost_factor_;
            double tentative_g = g_score[curr_idx] + edge_cost;

            if (tentative_g < g_score[next_idx]) {
                g_score[next_idx]      = tentative_g;
                parent_index[next_idx] = curr_idx;
                double h               = heuristic(nx, ny, goal_x, goal_y);
                open_set.push({nx, ny, tentative_g + h});
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    if (!found) {
        return path;
    }

    int curr = getIndex(goal_x, goal_y);
    while (curr != -1) {
        int cx = curr % width;
        int cy = curr / width;
        path.push_back({cx, cy});
        curr = parent_index[curr];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void PathPlannerNode::onGoalPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (!has_costmap_) {
        RCLCPP_WARN(this->get_logger(), "Costmap not received yet. Cannot plan path.");
        return;
    }

    // 1. ロボット現在位置をTFから取得
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not transform %s to %s: %s",
            map_frame_.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
        return;
    }

    double start_wx = tf_stamped.transform.translation.x;
    double start_wy = tf_stamped.transform.translation.y;
    double goal_wx  = msg->pose.position.x;
    double goal_wy  = msg->pose.position.y;

    auto [start_gx, start_gy] = worldToGrid(start_wx, start_wy);
    auto [goal_gx, goal_gy]   = worldToGrid(goal_wx, goal_wy);

    // 2. スタートとゴールの通行可能性チェック＆スナップ
    const int snap_radius_cells =
        static_cast<int>(std::ceil(0.4 / latest_costmap_.info.resolution));

    auto snapped_start = findNearestPassable(start_gx, start_gy, snap_radius_cells);
    if (!snapped_start.has_value()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Start pose (%d, %d) is inside obstacle and cannot be snapped.",
            start_gx,
            start_gy
        );
        // 空のPathをパブリッシュして失敗通知
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    auto snapped_goal = findNearestPassable(goal_gx, goal_gy, snap_radius_cells);
    if (!snapped_goal.has_value()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Goal pose (%d, %d) is inside obstacle and cannot be snapped.",
            goal_gx,
            goal_gy
        );
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    int actual_start_x = snapped_start->first;
    int actual_start_y = snapped_start->second;
    int actual_goal_x  = snapped_goal->first;
    int actual_goal_y  = snapped_goal->second;

    RCLCPP_INFO(
        this->get_logger(),
        "Planning path from grid (%d, %d) to (%d, %d)...",
        actual_start_x,
        actual_start_y,
        actual_goal_x,
        actual_goal_y
    );

    // 3. 8方向A*探索
    auto grid_path = aStarSearch(actual_start_x, actual_start_y, actual_goal_x, actual_goal_y);

    if (grid_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "A* search failed to find a path.");
        nav_msgs::msg::Path empty_path;
        empty_path.header.stamp    = this->now();
        empty_path.header.frame_id = map_frame_;
        path_pub_->publish(empty_path);
        return;
    }

    // 4. 仕様書:
    // 「探索された格子点列はB-spline等の重厚な平滑化・局所最適化を行わず、そのままワールド座標に変換して/planned_path（nav_msgs/msg/Path）として発行する。」
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = map_frame_;

    path_msg.poses.resize(grid_path.size());
    for (size_t i = 0; i < grid_path.size(); ++i) {
        auto [wx, wy]                     = gridToWorld(grid_path[i].first, grid_path[i].second);
        path_msg.poses[i].header.stamp    = path_msg.header.stamp;
        path_msg.poses[i].header.frame_id = map_frame_;
        path_msg.poses[i].pose.position.x = wx;
        path_msg.poses[i].pose.position.y = wy;
        path_msg.poses[i].pose.position.z = 0.0;

        // 姿勢の計算
        double yaw = 0.0;
        if (i + 1 < grid_path.size()) {
            auto [next_wx, next_wy] = gridToWorld(grid_path[i + 1].first, grid_path[i + 1].second);
            yaw                     = std::atan2(next_wy - wy, next_wx - wx);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            path_msg.poses[i].pose.orientation = tf2::toMsg(q);
        } else {
            // ゴール点では goal_pose の orientation をそのまま使用
            path_msg.poses[i].pose.orientation = msg->pose.orientation;
        }
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(
        this->get_logger(), "Published planned_path with %zu points.", path_msg.poses.size()
    );
}

}  // namespace gn10_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gn10_navigation::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
