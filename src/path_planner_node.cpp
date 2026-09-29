#include "gn10_navigation/path_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include "tf2/exceptions.h"

PathPlannerNode::PathPlannerNode() : Node("path_planner_node")
{
    // パラメータ宣言
    this->declare_parameter("cost_factor", 0.05);
    cost_factor_ = this->get_parameter("cost_factor").as_double();

    // QoSの設定
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    // pubとsubの設定
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&PathPlannerNode::getCostmapMsg, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::getGoalPose, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // tfの設定
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timerの設定
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PathPlannerNode::getRobotPose, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path planner started");
}

void PathPlannerNode::getCostmapMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg)
{
    // ① マップの基本情報を取得
    width_      = grid_msg->info.width;
    height_     = grid_msg->info.height;
    resolution_ = grid_msg->info.resolution;

    origin_x_ = grid_msg->info.origin.position.x;
    origin_y_ = grid_msg->info.origin.position.y;

    // ② 1次元配列を2次元配列にコピー
    planning_grid_.assign(height_, std::vector<int>(width_, 0));

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
            planning_grid_[y][x] = static_cast<uint8_t>(grid_msg->data[index]);
        }
    }

    // ③ 受信ログの出力
    RCLCPP_INFO(
        this->get_logger(),
        "Costmap received: width=%u, height=%u, resolution=%.3f",
        width_,
        height_,
        resolution_
    );

    std::map<int, int> value_count;
    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            value_count[planning_grid_[y][x]]++;
        }
    }

    for (const auto& [value, count] : value_count) {
        RCLCPP_INFO(this->get_logger(), "Cost %d: %d cells", value, count);
    }
}

void PathPlannerNode::getGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(), "Goal frame is not map: %s", msg->header.frame_id.c_str());
        return;
    }

    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;

    RCLCPP_INFO(this->get_logger(), "Goal received: x=%.3f, y=%.3f", goal_x_, goal_y_);

    auto [grid_x, grid_y] = worldToGrid(goal_x_, goal_y_);

    if (!isInsideGrid(grid_x, grid_y)) {
        RCLCPP_WARN(this->get_logger(), "Goal is outside the costmap: x=%d, y=%d", grid_x, grid_y);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal grid position: x=%d, y=%d", grid_x, grid_y);

    // 現在のロボット位置をGrid座標へ変換
    auto [start_x, start_y] = worldToGrid(robot_x_, robot_y_);

    // A*を実行
    auto path = aStar(start_x, start_y, grid_x, grid_y);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& cell : path) {
        auto [world_x, world_y] = gridToWorld(cell.first, cell.second);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;

        // 相手ゾーンを常に向いているため Yaw 0rad 固定（w=1.0）
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Published Path with %zu points.", path.size());
}

void PathPlannerNode::getRobotPose()
{
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_x_ = transform.transform.translation.x;
        robot_y_ = transform.transform.translation.y;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Could not get transform: %s", ex.what()
        );
        return;
    }

    auto [grid_x, grid_y] = worldToGrid(robot_x_, robot_y_);

    if (!isInsideGrid(grid_x, grid_y)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Robot is outside the costmap: x=%d, y=%d", grid_x, grid_y
        );
        return;
    }

    if (!isPassable(grid_x, grid_y)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Robot cell is occupied!");
    }
}

std::pair<int, int> PathPlannerNode::worldToGrid(double world_x, double world_y)
{
    int grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    int grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    return {grid_x, grid_y};
}

std::pair<double, double> PathPlannerNode::gridToWorld(int grid_x, int grid_y)
{
    double world_x = origin_x_ + static_cast<double>(grid_x) * resolution_;
    double world_y = origin_y_ + static_cast<double>(grid_y) * resolution_;
    return {world_x, world_y};
}

bool PathPlannerNode::isInsideGrid(int x, int y)
{
    return x >= 0 && x < static_cast<int>(width_) && y >= 0 && y < static_cast<int>(height_);
}

bool PathPlannerNode::isPassable(int x, int y)
{
    if (!isInsideGrid(x, y)) {
        return false;
    }
    // 254(絶対不可侵)以上は侵入不可
    return planning_grid_[y][x] < 254;
}

/**
 *  A* 探索
 */

double heuristic(int x, int y, int goal_x, int goal_y)
{
    const double dx = std::abs(x - goal_x);
    const double dy = std::abs(y - goal_y);

    // 斜めに進める回数
    const double diagonal = std::min(dx, dy);

    // 残りの上下左右移動の回数
    const double straight = std::max(dx, dy) - diagonal;

    // 斜め移動は √2、上下左右は 1
    return diagonal * std::sqrt(2.0) + straight;
}

struct CompareAStarNode {
    bool operator()(const AStarNode& a, const AStarNode& b) const
    {
        return a.f > b.f;
    }
};

std::vector<std::pair<int, int>> PathPlannerNode::aStar(
    int start_x, int start_y, int goal_x, int goal_y
)
{
    std::vector<std::pair<int, int>> path;

    // StartとGoalがマップ内か確認
    if (!isInsideGrid(start_x, start_y) || !isInsideGrid(goal_x, goal_y)) {
        RCLCPP_WARN(this->get_logger(), "A*: Start or Goal is outside the grid.");
        return path;
    }

    // StartとGoalが通行可能か確認
    if (!isPassable(start_x, start_y) || !isPassable(goal_x, goal_y)) {
        RCLCPP_WARN(this->get_logger(), "A*: Start or Goal is not passable.");
        return path;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> open_list;

    // 各セルまでの最短コスト
    std::vector<std::vector<double>> g_cost(
        height_, std::vector<double>(width_, std::numeric_limits<double>::infinity())
    );

    // 親ノード
    std::vector<std::vector<std::pair<int, int>>> parent(
        height_, std::vector<std::pair<int, int>>(width_, {-1, -1})
    );

    const double start_h = heuristic(start_x, start_y, goal_x, goal_y);
    AStarNode start_node{start_x, start_y, 0.0, start_h, start_h};

    open_list.push(start_node);
    g_cost[start_y][start_x] = 0.0;

    // 8方向
    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    bool found = false;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.g > g_cost[current.y][current.x]) {
            continue;
        }

        // Goalに到達
        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        // 周囲8方向を探索
        for (int i = 0; i < 8; ++i) {
            const int next_x = current.x + dx[i];
            const int next_y = current.y + dy[i];

            if (!isInsideGrid(next_x, next_y) || !isPassable(next_x, next_y)) {
                continue;
            }

            // 斜め移動時の角抜け（すり抜け）防止
            if (dx[i] != 0 && dy[i] != 0) {
                if (!isPassable(current.x + dx[i], current.y) ||
                    !isPassable(current.x, current.y + dy[i])) {
                    continue;
                }
            }

            const double move_cost = (dx[i] != 0 && dy[i] != 0) ? std::sqrt(2.0) : 1.0;

            // コストマップの連続グラデーションコスト(0〜253)をペナルティとして直接加算
            const double cost_penalty = static_cast<double>(planning_grid_[next_y][next_x]) * cost_factor_;

            const double new_g = current.g + move_cost + cost_penalty;

            if (new_g >= g_cost[next_y][next_x]) {
                continue;
            }

            const double new_h = heuristic(next_x, next_y, goal_x, goal_y);
            const double new_f = new_g + new_h;

            AStarNode next_node{next_x, next_y, new_g, new_h, new_f};

            g_cost[next_y][next_x] = new_g;
            parent[next_y][next_x] = {current.x, current.y};

            open_list.push(next_node);
        }
    }

    if (!found) {
        RCLCPP_WARN(this->get_logger(), "A*: Path not found.");
        return path;
    }

    // バックトラック
    int current_x = goal_x;
    int current_y = goal_y;

    while (!(current_x == start_x && current_y == start_y)) {
        path.push_back({current_x, current_y});
        const auto [parent_x, parent_y] = parent[current_y][current_x];
        current_x = parent_x;
        current_y = parent_y;
    }

    // Startも追加
    path.push_back({start_x, start_y});

    // Goal → Start になっているので逆順にする
    std::reverse(path.begin(), path.end());

    RCLCPP_INFO(this->get_logger(), "A*: Path found. Length=%zu", path.size());

    return path;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}