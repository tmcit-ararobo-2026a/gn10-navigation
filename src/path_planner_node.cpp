#include "gn10_navigation/path_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>

// メンバ関数じゃない計算を行うプログラム
namespace {

double calculateAngleDifference(double ax, double ay, double bx, double by)
{
    const double a_length = std::hypot(ax, ay);
    const double b_length = std::hypot(bx, by);

    if (a_length < 1e-9 || b_length < 1e-9) {
        return 0.0;
    }

    double cos_angle = (ax * bx + ay * by) / (a_length * b_length);
    cos_angle        = std::clamp(cos_angle, -1.0, 1.0);

    return std::acos(cos_angle) * 180.0 / M_PI;
}

// 8方向に障害物がないとしたときのゴールまでの最短距離（最小腰コストを求める）
double heuristic(int x, int y, int goal_x, int goal_y)
{
    const double dx       = std::abs(x - goal_x);
    const double dy       = std::abs(y - goal_y);
    const double diagonal = std::min(dx, dy);
    const double straight = std::max(dx, dy) - diagonal;

    return diagonal * std::sqrt(2.0) + straight;
}

struct CompareAStarNode {
    bool operator()(const AStarNode& a, const AStarNode& b) const
    {
        return a.f > b.f;
    }
};

}  // namespace

PathPlannerNode::PathPlannerNode() : Node("path_planner_node")
{
    // Qos config
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    // subscriber
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&PathPlannerNode::getCostmapMsg, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::getGoalPose, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // tf
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パラメータの初期化設定
    this->declare_parameter("map_configs.map_frame", "map");
    this->declare_parameter("map_configs.base_frame", "base_link");
    this->declare_parameter("snapp_config.start_snap_radius", 1.0);
    this->declare_parameter("a_star.plannning_cost", 3.0);

    // yamlから受信
    map_frame_         = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_        = this->get_parameter("map_configs.base_frame").as_string();
    start_snap_radius_ = this->get_parameter("snapp_config.start_snap_radius").as_double();
    cost_factor_       = this->get_parameter("a_star.plannning_cost").as_double();
}

void PathPlannerNode::getCostmapMsg(nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg)
{
    // ERROR msg
    const unsigned int new_width  = grid_msg->info.width;
    const unsigned int new_height = grid_msg->info.height;

    if (grid_msg->data.size() < static_cast<std::size_t>(new_width) * new_height) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Costmap data is too small: data=%zu, expected=%zu",
            grid_msg->data.size(),
            static_cast<std::size_t>(new_width) * new_height
        );

        return;
    }

    if (grid_msg->info.resolution <= 0.0F) {
        RCLCPP_ERROR(this->get_logger(), "Costmap resolution is invalid.");

        return;
    }

    if (grid_msg->header.frame_id != map_frame_) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Costmap frame is not %s: %s",
            map_frame_.c_str(),
            grid_msg->header.frame_id.c_str()
        );

        return;
    }

    // 値を代入
    resolution_ = grid_msg->info.resolution;
    width_      = grid_msg->info.width;
    height_     = grid_msg->info.height;
    origin_x_   = grid_msg->info.origin.position.x;
    origin_y_   = grid_msg->info.origin.position.y;

    // 1次元配列を2次元配列にコピー
    planning_grid_.assign(static_cast<std::size_t>(width_) * height_, 0);

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width_ + x;

            planning_grid_[index] = static_cast<uint8_t>(grid_msg->data[index]);
        }
    }

    // 受信ログの出力
    RCLCPP_INFO(
        this->get_logger(),
        "Costmap received: width=%u, height=%u, resolution=%.3f",
        width_,
        height_,
        resolution_
    );

    // コスト値と個数
    std::map<int, int> value_count;

    // 何個そのコストがあったのか計算
    for (const uint8_t value : planning_grid_) {
        value_count[value]++;
    }
    for (const auto& [value, count] : value_count) {
        RCLCPP_INFO(this->get_logger(), "Cost %d: %d cells", value, count);
    }
}

bool PathPlannerNode::getRobotPose()
{
    // tfに例外が発生して受信できなかった場合座標変換しない
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        robot_x_       = transform.transform.translation.x;
        robot_y_       = transform.transform.translation.y;
        robot_yaw_     = tf2::getYaw(transform.transform.rotation);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Could not get transform: %s", ex.what()
        );

        return false;
    }

    // コストマップ未受信(resolution_ = 0)のときに座標変換しない
    if (planning_grid_.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Costmap has not been received yet."
        );

        return false;
    }

    auto [grid_x, grid_y] = worldToGrid(robot_x_, robot_y_);

    // 座標が範囲内に収まっているかどうかcheckする関数
    if (!isInsideGrid(grid_x, grid_y)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Robot is outside the costmap: x=%d, y=%d",
            grid_x,
            grid_y
        );
        return false;
    }

    // 通行不可ならコメントを出す
    if (!isPassable(grid_x, grid_y)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Robot cell is occupied!"
        );
    }

    return true;
}

void PathPlannerNode::getGoalPose(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
    // mapを受信してから
    if (planning_grid_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Costmap has not been received yet.");
        return;
    }

    // frameが違かったら
    if (pose_msg->header.frame_id != map_frame_) {
        RCLCPP_WARN(
            this->get_logger(), "Goal frame is not map: %s", pose_msg->header.frame_id.c_str()
        );
        return;
    }

    // robotのpositionがわからなかったら
    if (!getRobotPose()) {
        RCLCPP_WARN(this->get_logger(), "Robot pose is unavailable. Planning aborted.");

        return;
    }

    goal_x_ = pose_msg->pose.position.x;
    goal_y_ = pose_msg->pose.position.y;

    RCLCPP_INFO(this->get_logger(), "Goal received: x=%.3f, y=%.3f", goal_x_, goal_y_);

    auto [goal_grid_x_, goal_grid_y_] = worldToGrid(goal_x_, goal_y_);

    int goal_grid_x = goal_grid_x_;
    int goal_grid_y = goal_grid_y_;

    // 範囲内じゃなかったら
    if (!isInsideGrid(goal_grid_x, goal_grid_y)) {
        RCLCPP_WARN(
            this->get_logger(), "Goal is outside the costmap: x=%d, y=%d", goal_grid_x, goal_grid_y
        );

        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal grid position: x=%d, y=%d", goal_grid_x, goal_grid_y);

    auto [robot_grid_x, robot_grid_y] = worldToGrid(robot_x_, robot_y_);

    int start_grid_x = robot_grid_x;
    int start_grid_y = robot_grid_y;

    /*
     * ロボットが膨張領域内にいる場合は、最寄りの通行可能セルからA*を開始する。
     */
    bool start_snapped = false;

    if (!isPassable(start_grid_x, start_grid_y)) {
        const int radius_cells = static_cast<int>(std::ceil(start_snap_radius_ / resolution_));

        const auto snapped = findNearestPassable(start_grid_x, start_grid_y, radius_cells);

        if (!snapped) {
            RCLCPP_WARN(
                this->get_logger(), "Robot is not on a passable cell and no free cell is nearby."
            );

            return;
        }

        start_grid_x  = snapped->first;
        start_grid_y  = snapped->second;
        start_snapped = true;

        RCLCPP_WARN(
            this->get_logger(),
            "Robot cell is occupied. Start snapped to x=%d, y=%d",
            start_grid_x,
            start_grid_y
        );
    }

    // A*の配列にstartとgoalのgridを代入
    auto path = aStar(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);

    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No A* path available.");
        return;
    }

    // 一旦コンパイルのために書き換え
    std::vector<std::pair<double, double>> smoothed_path;
    for (const auto& cell : path) {
        smoothed_path.push_back(gridToWorld(cell.first, cell.second));
    }
    /*
        // b-spineはA*が使う
        auto smoothed_path = bsplineSmoothPath(path);
        if (smoothed_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "B-spline path generation failed.");
            return;
        }*/

    // パスの端点を、セル中心ではなく実際のロボット位置・ゴール位置に合わせる。
    //(同じセル内なので通行可能性は変わらない。スナップした場合は行わない。)
    if (!start_snapped && smoothed_path.size() >= 2) {
        smoothed_path.front() = {robot_x_, robot_y_};
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = map_frame_;

    for (std::size_t i = 0; i < smoothed_path.size(); ++i) {
        const auto& point = smoothed_path[i];

        geometry_msgs::msg::PoseStamped pose;

        pose.header = path_msg.header;

        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;

        if (i + 1 == smoothed_path.size()) {
            // 最終点だけ、Goalで指定された姿勢にする
            pose.pose.orientation = pose_msg->pose.orientation;
        } else {
            // 移動中はロボットの現在の向きを維持する
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
            pose.pose.orientation.w = std::cos(robot_yaw_ * 0.5);
        }

        path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Published Path with %zu points.", path_msg.poses.size());
}

// peripheral
std::pair<int, int> PathPlannerNode::worldToGrid(double wx, double wy)
{
    return {
        static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
        static_cast<int>(std::floor((wy - origin_y_) / resolution_))
    };
}

std::pair<double, double> PathPlannerNode::gridToWorld(int gx, int gy)
{
    return {origin_x_ + (gx + 0.5) * resolution_, origin_y_ + (gy + 0.5) * resolution_};
}

std::size_t PathPlannerNode::gridtoIndex(int x, int y) const
{
    return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
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

    return planning_grid_[gridtoIndex(x, y)] < 254;
}

std::optional<std::pair<int, int>> PathPlannerNode::findNearestPassable(
    int x, int y, int max_radius_cells
)
{
    // 半径内の全セルを調べ、ユークリッド距離が最も近い通行可能セルを返す。

    if (max_radius_cells <= 0) {
        return std::nullopt;
    }

    double best_distance = std::numeric_limits<double>::infinity();

    std::optional<std::pair<int, int>> best;

    for (int dy = -max_radius_cells; dy <= max_radius_cells; ++dy) {
        for (int dx = -max_radius_cells; dx <= max_radius_cells; ++dx) {
            const int nx = x + dx;

            const int ny = y + dy;

            if (!isPassable(nx, ny)) {
                continue;
            }

            const double distance = std::hypot(dx, dy);

            if (distance > max_radius_cells) {
                continue;
            }

            if (distance < best_distance) {
                best_distance = distance;

                best = std::make_pair(nx, ny);
            }
        }
    }

    return best;
}

std::vector<std::pair<int, int>> PathPlannerNode::aStar(
    int start_x, int start_y, int goal_x, int goal_y
)
{
    std::vector<std::pair<int, int>> path;

    if (!isPassable(start_x, start_y) || !isPassable(goal_x, goal_y)) {
        RCLCPP_WARN(this->get_logger(), "A*: Start or Goal is not passable.");
        return path;
    }

    // 候補セルの待ち行列。合計推定コストが一番小さいやつから出てくる。
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> open_list;

    const std::size_t cell_count = static_cast<std::size_t>(width_) * height_;
    std::vector<double> g_cost(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);

    const double start_h = heuristic(start_x, start_y, goal_x, goal_y);
    open_list.push(AStarNode{start_x, start_y, 0.0, start_h, start_h});
    g_cost[gridtoIndex(start_x, start_y)] = 0.0;

    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    bool found = false;

    // listが空になるまで探索 より良いコスト計算
    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        // 古い(より悪いコストで登録された)ノードは捨てる
        if (current.g > g_cost[gridtoIndex(current.x, current.y)]) {
            continue;
        }

        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            const int next_x = current.x + dx[i];
            const int next_y = current.y + dy[i];

            if (!isPassable(next_x, next_y)) {
                continue;
            }

            // 斜め移動は、角を挟む2セルも通行可能なときだけ許可
            if (dx[i] != 0 && dy[i] != 0) {
                if (!isPassable(current.x + dx[i], current.y) ||
                    !isPassable(current.x, current.y + dy[i])) {
                    continue;
                }
            }

            const double move_cost = (dx[i] != 0 && dy[i] != 0) ? std::sqrt(2.0) : 1.0;
            const double cost_penalty =
                static_cast<double>(planning_grid_[gridtoIndex(next_x, next_y)]) * cost_factor_;
            const double new_g = current.g + move_cost + cost_penalty;

            if (new_g >= g_cost[gridtoIndex(next_x, next_y)]) {
                continue;
            }

            const double new_h = heuristic(next_x, next_y, goal_x, goal_y);

            g_cost[gridtoIndex(next_x, next_y)] = new_g;
            parent[gridtoIndex(next_x, next_y)] =
                static_cast<int>(gridtoIndex(current.x, current.y));

            open_list.push(AStarNode{next_x, next_y, new_g, new_h, new_g + new_h});
        }
    }

    if (!found) {
        RCLCPP_WARN(this->get_logger(), "A*: Path not found.");
        return path;
    }

    const int start_index = static_cast<int>(gridtoIndex(start_x, start_y));
    int current_index     = static_cast<int>(gridtoIndex(goal_x, goal_y));

    while (current_index != start_index) {
        if (current_index < 0) {
            RCLCPP_ERROR(this->get_logger(), "A*: Parent chain is broken.");
            path.clear();
            return path;
        }

        path.push_back(
            {current_index % static_cast<int>(width_), current_index / static_cast<int>(width_)}
        );
        current_index = parent[static_cast<std::size_t>(current_index)];
    }

    path.push_back({start_x, start_y});
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