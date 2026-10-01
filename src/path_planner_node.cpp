#include "gn10_navigation/path_planner_node.hpp"

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

    // パラメータの初期化設定
    this->declare_parameter("map_configs.map_frame", "map");
    this->declare_parameter("map_configs.base_frame", "base_link");
    this->declare_parameter("snapp_config.start_snap_radius", 1);

    // yamlから受信
    map_frame_         = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_        = this->get_parameter("map_configs.base_frame").as_string();
    start_snap_radius_ = this->get_parameter("snapp_config.start_snap_radius").as_double();
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

    auto [goal_grid_x, goal_grid_y] = worldToGrid(goal_x_, goal_y_);

    int grid_x = goal_grid_x;
    int grid_y = goal_grid_y;

    // 範囲内じゃなかったら
    if (!isInsideGrid(grid_x, grid_y)) {
        RCLCPP_WARN(this->get_logger(), "Goal is outside the costmap: x=%d, y=%d", grid_x, grid_y);

        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal grid position: x=%d, y=%d", grid_x, grid_y);

    auto [robot_grid_x, robot_grid_y] = worldToGrid(robot_x_, robot_y_);

    int start_x = robot_grid_x;
    int start_y = robot_grid_y;

    /*
     * ロボットが膨張領域内にいる場合は、最寄りの通行可能セルからA*を開始する。
     */
    bool start_snapped = false;

    if (!isPassable(start_x, start_y)) {
        const int radius_cells = static_cast<int>(std::ceil(start_snap_radius_ / resolution_));

        const auto snapped = findNearestPassable(start_x, start_y, radius_cells);

        if (!snapped) {
            RCLCPP_WARN(
                this->get_logger(), "Robot is not on a passable cell and no free cell is nearby."
            );

            return;
        }

        start_x       = snapped->first;
        start_y       = snapped->second;
        start_snapped = true;

        RCLCPP_WARN(
            this->get_logger(),
            "Robot cell is occupied. Start snapped to x=%d, y=%d",
            start_x,
            start_y
        );
    }
}

// peripheral
std::pair<int, int> PathPlannerNode::worldToGrid(double wx, double wy)
{
    return {
        static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
        static_cast<int>(std::floor((wy - origin_y_) / resolution_))
    };
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

std::size_t PathPlannerNode::gridtoIndex(int x, int y) const
{
    return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
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