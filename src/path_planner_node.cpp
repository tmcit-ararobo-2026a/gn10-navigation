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

    // パラメータの初期化設定
    this->declare_parameter("map_configs.map_frame", "map");
    this->declare_parameter("map_configs.base_frame", "base_link");

    // yamlから受信
    map_frame_  = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_ = this->get_parameter("map_configs.base_frame").as_string();
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
