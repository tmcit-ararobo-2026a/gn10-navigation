#include "gn10_navigation/path_planner_node.hpp"

#include <chrono>
#include <functional>
#include <map>

#include "tf2/exceptions.h"

PathPlannerNode::PathPlannerNode()
    : Node("path_planner_node")
{
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    costmap_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap",
            map_qos,
            std::bind(
                &PathPlannerNode::getCostmapMsg,
                this,
                std::placeholders::_1
            )
        );

    goal_pose_sub_ = 
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(
                &PathPlannerNode::getGoalPose,
                this,
                std::placeholders::_1
            )

        );   

    tf_buffer_ =
        std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(
            &PathPlannerNode::getRobotPose,
            this
        )
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Path planner started"
    );
}

void PathPlannerNode::getCostmapMsg(
    const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg)
{
    // ① マップの基本情報を取得
    width_ = grid_msg->info.width;
    height_ = grid_msg->info.height;
    resolution_ = grid_msg->info.resolution;

    origin_x_ = grid_msg->info.origin.position.x;
    origin_y_ = grid_msg->info.origin.position.y;

    // ② 1次元配列を2次元配列に変換
    grid_.assign(
        height_,
        std::vector<int>(width_, 0)
    );

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {

            // OccupancyGridの1次元配列上のインデックス
            const std::size_t index =
                static_cast<std::size_t>(y) * width_ + x;

            // 1次元配列から2次元配列へコピー
            grid_[y][x] =
                static_cast<uint8_t>(grid_msg->data[index]);
        }
    }

    // ③ 受信結果を表示
    RCLCPP_INFO(
        this->get_logger(),
        "Costmap received: width=%u, height=%u, resolution=%.3f",
        width_,
        height_,
        resolution_
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Grid cell (0, 0) = %d",
        grid_[0][0]
    );

    int free_count = 0;
    int occupied_count = 0;
    int unknown_count = 0;
    int other_count = 0;

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {

            const int value = grid_[y][x];

            if (value == 0) {
                free_count++;
            } else if (value == 100) {
                occupied_count++;
            } else if (value == -1) {
                unknown_count++;
            } else {
                other_count++;
            }
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Free: %d, Occupied: %d, Unknown: %d, Other: %d",
        free_count,
        occupied_count,
        unknown_count,
        other_count
    );

    std::map<int, int> value_count;

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            value_count[grid_[y][x]]++;
        }
    }

    for (const auto& [value, count] : value_count) {
        RCLCPP_INFO(
            this->get_logger(),
            "Cost %d: %d cells",
            value,
            count
        );
    }
}

void PathPlannerNode::getRobotPose()
{
    try {
        auto transform =
            tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero
            );

        robot_x_ =
            transform.transform.translation.x;

        robot_y_ =
            transform.transform.translation.y;

        RCLCPP_INFO(
            this->get_logger(),
            "Robot position: x=%.3f, y=%.3f",
            robot_x_,
            robot_y_
        );
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not get transform: %s",
            ex.what()
        );
    }

    auto [grid_x, grid_y] =
    worldToGrid(robot_x_, robot_y_);

    RCLCPP_INFO(
        this->get_logger(),
        "Robot grid position: x=%d, y=%d",
        grid_x,
        grid_y
    );
}

void PathPlannerNode::getGoalPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(
            this->get_logger(),
            "Goal frame is not map: %s",
            msg->header.frame_id.c_str()
        );
        return;
    }

    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;

    RCLCPP_INFO(
        this->get_logger(),
        "Goal received: x=%.3f, y=%.3f",
        goal_x_,
        goal_y_
    );

    auto [grid_x, grid_y] =
        worldToGrid(goal_x_, goal_y_);

    RCLCPP_INFO(
        this->get_logger(),
        "Goal grid position: x=%d, y=%d",
        grid_x,
        grid_y
    );
}

std::pair<int, int> PathPlannerNode::worldToGrid(
    double world_x,
    double world_y)
{
    int grid_x =
        static_cast<int>(
            (world_x - origin_x_) / resolution_
        );

    int grid_y =
        static_cast<int>(
            (world_y - origin_y_) / resolution_
        );

    return {grid_x, grid_y};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}