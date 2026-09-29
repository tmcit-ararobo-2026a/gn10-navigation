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
    this->declare_parameter("cost_factor", 0.05);

    this->declare_parameter("bspline.length_weight", 1.0);
    this->declare_parameter("bspline.curvature_weight", 1.0);
    this->declare_parameter("bspline.optimization_iterations", 20);
    this->declare_parameter("bspline.control_point_step", 0.05);
    this->declare_parameter("bspline.samples_per_segment", 10);

    cost_factor_ = this->get_parameter("cost_factor").as_double();

    bspline_length_weight_ = this->get_parameter("bspline.length_weight").as_double();

    bspline_curvature_weight_ = this->get_parameter("bspline.curvature_weight").as_double();

    bspline_optimization_iterations_ =
        this->get_parameter("bspline.optimization_iterations").as_int();

    bspline_control_point_step_ = this->get_parameter("bspline.control_point_step").as_double();

    bspline_samples_per_segment_ = this->get_parameter("bspline.samples_per_segment").as_int();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", map_qos, std::bind(&PathPlannerNode::getCostmapMsg, this, std::placeholders::_1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::getGoalPose, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PathPlannerNode::getRobotPose, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path planner started");
}

void PathPlannerNode::getCostmapMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg)
{
    width_      = grid_msg->info.width;
    height_     = grid_msg->info.height;
    resolution_ = grid_msg->info.resolution;

    origin_x_ = grid_msg->info.origin.position.x;
    origin_y_ = grid_msg->info.origin.position.y;

    planning_grid_.assign(height_, std::vector<int>(width_, 0));

    for (unsigned int y = 0; y < height_; ++y) {
        for (unsigned int x = 0; x < width_; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * width_ + x;

            planning_grid_[y][x] = static_cast<uint8_t>(grid_msg->data[index]);
        }
    }

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

    if (planning_grid_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Costmap has not been received yet.");
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

    auto [start_x, start_y] = worldToGrid(robot_x_, robot_y_);

    auto path = aStar(start_x, start_y, grid_x, grid_y);

    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No A* path available.");
        return;
    }

    auto smoothed_path = bsplineSmoothPath(path);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& point : smoothed_path) {
        geometry_msgs::msg::PoseStamped pose;

        pose.header = path_msg.header;

        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Published Path with %zu points.", path_msg.poses.size());
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
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Robot is outside the costmap: x=%d, y=%d",
            grid_x,
            grid_y
        );

        return;
    }

    if (!isPassable(grid_x, grid_y)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Robot cell is occupied!"
        );
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

    return planning_grid_[y][x] < 254;
}

double heuristic(int x, int y, int goal_x, int goal_y)
{
    const double dx = std::abs(x - goal_x);

    const double dy = std::abs(y - goal_y);

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

std::vector<std::pair<int, int>> PathPlannerNode::aStar(
    int start_x, int start_y, int goal_x, int goal_y
)
{
    std::vector<std::pair<int, int>> path;

    if (!isInsideGrid(start_x, start_y) || !isInsideGrid(goal_x, goal_y)) {
        RCLCPP_WARN(this->get_logger(), "A*: Start or Goal is outside the grid.");

        return path;
    }

    if (!isPassable(start_x, start_y) || !isPassable(goal_x, goal_y)) {
        RCLCPP_WARN(this->get_logger(), "A*: Start or Goal is not passable.");

        return path;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> open_list;

    std::vector<std::vector<double>> g_cost(
        height_, std::vector<double>(width_, std::numeric_limits<double>::infinity())
    );

    std::vector<std::vector<std::pair<int, int>>> parent(
        height_, std::vector<std::pair<int, int>>(width_, {-1, -1})
    );

    const double start_h = heuristic(start_x, start_y, goal_x, goal_y);

    AStarNode start_node{start_x, start_y, 0.0, start_h, start_h};

    open_list.push(start_node);
    g_cost[start_y][start_x] = 0.0;

    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    bool found = false;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();

        open_list.pop();

        if (current.g > g_cost[current.y][current.x]) {
            continue;
        }

        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            const int next_x = current.x + dx[i];

            const int next_y = current.y + dy[i];

            if (!isInsideGrid(next_x, next_y) || !isPassable(next_x, next_y)) {
                continue;
            }

            if (dx[i] != 0 && dy[i] != 0) {
                if (!isPassable(current.x + dx[i], current.y) ||
                    !isPassable(current.x, current.y + dy[i])) {
                    continue;
                }
            }

            const double move_cost = (dx[i] != 0 && dy[i] != 0) ? std::sqrt(2.0) : 1.0;

            const double cost_penalty =
                static_cast<double>(planning_grid_[next_y][next_x]) * cost_factor_;

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

    int current_x = goal_x;
    int current_y = goal_y;

    while (!(current_x == start_x && current_y == start_y)) {
        path.push_back({current_x, current_y});

        const auto [parent_x, parent_y] = parent[current_y][current_x];

        current_x = parent_x;
        current_y = parent_y;
    }

    path.push_back({start_x, start_y});

    std::reverse(path.begin(), path.end());

    RCLCPP_INFO(this->get_logger(), "A*: Path found. Length=%zu", path.size());

    return path;
}

// b spline
std::vector<std::pair<double, double>> PathPlannerNode::bsplineSmoothPath(
    const std::vector<std::pair<int, int>>& path
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> result;

    if (path.empty()) {
        return result;
    }

    for (const auto& cell : path) {
        result.push_back(gridToWorld(cell.first, cell.second));
    }

    if (result.size() < 4) {
        return result;
    }

    std::vector<Point> control_points = result;

    const int degree        = 3;
    const int control_count = static_cast<int>(control_points.size());

    const int max_t = control_count - degree;

    std::vector<double> knots(control_count + degree + 1);

    for (int i = 0; i < static_cast<int>(knots.size()); ++i) {
        if (i <= degree) {
            knots[i] = 0.0;
        } else if (i >= control_count) {
            knots[i] = static_cast<double>(max_t);
        } else {
            knots[i] = static_cast<double>(i - degree);
        }
    }

    auto evaluateBSpline = [&](const std::vector<Point>& points, double t) -> Point {
        if (t <= 0.0) {
            return points.front();
        }

        if (t >= static_cast<double>(max_t)) {
            return points.back();
        }

        int span = degree;

        for (int i = degree; i < control_count; ++i) {
            if (t >= knots[i] && t < knots[i + 1]) {
                span = i;
                break;
            }
        }

        std::vector<Point> d(degree + 1);

        for (int j = 0; j <= degree; ++j) {
            d[j] = points[span - degree + j];
        }

        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                const int i = span - degree + j;

                const double denominator = knots[i + degree - r + 1] - knots[i];

                double alpha = 0.0;

                if (denominator > 0.0) {
                    alpha = (t - knots[i]) / denominator;
                }

                d[j].first = (1.0 - alpha) * d[j - 1].first + alpha * d[j].first;

                d[j].second = (1.0 - alpha) * d[j - 1].second + alpha * d[j].second;
            }
        }

        return d[degree];
    };

    auto sampleBSpline = [&](const std::vector<Point>& points) {
        std::vector<Point> sampled;

        const int total_samples = max_t * bspline_samples_per_segment_;

        sampled.reserve(total_samples + 1);

        for (int i = 0; i <= total_samples; ++i) {
            const double t = static_cast<double>(max_t) * static_cast<double>(i) /
                             static_cast<double>(total_samples);

            sampled.push_back(evaluateBSpline(points, t));
        }

        return sampled;
    };

    auto initial_path = sampleBSpline(control_points);

    if (!isSmoothPathValid(initial_path)) {
        return result;
    }

    const double base_length = calculatePathLength(initial_path);

    const double base_curvature = calculateCurvatureCost(initial_path);

    double current_cost = calculatePathCost(initial_path, base_length, base_curvature);

    const int iterations = std::max(1, bspline_optimization_iterations_);

    const double step = std::max(0.001, bspline_control_point_step_);

    for (int iteration = 0; iteration < iterations; ++iteration) {
        bool improved = false;

        for (int i = 1; i < control_count - 1; ++i) {
            const Point current = control_points[i];

            const Point previous = control_points[i - 1];

            const Point next = control_points[i + 1];

            const Point midpoint{
                (previous.first + next.first) * 0.5, (previous.second + next.second) * 0.5
            };

            const double direction_x = midpoint.first - current.first;

            const double direction_y = midpoint.second - current.second;

            const double direction_length = std::hypot(direction_x, direction_y);

            if (direction_length < 1e-9) {
                continue;
            }

            const double nx = direction_x / direction_length;

            const double ny = direction_y / direction_length;

            std::vector<Point> candidates;

            candidates.push_back(current);

            const double candidate_steps[] = {0.25, 0.5, 0.75, 1.0};

            for (double ratio : candidate_steps) {
                const double distance = std::min(step, direction_length) * ratio;

                candidates.push_back(
                    {current.first + nx * distance, current.second + ny * distance}
                );
            }

            for (const auto& candidate : candidates) {
                if (candidate == current) {
                    continue;
                }

                control_points[i] = candidate;

                auto candidate_path = sampleBSpline(control_points);

                if (!isSmoothPathValid(candidate_path)) {
                    control_points[i] = current;

                    continue;
                }

                const double candidate_cost =
                    calculatePathCost(candidate_path, base_length, base_curvature);

                if (candidate_cost < current_cost) {
                    current_cost = candidate_cost;

                    improved = true;
                } else {
                    control_points[i] = current;
                }
            }
        }

        if (!improved) {
            break;
        }
    }

    result = sampleBSpline(control_points);

    if (!isSmoothPathValid(result)) {
        result.clear();

        for (const auto& cell : path) {
            result.push_back(gridToWorld(cell.first, cell.second));
        }

        RCLCPP_WARN(this->get_logger(), "Optimized B-spline path is invalid. Using A* path.");

        return result;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "B-spline optimized: length=%.3f -> %.3f, curvature=%.3f -> %.3f",
        base_length,
        calculatePathLength(result),
        base_curvature,
        calculateCurvatureCost(result)
    );

    return result;
}

double PathPlannerNode::calculatePathLength(const std::vector<std::pair<double, double>>& path)
{
    if (path.size() < 2) {
        return 0.0;
    }

    double length = 0.0;

    for (std::size_t i = 1; i < path.size(); ++i) {
        const double dx = path[i].first - path[i - 1].first;

        const double dy = path[i].second - path[i - 1].second;

        length += std::hypot(dx, dy);
    }

    return length;
}

double PathPlannerNode::calculateCurvatureCost(const std::vector<std::pair<double, double>>& path)
{
    if (path.size() < 3) {
        return 0.0;
    }

    double cost = 0.0;

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const double ax = path[i].first - path[i - 1].first;

        const double ay = path[i].second - path[i - 1].second;

        const double bx = path[i + 1].first - path[i].first;

        const double by = path[i + 1].second - path[i].second;

        const double a = std::hypot(ax, ay);

        const double b = std::hypot(bx, by);

        const double c = std::hypot(
            path[i + 1].first - path[i - 1].first, path[i + 1].second - path[i - 1].second
        );

        if (a < 1e-9 || b < 1e-9 || c < 1e-9) {
            continue;
        }

        const double cross = ax * by - ay * bx;

        const double curvature = (2.0 * std::abs(cross)) / (a * b * c);

        const double ds = 0.5 * (a + b);

        cost += curvature * curvature * ds;
    }

    return cost;
}

double PathPlannerNode::calculatePathCost(
    const std::vector<std::pair<double, double>>& path, double base_length, double base_curvature
)
{
    const double length = calculatePathLength(path);

    const double curvature = calculateCurvatureCost(path);

    const double normalized_length = length / std::max(base_length, 1e-9);

    double normalized_curvature = 0.0;

    if (base_curvature > 1e-9) {
        normalized_curvature = curvature / base_curvature;
    } else {
        normalized_curvature = curvature;
    }

    return bspline_length_weight_ * normalized_length +
           bspline_curvature_weight_ * normalized_curvature;
}

bool PathPlannerNode::isSmoothPathValid(const std::vector<std::pair<double, double>>& path)
{
    if (path.empty()) {
        return false;
    }

    for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& point = path[i];

        auto [grid_x, grid_y] = worldToGrid(point.first, point.second);

        if (!isInsideGrid(grid_x, grid_y)) {
            return false;
        }

        if (!isPassable(grid_x, grid_y)) {
            return false;
        }

        if (i > 0) {
            const auto& previous = path[i - 1];

            const double dx = point.first - previous.first;

            const double dy = point.second - previous.second;

            const double distance = std::hypot(dx, dy);

            const int samples =
                std::max(1, static_cast<int>(std::ceil(distance / (resolution_ * 0.5))));

            for (int j = 1; j <= samples; ++j) {
                const double ratio = static_cast<double>(j) / static_cast<double>(samples);

                const double x = previous.first + dx * ratio;

                const double y = previous.second + dy * ratio;

                auto [sample_x, sample_y] = worldToGrid(x, y);

                if (!isInsideGrid(sample_x, sample_y)) {
                    return false;
                }

                if (!isPassable(sample_x, sample_y)) {
                    return false;
                }
            }
        }
    }

    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PathPlannerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}