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

namespace {

double calculateAngleDifference(double ax, double ay, double bx, double by)
{
    const double a_length = std::hypot(ax, ay);
    const double b_length = std::hypot(bx, by);

    if (a_length < 1e-9 || b_length < 1e-9) {
        return 0.0;
    }

    double cos_angle = (ax * bx + ay * by) / (a_length * b_length);

    cos_angle = std::clamp(cos_angle, -1.0, 1.0);

    return std::acos(cos_angle) * 180.0 / M_PI;
}

}  // namespace

PathPlannerNode::PathPlannerNode() : Node("path_planner_node")
{
    this->declare_parameter("cost_factor", cost_factor_);

    this->declare_parameter("bspline.length_weight", bspline_length_weight_);

    this->declare_parameter("bspline.curvature_weight", bspline_curvature_weight_);

    this->declare_parameter("bspline.smoothness_weight", bspline_smoothness_weight_);

    this->declare_parameter("bspline.smoothing_iterations", bspline_smoothing_iterations_);

    this->declare_parameter("bspline.optimization_iterations", bspline_optimization_iterations_);

    this->declare_parameter("bspline.control_point_step", bspline_control_point_step_);

    this->declare_parameter("bspline.samples_per_segment", bspline_samples_per_segment_);

    this->declare_parameter("bspline.turning_angle_threshold", bspline_turning_angle_threshold_);

    this->declare_parameter("bspline.simplification_tolerance", bspline_simplification_tolerance_);

    cost_factor_ = this->get_parameter("cost_factor").as_double();

    bspline_length_weight_ = this->get_parameter("bspline.length_weight").as_double();

    bspline_curvature_weight_ = this->get_parameter("bspline.curvature_weight").as_double();

    bspline_smoothness_weight_ = this->get_parameter("bspline.smoothness_weight").as_double();

    bspline_optimization_iterations_ =
        this->get_parameter("bspline.optimization_iterations").as_int();

    bspline_control_point_step_ = this->get_parameter("bspline.control_point_step").as_double();

    bspline_samples_per_segment_ = this->get_parameter("bspline.samples_per_segment").as_int();

    bspline_turning_angle_threshold_ =
        this->get_parameter("bspline.turning_angle_threshold").as_double();

    bspline_simplification_tolerance_ =
        this->get_parameter("bspline.simplification_tolerance").as_double();

    bspline_smoothing_iterations_ = this->get_parameter("bspline.smoothing_iterations").as_int();

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
    width_  = grid_msg->info.width;
    height_ = grid_msg->info.height;

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

    if (smoothed_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "B-spline path generation failed.");

        return;
    }

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
    const int grid_x = static_cast<int>((world_x - origin_x_) / resolution_);

    const int grid_y = static_cast<int>((world_y - origin_y_) / resolution_);

    return {grid_x, grid_y};
}

std::pair<double, double> PathPlannerNode::gridToWorld(int grid_x, int grid_y)
{
    const double world_x = origin_x_ + static_cast<double>(grid_x) * resolution_;

    const double world_y = origin_y_ + static_cast<double>(grid_y) * resolution_;

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

std::vector<std::pair<double, double>> PathPlannerNode::extractImportantPoints(
    const std::vector<std::pair<int, int>>& path
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> points;

    if (path.empty()) {
        return points;
    }

    for (const auto& cell : path) {
        points.push_back(gridToWorld(cell.first, cell.second));
    }

    if (points.size() <= 2) {
        return points;
    }

    /*
     * まずRDPで、直線的な部分の不要な点を削除する。
     */
    std::vector<bool> keep(points.size(), false);

    keep.front() = true;
    keep.back()  = true;

    simplifyPathRecursive(
        points, 0, static_cast<int>(points.size()) - 1, bspline_simplification_tolerance_, keep
    );

    /*
     * 次に、一定以上の角度で曲がっている点を残す。
     */
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const double v1x = points[i].first - points[i - 1].first;

        const double v1y = points[i].second - points[i - 1].second;

        const double v2x = points[i + 1].first - points[i].first;

        const double v2y = points[i + 1].second - points[i].second;

        const double angle = calculateAngleDifference(v1x, v1y, v2x, v2y);

        if (angle >= bspline_turning_angle_threshold_) {
            keep[i] = true;
        }
    }

    /*
     * ここが重要。
     *
     * RDP + 角度判定だけでは4点未満になる場合がある。
     *
     * その場合は「均等間隔」で点を追加するのではなく、
     * A*経路の中から曲がり角が大きい順に重要な点を追加する。
     */
    std::size_t kept_count = 0;

    for (bool value : keep) {
        if (value) {
            ++kept_count;
        }
    }

    if (kept_count < 4) {
        struct ImportantCandidate {
            std::size_t index;
            double angle;
        };

        std::vector<ImportantCandidate> candidates;

        for (std::size_t i = 1; i + 1 < points.size(); ++i) {
            const double v1x = points[i].first - points[i - 1].first;

            const double v1y = points[i].second - points[i - 1].second;

            const double v2x = points[i + 1].first - points[i].first;

            const double v2y = points[i + 1].second - points[i].second;

            const double angle = calculateAngleDifference(v1x, v1y, v2x, v2y);

            candidates.push_back({i, angle});
        }

        /*
         * 曲がり角の大きい順に並べる。
         */
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const ImportantCandidate& a, const ImportantCandidate& b) {
                return a.angle > b.angle;
            }
        );

        /*
         * 重要な点を追加していく。
         */
        for (const auto& candidate : candidates) {
            if (kept_count >= 4) {
                break;
            }

            if (!keep[candidate.index]) {
                keep[candidate.index] = true;
                ++kept_count;
            }
        }
    }

    /*
     * 元のA*経路順を維持したまま、
     * keepされた点だけを取り出す。
     */
    std::vector<Point> important_points;

    for (std::size_t i = 0; i < keep.size(); ++i) {
        if (keep[i]) {
            important_points.push_back(points[i]);
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Important points: %zu / A* points: %zu",
        important_points.size(),
        points.size()
    );

    return important_points;
}

void PathPlannerNode::simplifyPathRecursive(
    const std::vector<std::pair<double, double>>& points,
    int start,
    int end,
    double tolerance,
    std::vector<bool>& keep
)
{
    if (end <= start + 1) {
        return;
    }

    double max_distance = 0.0;
    int max_index       = -1;

    for (int i = start + 1; i < end; ++i) {
        const double distance = pointToLineDistance(points[i], points[start], points[end]);

        if (distance > max_distance) {
            max_distance = distance;
            max_index    = i;
        }
    }

    if (max_index >= 0 && max_distance > tolerance) {
        keep[max_index] = true;

        simplifyPathRecursive(points, start, max_index, tolerance, keep);

        simplifyPathRecursive(points, max_index, end, tolerance, keep);
    }
}

double PathPlannerNode::pointToLineDistance(
    const std::pair<double, double>& point,
    const std::pair<double, double>& line_start,
    const std::pair<double, double>& line_end
)
{
    const double dx = line_end.first - line_start.first;

    const double dy = line_end.second - line_start.second;

    const double length_squared = dx * dx + dy * dy;

    if (length_squared < 1e-12) {
        return std::hypot(point.first - line_start.first, point.second - line_start.second);
    }

    double t = ((point.first - line_start.first) * dx + (point.second - line_start.second) * dy) /
               length_squared;

    t = std::clamp(t, 0.0, 1.0);

    const double closest_x = line_start.first + t * dx;

    const double closest_y = line_start.second + t * dy;

    return std::hypot(point.first - closest_x, point.second - closest_y);
}

std::vector<std::pair<double, double>> PathPlannerNode::evaluateBSplinePath(
    const std::vector<std::pair<double, double>>& control_points
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> sampled;

    if (control_points.size() < 4) {
        return sampled;
    }

    constexpr int degree = 3;

    const int control_count = static_cast<int>(control_points.size());

    const int max_t = control_count - degree;

    if (max_t <= 0) {
        return sampled;
    }

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

    auto evaluate = [&](double t) -> Point {
        if (t <= 0.0) {
            return control_points.front();
        }

        if (t >= static_cast<double>(max_t)) {
            return control_points.back();
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
            d[j] = control_points[span - degree + j];
        }

        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                const int index = span - degree + j;

                const double denominator = knots[index + degree - r + 1] - knots[index];

                double alpha = 0.0;

                if (denominator > 0.0) {
                    alpha = (t - knots[index]) / denominator;
                }

                d[j].first = (1.0 - alpha) * d[j - 1].first + alpha * d[j].first;

                d[j].second = (1.0 - alpha) * d[j - 1].second + alpha * d[j].second;
            }
        }

        return d[degree];
    };

    const int total_samples = std::max(1, max_t * bspline_samples_per_segment_);

    sampled.reserve(total_samples + 1);

    for (int i = 0; i <= total_samples; ++i) {
        const double t = static_cast<double>(max_t) * static_cast<double>(i) /
                         static_cast<double>(total_samples);

        sampled.push_back(evaluate(t));
    }

    return sampled;
}

std::vector<std::pair<double, double>> PathPlannerNode::bsplineSmoothPath(
    const std::vector<std::pair<int, int>>& path
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> result;

    if (path.empty()) {
        return result;
    }

    auto control_points = extractImportantPoints(path);

    /*
     * 重要点が4点未満なら、
     * 無理にB-splineにせずA*経路をそのまま使う。
     */
    if (control_points.size() < 4) {
        for (const auto& cell : path) {
            result.push_back(gridToWorld(cell.first, cell.second));
        }

        return result;
    }

    /*
     * smoothness_weight が大きいほど、
     * 制御点を中点方向へ強く動かす。
     */
    const double smoothness_alpha = bspline_smoothness_weight_ / (bspline_smoothness_weight_ + 1.0);

    const int smoothing_iterations = std::max(0, bspline_smoothing_iterations_);

    const double smoothing_step = std::max(0.001, bspline_control_point_step_) * smoothness_alpha;

    RCLCPP_INFO(
        this->get_logger(),
        "B-spline smoothing: weight=%.3f, alpha=%.3f, iterations=%d",
        bspline_smoothness_weight_,
        smoothness_alpha,
        smoothing_iterations
    );

    /*
     * まず制御点を平滑化する。
     */
    for (int iteration = 0; iteration < smoothing_iterations; ++iteration) {
        bool changed = false;

        for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
            const Point original = control_points[i];

            const Point previous = control_points[i - 1];

            const Point next = control_points[i + 1];

            const double midpoint_x = 0.5 * (previous.first + next.first);

            const double midpoint_y = 0.5 * (previous.second + next.second);

            const double dx = midpoint_x - original.first;

            const double dy = midpoint_y - original.second;

            const double distance = std::hypot(dx, dy);

            if (distance < 1e-9) {
                continue;
            }

            const double move_distance = std::min(smoothing_step, distance);

            const Point candidate{
                original.first + dx / distance * move_distance,

                original.second + dy / distance * move_distance
            };

            control_points[i] = candidate;

            auto candidate_path = evaluateBSplinePath(control_points);

            if (!isSmoothPathValid(candidate_path)) {
                control_points[i] = original;

                continue;
            }

            changed = true;
        }

        if (!changed) {
            break;
        }
    }

    auto initial_path = evaluateBSplinePath(control_points);

    if (!isSmoothPathValid(initial_path)) {
        RCLCPP_WARN(this->get_logger(), "Initial B-spline path is invalid. Using A* path.");

        for (const auto& cell : path) {
            result.push_back(gridToWorld(cell.first, cell.second));
        }

        return result;
    }

    const double base_length = calculatePathLength(initial_path);

    const double base_curvature = calculateCurvatureCost(initial_path);

    const double base_smoothness = calculateSmoothnessCost(initial_path);

    const double base_control_smoothness = calculateControlPointSmoothnessCost(control_points);

    double current_cost = calculatePathCost(
        initial_path,
        control_points,
        base_length,
        base_curvature,
        base_smoothness,
        base_control_smoothness
    );

    const int iterations = std::max(1, bspline_optimization_iterations_);

    const double step = std::max(0.001, bspline_control_point_step_);

    /*
     * 局所最適化。
     *
     * 以前は、
     *
     *   候補Aを採用
     *   ↓
     *   候補Bを評価
     *   ↓
     *   Bが悪かったので original に戻す
     *
     * となり、Aまで消えてしまう問題があった。
     *
     * 今回は「その制御点について一番良かった候補」
     * を最後に1回だけ採用する。
     */
    for (int iteration = 0; iteration < iterations; ++iteration) {
        bool improved = false;

        for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
            const Point original = control_points[i];

            const Point previous = control_points[i - 1];

            const Point next = control_points[i + 1];

            std::vector<Point> candidates;

            const double directions[][2] = {
                {        1.0,         0.0},
                {       -1.0,         0.0},
                {        0.0,         1.0},
                {        0.0,        -1.0},
                { 0.70710678,  0.70710678},
                { 0.70710678, -0.70710678},
                {-0.70710678,  0.70710678},
                {-0.70710678, -0.70710678}
            };

            const double midpoint_x = 0.5 * (previous.first + next.first);

            const double midpoint_y = 0.5 * (previous.second + next.second);

            const double to_mid_x = midpoint_x - original.first;

            const double to_mid_y = midpoint_y - original.second;

            const double to_mid_length = std::hypot(to_mid_x, to_mid_y);

            const double optimization_smoothing_step =
                step * (bspline_smoothness_weight_ / (bspline_smoothness_weight_ + 1.0));

            /*
             * 前後の制御点の中点方向。
             */
            if (to_mid_length > 1e-9) {
                candidates.push_back(
                    {original.first + to_mid_x / to_mid_length * optimization_smoothing_step,

                     original.second + to_mid_y / to_mid_length * optimization_smoothing_step}
                );
            }

            /*
             * 8方向。
             */
            for (const auto& direction : directions) {
                candidates.push_back(
                    {original.first + direction[0] * optimization_smoothing_step,

                     original.second + direction[1] * optimization_smoothing_step}
                );
            }

            /*
             * 現在位置を「最良候補」の初期値にする。
             */
            Point best_point = original;

            double best_cost = current_cost;

            /*
             * 各候補を評価する。
             */
            for (const auto& candidate : candidates) {
                if (candidate == original) {
                    continue;
                }

                control_points[i] = candidate;

                auto candidate_path = evaluateBSplinePath(control_points);

                /*
                 * 障害物に入る候補は却下。
                 */
                if (!isSmoothPathValid(candidate_path)) {
                    control_points[i] = original;

                    continue;
                }

                const double candidate_cost = calculatePathCost(
                    candidate_path,
                    control_points,
                    base_length,
                    base_curvature,
                    base_smoothness,
                    base_control_smoothness
                );

                /*
                 * 今までで最も良い候補なら保存。
                 */
                if (candidate_cost < best_cost) {
                    best_cost = candidate_cost;

                    best_point = candidate;
                }

                /*
                 * 次の候補は必ず元の位置から評価する。
                 */
                control_points[i] = original;
            }

            /*
             * 最も良かった候補だけを採用。
             */
            if (best_cost < current_cost) {
                control_points[i] = best_point;

                current_cost = best_cost;

                improved = true;
            } else {
                control_points[i] = original;
            }
        }

        if (!improved) {
            break;
        }
    }

    result = evaluateBSplinePath(control_points);

    if (!isSmoothPathValid(result)) {
        result.clear();

        for (const auto& cell : path) {
            result.push_back(gridToWorld(cell.first, cell.second));
        }

        RCLCPP_WARN(this->get_logger(), "Optimized B-spline path is invalid. Using A* path.");

        return result;
    }

    const double final_length = calculatePathLength(result);

    const double final_curvature = calculateCurvatureCost(result);

    const double final_smoothness = calculateSmoothnessCost(result);

    const double final_control_smoothness = calculateControlPointSmoothnessCost(control_points);

    RCLCPP_INFO(
        this->get_logger(),
        "B-spline optimized: "
        "control_points=%zu, "
        "length %.3f -> %.3f, "
        "curvature %.6f -> %.6f, "
        "smoothness %.6f -> %.6f, "
        "control_smoothness %.6f -> %.6f, "
        "smoothness_weight=%.3f",
        control_points.size(),
        base_length,
        final_length,
        base_curvature,
        final_curvature,
        base_smoothness,
        final_smoothness,
        base_control_smoothness,
        final_control_smoothness,
        bspline_smoothness_weight_
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
            path[i + 1].first - path[i - 1].first,

            path[i + 1].second - path[i - 1].second
        );

        if (a < 1e-9 || b < 1e-9 || c < 1e-9) {
            continue;
        }

        const double cross = ax * by - ay * bx;

        const double curvature = 2.0 * std::abs(cross) / (a * b * c);

        const double ds = 0.5 * (a + b);

        cost += curvature * curvature * ds;
    }

    return cost;
}

double PathPlannerNode::calculateSmoothnessCost(const std::vector<std::pair<double, double>>& path)
{
    if (path.size() < 4) {
        return 0.0;
    }

    std::vector<double> curvature;

    curvature.reserve(path.size() - 2);

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const double x0 = path[i - 1].first;

        const double y0 = path[i - 1].second;

        const double x1 = path[i].first;

        const double y1 = path[i].second;

        const double x2 = path[i + 1].first;

        const double y2 = path[i + 1].second;

        const double dx1 = x1 - x0;

        const double dy1 = y1 - y0;

        const double dx2 = x2 - x1;

        const double dy2 = y2 - y1;

        const double a = std::hypot(dx1, dy1);

        const double b = std::hypot(dx2, dy2);

        const double c = std::hypot(x2 - x0, y2 - y0);

        if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
            curvature.push_back(0.0);

            continue;
        }

        const double cross = dx1 * dy2 - dy1 * dx2;

        const double kappa = 2.0 * std::abs(cross) / (a * b * c);

        curvature.push_back(kappa);
    }

    double smoothness = 0.0;

    for (std::size_t i = 1; i < curvature.size(); ++i) {
        const double dkappa = curvature[i] - curvature[i - 1];

        smoothness += dkappa * dkappa;
    }

    return smoothness;
}

double PathPlannerNode::calculateControlPointSmoothnessCost(
    const std::vector<std::pair<double, double>>& control_points
)
{
    if (control_points.size() < 3) {
        return 0.0;
    }

    double cost = 0.0;

    for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
        const double second_x = control_points[i - 1].first - 2.0 * control_points[i].first +
                                control_points[i + 1].first;

        const double second_y = control_points[i - 1].second - 2.0 * control_points[i].second +
                                control_points[i + 1].second;

        cost += second_x * second_x + second_y * second_y;
    }

    return cost;
}

double PathPlannerNode::calculatePathCost(
    const std::vector<std::pair<double, double>>& path,
    const std::vector<std::pair<double, double>>& control_points,
    double base_length,
    double base_curvature,
    double base_smoothness,
    double base_control_smoothness
)
{
    const double length = calculatePathLength(path);

    const double curvature = calculateCurvatureCost(path);

    const double smoothness = calculateSmoothnessCost(path);

    const double control_smoothness = calculateControlPointSmoothnessCost(control_points);

    /*
     * 0除算や極端な値を避けるため、
     * 正規化の下限を1e-4にする。
     */
    const double length_ratio = length / std::max(base_length, 1e-4);

    const double curvature_ratio = curvature / std::max(base_curvature, 1e-4);

    const double smoothness_ratio = smoothness / std::max(base_smoothness, 1e-4);

    const double control_smoothness_ratio =
        control_smoothness / std::max(base_control_smoothness, 1e-4);

    return bspline_length_weight_ * length_ratio +

           bspline_curvature_weight_ * curvature_ratio +

           bspline_smoothness_weight_ * (0.5 * smoothness_ratio + 0.5 * control_smoothness_ratio);
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