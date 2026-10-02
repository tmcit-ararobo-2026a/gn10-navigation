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

// 点pointから線分(line_start, line_end)までの最短距離
double pointToLineDistance(
    const std::pair<double, double>& point,
    const std::pair<double, double>& line_start,
    const std::pair<double, double>& line_end
)
{
    const double dx = line_end.first - line_start.first;
    const double dy = line_end.second - line_start.second;

    const double length_squared = dx * dx + dy * dy;

    // 始点と終点がほぼ同じなら、点同士の距離
    if (length_squared < 1e-12) {
        return std::hypot(point.first - line_start.first, point.second - line_start.second);
    }

    // 線分上で一番近い位置を割合t(0〜1)で求める
    double t = ((point.first - line_start.first) * dx + (point.second - line_start.second) * dy) /
               length_squared;
    t        = std::clamp(t, 0.0, 1.0);

    const double closest_x = line_start.first + t * dx;
    const double closest_y = line_start.second + t * dy;

    return std::hypot(point.first - closest_x, point.second - closest_y);
}

// RDP: startとendを結ぶ線から最も遠い点が許容誤差より遠ければ残し、その点で2分割して再帰する
void simplifyPathRecursive(
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
    this->declare_parameter("bspline.simplification_tolerance", 0.1);
    this->declare_parameter("bspline.turning_angle_threshold", 15.0);
    this->declare_parameter("bspline.samples_per_segment", 10);
    this->declare_parameter("bspline.smoothness_weight", 1.0);
    this->declare_parameter("bspline.control_point_step", 0.05);
    this->declare_parameter("bspline.smoothing_iterations", 10);
    this->declare_parameter("cost_caluculate.length_weight", 1.0);
    this->declare_parameter("cost_caluculate.curvature_weight", 1.0);
    this->declare_parameter("cost_caluculate.obstacle_cost_weight", 1.0);
    this->declare_parameter("cost_caluculate.optimization_iterations", 20);

    // yamlから受信
    map_frame_         = this->get_parameter("map_configs.map_frame").as_string();
    base_frame_        = this->get_parameter("map_configs.base_frame").as_string();
    start_snap_radius_ = this->get_parameter("snapp_config.start_snap_radius").as_double();
    cost_factor_       = this->get_parameter("a_star.plannning_cost").as_double();
    bspline_simplification_tolerance_ =
        this->get_parameter("bspline.simplification_tolerance").as_double();
    bspline_turning_angle_threshold_ =
        this->get_parameter("bspline.turning_angle_threshold").as_double();
    bspline_samples_per_segment_ =
        static_cast<int>(this->get_parameter("bspline.samples_per_segment").as_int());
    bspline_smoothness_weight_  = this->get_parameter("bspline.smoothness_weight").as_double();
    bspline_control_point_step_ = this->get_parameter("bspline.control_point_step").as_double();
    bspline_smoothing_iterations_ =
        static_cast<int>(this->get_parameter("bspline.smoothing_iterations").as_int());
    bspline_length_weight_    = this->get_parameter("cost_caluculate.length_weight").as_double();
    bspline_curvature_weight_ = this->get_parameter("cost_caluculate.curvature_weight").as_double();
    bspline_obstacle_cost_weight_ =
        this->get_parameter("cost_caluculate.obstacle_cost_weight").as_double();
    bspline_optimization_iterations_ =
        static_cast<int>(this->get_parameter("cost_caluculate.optimization_iterations").as_int());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { getRobotPose(); });
    RCLCPP_INFO(this->get_logger(), "Path planner started");
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

    // b-spineはA*が使う
    auto smoothed_path = bsplineSmoothPath(path);
    if (smoothed_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "B-spline path generation failed.");
        return;
    }

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

    smoothed_path.back() = {goal_x_, goal_y_};
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

std::vector<std::pair<double, double>> PathPlannerNode::extractImportantPoints(
    const std::vector<std::pair<int, int>>& path
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> points;
    points.reserve(path.size());  // 使う分予約

    for (const auto& cell : path) {
        points.push_back(gridToWorld(cell.first, cell.second));
    }
    if (points.size() <= 2) {
        return points;
    }

    const std::size_t n = points.size();

    //  RDPで、直線的な部分の不要な点を削除する
    std::vector<bool> keep(n, false);
    keep[0]     = true;
    keep[n - 1] = true;

    simplifyPathRecursive(
        points, 0, static_cast<int>(n) - 1, bspline_simplification_tolerance_, keep
    );

    std::vector<std::size_t> rdp_indices;
    for (std::size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            rdp_indices.push_back(i);
        }
    }

    //  RDP後の点列で、一定以上曲がっている点だけ残す
    struct DroppedCandidate {
        std::size_t index;
        double angle;
    };
    std::vector<DroppedCandidate> dropped;

    std::vector<bool> final_keep(n, false);
    final_keep[0]     = true;
    final_keep[n - 1] = true;

    std::size_t kept_count = 2;
    std::size_t last_kept  = rdp_indices.front();

    for (std::size_t k = 1; k + 1 < rdp_indices.size(); ++k) {
        const std::size_t idx  = rdp_indices[k];
        const std::size_t next = rdp_indices[k + 1];

        const double v1x = points[idx].first - points[last_kept].first;
        const double v1y = points[idx].second - points[last_kept].second;
        const double v2x = points[next].first - points[idx].first;
        const double v2y = points[next].second - points[idx].second;

        const double angle = calculateAngleDifference(v1x, v1y, v2x, v2y);

        if (angle >= bspline_turning_angle_threshold_) {
            final_keep[idx] = true;
            ++kept_count;
            last_kept = idx;
        } else {
            dropped.push_back({idx, angle});
        }
    }

    // 4点未満なら、落とした点を曲がり角が大きい順に戻す
    //  (3次B-splineは制御点が4点以上必要)
    constexpr std::size_t min_points = 4;

    if (kept_count < min_points && !dropped.empty()) {
        std::sort(
            dropped.begin(),
            dropped.end(),
            [](const DroppedCandidate& a, const DroppedCandidate& b) { return a.angle > b.angle; }
        );

        for (const auto& candidate : dropped) {
            if (kept_count >= min_points) {
                break;
            }
            final_keep[candidate.index] = true;
            ++kept_count;
        }
    }

    // それでも足りなければ、現在の折れ線から最も離れた元経路上の点を追加する ---
    // まっすぐで距離がすべて0なら、最も広い区間の中央を追加する
    while (kept_count < min_points && kept_count < n) {
        double best_distance      = -1.0;
        std::size_t best_index    = 0;
        std::size_t widest_gap    = 1;
        std::size_t widest_mid    = 0;
        std::size_t segment_start = 0;

        for (std::size_t b = 1; b < n; ++b) {
            if (!final_keep[b]) {
                continue;
            }

            for (std::size_t i = segment_start + 1; i < b; ++i) {
                const double distance =
                    pointToLineDistance(points[i], points[segment_start], points[b]);

                if (distance > best_distance) {
                    best_distance = distance;
                    best_index    = i;
                }
            }

            if (b - segment_start > widest_gap) {
                widest_gap = b - segment_start;
                widest_mid = segment_start + (b - segment_start) / 2;
            }

            segment_start = b;
        }

        const std::size_t chosen = (best_distance > 1e-9) ? best_index : widest_mid;

        final_keep[chosen] = true;
        ++kept_count;
    }

    //  元の経路順のまま、残した点だけ取り出す
    std::vector<Point> important_points;
    important_points.reserve(kept_count);

    for (std::size_t i = 0; i < n; ++i) {
        if (final_keep[i]) {
            important_points.push_back(points[i]);
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Important points: %zu (RDP: %zu) / A* points: %zu",
        important_points.size(),
        rdp_indices.size(),
        n
    );

    return important_points;
}

bool PathPlannerNode::isSmoothPathValid(const std::vector<std::pair<double, double>>& path)
{
    if (path.empty()) {
        return false;
    }

    for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& point = path[i];

        const auto [grid_x, grid_y] = worldToGrid(point.first, point.second);

        if (!isPassable(grid_x, grid_y)) {  // 範囲外もここで弾かれる
            return false;
        }

        if (i == 0) {
            continue;
        }

        // 前の点との間を、半セル刻みで調べる
        const auto& previous = path[i - 1];

        const double dx       = point.first - previous.first;
        const double dy       = point.second - previous.second;
        const double distance = std::hypot(dx, dy);

        const int samples =
            std::max(1, static_cast<int>(std::ceil(distance / (resolution_ * 0.5))));

        for (int j = 1; j <= samples; ++j) {
            const double ratio = static_cast<double>(j) / static_cast<double>(samples);

            const auto [sample_x, sample_y] =
                worldToGrid(previous.first + dx * ratio, previous.second + dy * ratio);

            if (!isPassable(sample_x, sample_y)) {
                return false;
            }
        }
    }

    return true;
}

std::vector<std::pair<double, double>> PathPlannerNode::evaluateBSplinePath(
    const std::vector<std::pair<double, double>>& control_points
)
{
    using Point = std::pair<double, double>;

    std::vector<Point> sampled;

    constexpr int degree = 3;

    if (control_points.size() < degree + 1) {
        return sampled;
    }

    const int control_count = static_cast<int>(control_points.size());
    const int max_t         = control_count - degree;  // パラメータtの範囲は0〜max_t

    // ノット列: 両端はdegree+1個重複、間は等間隔
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

    // De Boorのアルゴリズムで、パラメータtの点を求める
    auto evaluate = [&](double t) -> Point {
        if (t <= 0.0) {
            return control_points.front();
        }
        if (t >= static_cast<double>(max_t)) {
            return control_points.back();
        }

        // tが属するノット区間を探す
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
                const int index          = span - degree + j;
                const double denominator = knots[index + degree - r + 1] - knots[index];

                double alpha = 0.0;
                if (denominator > 0.0) {
                    alpha = (t - knots[index]) / denominator;
                }

                d[j].first  = (1.0 - alpha) * d[j - 1].first + alpha * d[j].first;
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

    // 失敗したときに返す、A*経路そのもの
    auto make_astar_path = [&]() {
        std::vector<Point> result;
        for (const auto& cell : path) {
            result.push_back(gridToWorld(cell.first, cell.second));
        }
        return result;
    };

    if (path.empty()) {
        return {};
    }

    auto control_points = extractImportantPoints(path);

    // 3次B-splineは制御点が4点以上必要
    if (control_points.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Too few control points. Using A* path.");
        return make_astar_path();
    }

    // 平滑化の前に、まず元の制御点で曲線が有効か確認する
    if (!isSmoothPathValid(evaluateBSplinePath(control_points))) {
        RCLCPP_WARN(this->get_logger(), "Initial B-spline path is invalid. Using A* path.");
        return make_astar_path();
    }

    // 制御点の平滑化
    // weightが大きいほどalphaが1に近づき、1回の移動量が増える
    const double smoothness_weight = std::max(0.0, bspline_smoothness_weight_);
    const double smoothness_alpha  = smoothness_weight / (smoothness_weight + 1.0);
    const int smoothing_iterations = std::max(0, bspline_smoothing_iterations_);
    const double smoothing_step = std::max(0.001, bspline_control_point_step_) * smoothness_alpha;

    for (int iteration = 0; iteration < smoothing_iterations; ++iteration) {
        bool changed = false;

        // 始点と終点は動かさない
        for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
            const Point original = control_points[i];
            const Point previous = control_points[i - 1];
            const Point next     = control_points[i + 1];

            // 両隣の中点へ向かうベクトル
            const double dx       = 0.5 * (previous.first + next.first) - original.first;
            const double dy       = 0.5 * (previous.second + next.second) - original.second;
            const double distance = std::hypot(dx, dy);

            if (distance < 1e-9) {
                continue;
            }

            // 中点を行き過ぎないよう、stepと距離の小さい方だけ動く
            const double move = std::min(smoothing_step, distance);

            control_points[i] = {
                original.first + dx / distance * move, original.second + dy / distance * move
            };

            // 動かした結果が障害物に入るなら取り消す
            if (!isSmoothPathValid(evaluateBSplinePath(control_points))) {
                control_points[i] = original;
                continue;
            }

            changed = true;
        }

        if (!changed) {
            break;  // どの点も動かなければ終了
        }
    }

    // 　 最適化: 基準値(これとの比でコストを測る)
    const auto initial_path          = evaluateBSplinePath(control_points);
    const double base_length         = calculatePathLength(initial_path);
    const double base_curvature      = calculateCurvatureCost(initial_path);
    const double base_smoothness     = calculateSmoothnessCost(initial_path);
    const double base_control_smooth = calculateControlPointSmoothnessCost(control_points);

    double current_cost = calculatePathCost(
        initial_path,
        control_points,
        base_length,
        base_curvature,
        base_smoothness,
        base_control_smooth
    );

    const int iterations = std::max(1, bspline_optimization_iterations_);
    const double step    = std::max(0.001, bspline_control_point_step_);

    // 8方向
    const double directions[8][2] = {
        {        1.0,         0.0},
        {       -1.0,         0.0},
        {        0.0,         1.0},
        {        0.0,        -1.0},
        { 0.70710678,  0.70710678},
        { 0.70710678, -0.70710678},
        {-0.70710678,  0.70710678},
        {-0.70710678, -0.70710678}
    };

    for (int iteration = 0; iteration < iterations; ++iteration) {
        bool improved = false;

        // 始点・終点は動かさない
        for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
            const Point original = control_points[i];
            const Point previous = control_points[i - 1];
            const Point next     = control_points[i + 1];

            std::vector<Point> candidates;

            // 候補1: 両隣の中点方向
            const double mx = 0.5 * (previous.first + next.first) - original.first;
            const double my = 0.5 * (previous.second + next.second) - original.second;
            const double ml = std::hypot(mx, my);

            if (ml > 1e-9) {
                candidates.push_back(
                    {original.first + mx / ml * step, original.second + my / ml * step}
                );
            }

            // 候補2: 8方向
            for (const auto& d : directions) {
                candidates.push_back({original.first + d[0] * step, original.second + d[1] * step});
            }

            Point best_point = original;
            double best_cost = current_cost;

            for (const auto& candidate : candidates) {
                control_points[i] = candidate;

                const auto candidate_path = evaluateBSplinePath(control_points);

                // 障害物に入る候補は却下
                if (!isSmoothPathValid(candidate_path)) {
                    continue;
                }

                const double cost = calculatePathCost(
                    candidate_path,
                    control_points,
                    base_length,
                    base_curvature,
                    base_smoothness,
                    base_control_smooth
                );

                if (cost < best_cost) {
                    best_cost  = cost;
                    best_point = candidate;
                }
            }

            // 最良の候補だけ採用(なければ元に戻る)
            control_points[i] = best_point;

            if (best_cost < current_cost) {
                current_cost = best_cost;
                improved     = true;
            }
        }

        if (!improved) {
            break;  // どの点も改善しなければ終了
        }
    }

    const auto result = evaluateBSplinePath(control_points);

    if (!isSmoothPathValid(result)) {
        RCLCPP_WARN(this->get_logger(), "Optimized B-spline path is invalid. Using A* path.");
        return make_astar_path();
    }

    RCLCPP_INFO(
        this->get_logger(),
        "B-spline: control_points=%zu, samples=%zu, smoothing alpha=%.3f, iterations=%d",
        control_points.size(),
        result.size(),
        smoothness_alpha,
        smoothing_iterations
    );

    RCLCPP_INFO(
        this->get_logger(),
        "len=%.3f curv=%.6f smooth=%.6f ctrl=%.6f obs=%.3f",
        calculatePathLength(result),
        calculateCurvatureCost(result),
        calculateSmoothnessCost(result),
        calculateControlPointSmoothnessCost(control_points),
        calculateObstacleCost(result)
    );

    RCLCPP_INFO(
        this->get_logger(),
        "B-spline optimized: length %.3f -> %.3f, curvature %.6f -> %.6f, obstacle %.3f -> %.3f",
        base_length,
        calculatePathLength(result),
        base_curvature,
        calculateCurvatureCost(result),
        calculateObstacleCost(initial_path),
        calculateObstacleCost(result)
    );

    return result;
}

// cost計算関数群

// pathの長さを計算
double PathPlannerNode::calculatePathLength(const std::vector<std::pair<double, double>>& path)
{
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length +=
            std::hypot(path[i].first - path[i - 1].first, path[i].second - path[i - 1].second);
    }
    return length;
}

// 曲率の二乗を、道のり方向に積分したもの(急カーブほど大きい)
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

        // 3点を通る円の曲率 = 2|cross| / (a*b*c)
        const double cross     = ax * by - ay * bx;
        const double curvature = 2.0 * std::abs(cross) / (a * b * c);

        cost += curvature * curvature * 0.5 * (a + b);
    }

    return cost;
}

// 曲率の変化量の二乗和(曲がり方が急に変わるほど大きい)
double PathPlannerNode::calculateSmoothnessCost(const std::vector<std::pair<double, double>>& path)
{
    if (path.size() < 4) {
        return 0.0;
    }

    std::vector<double> curvature;
    curvature.reserve(path.size() - 2);

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const double dx1 = path[i].first - path[i - 1].first;
        const double dy1 = path[i].second - path[i - 1].second;
        const double dx2 = path[i + 1].first - path[i].first;
        const double dy2 = path[i + 1].second - path[i].second;

        const double a = std::hypot(dx1, dy1);
        const double b = std::hypot(dx2, dy2);
        const double c = std::hypot(
            path[i + 1].first - path[i - 1].first, path[i + 1].second - path[i - 1].second
        );

        if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
            curvature.push_back(0.0);
            continue;
        }

        curvature.push_back(2.0 * std::abs(dx1 * dy2 - dy1 * dx2) / (a * b * c));
    }

    double smoothness = 0.0;
    for (std::size_t i = 1; i < curvature.size(); ++i) {
        const double d = curvature[i] - curvature[i - 1];
        smoothness += d * d;
    }

    return smoothness;
}

// 通る各セルのコスト平均を0〜1に正規化(障害物に近い帯を通るほど大きい)
double PathPlannerNode::calculateObstacleCost(const std::vector<std::pair<double, double>>& path)
{
    if (path.empty()) {
        return 0.0;
    }

    double sum = 0.0;

    for (const auto& point : path) {
        const auto [gx, gy] = worldToGrid(point.first, point.second);

        if (!isInsideGrid(gx, gy)) {
            sum += 254.0;
            continue;
        }

        sum += static_cast<double>(planning_grid_[gridtoIndex(gx, gy)]);
    }

    return sum / (static_cast<double>(path.size()) * 254.0);
}

// 制御点の二階差分の二乗和(制御点列のギザギザ)
double PathPlannerNode::calculateControlPointSmoothnessCost(
    const std::vector<std::pair<double, double>>& control_points
)
{
    if (control_points.size() < 3) {
        return 0.0;
    }

    double cost = 0.0;

    for (std::size_t i = 1; i + 1 < control_points.size(); ++i) {
        const double sx = control_points[i - 1].first - 2.0 * control_points[i].first +
                          control_points[i + 1].first;
        const double sy = control_points[i - 1].second - 2.0 * control_points[i].second +
                          control_points[i + 1].second;
        cost += sx * sx + sy * sy;
    }

    return cost;
}

// 各項を「最初の値との比」にして重みづけ合計する
double PathPlannerNode::calculatePathCost(
    const std::vector<std::pair<double, double>>& path,
    const std::vector<std::pair<double, double>>& control_points,
    double base_length,
    double base_curvature,
    double base_smoothness,
    double base_control_smoothness
)
{
    const double length_ratio     = calculatePathLength(path) / std::max(base_length, 1e-4);
    const double curvature_ratio  = calculateCurvatureCost(path) / std::max(base_curvature, 1e-4);
    const double smoothness_ratio = calculateSmoothnessCost(path) / std::max(base_smoothness, 1e-4);
    const double control_ratio    = calculateControlPointSmoothnessCost(control_points) /
                                    std::max(base_control_smoothness, 1e-4);
    const double obstacle         = calculateObstacleCost(path);

    return bspline_length_weight_ * length_ratio + bspline_curvature_weight_ * curvature_ratio +
           bspline_smoothness_weight_ * (0.5 * smoothness_ratio + 0.5 * control_ratio) +
           bspline_obstacle_cost_weight_ * obstacle;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}