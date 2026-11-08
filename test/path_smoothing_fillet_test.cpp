#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#ifndef GN10_NAVIGATION_PATH_PLANNER_NO_MAIN
#define GN10_NAVIGATION_PATH_PLANNER_NO_MAIN
#endif
#include "gn10_navigation/path_planner_node.hpp"

class PathSmoothingFilletTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    nav_msgs::msg::OccupancyGrid createMockCostmap(int width, int height, double res, int8_t default_cost = 0)
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.info.resolution = static_cast<float>(res);
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(width * height, default_cost);
        return grid;
    }
};

TEST_F(PathSmoothingFilletTest, GeneratesDeterministicLargeFilletArc)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("smoothing.fillet.rdp_epsilon", 0.05);
    options.append_parameter_override("smoothing.fillet.target_radius", 1.3);
    options.append_parameter_override("smoothing.fillet.min_radius", 0.2);
    options.append_parameter_override("smoothing.fillet.max_cost", 35);
    options.append_parameter_override("smoothing.fillet.path_resolution", 0.05);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);

    // 10m x 10m の低コストグリッド (解像度 0.05m)
    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    // コストマップをノードに注入するためのダミーメッセージ処理
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    // (1.0, 1.0) -> (5.0, 1.0) -> (5.0, 5.0) の直角 90 度コーナー
    // グリッド座標列を生成 (0.05m 刻み)
    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int y = 21; y <= 100; ++y) {
        grid_path.push_back({100, y});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 5.0};

    auto path1 = planner->smoothPathFillet(grid_path, exact_start, exact_goal);
    ASSERT_FALSE(path1.empty());
    EXPECT_NEAR(path1.front().first, 1.0, 1e-4);
    EXPECT_NEAR(path1.front().second, 1.0, 1e-4);
    EXPECT_NEAR(path1.back().first, 5.0, 1e-4);
    EXPECT_NEAR(path1.back().second, 5.0, 1e-4);

    // 目標半径 1.3m の大円弧フィレットにより、
    // コーナー (5.0, 1.0) の内角二等分線上において、頂点からの最接近点は
    // dist = R * (1 / sin(45 deg) - 1) = 1.3 * (sqrt(2) - 1) ≈ 1.3 * 0.4142 ≈ 0.538 m
    // つまり頂点 (5.0, 1.0) から (-dx, dy) 方向に約 0.54m 離れた位置を通るはず
    double min_dist_to_corner = 1e9;
    for (const auto& pt : path1) {
        double d = std::hypot(pt.first - 5.0, pt.second - 1.0);
        if (d < min_dist_to_corner) {
            min_dist_to_corner = d;
        }
    }
    EXPECT_NEAR(min_dist_to_corner, 1.3 * (std::sqrt(2.0) - 1.0), 0.08);

    // 開始位置を微小シフト (2cm, 5cm) させた場合の決定論的一貫性検証
    // スタートが数cmズレても、コーナー付近の最接近距離が跳ねずに一貫していること
    const std::pair<double, double> shifted_start = {1.03, 1.02};
    auto path2 = planner->smoothPathFillet(grid_path, shifted_start, exact_goal);
    ASSERT_FALSE(path2.empty());

    double min_dist_shifted = 1e9;
    for (const auto& pt : path2) {
        double d = std::hypot(pt.first - 5.0, pt.second - 1.0);
        if (d < min_dist_shifted) {
            min_dist_shifted = d;
        }
    }
    // 開始位置のズレにかかわらず、コーナーフィレット形状が完全に一致
    EXPECT_NEAR(min_dist_shifted, min_dist_to_corner, 0.02);
}

TEST_F(PathSmoothingFilletTest, SafelyShrinksRadiusNearObstacles)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("smoothing.fillet.rdp_epsilon", 0.05);
    options.append_parameter_override("smoothing.fillet.target_radius", 1.3);
    options.append_parameter_override("smoothing.fillet.min_radius", 0.2);
    options.append_parameter_override("smoothing.fillet.max_cost", 35);
    options.append_parameter_override("smoothing.fillet.path_resolution", 0.05);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);

    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    // コーナー (5.0, 1.0) の内側 (4.5, 1.5) 付近に高コスト障害物を配置
    // (大半径 1.3m の円弧が通る領域をブロック)
    int obs_gx = static_cast<int>(4.6 / 0.05);
    int obs_gy = static_cast<int>(1.4 / 0.05);
    costmap.data[obs_gy * 200 + obs_gx] = 90; // 高コスト

    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int y = 21; y <= 100; ++y) {
        grid_path.push_back({100, y});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 5.0};

    auto path = planner->smoothPathFillet(grid_path, exact_start, exact_goal);
    ASSERT_FALSE(path.empty());

    // 障害物セル (cost 90) を踏んでいないこと
    for (const auto& pt : path) {
        int gx = static_cast<int>(std::floor(pt.first / 0.05));
        int gy = static_cast<int>(std::floor(pt.second / 0.05));
        if (gx >= 0 && gx < 200 && gy >= 0 && gy < 200) {
            EXPECT_LE(costmap.data[gy * 200 + gx], 35);
        }
    }
}

TEST_F(PathSmoothingFilletTest, SmoothlyBlendsCurvatureAtJunctions)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("smoothing.fillet.rdp_epsilon", 0.05);
    options.append_parameter_override("smoothing.fillet.target_radius", 1.3);
    options.append_parameter_override("smoothing.fillet.min_radius", 0.2);
    options.append_parameter_override("smoothing.fillet.max_cost", 35);
    options.append_parameter_override("smoothing.fillet.path_resolution", 0.05);
    options.append_parameter_override("smoothing.fillet.curvature_blend_iterations", 3);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);
    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int y = 21; y <= 100; ++y) {
        grid_path.push_back({100, y});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 5.0};

    auto blended_path = planner->smoothPathFillet(grid_path, exact_start, exact_goal);
    ASSERT_FALSE(blended_path.empty());

    // 始点・終点が正確に保持されていること
    EXPECT_NEAR(blended_path.front().first, 1.0, 1e-4);
    EXPECT_NEAR(blended_path.front().second, 1.0, 1e-4);
    EXPECT_NEAR(blended_path.back().first, 5.0, 1e-4);
    EXPECT_NEAR(blended_path.back().second, 5.0, 1e-4);

    // 点間のステップ距離が過度に離れておらず、滑らかに連続していること
    for (std::size_t i = 0; i + 1 < blended_path.size(); ++i) {
        double d = std::hypot(
            blended_path[i + 1].first - blended_path[i].first,
            blended_path[i + 1].second - blended_path[i].second
        );
        EXPECT_LE(d, 0.08); // resolution 0.05m + マージン
    }

    // 隣接3点の内角変化 (ヘディング角の変化) が急峻にスパイクしていないこと
    for (std::size_t i = 1; i + 1 < blended_path.size(); ++i) {
        double d1x = blended_path[i].first - blended_path[i - 1].first;
        double d1y = blended_path[i].second - blended_path[i - 1].second;
        double d2x = blended_path[i + 1].first - blended_path[i].first;
        double d2y = blended_path[i + 1].second - blended_path[i].second;
        double l1 = std::hypot(d1x, d1y);
        double l2 = std::hypot(d2x, d2y);
        if (l1 > 1e-3 && l2 > 1e-3) {
            double dot = std::clamp((d1x * d2x + d1y * d2y) / (l1 * l2), -1.0, 1.0);
            double angle_change = std::acos(dot);
            // 0.05m 刻みにおいて、急角度の屈曲 (約 15 度以上) がないこと
            EXPECT_LE(angle_change, 0.25);
        }
    }
}
