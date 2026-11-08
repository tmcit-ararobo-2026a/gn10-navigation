#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "gn10_navigation/path_planner_node.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <memory>
#include <vector>

class PathSmoothingElasticBandTest : public ::testing::Test
{
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

    nav_msgs::msg::OccupancyGrid createMockCostmap(
        int width, int height, double resolution, int default_cost = 0
    )
    {
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.frame_id        = "map";
        costmap.info.resolution        = resolution;
        costmap.info.width             = width;
        costmap.info.height            = height;
        costmap.info.origin.position.x = 0.0;
        costmap.info.origin.position.y = 0.0;
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;
        costmap.data.assign(width * height, static_cast<int8_t>(default_cost));
        return costmap;
    }
};

TEST_F(PathSmoothingElasticBandTest, SmoothsOpenFieldPath)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("elastic_band.iterations", 40);
    options.append_parameter_override("elastic_band.elastic_strength", 0.5);
    options.append_parameter_override("elastic_band.repulsive_strength", 0.05);
    options.append_parameter_override("elastic_band.repulsive_cost_threshold", 15);
    options.append_parameter_override("elastic_band.max_cost", 40);
    options.append_parameter_override("elastic_band.path_resolution", 0.05);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);

    // 10m x 10m の自由空間グリッド (解像度 0.05m, コスト 0)
    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    // (1.0, 1.0) -> (5.0, 1.0) -> (5.0, 5.0) の直角 90 度コーナー
    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int y = 21; y <= 100; ++y) {
        grid_path.push_back({100, y});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 5.0};

    auto smoothed = planner->smoothPathElasticBand(grid_path, exact_start, exact_goal);
    ASSERT_FALSE(smoothed.empty());

    // 始点・終点が正確に保持されていること
    EXPECT_NEAR(smoothed.front().first, 1.0, 1e-4);
    EXPECT_NEAR(smoothed.front().second, 1.0, 1e-4);
    EXPECT_NEAR(smoothed.back().first, 5.0, 1e-4);
    EXPECT_NEAR(smoothed.back().second, 5.0, 1e-4);

    // 点間距離が均一に保たれていること (解像度 0.05m + 許容マージン)
    for (std::size_t i = 0; i + 1 < smoothed.size(); ++i) {
        double d = std::hypot(
            smoothed[i + 1].first - smoothed[i].first,
            smoothed[i + 1].second - smoothed[i].second
        );
        EXPECT_LE(d, 0.08);
    }

    // コーナー頂点 (5.0, 1.0) の内角を滑らかにラウンドしていること
    double min_dist_to_corner = 1e9;
    for (const auto& pt : smoothed) {
        double d = std::hypot(pt.first - 5.0, pt.second - 1.0);
        if (d < min_dist_to_corner) {
            min_dist_to_corner = d;
        }
    }
    // 直角の角が内側に適度に平滑化され、頂点から離れていること (> 0.05m)
    EXPECT_GT(min_dist_to_corner, 0.05);
}

TEST_F(PathSmoothingElasticBandTest, RepelsFromHighCostObstacle)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("elastic_band.iterations", 50);
    options.append_parameter_override("elastic_band.elastic_strength", 0.4);
    options.append_parameter_override("elastic_band.repulsive_strength", 0.08);
    options.append_parameter_override("elastic_band.repulsive_cost_threshold", 15);
    options.append_parameter_override("elastic_band.max_cost", 40);
    options.append_parameter_override("elastic_band.path_resolution", 0.05);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);

    // 10m x 10m グリッド
    auto costmap = createMockCostmap(200, 200, 0.05, 0);

    // 障害物 (x=3.0, y=1.0 付近) にコスト勾配 (インフレーション) を設置
    // 中心 (x=60, y=20) は障害物 cost=90, 周囲に勾配
    for (int dy = -10; dy <= 10; ++dy) {
        for (int dx = -10; dx <= 10; ++dx) {
            int gx = 60 + dx;
            int gy = 20 + dy;
            if (gx >= 0 && gx < 200 && gy >= 0 && gy < 200) {
                double dist = std::hypot(dx, dy) * 0.05;
                if (dist <= 0.4) {
                    int c = static_cast<int>(std::round(90.0 * (1.0 - dist / 0.4)));
                    costmap.data[gy * 200 + gx] = static_cast<int8_t>(std::clamp(c, 0, 99));
                }
            }
        }
    }
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    // A* が y=1.3 (グリッド gy=26) を迂回する経路
    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 50; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int x = 51; x <= 69; ++x) {
        grid_path.push_back({x, 26});  // 迂回
    }
    for (int x = 70; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 1.0};

    auto smoothed = planner->smoothPathElasticBand(grid_path, exact_start, exact_goal);
    ASSERT_FALSE(smoothed.empty());

    // 全点が max_cost (40) 以下を厳守していること
    for (const auto& pt : smoothed) {
        int gx = static_cast<int>(std::round(pt.first / 0.05));
        int gy = static_cast<int>(std::round(pt.second / 0.05));
        if (gx >= 0 && gx < 200 && gy >= 0 && gy < 200) {
            int cost = costmap.data[gy * 200 + gx];
            EXPECT_LE(cost, 40) << "Point (" << pt.first << ", " << pt.second << ") penetrated high cost " << cost;
        }
    }
}

TEST_F(PathSmoothingElasticBandTest, DeterministicConsistency)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("elastic_band.iterations", 30);
    options.append_parameter_override("elastic_band.elastic_strength", 0.5);
    options.append_parameter_override("elastic_band.repulsive_strength", 0.05);
    options.append_parameter_override("elastic_band.repulsive_cost_threshold", 15);
    options.append_parameter_override("elastic_band.max_cost", 40);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);
    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 80; ++x) {
        grid_path.push_back({x, 20});
    }

    const std::pair<double, double> start1 = {1.0, 1.0};
    const std::pair<double, double> goal   = {4.0, 1.0};

    auto path1 = planner->smoothPathElasticBand(grid_path, start1, goal);
    auto path2 = planner->smoothPathElasticBand(grid_path, start1, goal);

    ASSERT_EQ(path1.size(), path2.size());
    for (std::size_t i = 0; i < path1.size(); ++i) {
        EXPECT_NEAR(path1[i].first, path2[i].first, 1e-9);
        EXPECT_NEAR(path1[i].second, path2[i].second, 1e-9);
    }
}

TEST_F(PathSmoothingElasticBandTest, HybridReducesCurvatureWithFillet)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("elastic_band.iterations", 30);
    options.append_parameter_override("elastic_band.elastic_strength", 1.0);
    options.append_parameter_override("elastic_band.repulsive_strength", 0.05);
    options.append_parameter_override("elastic_band.repulsive_cost_threshold", 40);
    options.append_parameter_override("elastic_band.max_cost", 60);
    options.append_parameter_override("elastic_band.path_resolution", 0.05);

    options.append_parameter_override("fillet.rdp_epsilon", 0.15);
    options.append_parameter_override("fillet.target_radius", 2.0);
    options.append_parameter_override("fillet.min_radius", 0.2);
    options.append_parameter_override("fillet.max_cost", 60);
    options.append_parameter_override("fillet.path_resolution", 0.05);
    options.append_parameter_override("fillet.curvature_blend_iterations", 3);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);
    auto costmap = createMockCostmap(200, 200, 0.05, 0);
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    // (1.0, 1.0) -> (5.0, 1.0) -> (5.0, 5.0) の直角 90 度コーナー
    std::vector<std::pair<int, int>> grid_path;
    for (int x = 20; x <= 100; ++x) {
        grid_path.push_back({x, 20});
    }
    for (int y = 21; y <= 100; ++y) {
        grid_path.push_back({100, y});
    }

    const std::pair<double, double> exact_start = {1.0, 1.0};
    const std::pair<double, double> exact_goal  = {5.0, 5.0};

    // Elastic Band 単体
    auto eb_only = planner->smoothPathElasticBand(grid_path, exact_start, exact_goal);
    // ハイブリッド (EB + Fillet)
    auto hybrid = planner->smoothPath(grid_path, exact_start, exact_goal);

    ASSERT_FALSE(eb_only.empty());
    ASSERT_FALSE(hybrid.empty());

    // コーナー頂点 (5.0, 1.0) への最近傍距離
    double dist_eb = 1e9;
    for (const auto& pt : eb_only) {
        dist_eb = std::min(dist_eb, std::hypot(pt.first - 5.0, pt.second - 1.0));
    }

    double dist_hybrid = 1e9;
    for (const auto& pt : hybrid) {
        dist_hybrid = std::min(dist_hybrid, std::hypot(pt.first - 5.0, pt.second - 1.0));
    }

    // Fillet により大きな円弧 (R=2.0m) が入るため、頂点からのクリアランスが大幅に拡大
    // (＝旋回半径が大きくなり、最大曲率が低減されている)
    EXPECT_GT(dist_hybrid, dist_eb + 0.2);
    EXPECT_GT(dist_hybrid, 0.4);
}

TEST_F(PathSmoothingElasticBandTest, ReproduceBucket1ToBucket2)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("elastic_band.iterations", 30);
    options.append_parameter_override("elastic_band.elastic_strength", 1.0);
    options.append_parameter_override("elastic_band.repulsive_strength", 0.05);
    options.append_parameter_override("elastic_band.repulsive_cost_threshold", 40);
    options.append_parameter_override("elastic_band.max_cost", 60);
    options.append_parameter_override("elastic_band.path_resolution", 0.05);

    options.append_parameter_override("fillet.rdp_epsilon", 0.15);
    options.append_parameter_override("fillet.target_radius", 2.0);
    options.append_parameter_override("fillet.min_radius", 0.2);
    options.append_parameter_override("fillet.max_cost", 60);
    options.append_parameter_override("fillet.path_resolution", 0.05);
    options.append_parameter_override("fillet.curvature_blend_iterations", 3);

    auto planner = std::make_shared<gn10_navigation::PathPlannerNode>(options);
    auto costmap = createMockCostmap(240, 260, 0.05, 0);
    costmap.info.origin.position.x = -6.0;
    costmap.info.origin.position.y = -6.5;
    planner->onCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap));

    // FixedBucket1 (0.56, 2.0) -> FixedBucket2 (-1.2724, 0.8996)
    // 粗い格子パスを作成
    std::vector<std::pair<int, int>> grid_path;
    int sx = 131, sy = 169;
    int gx = 94, gy = 146;
    for (int step = 0; step <= 50; ++step) {
        double t = static_cast<double>(step) / 50.0;
        grid_path.push_back({
            static_cast<int>(std::round(sx + t * (gx - sx))),
            static_cast<int>(std::round(sy + t * (gy - sy)))
        });
    }

    std::pair<double, double> exact_start = {0.56, 2.0};
    std::pair<double, double> approach_goal = {-1.275, 0.825};

    auto path = planner->smoothPath(grid_path, exact_start, approach_goal);
    ASSERT_FALSE(path.empty());
    for (std::size_t i = 0; i < path.size(); ++i) {
        EXPECT_TRUE(std::isfinite(path[i].first)) << "path[" << i << "].x is not finite";
        EXPECT_TRUE(std::isfinite(path[i].second)) << "path[" << i << "].y is not finite";
        EXPECT_LT(std::abs(path[i].first), 50.0) << "path[" << i << "].x is " << path[i].first;
        EXPECT_LT(std::abs(path[i].second), 50.0) << "path[" << i << "].y is " << path[i].second;
    }
}
