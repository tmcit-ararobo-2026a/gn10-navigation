// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include "gn10_navigation/grid_safety.hpp"

#include <gtest/gtest.h>

namespace gn10_navigation {
TEST(GridSafety, OverlappingCostsNeverDecreaseRegardlessOfDrawingOrder)
{
    for (uint8_t first : {120, 200, 254}) {
        for (uint8_t second : {120, 200, 254}) {
            const auto merged =
                static_cast<uint8_t>(merge_cost(static_cast<int8_t>(first), second));
            EXPECT_GE(merged, first);
            EXPECT_GE(merged, second);
            EXPECT_EQ(
                merge_cost(static_cast<int8_t>(first), second),
                merge_cost(static_cast<int8_t>(second), first)
            );
        }
    }
}
TEST(GridSafety, ChecksEndpointsAndEveryTouchedCell)
{
    std::vector<uint8_t> cells(25, 0);
    GridSafety grid(cells, 5, 5, 1, 0, 0);
    EXPECT_TRUE(grid.segment_passable(0.5, 0.5, 4.5, 4.5));
    cells[1] = 254;
    EXPECT_FALSE(grid.segment_passable(0.5, 0.5, 4.5, 4.5));
    EXPECT_FALSE(grid.segment_passable(4.5, 4.5, 0.5, 0.5));
    EXPECT_FALSE(grid.point_passable(1.0, 1.0));
    cells[1]  = 0;
    cells[12] = 255;
    EXPECT_FALSE(grid.segment_passable(0.5, 2.5, 4.5, 2.5));
    EXPECT_FALSE(grid.segment_passable(2.5, 2.5, 2.8, 2.8));
    EXPECT_FALSE(grid.segment_passable(2.1, 1.2, 2.9, 2.1));
}
TEST(GridSafety, BoundarySegmentsCheckBothAdjacentColumnsAndRows)
{
    std::vector<uint8_t> cells(25, 0);
    GridSafety grid(cells, 5, 5, 1, 0, 0);
    EXPECT_TRUE(grid.segment_passable(2.0, 0.5, 2.0, 4.5));
    cells[11] = 254;
    EXPECT_FALSE(grid.segment_passable(2.0, 0.5, 2.0, 4.5));
    cells[11] = 0;
    cells[7]  = 254;
    EXPECT_FALSE(grid.segment_passable(0.5, 2.0, 4.5, 2.0));
}
TEST(GridSafety, InvalidMapAndCoordinatesFailClosed)
{
    std::vector<uint8_t> cells(25, 0);
    GridSafety grid(cells, 5, 5, 1, 0, 0);
    EXPECT_FALSE(grid.segment_passable(-0.1, 0.5, 1.5, 1.5));
    EXPECT_FALSE(grid.point_passable(5, 1));
    EXPECT_FALSE(grid.point_passable(std::numeric_limits<double>::quiet_NaN(), 1));
    EXPECT_FALSE(grid.point_passable(1e300, 1));
    GridSafety invalid(cells, 5, 5, 0, 0, 0);
    EXPECT_FALSE(invalid.point_passable(0.5, 0.5));
}
}  // namespace gn10_navigation
