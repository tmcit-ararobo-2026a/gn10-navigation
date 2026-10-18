// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace gn10_navigation {
/** @brief Preserve custom unsigned 0..254 costs stored in an int8 ROS field. */
inline int8_t merge_cost(int8_t previous, uint8_t incoming)
{
    return static_cast<int8_t>(std::max(static_cast<uint8_t>(previous), incoming));
}

/** @brief Axis-aligned grid collision view with conservative supercover traversal. */
class GridSafety
{
public:
    /** @brief View a validated grid without copying its storage. */
    GridSafety(
        const std::vector<uint8_t>& data,
        unsigned width,
        unsigned height,
        double resolution,
        double origin_x,
        double origin_y
    )
        : data_(data),
          width_(width),
          height_(height),
          resolution_(resolution),
          origin_x_(origin_x),
          origin_y_(origin_y)
    {
    }

    /** @brief Reject out-of-map, nonfinite and lethal/unknown cells, including boundaries. */
    bool point_passable(double world_x, double world_y) const
    {
        double x, y;
        if (!grid_coordinates(world_x, world_y, x, y)) {
            return false;
        }
        const int cell_x = static_cast<int>(std::floor(x));
        const int cell_y = static_cast<int>(std::floor(y));
        if (!cell_passable(cell_x, cell_y)) {
            return false;
        }
        const bool boundary_x = x == std::floor(x) && cell_x > 0;
        const bool boundary_y = y == std::floor(y) && cell_y > 0;
        if ((boundary_x && !cell_passable(cell_x - 1, cell_y)) ||
            (boundary_y && !cell_passable(cell_x, cell_y - 1)) ||
            (boundary_x && boundary_y && !cell_passable(cell_x - 1, cell_y - 1))) {
            return false;
        }
        return true;
    }

    /** @brief Check every touched cell, including corner-adjacent and boundary cells. */
    bool segment_passable(double start_x, double start_y, double goal_x, double goal_y) const
    {
        if (!point_passable(start_x, start_y) || !point_passable(goal_x, goal_y)) {
            return false;
        }
        double first_x, first_y, last_x, last_y;
        if (!grid_coordinates(start_x, start_y, first_x, first_y) ||
            !grid_coordinates(goal_x, goal_y, last_x, last_y)) {
            return false;
        }
        int x                = static_cast<int>(std::floor(first_x));
        int y                = static_cast<int>(std::floor(first_y));
        const int end_x      = static_cast<int>(std::floor(last_x));
        const int end_y      = static_cast<int>(std::floor(last_y));
        const double delta_x = last_x - first_x;
        const double delta_y = last_y - first_y;
        int step_x = 0, step_y = 0;
        double next_x = std::numeric_limits<double>::infinity();
        double next_y = next_x, step_time_x = next_x, step_time_y = next_x;
        if (delta_x > 0) {
            step_x      = 1;
            next_x      = (x + 1 - first_x) / delta_x;
            step_time_x = 1.0 / delta_x;
        } else if (delta_x < 0) {
            step_x      = -1;
            next_x      = (x - first_x) / delta_x;
            step_time_x = -1.0 / delta_x;
        }
        if (delta_y > 0) {
            step_y      = 1;
            next_y      = (y + 1 - first_y) / delta_y;
            step_time_y = 1.0 / delta_y;
        } else if (delta_y < 0) {
            step_y      = -1;
            next_y      = (y - first_y) / delta_y;
            step_time_y = -1.0 / delta_y;
        }
        const bool on_vertical_boundary   = delta_x == 0 && first_x == std::floor(first_x) && x > 0;
        const bool on_horizontal_boundary = delta_y == 0 && first_y == std::floor(first_y) && y > 0;
        const auto touched_passable       = [&](int column, int row) {
            return cell_passable(column, row) &&
                   (!on_vertical_boundary || cell_passable(column - 1, row)) &&
                   (!on_horizontal_boundary || cell_passable(column, row - 1)) &&
                   (!(on_vertical_boundary && on_horizontal_boundary) ||
                    cell_passable(column - 1, row - 1));
        };
        // Each iteration crosses a grid boundary; at most width+height crossings exist.
        while (x != end_x || y != end_y) {
            if (std::abs(next_x - next_y) <= 1e-12) {
                if (!touched_passable(x + step_x, y) || !touched_passable(x, y + step_y)) {
                    return false;
                }
                x += step_x;
                y += step_y;
                next_x += step_time_x;
                next_y += step_time_y;
            } else if (next_x < next_y) {
                x += step_x;
                next_x += step_time_x;
            } else {
                y += step_y;
                next_y += step_time_y;
            }
            if (!touched_passable(x, y)) {
                return false;
            }
        }
        return true;
    }

private:
    /** @brief Validate coordinate conversion before narrowing to integer cell indices. */
    bool grid_coordinates(double world_x, double world_y, double& x, double& y) const
    {
        if (!std::isfinite(world_x) || !std::isfinite(world_y) || !std::isfinite(resolution_) ||
            resolution_ <= 0 || width_ == 0 || height_ == 0 ||
            width_ > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            height_ > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            data_.size() != static_cast<std::size_t>(width_) * height_) {
            return false;
        }
        x = (world_x - origin_x_) / resolution_;
        y = (world_y - origin_y_) / resolution_;
        return std::isfinite(x) && std::isfinite(y) && x >= 0 && y >= 0 && x < width_ &&
               y < height_;
    }
    /** @brief Test custom costs, rejecting 254 and unknown 255. */
    bool cell_passable(int x, int y) const
    {
        return x >= 0 && y >= 0 && static_cast<unsigned>(x) < width_ &&
               static_cast<unsigned>(y) < height_ &&
               data_[static_cast<std::size_t>(y) * width_ + static_cast<unsigned>(x)] < 254;
    }
    const std::vector<uint8_t>& data_;
    unsigned width_, height_;
    double resolution_, origin_x_, origin_y_;
};
}  // namespace gn10_navigation
