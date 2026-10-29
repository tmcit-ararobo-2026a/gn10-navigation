// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <vector>

namespace gn10_navigation {
// Blue outer polygon in config/robot_shape_dimensioned.svg, counterclockwise [m].
// Origin: original shape centroid (SVG 819.90, 850.332381 mm).
// +x forward (SVG up), +y left (SVG left).
inline const std::vector<double> kDefaultFootprint = {
    0.525332381, 0.209900000,
    -0.080887619, 0.559900000,
    -0.394447619, 0.350000000,
    -0.394447619, -0.350000000,
    -0.080887619, -0.559910000,
    0.525332381, -0.209910000,
    0.590332381, -0.080000000,
    0.590332381, 0.080000000,
};
// Reduced center clearance for close approaches, shared by all obstacles.
// Below the 0.595729 m circumscribed radius: contact is possible at some headings.
inline constexpr double kDefaultRobotRadius = 0.45;
inline constexpr double kDefaultInflationMargin = 0.30;
}  // namespace gn10_navigation
