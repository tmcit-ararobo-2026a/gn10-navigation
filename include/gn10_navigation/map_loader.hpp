#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "gn10_navigation/field_object.hpp"

class MapLoader
{
public:
    // JSONファイルからマップを読み込み
    static std::vector<FieldObject> loadFromJSON(const std::string& file_path);
};