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

    // ROS2パラメータ(文字列リスト)からマップを読み込み
    static std::vector<FieldObject> loadFromParams(const std::vector<std::string>& param_strings);

    // ハードコードされた標準NHK2026フィールドマップ（フォールバック用）
    static std::vector<FieldObject> createNHK2026FieldMap();
};