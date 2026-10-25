#include "gn10_navigation/map_loader.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace gn10_navigation {

std::vector<FieldObject> MapLoader::loadFromJSON(const std::string& file_path)
{
    std::vector<FieldObject> map;
    std::ifstream f(file_path);
    if (!f.is_open()) {
        std::cerr << "[MapLoader] Failed to open map file: " << file_path << std::endl;
        return map;
    }

    try {
        json j = json::parse(f);
        for (const auto& item : j) {
            FieldObject obj;
            // comment が存在すれば読み込む（無ければ空文字列）
            if (item.contains("comment")) {
                obj.comment = item.at("comment").get<std::string>();
            } else {
                obj.comment = "";
            }

            std::string type_str = item.at("type").get<std::string>();
            if (type_str == "CYLINDER") {
                obj.type = CYLINDER;
            } else if (type_str == "VISUAL_BOX") {
                obj.type = VISUAL_BOX;
            } else if (type_str == "ZONE" || type_str == "PENALTY_ZONE") {
                obj.type = ZONE;
            } else {
                obj.type = BOX;
            }

            obj.center_x = item.at("x").get<float>();
            obj.center_y = item.at("y").get<float>();
            obj.z_min    = item.at("z_min").get<float>();
            obj.z_max    = item.at("z_max").get<float>();

            // 形状パラメータ（新仕様: radius, half_x, half_y / 旧仕様: param1, param2）
            if (obj.type == CYLINDER) {
                if (item.contains("radius")) {
                    obj.set_radius(item.at("radius").get<float>());
                } else if (item.contains("param1")) {
                    obj.set_radius(item.at("param1").get<float>());
                } else {
                    obj.set_radius(0.0f);
                }
            } else {
                // BOX / VISUAL_BOX / ZONE
                float hx = 0.0f;
                float hy = 0.0f;
                if (item.contains("half_x")) {
                    hx = item.at("half_x").get<float>();
                } else if (item.contains("half_width")) {
                    hx = item.at("half_width").get<float>();
                } else if (item.contains("size_x")) {
                    hx = item.at("size_x").get<float>() * 0.5f;
                } else if (item.contains("param1")) {
                    hx = item.at("param1").get<float>();
                }

                if (item.contains("half_y")) {
                    hy = item.at("half_y").get<float>();
                } else if (item.contains("half_depth")) {
                    hy = item.at("half_depth").get<float>();
                } else if (item.contains("size_y")) {
                    hy = item.at("size_y").get<float>() * 0.5f;
                } else if (item.contains("param2")) {
                    hy = item.at("param2").get<float>();
                }
                obj.set_half_sizes(hx, hy);
            }

            // コスト値の読み込み（新仕様: 数値指定 [0..254]）
            if (item.contains("cost")) {
                obj.cost = item.at("cost").get<int>();
            } else {
                obj.cost = -1;  // 未指定（ノード側でcommentまたはデフォルト判定）
            }

            // 通行可能フラグ（passable: 避けたい場所だが通行は許可）
            if (item.contains("passable")) {
                obj.passable = item.at("passable").get<bool>();
            } else if (item.contains("is_obstacle")) {
                obj.passable = !item.at("is_obstacle").get<bool>();
            } else if (obj.type == ZONE) {
                obj.passable = true;  // ZONE タイプはデフォルトで通過可能ペナルティ領域
            } else {
                obj.passable = false; // デフォルトは障害物（不可侵）
            }

            map.push_back(obj);
        }
    } catch (const std::exception& e) {
        std::cerr << "[MapLoader] JSON parse error: " << e.what() << std::endl;
    }

    return map;
}

}  // namespace gn10_navigation
