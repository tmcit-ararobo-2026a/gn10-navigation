#include "gn10_navigation/map_loader.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

using json = nlohmann::json;

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
            obj.type             = (type_str == "CYLINDER") ? CYLINDER :
                                   (type_str == "VISUAL_BOX") ? VISUAL_BOX : BOX;
            obj.center_x         = item.at("x").get<float>();
            obj.center_y         = item.at("y").get<float>();
            obj.z_min            = item.at("z_min").get<float>();
            obj.z_max            = item.at("z_max").get<float>();
            obj.param1           = item.at("param1").get<float>();
            obj.param2           = item.at("param2").get<float>();
            map.push_back(obj);
        }
    } catch (const std::exception& e) {
        std::cerr << "[MapLoader] JSON parse error: " << e.what() << std::endl;
    }

    return map;
}