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

std::vector<FieldObject> MapLoader::loadFromParams(const std::vector<std::string>& param_strings)
{
    // 形式: "TYPE,x,y,z_min,z_max,param1,param2"
    std::vector<FieldObject> map;
    for (const auto& s : param_strings) {
        std::stringstream ss(s);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        if (tokens.size() == 7) {
            FieldObject obj;
            obj.type     = (tokens[0] == "CYLINDER") ? CYLINDER :
                           (tokens[0] == "VISUAL_BOX") ? VISUAL_BOX : BOX;
            obj.center_x = std::stof(tokens[1]);
            obj.center_y = std::stof(tokens[2]);
            obj.z_min    = std::stof(tokens[3]);
            obj.z_max    = std::stof(tokens[4]);
            obj.param1   = std::stof(tokens[5]);
            obj.param2   = std::stof(tokens[6]);
            map.push_back(obj);
        }
    }
    return map;
}

std::vector<FieldObject> MapLoader::createNHK2026FieldMap()
{
    std::vector<FieldObject> map;

    // 1. CAD fence: 150 mm L-section, 24 mm foot and inner upright.
    // The floor foot is displayed but not matched because ground filtering
    // removes its returns. The 24 mm upright is the LiDAR-facing surface.
    map.push_back({VISUAL_BOX, 0.000f, 5.775f, 0.000f, 0.024f, 5.400f, 0.075f});
    map.push_back({VISUAL_BOX, 0.000f, -5.775f, 0.000f, 0.024f, 5.400f, 0.075f});
    map.push_back({VISUAL_BOX, -5.325f, 0.000f, 0.000f, 0.024f, 0.075f, 5.700f});
    map.push_back({VISUAL_BOX, 5.325f, 0.000f, 0.000f, 0.024f, 0.075f, 5.700f});
    map.push_back({BOX, 0.000f, 5.712f, 0.024f, 0.150f, 5.400f, 0.012f});
    map.push_back({BOX, 0.000f, -5.712f, 0.024f, 0.150f, 5.400f, 0.012f});
    map.push_back({BOX, -5.262f, 0.000f, 0.024f, 0.150f, 0.012f, 5.700f});
    map.push_back({BOX, 5.262f, 0.000f, 0.024f, 0.150f, 0.012f, 5.700f});

    // 2. 教壇 (X: -5.25~5.25m, Y: -0.3~0.3m, H: 0.2m)
    map.push_back({BOX, 0.000f, 0.000f, 0.000f, 0.200f, 5.250f, 0.300f});

    struct ObjectSpec {
        ObjectType type;
        float x, y;
        float param1, param2;  // CYL: radius, 0 / BOX: half_w, half_d
        float z_min, z_max;
    };

    constexpr float bucket_radius = 0.273f / 2.0f;  // バケツ半径 0.1365m
    std::vector<ObjectSpec> base_specs;

    // バケツ① (φ0.273 x H0.255)
    base_specs.push_back({CYLINDER, 0.550f, 0.870f, bucket_radius, 0.000f, 0.000f, 0.255f});

    // バケツ② (台座 0.3x0.3xH0.6 + バケツ①)
    base_specs.push_back({BOX, -1.270f, 1.480f, 0.150f, 0.150f, 0.000f, 0.600f});
    base_specs.push_back({CYLINDER, -1.270f, 1.480f, bucket_radius, 0.000f, 0.600f, 0.855f});

    // バケツ③ (台座 0.3x0.3xH0.3 + バケツ①)
    base_specs.push_back({BOX, 2.370f, 1.480f, 0.150f, 0.150f, 0.000f, 0.300f});
    base_specs.push_back({CYLINDER, 2.370f, 1.480f, bucket_radius, 0.000f, 0.300f, 0.555f});

    // 椅子 (W0.36 x D0.40, H0.807)
    base_specs.push_back({BOX, 0.550f, 4.980f, 0.180f, 0.200f, 0.000f, 0.807f});

    // Each side has desk 1, desk 2, a control-station desk and a refill desk.
    constexpr float desk_coords[4][2] = {
        {-2.295f, 3.855f},
        { 3.395f, 3.855f},
        {-4.895f, 5.445f},
        {-4.750f, 1.105f}
    };
    for (const auto& coord : desk_coords) {
        base_specs.push_back({BOX, coord[0], coord[1], 0.325f, 0.225f, 0.000f, 0.760f});
    }

    // 旗 (土台 0.39x0.39xH0.18 + 支柱 φ0.06 x H3.0)
    base_specs.push_back({BOX, 0.550f, 3.025f, 0.195f, 0.195f, 0.000f, 0.180f});
    base_specs.push_back({CYLINDER, 0.550f, 3.025f, 0.030f, 0.000f, 0.180f, 3.000f});
    base_specs.push_back({BOX, 0.850f, 3.025f, 0.300f, 0.020f, 1.200f, 3.000f});

    // 領域A (+Y) と 領域B (-Y) に対称展開
    for (const auto& spec : base_specs) {
        for (const float y_sign : {1.0f, -1.0f}) {
            map.push_back(
                {spec.type,
                 spec.x,
                 spec.y * y_sign,
                 spec.z_min,
                 spec.z_max,
                 spec.param1,
                 spec.param2}
            );
        }
    }

    return map;
}
