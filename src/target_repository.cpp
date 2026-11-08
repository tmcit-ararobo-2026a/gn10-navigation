#include "gn10_navigation/target_repository.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#ifndef GN10_NAVIGATION_SOURCE_DIR
#define GN10_NAVIGATION_SOURCE_DIR ""
#endif

namespace gn10_navigation {

TargetRepository::TargetRepository(const std::string& default_file_path)
    : file_path_(default_file_path)
{
    resolved_file_path_ = resolveFilePath(file_path_);
}

void TargetRepository::setFilePath(const std::string& file_path)
{
    file_path_          = file_path;
    resolved_file_path_ = resolveFilePath(file_path_);
}

std::string TargetRepository::resolveFilePath(const std::string& path) const
{
    namespace fs = std::filesystem;

    // 1. 環境変数が設定されている場合はそれを最優先
    const char* env_path = std::getenv("GN10_SHOOTING_POINTS_PATH");
    if (env_path != nullptr && fs::exists(env_path)) {
        return fs::absolute(env_path).string();
    }

    std::string home_dir = "";
    const char* env_home = std::getenv("HOME");
    if (env_home != nullptr) {
        home_dir = env_home;
    }

    fs::path target_path(path);
    std::string target_filename = target_path.filename().string();
    bool is_red =
        (path.find("red") != std::string::npos || target_filename.find("red") != std::string::npos);

    // 候補ファイル名の決定
    std::vector<std::string> search_filenames;
    if (!target_filename.empty() && target_filename != "." &&
        target_filename != "shooting_points.yaml") {
        search_filenames.push_back(target_filename);
    } else if (is_red) {
        search_filenames.push_back("shooting_points_red.yaml");
        search_filenames.push_back("shooting_point_red.yaml");
    } else {
        search_filenames.push_back("shooting_points_blue.yaml");
        search_filenames.push_back("shooting_point_blue.yaml");
        search_filenames.push_back("shooting_points.yaml");
        search_filenames.push_back("shooting_point.yaml");
    }

    // 2. data/shooting-points リポジトリ内の候補を列挙
    std::vector<fs::path> candidate_data_paths;
    for (const auto& fn : search_filenames) {
        if (!home_dir.empty()) {
            candidate_data_paths.push_back(
                fs::path(home_dir) / "ros2_ws/data/shooting-points" / fn
            );
            candidate_data_paths.push_back(
                fs::path(home_dir) / "ros2_ws/data/shooting-point-config" / fn
            );
        }
        candidate_data_paths.push_back(fs::path("data/shooting-points") / fn);
        candidate_data_paths.push_back(fs::path("../data/shooting-points") / fn);
    }

    // 3. 指定された path が明示的かつ直接存在し、かつ install/share や旧 src/config
    // ではない場合はそれを尊重
    bool is_share_or_install_path =
        (path.find("install/") != std::string::npos ||
         path.find("share/gn10_navigation") != std::string::npos);
    bool is_old_src_config = (path.find("src/gn10-navigation/config") != std::string::npos);

    if (!is_share_or_install_path && !is_old_src_config && fs::exists(path)) {
        return fs::absolute(path).string();
    }

    // 4. data/shooting-points の候補が存在すれば最優先で使用
    for (const auto& cand : candidate_data_paths) {
        if (fs::exists(cand)) {
            return fs::absolute(cand).string();
        }
    }

    // 5. パッケージソースディレクトリからの解決（従来のフォールバック）
    std::string source_dir = GN10_NAVIGATION_SOURCE_DIR;
    if (!source_dir.empty() && fs::exists(source_dir)) {
        // ソースディレクトリ基準で data/shooting-points を探索
        for (const auto& fn : search_filenames) {
            fs::path sibling_data =
                fs::path(source_dir).parent_path().parent_path() / "data" / "shooting-points" / fn;
            if (fs::exists(sibling_data)) {
                return fs::absolute(sibling_data).string();
            }
        }

        fs::path p_in_source = fs::path(source_dir) / path;
        if (fs::exists(p_in_source)) {
            return fs::absolute(p_in_source).string();
        }

        // ファイル名で config/ 配下を探索
        for (const auto& fn : search_filenames) {
            fs::path p_config = fs::path(source_dir) / "config" / fn;
            if (fs::exists(p_config)) {
                return fs::absolute(p_config).string();
            }
        }
    }

    // 6. 指定された path そのものが存在すればそれを使用 (install/share 含む)
    if (!path.empty() && fs::exists(path)) {
        return fs::absolute(path).string();
    }

    // 7. 保存先として data/shooting-points をデフォルトターゲットにする
    if (!candidate_data_paths.empty()) {
        return fs::absolute(candidate_data_paths.front()).string();
    }

    return path;
}

bool TargetRepository::loadFromFile(const std::string& file_path)
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["shooting_points"]) {
            std::cerr << "[TargetRepository] YAML does not contain 'shooting_points' key: "
                      << file_path << std::endl;
            return false;
        }

        shooting_points_.clear();
        auto sp = config["shooting_points"];

        if (sp.IsMap()) {
            for (auto it = sp.begin(); it != sp.end(); ++it) {
                uint8_t id = static_cast<uint8_t>(it->first.as<int>());
                TargetPoint pt;
                if (it->second["name"]) {
                    pt.name = it->second["name"].as<std::string>();
                }
                if (it->second["x"]) {
                    pt.x = it->second["x"].as<double>();
                }
                if (it->second["y"]) {
                    pt.y = it->second["y"].as<double>();
                }
                if (it->second["yaw"]) {
                    pt.yaw = it->second["yaw"].as<double>();
                }
                if (it->second["target_velocity"]) {
                    pt.target_velocity = it->second["target_velocity"].as<double>();
                }
                if (it->second["memo"]) {
                    pt.memo = it->second["memo"].as<std::string>();
                }
                shooting_points_[id] = pt;
            }
        } else if (sp.IsSequence()) {
            for (size_t i = 0; i < sp.size(); ++i) {
                uint8_t id = static_cast<uint8_t>(i);
                auto node  = sp[i];
                if (node["target_id"]) {
                    id = static_cast<uint8_t>(node["target_id"].as<int>());
                } else if (node["id"]) {
                    id = static_cast<uint8_t>(node["id"].as<int>());
                }
                TargetPoint pt;
                if (node["name"]) {
                    pt.name = node["name"].as<std::string>();
                }
                if (node["x"]) {
                    pt.x = node["x"].as<double>();
                }
                if (node["y"]) {
                    pt.y = node["y"].as<double>();
                }
                if (node["yaw"]) {
                    pt.yaw = node["yaw"].as<double>();
                }
                if (node["target_velocity"]) {
                    pt.target_velocity = node["target_velocity"].as<double>();
                }
                if (node["memo"]) {
                    pt.memo = node["memo"].as<std::string>();
                }
                shooting_points_[id] = pt;
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[TargetRepository] Error reading YAML '" << file_path << "': " << e.what()
                  << std::endl;
        return false;
    }
}

bool TargetRepository::saveToFile(const std::string& file_path) const
{
    try {
        YAML::Node root;
        for (const auto& [id, pt] : shooting_points_) {
            YAML::Node node;
            node["target_id"]       = static_cast<int>(id);
            node["name"]            = pt.name;
            node["x"]               = pt.x;
            node["y"]               = pt.y;
            node["yaw"]             = pt.yaw;
            node["target_velocity"] = pt.target_velocity;
            node["memo"]            = pt.memo;
            root["shooting_points"].push_back(node);
        }

        std::filesystem::path p(file_path);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }

        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            std::cerr << "[TargetRepository] Cannot open file for writing: " << file_path
                      << std::endl;
            return false;
        }
        fout << root;
        fout.close();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[TargetRepository] Error writing YAML '" << file_path << "': " << e.what()
                  << std::endl;
        return false;
    }
}

bool TargetRepository::load()
{
    return loadFromFile(resolved_file_path_);
}

bool TargetRepository::save() const
{
    return saveToFile(resolved_file_path_);
}

std::string TargetRepository::toYamlString() const
{
    YAML::Node root;
    for (const auto& [id, pt] : shooting_points_) {
        YAML::Node node;
        node["target_id"]       = static_cast<int>(id);
        node["name"]            = pt.name;
        node["x"]               = pt.x;
        node["y"]               = pt.y;
        node["yaw"]             = pt.yaw;
        node["target_velocity"] = pt.target_velocity;
        node["memo"]            = pt.memo;
        root["shooting_points"].push_back(node);
    }
    std::stringstream ss;
    ss << root;
    return ss.str();
}

bool TargetRepository::has(uint8_t id) const
{
    return shooting_points_.find(id) != shooting_points_.end();
}

std::optional<TargetPoint> TargetRepository::get(uint8_t id) const
{
    auto it = shooting_points_.find(id);
    if (it != shooting_points_.end()) {
        return it->second;
    }
    return std::nullopt;
}

void TargetRepository::update(uint8_t id, const TargetPoint& point)
{
    shooting_points_[id] = point;
}

const std::map<uint8_t, TargetPoint>& TargetRepository::getAll() const
{
    return shooting_points_;
}

size_t TargetRepository::size() const
{
    return shooting_points_.size();
}

void TargetRepository::clear()
{
    shooting_points_.clear();
}

}  // namespace gn10_navigation
