#include "gn10_navigation/target_repository.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>

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

    // share / install ディレクトリが指定された場合は、ソースリポジトリ側へのマッピングを試みる
    bool is_share_or_install_path =
        (path.find("install/") != std::string::npos ||
         path.find("share/gn10_navigation") != std::string::npos);

    if (!is_share_or_install_path && fs::exists(path)) {
        return fs::absolute(path).string();
    }

    // パッケージソースディレクトリ（コンパイル時マクロ GN10_NAVIGATION_SOURCE_DIR）からの解決
    std::string source_dir = GN10_NAVIGATION_SOURCE_DIR;
    if (!source_dir.empty() && fs::exists(source_dir)) {
        fs::path p_in_source = fs::path(source_dir) / path;
        if (fs::exists(p_in_source)) {
            return fs::absolute(p_in_source).string();
        }

        // ファイル名だけで config/ 配下を探索
        fs::path filename = fs::path(path).filename();
        fs::path p_config = fs::path(source_dir) / "config" / filename;
        if (fs::exists(p_config)) {
            return fs::absolute(p_config).string();
        }
    }

    // ワークスペース相対パス "src/gn10-navigation/..." の探索
    fs::path ws_src_path = fs::path("src/gn10-navigation") / path;
    if (fs::exists(ws_src_path)) {
        return fs::absolute(ws_src_path).string();
    }

    fs::path ws_src_config = fs::path("src/gn10-navigation/config") / fs::path(path).filename();
    if (fs::exists(ws_src_config)) {
        return fs::absolute(ws_src_config).string();
    }

    // 保存先として再ビルドで消えないソースリポジトリ内をデフォルトターゲットにする
    if (!source_dir.empty() && fs::exists(source_dir)) {
        fs::path target = fs::path(source_dir) / path;
        return fs::absolute(target).string();
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
