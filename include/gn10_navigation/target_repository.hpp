#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "gn10_navigation/navigation_types.hpp"

namespace gn10_navigation {

struct TargetPoint {
    std::string name;
    double x               = 0.0;
    double y               = 0.0;
    double yaw             = 0.0;
    double target_velocity = 0.0;
    std::string memo;
};

/**
 * @brief ターゲット射出ポイントデータの管理・ファイル永続化を担当するリポジトリクラス。
 *        shareディレクトリ（installディレクトリ）を使用せず、パッケージソースリポジトリ側でYAMLファイルを管理する。
 */
class TargetRepository
{
public:
    explicit TargetRepository(const std::string& default_file_path = "config/shooting_points.yaml");
    ~TargetRepository() = default;

    std::string resolveFilePath(const std::string& path) const;

    bool loadFromFile(const std::string& file_path);
    bool saveToFile(const std::string& file_path) const;

    bool load();
    bool save() const;
    std::string toYamlString() const;

    bool has(uint8_t id) const;
    std::optional<TargetPoint> get(uint8_t id) const;
    void update(uint8_t id, const TargetPoint& point);
    const std::map<uint8_t, TargetPoint>& getAll() const;
    size_t size() const;
    void clear();

    std::string getResolvedFilePath() const
    {
        return resolved_file_path_;
    }
    void setFilePath(const std::string& file_path);

private:
    std::string file_path_;
    std::string resolved_file_path_;
    std::map<uint8_t, TargetPoint> shooting_points_;
};

}  // namespace gn10_navigation
