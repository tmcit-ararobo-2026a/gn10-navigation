#pragma once

#include <string>

namespace gn10_navigation {

enum ObjectType { CYLINDER, BOX, VISUAL_BOX, ZONE };

struct FieldObject {
    std::string comment;
    ObjectType type{BOX};
    float center_x{0.0f};
    float center_y{0.0f};
    float z_min{0.0f};
    float z_max{0.0f};

    // 形状パラメータ（新仕様: radius, half_x, half_y / 後方互換: param1, param2）
    float param1{0.0f};  // CYLINDER: radius, BOX: half_x
    float param2{0.0f};  // BOX: half_y

    // コスト・通行特性
    int cost{-1};         // 明示的コスト [0..254] (-1: 未指定、従来のcomment判定やデフォルト値を適用)
    bool passable{false}; // true: 通過可能ペナルティ領域（障害物ではなく避けたい場所）

    // 分かりやすいアクセサ
    float radius() const { return param1; }
    void set_radius(float r)
    {
        param1 = r;
        param2 = 0.0f;
    }

    float half_x() const { return param1; }
    float half_y() const { return param2; }
    void set_half_sizes(float hx, float hy)
    {
        param1 = hx;
        param2 = hy;
    }
};

}  // namespace gn10_navigation
