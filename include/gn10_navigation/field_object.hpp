#pragma once

#include <string>

enum ObjectType { CYLINDER, BOX, VISUAL_BOX };

struct FieldObject {
    std::string comment;
    ObjectType type;
    float center_x, center_y;
    float z_min, z_max;
    float param1, param2;
};