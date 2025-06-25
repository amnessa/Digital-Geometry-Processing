#pragma once

#include <Eigen/Dense>

struct BindingData {
    int joint_index;
    int triangle_index;
    Eigen::Vector3d barycentric_coords;
};
