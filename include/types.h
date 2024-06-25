#pragma once

#include <Eigen/Dense>

namespace roi_detector
{
using Boundary = Eigen::Matrix4Xf;
using Boundaries = std::vector<Boundary, Eigen::aligned_allocator<Boundary>>;
}
