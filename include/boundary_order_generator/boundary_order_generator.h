#pragma once

#include "../types.h"

#include <memory>

namespace roi_detector
{
/**
 * @brief Orders a boundary such that the points are spatially sequential
 */
class BoundaryOrderGenerator
{
public:
  using Ptr = std::shared_ptr<BoundaryOrderGenerator>;
  using ConstPtr = std::shared_ptr<const BoundaryOrderGenerator>;

  virtual ~BoundaryOrderGenerator() = default;
  virtual Boundary order(const Boundary& boundary) const = 0;
};

}
