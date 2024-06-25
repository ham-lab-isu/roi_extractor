#pragma once

#include "boundary_order_generator.h"

namespace roi_detector
{
class TSPBoundaryOrderGenerator : public BoundaryOrderGenerator
{
public:
  TSPBoundaryOrderGenerator() = default;
  //  TSPBoundaryOrderGenerator::TSPBoundaryOrderGenerator(...);
  Boundary order(const Boundary& boundary) const override;

private:
  //
};

} // namespace snp_isu_tpp

