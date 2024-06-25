#pragma once

#include "boundary_order_generator.h"

namespace roi_detector
{
class KdTreeBoundaryOrderGenerator : public BoundaryOrderGenerator
{
public:
  KdTreeBoundaryOrderGenerator(int k = 10);
  Boundary order(const Boundary& boundary) const override;

private:
  int k_;
};

} // namespace snp_isu_tpp
