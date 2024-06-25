#pragma once

#include "boundary_order_generator.h"

namespace roi_detector
{
class CrustBoundaryOrderGenerator : public BoundaryOrderGenerator
{
public:
  CrustBoundaryOrderGenerator() = default;
  Boundary order(const Boundary& boundary) const override;

private:
};

} // namespace snp_isu_tpp

