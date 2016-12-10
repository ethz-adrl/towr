/*
 * a_constraint.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/opt/a_constraint.h>
#include <iostream>
#include <iomanip>

namespace xpp {
namespace opt {

const AConstraint::Bound AConstraint::kNoBound_                 = Bound(-1.0e20, +1.0e20);
const AConstraint::Bound AConstraint::kEqualityBound_           = Bound(0.0, 0.0);
const AConstraint::Bound AConstraint::kInequalityBoundPositive_ = Bound(0.0, 1.0e20);


AConstraint::AConstraint ()
{
  name_ = "NoName";
}

AConstraint::~AConstraint ()
{
  // TODO Auto-generated destructor stub
}

int
AConstraint::GetNumberOfConstraints () const
{
  return GetBounds().size();
}

void
xpp::opt::AConstraint::PrintStatus (double tol) const
{
  auto bounds = GetBounds();
  auto g = EvaluateConstraint();

  std::cout << std::setw(17) << std::left << name_;
  std::cout << "[" << std::setw(3) << std::right << g.rows() << "]:  ";

  int i=0;
  for (auto b : bounds) {
    bool g_too_small = g(i) < b.lower_ - tol;
    bool g_too_large = g(i) > b.upper_ + tol;

    if (g_too_small || g_too_large)
      std::cout << i << ",";
    i++;
  }

  std::cout << std::endl;
}

} /* namespace opt */
} /* namespace xpp */

