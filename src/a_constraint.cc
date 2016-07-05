/*
 * a_constraint.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/a_constraint.h>

namespace xpp {
namespace zmp {

const AConstraint::Bound AConstraint::kNoBound_                 = Bound(-1.0e20, +1.0e20);
const AConstraint::Bound AConstraint::kEqualityBound_           = Bound(0.0, 0.0);
const AConstraint::Bound AConstraint::kInequalityBoundPositive_ = Bound(0.0, 1.0e20);


AConstraint::AConstraint ()
{
  // TODO Auto-generated constructor stub
}

AConstraint::~AConstraint ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace zmp */
} /* namespace xpp */
