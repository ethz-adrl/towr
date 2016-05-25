/*
 * a_constraint.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_CONSTRAINT_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {



class AConstraint {
public:
  struct Bound {
    Bound(double lower = 0.0, double upper = 0.0) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };
  typedef Eigen::VectorXd VectorXd;
  typedef std::vector<Bound> VecBound;


  AConstraint ();
  virtual ~AConstraint ();

  virtual VectorXd EvaluateConstraint () const = 0;
  virtual VecBound GetBounds () const = 0;

protected:
  static const Bound kEqualityBound_;
  static const Bound kInequalityBoundPositive_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_CONSTRAINT_H_ */
