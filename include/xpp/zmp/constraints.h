/*
 * constraints.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {


class Constraints {

public:

  struct Bound {
    Bound(double lower, int upper) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };

public:
  Constraints ();
  virtual
  ~Constraints ();


//  void AddConstraint(Bound bound,

  Eigen::VectorXd EvalContraints(const Eigen::VectorXd& x) const;
  std::vector<Bound> GetBounds() const;

private:

  int n_equality_constraints_;
  int n_inequality_constraints_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
