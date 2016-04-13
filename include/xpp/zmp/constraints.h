/*
 * constraints.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_

#include <Eigen/Dense>


#include <xpp/zmp/problem_specification.h>
#include <xpp/zmp/spline_constraints.h>
#include <xpp/zmp/zmp_constraint.h>

namespace xpp {
namespace zmp {


class Constraints : public ProblemSpecification {

public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef xpp::zmp::SplineConstraints::State State;

  struct Bound {
    Bound(double lower = 0.0, double upper = 0.0) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };

public:
  Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
               const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
               double walking_height);
  virtual
  ~Constraints () {};

  Eigen::VectorXd EvalContraints(const Eigen::VectorXd& x_coeff,
                                 const StdVecEigen2d& footholds);


  MatVec spline_junction_constraints_;
  MatVec spline_initial_acc_constraints_;
  MatVec spline_final_constraints_;

  Eigen::VectorXd constraints_;
  std::vector<Constraints::Bound> bounds_;

  ZmpConstraint zmp_constraint_;

  const double gap_center_x_ = 0.25;
  const double gap_width_x_  = 0.3;
private:

  bool first_constraint_eval_ = true;


  // Add constraints here
  Eigen::VectorXd KeepZmpInSuppPolygon(const Eigen::VectorXd& x_coeff);

  Eigen::VectorXd FixFootholdPosition(const StdVecEigen2d& footholds);

  /**
   * This also includes the constraint on initial and final state!
   * FIXME move to separate function.
   * @param x_coeff
   * @return
   */
  Eigen::VectorXd SmoothAccJerkAtSplineJunctions(const Eigen::VectorXd& x_coeff);
  Eigen::VectorXd InitialAcceleration(const Eigen::VectorXd& x_coeff);
  Eigen::VectorXd FinalState(const Eigen::VectorXd& x_coeff);

  Eigen::VectorXd AddObstacle();

  Eigen::VectorXd RestrictFootholdToCogPos(const Eigen::VectorXd& x_coeff);

  void AddBounds(int m_constraints, double lower, double upper);
  void CombineToEigenVector(const std::vector<Eigen::VectorXd>& g_std, Eigen::VectorXd& g_eig) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
