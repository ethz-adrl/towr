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

#include <functional>

namespace xpp {
namespace zmp {


class Constraints : public ProblemSpecification {

public:
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef xpp::utils::MatVec MatVec;
  typedef Eigen::VectorXd VectorXd;
  typedef Eigen::Vector2d Vector2d;
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

  enum ConstraintType {
    EQUALITY=0,  //0.0 -> 0.0
    INEQUALITY=1, //0.0 -> inf
    COGTOFOOTHOLD=2,
  };

  struct Constraint {
    VectorXd values_;
    ConstraintType type_;
  };


public:
  Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
               const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
               double walking_height);
  virtual
  ~Constraints () {};


  std::vector<Bound> GetBounds();

  Eigen::VectorXd EvalContraints(const VectorXd& x_coeff,
                                 const StdVecEigen2d& footholds) const;

  MatVec spline_junction_constraints_;
  MatVec spline_initial_acc_constraints_;
  MatVec spline_final_constraints_;

  ZmpConstraint zmp_constraint_;

  const double gap_center_x_ = 0.25;
  const double gap_width_x_  = 0.3;
private:

  int n_constraints_;

  // Add constraints here
  Constraint KeepZmpInSuppPolygon(const VectorXd& x_coeff,
                                  const SupportPolygonContainer& support_polygon_container) const;
  Constraint FixFootholdPosition(const StdVecEigen2d& footholds) const;
  Constraint SmoothAccJerkAtSplineJunctions(const VectorXd& x_coeff) const;
  Constraint InitialAcceleration(const VectorXd& x_coeff) const;
  Constraint FinalState(const VectorXd& x_coeff) const;
  Constraint AddObstacle(const StdVecEigen2d& footholds) const;
  Constraint RestrictFootholdToCogPos(const VectorXd& x_coeff,
                                      const StdVecEigen2d& footholds) const;

  void AddBounds(int m_constraints, ConstraintType type,
                 std::vector<Bound>& bounds) const;
  std::vector<Constraint> GetConstraintsOnly(const VectorXd& x_coeff,
                                             const StdVecEigen2d& footholds) const;
  VectorXd CombineToEigenVector(const std::vector<Constraint>& g_std) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
