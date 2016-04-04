/*
 * constraints.h
 *
 *  Created on: Mar 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>

#include <xpp/zmp/zmp_constraint.h>

namespace xpp {
namespace zmp {


class Constraints {

public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVecEigen2d;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  typedef SupportPolygonContainer::VecFoothold VecFoothold;

  struct Bound {
    Bound(double lower, double upper) {
      lower_ = lower;
      upper_ = upper;
    }
    double lower_;
    double upper_;
  };

public:
  Constraints (const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
               const xpp::zmp::ContinuousSplineContainer& zmp_spline_container,
               const MatVec& qp_equality_constraints);
  virtual
  ~Constraints () {};

  Eigen::VectorXd EvalContraints(const Eigen::VectorXd& x_coeff,
                                 const StdVecEigen2d& footholds);

  xpp::zmp::ContinuousSplineContainer zmp_spline_container_;
  xpp::hyq::SupportPolygonContainer supp_polygon_container_;

  MatVec spline_junction_constraints_;

  Eigen::VectorXd g_;
  std::vector<Constraints::Bound> bounds_;
  const VecFoothold planned_footholds_;

  ZmpConstraint zmp_constraint_;
private:

  MatVec x_zmp_;
  MatVec y_zmp_;
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

  Eigen::VectorXd RestrictMaxStepLength(const StdVecEigen2d& footholds);

  Eigen::VectorXd RestrictFootholdToCogPos(const Eigen::VectorXd& x_coeff);

  void AddBounds(int m_constraints, double lower, double upper);
  void CombineToEigenVector(const std::vector<Eigen::VectorXd>& g_std, Eigen::VectorXd& g_eig) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONSTRAINTS_H_ */
