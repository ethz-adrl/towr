/**
@file   zmp_optimizer.cc
@author Alexander Winkler (winklera@ethz.ch)
@date   Oct 21,  2014
@brief  Dynamic Walking using Zero-Moment-Point (ZMP) Criteria
 */

#ifndef _XPP_ZMP_OPTIMIZER_H_
#define _XPP_ZMP_OPTIMIZER_H_

#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/zmp_constraint.h>

#include <log4cxx/logger.h>
#include <Eigen/Dense>

#include <memory> //std::shared_ptr
#include <array>
#include <vector>


/**
@brief Dynamic ("pp" = ++) locomotion ("x"-coordinate)

All functions related to hyq, all utilities and all optimizations are a nested
namespace of this.
 */
namespace xpp {

/**
@brief Zero-Moment-Point (ZMP) optimization

All classes that optimize the CoG trajectory while keeping the ZMP inside the
current support triangle are located in this namespace
 */
namespace zmp {


/**
\class ZmpOptimizer
\brief Optimizes spline coefficients w.r.t Zero-Moment-Point criteria.

This class provides and interface to optimize the trajectory of the CoM of HyQ
for a specific sequence of footsteps. The resulting trajectory will always keep
the ZMP inside the current support triangle. The QP-solver implemented in
eigen_quadprog.hpp performs the optimization.

 */
class QpOptimizer {
public:
  typedef ::xpp::hyq::SupportPolygon SuppTriangle;
  typedef ::xpp::utils::Vec2d Position;
  typedef std::array<double,2> WeightsXYArray;
  typedef xpp::utils::MatVec MatVec;

public:
  QpOptimizer();

  /**
   * @param spline_structure the amount and sequence of splines with empty coefficients
   */
  QpOptimizer(const ContinuousSplineContainer& spline_structure,
              const xpp::hyq::SupportPolygonContainer& supp_triangle_container,
              const WeightsXYArray& weight,
              double walking_height);

  virtual ~QpOptimizer();

/*!
 @brief Optimizes the trajectory of the CoM according to a quadratic
        cost function while keeping the ZMP in the current support
        triangle formed by the stance feet.

 @param start_stance Position of the four feet before walking
 @param t_stance_initial The time permitted for the first 4-leg-support cog shift
 @param steps Footholds the robot must execute
 @param[out] splines optimized cog trajectory
 @throws std::runtime_error Throws if no solution to the QP problem was found.
 */

  Eigen::VectorXd SolveQp();


  MatVec cf_;
  MatVec eq_;

  ContinuousSplineContainer zmp_splines_;
  ZmpConstraint zmp_constraint_;

private:
  void SetupQpMatrices(const WeightsXYArray& weight,
                       const xpp::hyq::SupportPolygonContainer& supp_poly_container,
                       const Position& end_cog,
                       double height_robot);

  MatVec ineq_;
  MatVec CreateMinAccCostFunction(const WeightsXYArray& weight) const;
  MatVec CreateEqualityContraints(const Position &end_cog) const;
  MatVec CreateInequalityContraints(double walking_height,
                                    const xpp::hyq::SupportPolygonContainer& supp_poly_container) const;


  static log4cxx::LoggerPtr log_;
  static log4cxx::LoggerPtr log_matlab_;
};


} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_OPTIMIZER_H_
