/*
 * SplineBuilder.h
 *
 *  Created on: May 28, 2014
 *      Author: awinkler
 */
/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time.
 */
#ifndef _XPP_ZMP_SPLINECONTAINER_H_
#define _XPP_ZMP_SPLINECONTAINER_H_

#include "zmp_spline.h"
#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/supp_triangle.h>

#include <vector>

namespace xpp {
namespace zmp {

/**
@class SplineContainer
@brief holds multiple splines and knows when these are active in the sequence.
*/
class SplineContainer {
public:
typedef std::vector<ZmpSpline> Splines;
typedef xpp::utils::Point2d Lin2d;

public:
  SplineContainer();
  SplineContainer(const Splines& splines);
  virtual ~SplineContainer();

  /**
  @brief Calculates the state of a spline at a specific point in time.

  @param double specific time of spline
  @param Derivative which value (pos,vel,acc) at this time we are interested in
  @return x and y state of position,velocity OR acceleration
  */
  void GetCOGxy(double t_global, Lin2d& cog_xy);
  void AddSpline(const ZmpSpline &spline);
  double GetTotalTime() const;


  // Creates a sequence of Splines without the optimized coefficients
  void ConstructSplineSequence(const std::vector<xpp::hyq::LegID>& step_sequence,
                                        double t_stance,
                                        double t_swing,
                                        double t_stance_initial,
                                        double t_stance_final);


  // this stuff might have to be moved to optimization affine class
  static const int kOptCoeff = kCoeffCount-2;
  static int var_index(int spline, int dim, int coeff);
  int GetOptCoeffCount() const;
  /**
   * Creates a Vector whose scalar product the optimized coefficients (a,b,c,d)
   * has the same effect as the original e coefficient in the spline equation
   * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
   *
   * @param spline_info
   * @param k the number of spline for which the e coefficient should be represented
   * @param dim X=0, Y=1
   * @param start_v the initial velocity of the first spline
   * @param[out] Ek the vector to represent e through a,b,c,d
   * @param[out] non_dependent the influence of e that are not dependent on a,b,c,d
   */
  void DescribeEByPrev(int k, int dim,
      double start_v, Eigen::VectorXd& Ek, double& non_dependent) const;

  /**
   * Creates a Vector whose scalar product the optimized coefficients (a,b,c,d)
   * has the same effect as the original f coefficient in the spline equation
   * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
   *
   * @param spline_info
   * @param k the number of spline for which the e coefficient should be represented
   * @param dim X=0, Y=1
   * @param start_p the initial position of the first spline
   * @param start_v the initial velocity of the first spline
   * @param[out] Ek the vector to represent e through a,b,c,d
   * @param[out] non_dependent the influence of f that are not dependent on a,b,c,d
   */
  void DescribeFByPrev(int k, int dim,
      double start_v, double start_p, Eigen::VectorXd& Ek, double& non_dependent) const;
  void AddOptimizedCoefficients(
      const Eigen::Vector2d& start_cog_p,
      const Eigen::Vector2d& start_cog_v,
      const Eigen::VectorXd& optimized_coeff);

  Eigen::VectorXd
  GetXyDimAlternatingVector(double x, double y) const;


  Splines splines_;
private:
  uint curr_spline_;
  static log4cxx::LoggerPtr log_;
};

} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
