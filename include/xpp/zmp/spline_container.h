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

Should have no explicit foothold locations or discretization time. Only the framework
of splines, where the actual coefficients must be filled through an optimization.
*/
class SplineContainer {
public:
typedef std::vector<ZmpSpline> Splines;
typedef xpp::utils::Point2d Lin2d;
typedef xpp::hyq::LegID LegID;

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
  double GetTotalTime(bool exclude_4ls_splines = false) const;
  int GetTotalNodes(double dt, bool exclude_4ls_splines) const;
  int GetSplineID(int node, double dt) const;


  // Creates a sequence of Splines without the optimized coefficients
  void ConstructSplineSequence(const std::vector<LegID>& step_sequence,
                                        double t_stance,
                                        double t_swing,
                                        double t_stance_initial,
                                        double t_stance_final);

  static bool Insert4LSPhase(LegID prev, LegID next);



  Splines splines_;
protected:
  uint curr_spline_;
  static log4cxx::LoggerPtr log_;
};

} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
