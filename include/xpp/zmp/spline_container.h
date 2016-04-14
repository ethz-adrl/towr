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

#include <xpp/zmp/zmp_spline.h>
#include <xpp/utils/geometric_structs.h>
#include <xpp/hyq/leg_data_map.h>

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
typedef xpp::utils::Point2d Point2d;
typedef xpp::hyq::LegID LegID;
static const int kDim2d = xpp::utils::kDim2d;

public:
  SplineContainer();
  SplineContainer(const Splines& splines);
  virtual ~SplineContainer() {};

  /**
  @brief Calculates the state of a spline at a specific point in time.

  @param double specific time of spline
  @param Derivative which value (pos,vel,acc) at this time we are interested in
  @return x and y state of position,velocity OR acceleration
  */
  void GetCOGxy(double t_global, Point2d& cog_xy, const Splines& splines) const;
  void GetCOGxy(double t_global, Point2d& cog_xy) const
  {
    GetCOGxy(t_global, cog_xy, splines_);
  }
  int GetSplineID(double t_global) const;
  int GetFourLegSupport(double t_global) const;
  int GetStep(double t_global) const;
  void AddSpline(const ZmpSpline &spline);
  double GetTotalTime(const Splines& splines, bool exclude_4ls_splines) const;
  double GetTotalTime(bool exclude_4ls_splines = false) const
  {
    return GetTotalTime(splines_, exclude_4ls_splines);
  }



  // Creates a sequence of Splines without the optimized coefficients
  void ConstructSplineSequence(const std::vector<LegID>& step_sequence,
                                        double t_stance,
                                        double t_swing,
                                        double t_stance_initial,
                                        double t_stance_final);

  Splines splines_;
};

} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
