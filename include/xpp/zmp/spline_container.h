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
typedef std::vector<ZmpSpline> VecSpline;
typedef xpp::utils::Point2d Point2d;
typedef xpp::hyq::LegID LegID;
static const int kDim2d = xpp::utils::kDim2d;

public:
  SplineContainer();
  SplineContainer(const VecSpline& splines);
  virtual ~SplineContainer() {};

  VecSpline GetSplines()        const { return splines_; }
  ZmpSpline GetSpline(size_t i) const { return splines_.at(i); }
  ZmpSpline GetFirstSpline()    const { return splines_.front(); };
  ZmpSpline GetLastSpline()     const { return splines_.back(); };
  int GetSplineCount()          const { return splines_.size(); };

  /**
  @brief Calculates the state of a spline at a specific point in time.

  @param double specific time of spline
  @param Derivative which value (pos,vel,acc) at this time we are interested in
  @return x and y state of position,velocity OR acceleration
  */
  static void GetCOGxy(double t_global, Point2d& cog_xy, const VecSpline& splines);
  void GetCOGxy(double t_global, Point2d& cog_xy) const
  {
    GetCOGxy(t_global, cog_xy, splines_);
  }
  static int GetSplineID(double t_global, const VecSpline& splines);
  int GetSplineID(double t_global) const
  {
    return GetSplineID(t_global, splines_);
  }

  int GetFourLegSupport(double t_global) const;
  int GetStep(double t_global) const;
  static double GetTotalTime(const VecSpline& splines, bool exclude_4ls_splines = false);
  double GetTotalTime(bool exclude_4ls_splines = false) const
  {
    return GetTotalTime(splines_, exclude_4ls_splines);
  }


  // Creates a sequence of Splines without the optimized coefficients
  static VecSpline ConstructSplineSequence(const std::vector<LegID>& step_sequence,
                                           double t_stance,
                                           double t_swing,
                                           double t_stance_initial,
                                           double t_stance_final);

protected:
  VecSpline splines_;
  bool splines_initialized_ = false;
  void CheckIfSplinesInitialized() const;
};

} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
