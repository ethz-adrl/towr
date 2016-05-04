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
  typedef std::vector<xpp::hyq::LegID> VecLegID;
  typedef xpp::hyq::LegID LegID;
  static const int kDim2d = xpp::utils::kDim2d;

public:
  SplineContainer() {};
  SplineContainer(const VecSpline& splines);
  SplineContainer(const std::vector<xpp::hyq::LegID>& step_sequence,
                  double t_stance,
                  double t_swing,
                  double t_stance_initial,
                  double t_stance_final);
  virtual ~SplineContainer() {};

public:
  static VecSpline ConstructSplineSequence(const VecLegID& step_sequence,
                                           double t_stance,
                                           double t_swing,
                                           double t_stance_initial,
                                           double t_stance_final);

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
  static Point2d GetCOGxy(double t_global, const VecSpline& splines);
  Point2d GetCOGxy(double t_global) const { return GetCOGxy(t_global, splines_); }

  static int GetSplineID(double t_global, const VecSpline& splines);
  int GetSplineID(double t_global) const { return GetSplineID(t_global, splines_); }

  /** Returns the time that the spline active at t_global has been running */
  static double GetLocalTime(double t_global, const VecSpline& splines);
  double GetLocalTime(double t_global) const { return GetLocalTime(t_global, splines_); };

  bool IsFourLegSupport(double t_global) const;

  static double GetTotalTime(const VecSpline& splines);
  double GetTotalTime() const { return GetTotalTime(splines_); }

  static constexpr double dt_ = 0.1; // This is needed for creating support triangle inequality constraints
  int GetTotalNodes() const { return 1 + std::floor(GetTotalTime()/dt_); };
  int GetNodeCount(size_t i) const { return GetSpline(i).GetNodeCount(dt_); };

protected:
  VecSpline splines_;
  void CheckIfSplinesInitialized() const;
  void Init(const std::vector<xpp::hyq::LegID>& step_sequence,
            double t_stance,
            double t_swing,
            double t_stance_initial,
            double t_stance_final);

private:
  // Creates a sequence of Splines without the optimized coefficients
  bool splines_initialized_ = false;
  static constexpr double eps_ = 1e-10; // maximum inaccuracy when adding double numbers

};

} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
