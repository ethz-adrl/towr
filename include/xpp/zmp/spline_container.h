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


/** The duration of each polynom in the sequence that describes some trajectory */
struct SplineTimes
{
  SplineTimes() {};
  /**
  * @param t_stance time to transition between disjoint support polygons
  * @param t_swing time for one step
  * @param t_init time before executing the first step
  * @param t_final time after executing the last step to still move the CoG
  */
  SplineTimes(double t_stance, double t_swing, double t_init, double t_final)
      :t_stance_(t_stance), t_swing_(t_swing), t_stance_initial_(t_init), t_stance_final_(t_final) {}

  double t_stance_;
  double t_swing_;
  double t_stance_initial_;
  double t_stance_final_;

  void SetDefault() {
    t_stance_         = 0.2; //s
    t_swing_          = 0.7; //s
    t_stance_initial_ = 0.4; //s
    t_stance_final_   = 0.2; //s
  }
};


/** @brief holds multiple splines and knows when these are active in the sequence.
  *
  * Should have no explicit foothold locations or discretization time. Only the framework
  * of splines, where the actual coefficients must be filled through an optimization.
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
  SplineContainer(const std::vector<xpp::hyq::LegID>& step_sequence, const SplineTimes&);
  virtual ~SplineContainer() {};

public:
  static VecSpline ConstructSplineSequence(const VecLegID& step_sequence,
                                           const SplineTimes&);

  //////////////////////////////////////////////////////////
  // implement these
  static VecSpline ConstructSplineStepSequence(const VecLegID& step_sequence,
                                               const SplineTimes&);
  // these should be private
  static ZmpSpline ConstructStanceSpline(double t_stance);
  static ZmpSpline ConstructStepSpline(double t_step);
  //////////////////////////////////////////////////////////


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

  /** If the trajectory has to be discretized, use this for consistent time steps.
   *  t(0)------t(1)------t(2)------...------t(N-1)---|------t(N)
   *
   *  so first and last time are t0 and and tN, but there might be a
   *  timestep > delta t before the last node.
   */
  static std::vector<double> GetDiscretizedGlobalTimes(const VecSpline& splines);
  std::vector<double> GetDiscretizedGlobalTimes() const {
    return GetDiscretizedGlobalTimes(splines_);
  }

  int GetTotalNodes() const { return GetDiscretizedGlobalTimes().size(); };

protected:
  VecSpline splines_;
  void CheckIfSplinesInitialized() const;
  void Init(const std::vector<xpp::hyq::LegID>& step_sequence, const SplineTimes&);

private:
  bool splines_initialized_ = false;
  static constexpr double eps_ = 1e-10; // maximum inaccuracy when adding double numbers

};




} // namespace zmp
} // namespace xpp

#endif // _XPP_ZMP_SPLINECONTAINER_H_
