/**
 @file    com_spline.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Declares the class ComSpline
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_

#include "com_motion.h"
#include "zmp_spline.h"

#include <memory>

namespace xpp {
namespace zmp {


/** The duration of each polynome in the sequence that describes some trajectory */
struct SplineTimes
{
  SplineTimes() {};

  /** @param t_swing time for one step
    * @param t_init time before executing the first step
    */
  SplineTimes(double t_swing, double t_init)
      :t_swing_(t_swing), t_stance_initial_(t_init) {}

  double t_swing_;
  double t_stance_initial_;

  void SetDefault() {
    t_swing_          = 0.7; //s
    t_stance_initial_ = 0.4; //s
  }
};



/** Represents the Center of Mass (CoM) motion as a Spline (sequence of polynomials).
  *
  * This class is responsible for abstracting polynomial coefficients of multiple
  * polynomes into a CoM position/velocity and acceleration.
  */
class ComSpline : public ComMotion {
public:
  typedef std::vector<ZmpSpline> VecSpline;
  typedef xpp::utils::Point2d Point2d;
  typedef std::shared_ptr<ComSpline> Ptr;
  static const int kDim2d = xpp::utils::kDim2d;

  ComSpline ();
  virtual ~ComSpline ();

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
  double GetLocalTime(double t_global) const { return GetLocalTime(t_global, splines_); };

  static double GetTotalTime(const VecSpline& splines);
  double GetTotalTime() const { return GetTotalTime(splines_); }

  /** If the trajectory has to be discretized, use this for consistent time steps.
   *  t(0)------t(1)------t(2)------...------t(N-1)---|------t(N)
   *
   *  so first and last time are t0 and and tN, but there might be a
   *  timestep > delta t before the last node.
   */
  // fixme this should maybe go somewhere else (constraints/costs).
  static std::vector<double> GetDiscretizedGlobalTimes(const VecSpline& splines);
  std::vector<double> GetDiscretizedGlobalTimes() const {
    return GetDiscretizedGlobalTimes(splines_);
  }

  int GetTotalNodes() const { return GetDiscretizedGlobalTimes().size(); };

  /** Builds all the 5th order polynomials while not modifying it's internals.
    *
    * @param optimized_coeff the varying parameters
    * @return a copy of the polynomials that these parameters produce.
    */
  virtual VecSpline BuildOptimizedSplines(const VectorXd& optimized_coeff) const = 0;


protected:
  VecSpline splines_;
  void CheckIfSplinesInitialized() const;
  void AddSplinesStepSequence(int step_count, double t_swing);
  void AddStanceSpline(double t_stance);

private:
  static double GetLocalTime(double t_global, const VecSpline& splines);
  bool splines_initialized_ = false;
  static constexpr double eps_ = 1e-10; // maximum inaccuracy when adding double numbers
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COM_SPLINE_H_ */
