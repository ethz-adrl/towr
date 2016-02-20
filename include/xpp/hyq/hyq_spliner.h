/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#ifndef _XPP_HYQ_SPLINER_H_
#define _XPP_HYQ_SPLINER_H_

#include "hyq_state.h"
#include <xpp/utils/orientation.h>
#include <xpp/utils/spliner_3d.h>

#include <log4cxx/logger.h>
#include <Eigen/Dense>

namespace xpp {
namespace hyq {

/**
@brief Splines the base pose (position + orientation).

No velocity and accelerations of orientation, only roll, pitch, yaw.
For that transfer roll, pitch yaw velocities and accelerations into fixed global
frame values omega (rollPitchYawToEar)
 */
class HyqSpliner {
public:

public:
  HyqSpliner();
  virtual ~HyqSpliner() {};

  /**
  @brief transforms a HyqState into a collection of Points, including
         body position, body orientation, and feet position
  @param[in] time_to_reach how long the robot has to achieve this state
   */
  void AddNode(const HyqState& state, double t_max);


  void SetCurrGoal(uint des_goal);

  /**
  @brief function to access the current state AFTER states have been set.
  @param[in]  t_global global time.
  @return     curr position, orientation and current foothold of hyq.
   */
  HyqState getPoint(const double t_global);

  void SetParams(double upswing,
                 double lift_height,
                 double outward_swing_distance);

  void ClearNodes() { nodes_.clear(); }


private:
  typedef ::xpp::utils::QuinticSpliner Spliner;
  typedef ::xpp::utils::Spliner3d< Spliner > Spliner3d;
  typedef Spliner3d::Point Point;

  struct SplineNode {
    Point pos;
    Point ori;
    LegDataMap<Point> feet;
    int swingleg;             // leg to reach this state
    double T;                 // time to reach this state
    SplineNode(const Point& _pos, const Point& _ori,
               const LegDataMap<Point>& _feet, int _swingleg, double _T)
        : pos(_pos), ori(_ori), feet(_feet), swingleg(_swingleg), T(_T) {};
  };

  std::vector<SplineNode> nodes_; // the discrete states to spline through
  uint curr_goal_;                 // current goal node

  double kUpswingPercent;       // how long to swing up during swing
  double kLiftHeight;           // how high to lift the leg
  double kOutwardSwingDistance; // how far to swing leg outward (y-dir)

  Spliner3d pos_spliner_, ori_spliner_;
  LegDataMap< Spliner3d > feet_spliner_up_, feet_spliner_down_;

  log4cxx::LoggerPtr log_;

};

} // namespace hyq
} // namespace xpp
#endif // _XPP_HYQ_SPLINER_H_
