/**
 @file    motion_optimizer_facade.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   The facade for the motion optimization, ROS independent
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include <ifopt/problem.h>

#include <xpp_states/endeffectors.h>
#include <xpp_states/robot_state_cartesian.h>
#include <xpp_states/state.h>

#include <towr/constraints/height_map.h>
#include <towr/models/robot_model.h>
#include <towr/optimization_parameters.h>


namespace xpp {


/** Simplified interface to the complete motion optimization framework.
  */
class TOWR {
public:
  using VariablesCompPtr = opt::Composite::Ptr;
  using RobotStateVec    = std::vector<RobotStateCartesian>;

  TOWR () = default;
  virtual ~TOWR () = default;

  void SetInitialState(const RobotStateCartesian&);
  void SetParameters(const State3dEuler& final_base,
                     double total_time,
                     const RobotModel& model,
                     HeightMap::Ptr terrain);

  void SolveNLP();

  RobotStateVec GetTrajectory(double dt) const;
  std::vector<RobotStateVec> GetIntermediateSolutions(double dt) const;


private:
  State3dEuler inital_base_;
  EndeffectorsPos initial_ee_W_;

  RobotModel model_;
  HeightMap::Ptr terrain_;
  OptimizationParameters params_;
  State3dEuler final_base_;

  opt::Problem BuildNLP() const;
  mutable opt::Problem nlp_;

  RobotStateVec GetTrajectory(const VariablesCompPtr&, double dt) const;
  void SetTerrainHeightFromAvgFootholdHeight(HeightMap::Ptr& terrain) const;


  Vector3d GetUnique(const Vector3d& zyx_non_unique) const;
};

} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_OPTIMIZER_FACADE_H_ */
