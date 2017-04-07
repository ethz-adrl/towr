/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/optimization_variables.h>
#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/endeffectors_motion.h>
#include <xpp/opt/contact_schedule.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/base_motion.h>


namespace xpp {
namespace opt {

MotionOptimizerFacade::MotionOptimizerFacade ()
{
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::BuildDefaultStartStance (const MotionParameters& params)
{
  State3d base;
  base.lin.p.z() = params.geom_walking_height_;
  EndeffectorsBool contact_state(params.robot_ee_.size());
  contact_state.SetAll(true);

  start_geom_.SetBase(base);
  start_geom_.SetContactState(contact_state);
  start_geom_.SetEEState(kPos, params.GetNominalStanceInBase());
}

void
MotionOptimizerFacade::OptimizeMotion (NlpSolver solver)
{
  auto goal_com = goal_geom_;
  goal_com.p += motion_parameters_->offset_geom_to_com_;


  // initialize the contact schedule
  contact_schedule_ = std::make_shared<ContactSchedule>(motion_parameters_->GetOneCycle());

  // initialize the ee_motion with the fixed parameters
  ee_motion_ = std::make_shared<EndeffectorsMotion>(start_geom_.GetEEPos(),
                                                    *contact_schedule_);

  double com_height = motion_parameters_->geom_walking_height_
                    + motion_parameters_->offset_geom_to_com_.z();
  com_motion_ = MotionFactory::CreateComMotion(ee_motion_->GetTotalTime(),
                                               motion_parameters_->polynomials_per_second_,
                                               com_height);
  com_motion_->SetOffsetGeomToCom(motion_parameters_->offset_geom_to_com_);


  nlp_facade_.OptimizeMotion(start_geom_,
                             goal_com.Get2D(),
                             ee_motion_,
                             com_motion_,
                             contact_schedule_,
                             motion_parameters_,
                             solver);
}

MotionOptimizerFacade::RobotStateVec
MotionOptimizerFacade::GetTrajectory (double dt)
{
  RobotStateVec trajectory;

  double t=0.0;
  double T = ee_motion_->GetTotalTime();
  while (t<T) {

    RobotStateCartesian state(start_geom_.GetEECount());
    state.SetBase(com_motion_->GetBase(t));
    state.SetEEState(ee_motion_->GetEndeffectors(t));
    state.SetContactState(contact_schedule_->IsInContact(t));
    state.SetTime(t);

    trajectory.push_back(state);
    t += dt;
  }

  return trajectory;
}

void
MotionOptimizerFacade::SetMotionParameters (const MotionParametersPtr& params)
{
  motion_parameters_ = params;
}

} /* namespace opt */
} /* namespace xpp */


