/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/optimization_variables.h>

namespace xpp {
namespace opt {

using MotionStructure = xpp::opt::MotionStructure;

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  // TODO Auto-generated constructor stub
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::Init (double max_step_length, double dt_nodes,
                             double t_swing, double t_first_phase,
                             double des_walking_height,
                             double lift_height,
                             double outward_swing,
                             double trajectory_dt,
                             VisualizerPtr visualizer)
{
  max_step_length_ = max_step_length;
  dt_nodes_ = dt_nodes;
  t_swing_ = t_swing;
  t_first_phase_ = t_first_phase;
  des_walking_height_ = des_walking_height;

  whole_body_mapper_.SetParams(0.5, lift_height, outward_swing, trajectory_dt);

  nlp_facade_.AttachNlpObserver(visualizer);
}

void
MotionOptimizerFacade::OptimizeMotion ()
{
  // create the fixed motion structure
  step_sequence_planner_.Init(curr_state_.base_.lin.Get2D(), goal_cog_.Get2D(),
                              curr_state_.GetStanceLegsInWorld(),
                              des_walking_height_, max_step_length_,
                              curr_state_.SwinglegID());

  MotionStructure motion_structure;

  auto step_sequence = step_sequence_planner_.DetermineStepSequence();
  bool start_with_com_shift = true;//step_sequence_planner_.StartWithStancePhase();
  bool insert_final_stance = true;

  double t_first_phase = t_first_phase_;//t_left_; // mpc this changes by goal publisher

  // zmp_ see if i can eliminate these somehow
  auto stance = curr_state_.GetStanceLegsInWorld();
  std::vector<ContactDerived> contacts_initial;
  for (auto f : stance) {
    ContactDerived c;
    c.ee = static_cast<EndeffectorID>(f.leg);
    c.id = f.id;
    c.p = f.p;

    contacts_initial.push_back(c);
  }


  motion_structure.Init(contacts_initial, step_sequence, t_swing_, t_first_phase,
                        start_with_com_shift, insert_final_stance, dt_nodes_ );

  nlp_facade_.SolveNlp(curr_state_.base_.lin.Get2D(),
                       goal_cog_.Get2D(),
                       des_walking_height_,
                       motion_structure);

  whole_body_mapper_.Init(motion_structure.GetPhases(),
                          nlp_facade_.GetComMotion(),
                          nlp_facade_.GetFootholds(),
                          des_walking_height_,
                          curr_state_);

  optimized_trajectory_ = whole_body_mapper_.BuildWholeBodyTrajectoryJoints();
}

MotionOptimizerFacade::HyqStateVec
MotionOptimizerFacade::GetTrajectory () const
{
  return optimized_trajectory_;
}

void
MotionOptimizerFacade::SetCurrent (const HyqState& curr)
{
  curr_state_ = curr;
}

} /* namespace opt */
} /* namespace xpp */


