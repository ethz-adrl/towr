/**
 @file    motion_optimizer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Nov 20, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_optimizer_facade.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/opt/com_spline.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace opt {

using MotionStructure = xpp::opt::MotionStructure;
using Contacts        = xpp::hyq::SupportPolygonContainer;

MotionOptimizerFacade::MotionOptimizerFacade ()
{
  // TODO Auto-generated constructor stub
}

MotionOptimizerFacade::~MotionOptimizerFacade ()
{
  // TODO Auto-generated destructor stub
}

void
MotionOptimizerFacade::OptimizeMotion ()
{
  // create the fixed motion structure
  step_sequence_planner_.Init(curr_state_.base_.lin.Get2D(), goal_cog_.Get2D(),
                              curr_state_.GetStanceLegsInWorld(),
                              des_robot_height_, max_step_length_,
                              curr_state_.SwinglegID(), supp_polygon_margins_);

  auto step_sequence        = step_sequence_planner_.DetermineStepSequence();
  bool start_with_com_shift = step_sequence_planner_.StartWithStancePhase();

  MotionStructure motion_structure;
  motion_structure.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, t_swing_, t_stance_initial_,
                        start_with_com_shift, true);

  Contacts contacts;
  contacts.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, supp_polygon_margins_);

  nlp_facade_.SolveNlp(curr_state_.base_.lin.Get2D(),
                       goal_cog_.Get2D(),
                       des_robot_height_,
                       motion_structure,
                       contacts,
                       dt_zmp_);

  auto& com_spline = dynamic_cast<xpp::opt::ComSpline&>(*nlp_facade_.GetMotion());

  whole_body_mapper_.Init(nlp_facade_.GetPhases(),
                          com_spline.GetPolynomials(),
                          nlp_facade_.GetFootholds(),
                          des_robot_height_,
                          curr_state_);
}

MotionOptimizerFacade::HyqStateVec
MotionOptimizerFacade::GetTrajectory () const
{
  return whole_body_mapper_.BuildWholeBodyTrajectoryJoints();
}

} /* namespace opt */
} /* namespace xpp */
