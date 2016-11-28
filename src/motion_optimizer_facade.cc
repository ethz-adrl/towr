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
MotionOptimizerFacade::Init (double max_step_length, double dt_zmp,
                             double diag_supp_poly_margin,
                             double t_swing, double t_stance_initial,
                             double des_walking_height,
                             double lift_height,
                             double outward_swing,
                             double trajectory_dt,
                             VisualizerPtr visualizer)
{
  max_step_length_ = max_step_length;
  dt_zmp_ = dt_zmp;
  supp_polygon_margins_ = xpp::hyq::SupportPolygon::GetDefaultMargins();
  supp_polygon_margins_[hyq::DIAG] =  diag_supp_poly_margin;
  t_swing_ = t_swing;
  t_stance_initial_ = t_stance_initial;
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
                              curr_state_.SwinglegID(), supp_polygon_margins_);

//  auto step_sequence        = step_sequence_planner_.DetermineStepSequence();
//  bool start_with_com_shift = step_sequence_planner_.StartWithStancePhase();
  std::vector<xpp::hyq::LegID> step_sequence = {hyq::LH};
  bool start_with_com_shift = true;
  bool insert_final_stance = true;

  MotionStructure motion_structure;
  double t_stance_initial = t_left_; // mpc for only body shift, this changes by goal publisher

  motion_structure.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, t_swing_, t_stance_initial,
                        start_with_com_shift, insert_final_stance);

  Contacts contacts;
  contacts.Init(curr_state_.GetStanceLegsInWorld(), step_sequence, supp_polygon_margins_);

  nlp_facade_.SolveNlp(curr_state_.base_.lin.Get2D(),
                       goal_cog_.Get2D(),
                       des_walking_height_,
                       motion_structure,
                       contacts,
                       dt_zmp_);

  auto& com_spline = dynamic_cast<xpp::opt::ComSpline&>(*nlp_facade_.GetMotion());

  whole_body_mapper_.Init(nlp_facade_.GetPhases(),
                          com_spline.GetPolynomials(),
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


