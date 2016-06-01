/*
 * nlp_optimizer.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/nlp_optimizer.h>
#include <xpp/zmp/nlp_ipopt_zmp.h>

// this looks like i need the factor method
#include <xpp/zmp/continuous_spline_container.h>
#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/initial_acceleration_equation.h>
#include <xpp/zmp/final_state_equation.h>
#include <xpp/zmp/spline_junction_equation.h>
#include <xpp/zmp/zmp_constraint.h>
#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/zmp/constraint_container.h>
// cost function stuff
#include <xpp/zmp/a_quadratic_cost.h>
#include <xpp/zmp/range_of_motion_cost.h>
#include <xpp/zmp/total_acceleration_equation.h>
#include <xpp/zmp/cost_container.h>
//
#include <xpp/ros/optimization_visualizer.h>

namespace xpp {
namespace zmp {


NlpOptimizer::NlpOptimizer ()
    :subject_(0,0), // initialized with zero values
     visualizer_(xpp::ros::dummy_visualizer)
{
  app_.RethrowNonIpoptException(true); // this allows to see the error message of exceptions thrown inside ipopt
  status_ = app_.Initialize();
  if (status_ != Ipopt::Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }



  // fixme add the constraint registration with subject here

}


void
NlpOptimizer::SolveNlp(const State& initial_state,
                       const State& final_state,
                       const std::vector<xpp::hyq::LegID>& step_sequence,
                       const VecFoothold& start_stance,
                       const SplineTimes& times,
                       double robot_height)
{
  typedef xpp::hyq::SupportPolygon SupportPolygon;

  // create the general spline structure
  spline_structure_.Init(initial_state.p,
                        initial_state.v ,
                        step_sequence,
                        times);

  step_sequence_ = step_sequence;


  xpp::hyq::SupportPolygonContainer supp_polygon_container;
  supp_polygon_container.Init(start_stance,
                              step_sequence_,
                              SupportPolygon::GetDefaultMargins());

  NlpStructure nlp_structure(spline_structure_.GetTotalFreeCoeff(),
                             supp_polygon_container.GetNumberOfSteps());




  // fixme this should also change if the goal or any other parameter changes apart from the start position
//  bool init_with_zeros = true;
//  bool num_steps_changed = initial_variables_.footholds_.size() != nlp_structure.n_steps_;
//  if (num_steps_changed || init_with_zeros) {
//    SetInitialVariables(nlp_structure, supp_polygon_container);
//  } else {} // use previous values


//  Constraints constraints(supp_polygon_container, spline_structure_, nlp_structure, robot_height, initial_state.a, final_state);


  // This should all be hidden inside a factory method
  // the linear equations
  InitialAccelerationEquation eq_acc(initial_state.a, spline_structure_.GetTotalFreeCoeff());
  FinalStateEquation eq_final(final_state, spline_structure_);
  SplineJunctionEquation eq_junction(spline_structure_);


  subject_.Init(spline_structure_.GetTotalFreeCoeff(), supp_polygon_container.GetNumberOfSteps());
  subject_.SetFootholds(supp_polygon_container.GetFootholdsInitializedToStart());

  // add the constraints
  LinearEqualityConstraint c_acc(subject_);
  c_acc.Init(eq_acc.BuildLinearEquation());

  LinearEqualityConstraint c_final(subject_);
  c_final.Init(eq_final.BuildLinearEquation());

  LinearEqualityConstraint c_junction(subject_);
  c_junction.Init(eq_junction.BuildLinearEquation());

  ZmpConstraint c_zmp(subject_);
  c_zmp.Init(spline_structure_, supp_polygon_container, robot_height);

  RangeOfMotionConstraint c_rom(subject_);
  c_rom.Init(spline_structure_, supp_polygon_container);


  ConstraintContainer constraint_container;
  constraint_container.AddConstraint(c_acc);
  constraint_container.AddConstraint(c_final);
  constraint_container.AddConstraint(c_junction);
  constraint_container.AddConstraint(c_zmp);
  constraint_container.AddConstraint(c_rom);


  // costs
  TotalAccelerationEquation eq_total_acc(spline_structure_);

  AQuadraticCost cost_acc(subject_);
  cost_acc.Init(eq_total_acc.BuildLinearEquation());

  RangeOfMotionCost cost_rom(subject_);
  cost_rom.Init(spline_structure_, supp_polygon_container);

  CostContainer cost_container(subject_);
  cost_container.AddCost(cost_acc);
  cost_container.AddCost(cost_rom);


  // fixme make this a class member so ros registration only has to happen once
  xpp::ros::OptimizationVisualizer optimization_visualizer(subject_);
  optimization_visualizer.Init(step_sequence_, spline_structure_);

  IpoptPtr nlp_ptr = new Ipopt::NlpIpoptZmp(subject_, // optmization variables
                                            cost_container,
                                            constraint_container,
                                            optimization_visualizer);

  SolveIpopt(nlp);
  subject_.RemoveObservers();
}

void
NlpOptimizer::SolveIpopt (const IpoptPtr& nlp)
{
  status_ = app_.OptimizeTNLP(nlp);
  if (status_ == Ipopt::Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app_.Statistics()->IterationCount();
    std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

    Ipopt::Number final_obj = app_.Statistics()->FinalObjective();
    std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
  }
}

NlpOptimizer::VecFoothold
NlpOptimizer::GetFootholds () const
{
  OptimizationVariables::StdVecEigen2d footholds_xy = subject_.GetFootholdsStd();

  VecFoothold opt_footholds(footholds_xy.size());
  xpp::hyq::Foothold::SetXy(footholds_xy, opt_footholds);

  uint i=0;
  for (hyq::Foothold& f : opt_footholds)
    f.leg = step_sequence_.at(i++);

  return opt_footholds;
}

NlpOptimizer::VecSpline
NlpOptimizer::GetSplines ()
{
  spline_structure_.AddOptimizedCoefficients(subject_.GetSplineCoefficients());
  return spline_structure_.GetSplines();
}

} /* namespace zmp */
} /* namespace xpp */
