/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <towr/variables/spline.h>


namespace towr {

using namespace xpp;


Spline::Spline(const NodeValues::Ptr& nodes, const VecTimes& phase_durations)
{
  node_values_ = nodes;
  poly_durations_ = ConvertPhaseToSplineDurations(phase_durations);
  assert(node_values_->GetPolynomialCount() == poly_durations_.size());

  for (int i=0; i<node_values_->GetPolynomialCount(); ++i) {
    auto p = std::make_shared<CubicHermitePoly>(node_values_->n_dim_);
    cubic_polys_.push_back(p);
  }

  UpdatePolynomials();

  durations_change_ = false;
  jac_structure_ = Jacobian(node_values_->n_dim_, node_values_->GetRows());
}

//Spline::Spline(const NodeValues::Ptr& nodes,
//               const ContactSchedule::Ptr& contact_schedule)
//    :Spline(nodes, contact_schedule->GetDurations())
//{
//  contact_schedule_ = contact_schedule;
//  durations_change_ = true;
//}




int
Spline::GetSegmentID(double t_global, const VecTimes& durations)
{
  double eps = 1e-10; // double precision
  assert(t_global >= 0.0);

   double t = 0;
   int i=0;
   for (double d: durations) {
     t += d;

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }

   assert(false); // this should never be reached
}

Spline::LocalInfo
Spline::GetLocalTime (double t_global, const VecTimes& durations)
{
  int id = GetSegmentID(t_global, durations);

  double t_local = t_global;
  for (int i=0; i<id; i++)
    t_local -= durations.at(i);

  return std::make_pair(id, t_local);
}

const StateLinXd
Spline::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);

  return cubic_polys_.at(id)->GetPoint(t_local);
}

void Spline::UpdatePhaseDurations(const VecTimes& phase_durations)
{
  durations_change_ = true;
//  phase_durations_ = phase_durations;
  poly_durations_  = ConvertPhaseToSplineDurations(phase_durations);
  UpdatePolynomials();
}

Spline::VecTimes
Spline::ConvertPhaseToSplineDurations(const VecTimes& phase_durations) const
{
  VecTimes spline_durations;

  for (auto info : node_values_->polynomial_info_)
    spline_durations.push_back(phase_durations.at(info.phase_)/info.num_polys_in_phase_);

  return spline_durations;
}

//
//void Spline::UpdateNodes(const VecNodes& nodes)
//{
//  nodes_ = nodes;
//  UpdatePolynomials();
//}

void
Spline::UpdatePolynomials ()
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    VecNodes nodes = node_values_->GetBoundaryNodes(i);
    cubic_polys_.at(i)->SetNodes(nodes.front(), nodes.back(), poly_durations_.at(i));
  }
}


VectorXd
Spline::GetDerivativeOfPosWrtPhaseDuration (double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);

  auto info = node_values_->polynomial_info_.at(id);

  double percent_of_phase = 1./info.num_polys_in_phase_;
  double inner_derivative = percent_of_phase;
  VectorXd vel = GetPoint(t_global).v_;
  VectorXd dxdT = cubic_polys_.at(id)->GetDerivativeOfPosWrtDuration(t_local);

  return inner_derivative*dxdT - info.poly_id_in_phase_*percent_of_phase*vel;
}


Spline::Jacobian
Spline::GetJacobian (double t_global, MotionDerivative dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);


  // if durations change, the polynomial active at a specified global time
  // changes. Therefore, all elements of the Jacobian could be non-zero
  if (fill_jacobian_structure_) {
    // assume every global time time can fall into every polynomial
    for (int i=0; i<node_values_->polynomial_info_.size(); ++i)
      FillJacobian(i, 0.0, dxdt, jac_structure_, true);
  }
  fill_jacobian_structure_ = false;


  Jacobian jac(node_values_->n_dim_, node_values_->GetRows());
  if (durations_change_)
    jac = jac_structure_;

  FillJacobian(id, t_local, dxdt, jac, false);

  // needed to avoid Eigen::assert failure "wrong storage order" triggered
  // in dynamic_constraint.cc
  jac.makeCompressed();

  return jac;
}

void
Spline::FillJacobian (int poly_id, double t_local, MotionDerivative dxdt,
                      Jacobian& jac, bool fill_with_zeros) const
{
  for (int idx=0; idx<jac.cols(); ++idx) {
    for (auto info : node_values_->GetNodeInfo(idx)) {
      for (Side side : {Side::Start, Side::End}) { // every jacobian is affected by two nodes

        int node = node_values_->GetNodeId(poly_id,side);

        if (node == info.id_) {
          double val = cubic_polys_.at(poly_id)->GetDerivativeOf(dxdt, side, info.deriv_, t_local);

          // if only want structure
          if (fill_with_zeros)
            val = 0.0;

          jac.coeffRef(info.dim_, idx) += val;
        }
      }
    }
  }
}


//bool
//Spline::IsConstantPhase (double t_global) const
//{
//  int phase_id = GetSegmentID(t_global, phase_durations_);
//
//  // always alternating
//  bool first_phase_in_contact = nodes_->polynomial_info_.front().is_constant_;
//  if (phase_id%2==0)
//   return first_phase_in_contact;
//  else
//   return !first_phase_in_contact;
//}


} /* namespace towr */

