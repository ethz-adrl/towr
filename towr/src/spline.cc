/**
 @file    com_spline.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <towr/variables/spline.h>

#include <towr/variables/contact_schedule.h>
#include "../include/towr/variables/node_variables.h"


namespace towr {

using namespace xpp;


Spline::Spline(NodesObserver::SubjectPtr const nodes, const VecTimes& phase_durations)
   : NodesObserver(nodes)
{
  Init(phase_durations);
  jac_wrt_nodes_structure_ = Jacobian(node_values_->GetDim(), node_values_->GetRows());
}

Spline::Spline(NodesObserver::SubjectPtr const nodes, ContactSchedule* const contact_schedule)
    :NodesObserver(nodes),
     ContactScheduleObserver(contact_schedule)

{
  Init(contact_schedule->GetDurations());

  // if durations change, the polynomial active at a specified global time
  // changes. Therefore, all elements of the Jacobian could be non-zero
  // and must make sure that Jacobian structure never changes during
  // the iterations.
  // assume every global time time can fall into every polynomial
  jac_wrt_nodes_structure_ = Jacobian(node_values_->GetDim(), node_values_->GetRows());
  for (int i=0; i<node_values_->GetPolynomialCount(); ++i)
    FillJacobian(i, 0.0, kPos, jac_wrt_nodes_structure_, true);
}

void Spline::Init(const VecTimes& durations)
{
  SetPolyFromPhaseDurations(durations);
  uint n_polys = node_values_->GetPolynomialCount();
  assert(n_polys == poly_durations_.size());

  cubic_polys_.assign(n_polys, CubicHermitePoly(node_values_->GetDim()));

  UpdatePolynomials();
}

int
Spline::GetSegmentID(double t_global, const VecTimes& durations) const
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
Spline::GetLocalTime (double t_global, const VecTimes& durations) const
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

  return cubic_polys_.at(id).GetPoint(t_local);
}

void
Spline::UpdatePolynomials ()
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    VecNodes nodes = node_values_->GetBoundaryNodes(i);
    cubic_polys_.at(i).SetNodes(nodes.front(), nodes.back(), poly_durations_.at(i));
  }
}

void Spline::UpdatePhaseDurations()
{
  SetPolyFromPhaseDurations(contact_schedule_->GetDurations());
  UpdatePolynomials();
}

void
Spline::SetPolyFromPhaseDurations(const VecTimes& phase_durations)
{
  poly_durations_ = node_values_->ConvertPhaseToPolyDurations(phase_durations);
}


Spline::Jacobian
Spline::GetJacobianWrtNodes (double t_global, MotionDerivative dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, poly_durations_);

  Jacobian jac = jac_wrt_nodes_structure_;
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
    for (auto info : node_values_->GetNodeInfoAtOptIndex(idx)) {
      for (Side side : {Side::Start, Side::End}) { // every jacobian is affected by two nodes

        int node = node_values_->GetNodeId(poly_id,side);

        if (node == info.node_id_) {
          double val = cubic_polys_.at(poly_id).GetDerivativeOf(dxdt, side, info.node_deriv_, t_local);

          // if only want structure
          if (fill_with_zeros)
            val = 0.0;

          jac.coeffRef(info.node_deriv_dim_, idx) += val;
        }
      }
    }
  }
}

Spline::Jacobian
Spline::GetJacobianOfPosWrtDurations (double t_global) const
{
  VectorXd dx_dT  = GetDerivativeOfPosWrtPhaseDuration(t_global);
  VectorXd xd     = GetPoint(t_global).v_;
  int current_phase = GetSegmentID(t_global, contact_schedule_->GetDurations());

  return contact_schedule_->GetJacobianOfPos(current_phase, dx_dT, xd);
}

VectorXd
Spline::GetDerivativeOfPosWrtPhaseDuration (double t_global) const
{
  int poly_id; double t_local;
  std::tie(poly_id, t_local) = GetLocalTime(t_global, poly_durations_);

  VectorXd vel  = GetPoint(t_global).v_;
  VectorXd dxdT = cubic_polys_.at(poly_id).GetDerivativeOfPosWrtDuration(t_local);

  double inner_derivative = node_values_->GetDerivativeOfPolyDurationWrtPhaseDuration(poly_id);
  double prev_polys_in_phase = node_values_->GetNumberOfPrevPolynomialsInPhase(poly_id);

  // where does this minus stuff come from?
  // from number of polynomials before current polynomial that
  // cause shifting of entire spline
  return inner_derivative*(dxdT - prev_polys_in_phase*vel);
}


bool
Spline::IsConstantPhase (double t_global) const
{
  int poly_id = GetSegmentID(t_global, poly_durations_);
  return node_values_->IsInConstantPhase(poly_id);
}


} /* namespace towr */

