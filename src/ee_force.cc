/**
 @file    ee_force.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 15, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/ee_force.h>

namespace xpp {
namespace opt {

EEForce::EEForce () : Component(-1, "ee_force_single")
{
}

EEForce::~EEForce ()
{
}

void
EEForce::AddPhase (double T, double dt, bool is_contact)
{
  if (is_contact)
    AddContactPhase(T, dt);
  else
    AddSwingPhase(T);

  spline_.SetSegmentsPtr(polynomials_); // update spline here
  int num_nodes = polynomials_.size()+1;
  SetRows(num_nodes*dim_.size());

  VectorXd initial_forces(GetRows());
  for (int node=0; node<num_nodes; ++node) {
    initial_forces(kDim3d*node + X) = 0.0;
    initial_forces(kDim3d*node + Y) = 0.0;
    initial_forces(kDim3d*node + Z) = 200; // 1/4 of hyq weight
  }

  // initial forces should be different from zero
  SetValues(initial_forces);
}

VectorXd
EEForce::GetValues () const
{
  VectorXd x(GetRows());

  int idx=0;
  for (const auto& p : polynomials_)
    for (auto d : dim_)
      x(idx++)   = p->start_.p_(d); // goal value is same as next start value

  // add last node
  for (auto d : dim_)
    x(idx++)   = polynomials_.back()->end_.p_(d);

  return x;
}

void
EEForce::SetValues (const VectorXd& x)
{
  int idx=0;
  for (auto& p : polynomials_) {
    for (auto d : dim_) {
      p->start_.p_(d) = x(idx);
      p->end_.p_(d)   = x(idx + dim_.size());
      p->UpdateCoefficients();
      idx++;
    }
  }
}

VecBound
EEForce::GetBounds () const
{
  VecBound bounds;

  for (int node=0; node<GetRows()/kDim3d; ++node) {
    bounds.push_back(Bound(-max_load_, max_load_)); // X
    bounds.push_back(Bound(-max_load_, max_load_)); // Y
    bounds.push_back(Bound(0.0, max_load_));        // Z, //unilateral contact forces
  }

  int idx=0;
  for (int s=0; s<polynomials_.size(); ++s) {

    for (auto d : dim_) {

      if (!is_in_contact_.at(s)) {// swing phase
        // forbid contact force at start and end of node
        bounds.at(idx)             = Bound(0.0, 0.0); // [N]
        bounds.at(idx+dim_.size()) = Bound(0.0, 0.0); // [N]
      }

      idx++;
    }
  }

  return bounds;
}

Vector3d
EEForce::GetForce (double t_global) const
{
  return spline_.GetPoint(t_global).p_;
}

void
EEForce::AddContactPhase (double T, double dt)
{

  double t_left = T;
  while (t_left > 0.0) {
    double duration = t_left>dt?  dt : t_left;
    polynomials_.push_back(MakePoly(duration));
    is_in_contact_.push_back(true);
    t_left -= dt;
  }
}

void
EEForce::AddSwingPhase (double T)
{
  // one polynomials for entire swing-phase, with value zero
  polynomials_.push_back(MakePoly(T));
  is_in_contact_.push_back(false);
}

int
EEForce::Index (double t_global, Polynomial::PointType p, Coords3D dim) const
{
  // this must be adapted/extended when changing the polynomial type
  int node_start = spline_.GetSegmentID(t_global);
  return (node_start+p)*dim_.size() + dim; // adjacent node are stored next to each other
}

JacobianRow
EEForce::GetJacobian (Coords3D dim, double t_global) const
{
  int idx_node_start = Index(t_global, Polynomial::Start, dim);
  int idx_node_goal  = Index(t_global, Polynomial::Goal, dim);

  // figure out which polynomial is active at the current time
  int phase      = spline_.GetSegmentID(t_global);
  auto poly      = polynomials_.at(phase);

  JacobianRow jac(GetRows());
  double t_local = spline_.GetLocalTime(t_global);
  jac.insert(idx_node_start) = poly->GetDerivativeOfPosWrtPos(t_local, Polynomial::Start);
  jac.insert(idx_node_goal)  = poly->GetDerivativeOfPosWrtPos(t_local, Polynomial::Goal);

  return jac;
}

EEForce::PolyPtr
EEForce::MakePoly (double T) const
{
  // initialize with nonzero values, to avoid CoP exception
  auto p = std::make_shared<LinearPolynomial>();
  auto start_values = StateLinXd(dim_.size());
  p->SetBoundary(T, start_values, start_values);
  return p;
}

} /* namespace opt */
} /* namespace xpp */
