/**
 @file    linear_spline_equations.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/opt/linear_spline_equations.h>
#include <xpp/opt/com_spline.h>

namespace xpp {
namespace opt {

LinearSplineEquations::LinearSplineEquations (const BaseMotion& com_motion )
{
  // cast com motion to spline, because i need some specific features of that
  auto base_ptr = com_motion.clone();

  ComSpline *tmp = dynamic_cast<ComSpline*>(base_ptr.get());

  if(tmp != nullptr)
  {
    base_ptr.release();
    com_spline_.reset(tmp);
  }

  com_spline_->SetCoefficientsZero(); // the values my motion function approximation is around
}

LinearSplineEquations::~LinearSplineEquations ()
{
  // TODO Auto-generated destructor stub
}

MatVec
LinearSplineEquations::MakeInitial (const StateLin2d& init) const
{
  auto derivatives = com_spline_->GetInitialFreeMotions();

  int n_constraints = kDim2d *derivatives.size();
  MatVec M(n_constraints, com_spline_->GetTotalFreeCoeff());

  int i = 0; // constraint count
  double t_global = 0.0;
  for (const Coords3D dim : {X,Y})
  {
    for (auto dxdt :  derivatives) {
      VecScalar diff_dxdt = com_spline_->GetLinearApproxWrtCoeff(t_global, dxdt, dim);
      diff_dxdt.s -= init.GetByIndex(dxdt, dim);
      M.WriteRow(diff_dxdt, i++);
    }
  }

  assert(i==n_constraints);
  return M;
}

MatVec
LinearSplineEquations::MakeFinal (const StateLin2d& final_state,
                                  const MotionDerivatives& derivatives) const
{
  int n_constraints = derivatives.size()*kDim2d;
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int c = 0; // constraint count
  double T = com_spline_->GetTotalTime();
  for (const Coords3D dim : {X,Y})
  {
    for (auto dxdt :  derivatives) {
      VecScalar diff_dxdt = com_spline_->GetLinearApproxWrtCoeff(T, dxdt, dim);
      diff_dxdt.s -= final_state.GetByIndex(dxdt, dim);
      M.WriteRow(diff_dxdt, c++);
    }
  }

  assert(c==n_constraints);
  return M;
}

MatVec
LinearSplineEquations::MakeJunction () const
{
  auto derivatives = com_spline_->GetJunctionFreeMotions();

  int id_last = com_spline_->GetLastPolynomial().GetId();
  int n_constraints = derivatives.size() * id_last * kDim2d;
  int n_spline_coeff = com_spline_->GetTotalFreeCoeff();
  MatVec M(n_constraints, n_spline_coeff);

  int i = 0; // constraint count

  for (int id = 0; id < id_last; ++id) {
    double T = com_spline_->GetPolynomial(id).GetDuration();

    for (auto dim : {X,Y}) {
      for (auto dxdt :  derivatives) {
        VecScalar curr, next;

        // coefficients are all set to zero
        curr.s = com_spline_->GetCOGxyAtPolynomial(T,   id  ).GetByIndex(dxdt, dim);
        next.s = com_spline_->GetCOGxyAtPolynomial(0.0, id+1).GetByIndex(dxdt, dim);

        curr.v = com_spline_->GetJacobianWrtCoeffAtPolynomial(dxdt,   T,   id, dim);
        next.v = com_spline_->GetJacobianWrtCoeffAtPolynomial(dxdt, 0.0, id+1, dim);

        M.WriteRow(curr-next, i++);
      }
    }
  }
  assert(i==n_constraints);
  return M;
}

Eigen::MatrixXd
LinearSplineEquations::MakeAcceleration (const ValXY& weight) const
{
  // total number of coefficients to be optimized
  int n_coeff = com_spline_->GetTotalFreeCoeff();

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_coeff, n_coeff);
  for (const ComPolynomial& p : com_spline_->GetPolynomials()) {
    std::array<double,8> t_span = CalcExponents<8>(p.GetDuration());

    for (const Coords3D dim : {X,Y}) {
      const int a = com_spline_->Index(p.GetId(), dim, ComSpline::PolyCoeff::A);
      const int b = com_spline_->Index(p.GetId(), dim, ComSpline::PolyCoeff::B);
      const int c = com_spline_->Index(p.GetId(), dim, ComSpline::PolyCoeff::C);
      const int d = com_spline_->Index(p.GetId(), dim, ComSpline::PolyCoeff::D);

      // for explanation of values see M.Kalakrishnan et al., page 248
      // "Learning, Planning and Control for Quadruped Robots over challenging
      // Terrain", IJRR, 2010
      M(a, a) = 400.0 / 7.0      * t_span[7] * weight[dim];
      M(a, b) = 40.0             * t_span[6] * weight[dim];
      M(a, c) = 120.0 / 5.0      * t_span[5] * weight[dim];
      M(a, d) = 10.0             * t_span[4] * weight[dim];
      M(b, b) = 144.0 / 5.0      * t_span[5] * weight[dim];
      M(b, c) = 18.0             * t_span[4] * weight[dim];
      M(b, d) = 8.0              * t_span[3] * weight[dim];
      M(c, c) = 12.0             * t_span[3] * weight[dim];
      M(c, d) = 6.0              * t_span[2] * weight[dim];
      M(d, d) = 4.0              * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < M.cols(); ++c)
        for (int r = c + 1; r < M.rows(); ++r)
          M(r, c) = M(c, r);
    }
  }

  return M;
}

Eigen::MatrixXd
LinearSplineEquations::MakeJerk (const ValXY& weight) const
{
  // total number of coefficients to be optimized
  int n_coeff = com_spline_->GetTotalFreeCoeff();

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n_coeff, n_coeff);
  for (const ComPolynomial& p : com_spline_->GetPolynomials()) {
    std::array<double,8> t_span = CalcExponents<8>(p.GetDuration());

    for (const Coords3D dim : {X,Y}) {
      const int a = com_spline_->Index(p.GetId(), dim, Polynomial::PolynomialCoeff::A);
      const int b = com_spline_->Index(p.GetId(), dim, Polynomial::PolynomialCoeff::B);
      const int c = com_spline_->Index(p.GetId(), dim, Polynomial::PolynomialCoeff::C);

      M(a, a) = 60.*60. / 5.      * t_span[5] * weight[dim];
      M(a, b) = 60.*24. / 4.      * t_span[4] * weight[dim];
      M(a, c) = 60.* 6. / 3.      * t_span[3] * weight[dim];
      M(b, b) = 24.*24. / 3.      * t_span[3] * weight[dim];
      M(b, c) = 24.* 6. / 2.      * t_span[2] * weight[dim];
      M(c, c) =  6.* 6. / 1.      * t_span[1] * weight[dim];

      // mirrow values over diagonal to fill bottom left triangle
      for (int c = 0; c < M.cols(); ++c)
        for (int r = c + 1; r < M.rows(); ++r)
          M(r, c) = M(c, r);
    }
  }

  return M;
}

} /* namespace opt */
} /* namespace xpp */

