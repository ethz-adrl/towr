/**
@file    com_spline6.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline6, which realizes a ComSpline
 */

#include <xpp/zmp/com_spline6.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper; // X,Y,Z

ComSpline6::ComSpline6 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline6::~ComSpline6 ()
{
  // TODO Auto-generated destructor stub
}

void
ComSpline6::Init (int step_count, const SplineTimes& times,
                  bool insert_initial_stance)
{
  ComSpline::Init(step_count, times, insert_initial_stance);

  // initialize all coefficients to zero
  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  SetCoefficients(abcd);
}

void
ComSpline6::SetCoefficients (const VectorXd& optimized_coeff)
{
  CheckIfSplinesInitialized();

  for (size_t p=0; p<polynomials_.size(); ++p) {
    CoeffValues coeff_values;

    for (const Coords3D dim : {X,Y}) {
      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;
      cv[A] = optimized_coeff[Index(p,dim,A)];
      cv[B] = optimized_coeff[Index(p,dim,B)];
      cv[C] = optimized_coeff[Index(p,dim,C)];
      cv[D] = optimized_coeff[Index(p,dim,D)];
      cv[E] = optimized_coeff[Index(p,dim,E)];
      cv[F] = optimized_coeff[Index(p,dim,F)];
    }

    polynomials_.at(p).SetSplineCoefficients(coeff_values);
  }
}

} // namespace zmp
} // namespace xpp


