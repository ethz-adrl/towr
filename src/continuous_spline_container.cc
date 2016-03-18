/*
 * continuous_spline_container.cc
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#include <xpp/zmp/continuous_spline_container.h>

namespace xpp {
namespace zmp {

ContinuousSplineContainer::ContinuousSplineContainer ()
{
  // TODO Auto-generated constructor stub

}

ContinuousSplineContainer::~ContinuousSplineContainer ()
{
  // TODO Auto-generated destructor stub
}


int ContinuousSplineContainer::GetTotalFreeCoeff() const
{
  return splines_.size() * kFreeCoeffPerSpline * kDim2d;
}


int ContinuousSplineContainer::var_index(int spline, int dim, int coeff)
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}


void ContinuousSplineContainer::DescribeEByPrev(int k,
                                   int dim, double start_v, Eigen::VectorXd& Ek,
                                   double& non_dependent) const
{
  Ek.setZero();
  for (int i=0; i<k; ++i) {
    double Ti = splines_.at(i).duration_;

    Ek(var_index(i,dim,A)) += 5*std::pow(Ti,4);
    Ek(var_index(i,dim,B)) += 4*std::pow(Ti,3);
    Ek(var_index(i,dim,C)) += 3*std::pow(Ti,2);
    Ek(var_index(i,dim,D)) += 2*std::pow(Ti,1);
  }

  non_dependent = start_v;
}


void ContinuousSplineContainer::DescribeFByPrev(int k,
                                   int dim, double start_v, double start_p,
                                   Eigen::VectorXd& Fk, double& non_dependent) const
{
  Eigen::VectorXd T0tok(k); T0tok.setZero();
  Fk.setZero();
  for (int i=0; i<k; ++i) {

    double Ti = splines_.at(i).duration_;

    T0tok(i) = Ti; // initial velocity acts over the entire time T of EVERY spline

    Fk[var_index(i,dim,A)] += std::pow(Ti,5);
    Fk[var_index(i,dim,B)] += std::pow(Ti,4);
    Fk[var_index(i,dim,C)] += std::pow(Ti,3);
    Fk[var_index(i,dim,D)] += std::pow(Ti,2);

    for (int j = 0; j<i; ++j) {
      double Tj = splines_.at(j).duration_;
      Fk[var_index(j,dim,A)] += 5*std::pow(Tj,4) * Ti;
      Fk[var_index(j,dim,B)] += 4*std::pow(Tj,3) * Ti;
      Fk[var_index(j,dim,C)] += 3*std::pow(Tj,2) * Ti;
      Fk[var_index(j,dim,D)] += 2*std::pow(Tj,1) * Ti;
    }
  }

  non_dependent = start_v*T0tok.sum() + start_p;
}


void
ContinuousSplineContainer::AddOptimizedCoefficients(
    const Eigen::Vector2d& start_cog_p,
    const Eigen::Vector2d& start_cog_v,
    const Eigen::VectorXd& optimized_coeff)
{
  Eigen::VectorXd Ek(optimized_coeff.size());
  Eigen::VectorXd Fk(optimized_coeff.size());
  double non_dependent_e, non_dependent_f;

  for (size_t k=0; k<splines_.size(); ++k) {
    CoeffValues coeff_values;

    for (int dim=xpp::utils::X; dim<=xpp::utils::Y; dim++) {

      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      cv[A] = optimized_coeff[var_index(k,dim,A)];
      cv[B] = optimized_coeff[var_index(k,dim,B)];
      cv[C] = optimized_coeff[var_index(k,dim,C)];
      cv[D] = optimized_coeff[var_index(k,dim,D)];

      // calculate e and f coefficients from previous values
      Ek.setZero();
      Fk.setZero();
      DescribeEByPrev(k, dim, start_cog_v(dim), Ek, non_dependent_e);
      DescribeFByPrev(k, dim, start_cog_v(dim), start_cog_p(dim), Fk,non_dependent_f);

      cv[E] = Ek.transpose()*optimized_coeff + non_dependent_e;
      cv[F] = Fk.transpose()*optimized_coeff + non_dependent_f;

    } // dim:X..Y

    splines_.at(k).set_spline_coeff(coeff_values);

  } // k=0..n_spline_infos_
}


Eigen::VectorXd
ContinuousSplineContainer::GetXyDimAlternatingVector(double x, double y) const
{
  Eigen::VectorXd x_abcd(kFreeCoeffPerSpline);
  x_abcd.fill(x);

  Eigen::VectorXd y_abcd(kFreeCoeffPerSpline);
  y_abcd.fill(y);

  int coeff = splines_.size() * kFreeCoeffPerSpline * kDim2d;
  Eigen::VectorXd vec(coeff);
  vec.setZero();

  for (const ZmpSpline& s : splines_) {
    vec.middleRows(var_index(s.id_,xpp::utils::X,A), kFreeCoeffPerSpline) = x_abcd;
    vec.middleRows(var_index(s.id_,xpp::utils::Y,A), kFreeCoeffPerSpline) = y_abcd;
  }

  return vec;
}


} /* namespace zmp */
} /* namespace xpp */
