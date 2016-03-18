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


void ContinuousSplineContainer::Init(const Eigen::Vector2d& start_cog_p,
                                     const Eigen::Vector2d& start_cog_v,
                                     const std::vector<xpp::hyq::LegID>& step_sequence,
                                     double t_stance,
                                     double t_swing,
                                     double t_stance_initial,
                                     double t_stance_final)
{
  SetInitialPosVel(start_cog_p, start_cog_v);
  ConstructSplineSequence(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);
  initialized_ = true;
}

void ContinuousSplineContainer::SetInitialPosVel(const Eigen::Vector2d& start_cog_p,
                                                 const Eigen::Vector2d& start_cog_v)
{
  start_cog_p_ = start_cog_p;
  start_cog_v_ = start_cog_v;
}


int ContinuousSplineContainer::GetTotalFreeCoeff() const
{
  CheckIfInitialized();
  return splines_.size() * kFreeCoeffPerSpline * kDim2d;
}


int ContinuousSplineContainer::Idx(int spline, int dim, int coeff)
{
  return kFreeCoeffPerSpline * kDim2d * spline + kFreeCoeffPerSpline * dim + coeff;
}


void ContinuousSplineContainer::DescribeEByPrev(int k,
                                   int dim, Eigen::VectorXd& Ek,
                                   double& non_dependent) const
{
  CheckIfInitialized();

  Ek.setZero();
  for (int i=0; i<k; ++i) {
    double Ti = splines_.at(i).duration_;

    Ek(Idx(i,dim,A)) += 5*std::pow(Ti,4);
    Ek(Idx(i,dim,B)) += 4*std::pow(Ti,3);
    Ek(Idx(i,dim,C)) += 3*std::pow(Ti,2);
    Ek(Idx(i,dim,D)) += 2*std::pow(Ti,1);
  }

  non_dependent = start_cog_v_(dim);
}


void ContinuousSplineContainer::DescribeFByPrev(int k, int dim,
                                   Eigen::VectorXd& Fk, double& non_dependent) const
{
  CheckIfInitialized();

  Eigen::VectorXd T0tok(k); T0tok.setZero();
  Fk.setZero();
  for (int i=0; i<k; ++i) {

    double Ti = splines_.at(i).duration_;

    T0tok(i) = Ti; // initial velocity acts over the entire time T of EVERY spline

    Fk[Idx(i,dim,A)] += std::pow(Ti,5);
    Fk[Idx(i,dim,B)] += std::pow(Ti,4);
    Fk[Idx(i,dim,C)] += std::pow(Ti,3);
    Fk[Idx(i,dim,D)] += std::pow(Ti,2);

    for (int j = 0; j<i; ++j) {
      double Tj = splines_.at(j).duration_;
      Fk[Idx(j,dim,A)] += 5*std::pow(Tj,4) * Ti;
      Fk[Idx(j,dim,B)] += 4*std::pow(Tj,3) * Ti;
      Fk[Idx(j,dim,C)] += 3*std::pow(Tj,2) * Ti;
      Fk[Idx(j,dim,D)] += 2*std::pow(Tj,1) * Ti;
    }
  }

  non_dependent = start_cog_v_(dim)*T0tok.sum() + start_cog_p_(dim);
}


void
ContinuousSplineContainer::AddOptimizedCoefficients(
    const Eigen::VectorXd& optimized_coeff)
{
  CheckIfInitialized();

  Eigen::VectorXd Ek(optimized_coeff.size());
  Eigen::VectorXd Fk(optimized_coeff.size());
  double non_dependent_e, non_dependent_f;

  for (size_t k=0; k<splines_.size(); ++k) {
    CoeffValues coeff_values;

    for (int dim=xpp::utils::X; dim<=xpp::utils::Y; dim++) {

      double* cv = (dim == xpp::utils::X) ? coeff_values.x : coeff_values.y;

      // fill in only first 4 optimized coefficients
      cv[A] = optimized_coeff[Idx(k,dim,A)];
      cv[B] = optimized_coeff[Idx(k,dim,B)];
      cv[C] = optimized_coeff[Idx(k,dim,C)];
      cv[D] = optimized_coeff[Idx(k,dim,D)];

      // calculate e and f coefficients from previous values
      Ek.setZero();
      Fk.setZero();
      DescribeEByPrev(k, dim, Ek, non_dependent_e);
      DescribeFByPrev(k, dim, Fk, non_dependent_f);

      cv[E] = Ek.transpose()*optimized_coeff + non_dependent_e;
      cv[F] = Fk.transpose()*optimized_coeff + non_dependent_f;

    } // dim:X..Y

    splines_.at(k).set_spline_coeff(coeff_values);

  } // k=0..n_spline_infos_
}


Eigen::VectorXd
ContinuousSplineContainer::GetXyDimAlternatingVector(double x, double y) const
{
  CheckIfInitialized();

  Eigen::VectorXd x_abcd(kFreeCoeffPerSpline);
  x_abcd.fill(x);

  Eigen::VectorXd y_abcd(kFreeCoeffPerSpline);
  y_abcd.fill(y);

  int coeff = splines_.size() * kFreeCoeffPerSpline * kDim2d;
  Eigen::VectorXd vec(coeff);
  vec.setZero();

  for (const ZmpSpline& s : splines_) {
    vec.middleRows(Idx(s.id_,xpp::utils::X,A), kFreeCoeffPerSpline) = x_abcd;
    vec.middleRows(Idx(s.id_,xpp::utils::Y,A), kFreeCoeffPerSpline) = y_abcd;
  }

  return vec;
}

void
ContinuousSplineContainer::CheckIfInitialized() const
{
  if (!initialized_) {

    throw std::runtime_error("ContinousSplineContainer not initialized. Call Init() first");
  }
}

} /* namespace zmp */
} /* namespace xpp */
