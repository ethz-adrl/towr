/**
@file    spline_container.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Holds coefficients of multiple fifth-order splines and returns state
         (pos,vel,acc) of appropriate spline for a specific time
 */

#include <xpp/zmp/spline_container.h>

#include <cmath>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <cassert>

namespace xpp {
namespace zmp {

log4cxx::LoggerPtr SplineContainer::log_(log4cxx::Logger::getLogger("xpp.zmp.splinecontainer"));

SplineContainer::SplineContainer()
{
  curr_spline_ = 0;
  splines_.clear();
}


SplineContainer::~SplineContainer()
{
  // TODO Auto-generated destructor stub
}

double SplineContainer::GetTotalTime() const
{
  double T = 0.0;
  for (ZmpSpline s: splines_) {
    T += s.duration_;
  };
  return T;
}

void SplineContainer::AddSpline(const ZmpSpline &spline)
{
  splines_.push_back(spline);
}


// Creates a sequence of Splines without the optimized coefficients
void SplineContainer::ConstructSplineSequence(
    const std::vector<xpp::hyq::LegID>& step_sequence,
                                      double t_stance,
                                      double t_swing,
                                      double t_stance_initial,
                                      double t_stance_final)
{
  splines_.clear();
  int step = 0;
  unsigned int id = 0; // unique identifiers for each spline

  const int kSplinesPer4ls = 1;
  const int kSplinesPerStep = 1;

  for (size_t i = 0; i < step_sequence.size(); ++i)
  {
    // 1. insert 4ls-phase when switching between disjoint support triangles
    // Attention: these 4ls-phases much coincide with the ones in the zmp optimizer
    if (i==0) {
      AddSpline(ZmpSpline(id++, t_stance_initial, true, step));
    } else {
      xpp::hyq::LegID swing_leg = step_sequence[i];
      xpp::hyq::LegID swing_leg_prev = step_sequence[i-1];
      if (xpp::hyq::SuppTriangle::Insert4LSPhase(swing_leg_prev, swing_leg))
        for (int s = 0; s < kSplinesPer4ls; s++)
          AddSpline(ZmpSpline(id++, t_stance/kSplinesPer4ls, true, step));
    }


    // insert swing phase splines
    for (int s = 0; s < kSplinesPerStep; s++)
      AddSpline(ZmpSpline(id++, t_swing/kSplinesPerStep, false, step));



    // always have last 4ls spline for robot to move into center of feet
    if (i==step_sequence.size()-1)
      AddSpline(ZmpSpline(id++, t_stance_final, true, step));

    step++;
  }
}



int SplineContainer::GetOptCoeffCount() const
{
  return splines_.size() * kOptCoeff * kDim2d;
}


int SplineContainer::var_index(int spline, int dim, int coeff)
{
  return kOptCoeff * kDim2d * spline + kOptCoeff * dim + coeff;
}


void SplineContainer::DescribeEByPrev(int k,
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


void SplineContainer::DescribeFByPrev(int k,
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
SplineContainer::AddOptimizedCoefficients(
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
SplineContainer::GetXyDimAlternatingVector(double x, double y) const
{
  Eigen::VectorXd x_abcd(kOptCoeff);
  x_abcd.fill(x);

  Eigen::VectorXd y_abcd(kOptCoeff);
  y_abcd.fill(y);

  int coeff = splines_.size() * kOptCoeff * kDim2d;
  Eigen::VectorXd vec(coeff);
  vec.setZero();

  for (const ZmpSpline& s : splines_) {
    vec.middleRows(var_index(s.id_,xpp::utils::X,A), kOptCoeff) = x_abcd;
    vec.middleRows(var_index(s.id_,xpp::utils::Y,A), kOptCoeff) = y_abcd;
  }

  return vec;
}

void SplineContainer::GetCOGxy(double t_global, Lin2d& cog_xy)
{
  /** Transform global time to local spline time dt */
  double t_local = t_global;
  for (uint s = 0; s < curr_spline_; s++)
    t_local -= splines_[s].duration_;

  cog_xy.p = splines_[curr_spline_].GetState(kPos, t_local);
  cog_xy.v = splines_[curr_spline_].GetState(kVel, t_local);
  cog_xy.a = splines_[curr_spline_].GetState(kAcc, t_local);

  if (t_local > splines_[curr_spline_].duration_) {
    ++curr_spline_;
    assert(curr_spline_ < splines_.size()); // make sure the current spline is in the buffer
    LOG4CXX_TRACE(log_, std::setprecision(1) << std::fixed << " switched to zmp-spline " << curr_spline_);
  }
}

} // namespace zmp
} // namespace xpp
