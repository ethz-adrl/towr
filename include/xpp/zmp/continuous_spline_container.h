/*
 * continuous_spline_container.h
 *
 *  Created on: Mar 18, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONTINUOUS_SPLINE_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONTINUOUS_SPLINE_CONTAINER_H_

#include <xpp/zmp/spline_container.h>

namespace xpp {
namespace zmp {

/**
 * This class represents a collection of fifth order polynomials
 * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f, that are
 * continous in position and velocity at their borders. This means that the e
 * and f coefficients of all splines can be uniquely determined from the other
 * coefficients and the initial position/velocity.
 */
class ContinuousSplineContainer : public SplineContainer
{
public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef Eigen::Vector2d Vector2d;

public:
  ContinuousSplineContainer () {};
  virtual
  ~ContinuousSplineContainer () {};

  static constexpr double dt_ = 0.1; // This is needed for creating support triangle inequality constraints
public:

  void Init(const Eigen::Vector2d& start_cog_p,
            const Eigen::Vector2d& start_cog_v,
            const std::vector<xpp::hyq::LegID>& step_sequence,
            double t_stance,
            double t_swing,
            double t_stance_initial,
            double t_stance_final);

  /**
   * The index number of the coefficient \c coeff, for dimension \c dim and
   * spline number \c spline.
   */
  static int Index(int spline, int dim, int coeff);
  int GetTotalFreeCoeff() const;
  int GetTotalNodesNo4ls() const;
  int GetTotalNodes4ls() const;

  void AddOptimizedCoefficients(const Eigen::VectorXd& optimized_coeff,
                                VecSpline& splines) const;
  void AddOptimizedCoefficients(const Eigen::VectorXd& optimized_coeff)
  {
    AddOptimizedCoefficients(optimized_coeff, splines_);
  }
  void UpdateInitialPosVel(const Vector2d& start_cog_p, const Vector2d& start_cog_v);
  VecScalar RelationshipToABCD(int spline_id_k, int dim, SplineCoeff c) const;

private:
  static const int kFreeCoeffPerSpline = kCoeffCount-2;

  std::array<MatVec, 2> relationship_e_to_abcd_;
  std::array<MatVec, 2> relationship_f_to_abdc_;
  /**
   * Creates a Vector whose scalar product with the optimized coefficients (a,b,c,d)
   * has the same effect as the original e/f coefficients in the spline equation
   * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
   *
   * @param dim X=0, Y=1
   * @param start_p the initial position of the first spline
   * @param start_v the initial velocity of the first spline
   * @returns matrix and vector that describe the coefficient
   */
  MatVec DescribeEByPrev(int dim, double start_cog_v) const;
  MatVec DescribeFByPrev(int dim, double start_cog_p, double start_cog_v) const;


  bool initialized_ = false;
  void CheckIfInitialized() const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONTINUOUS_SPLINE_CONTAINER_H_ */
