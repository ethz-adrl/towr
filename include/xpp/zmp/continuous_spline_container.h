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

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // since member fixed sized eigen matrices (Vector2d)
  ContinuousSplineContainer ();
  virtual
  ~ContinuousSplineContainer ();

public:

  void Init(const Eigen::Vector2d& start_cog_p,
            const Eigen::Vector2d& start_cog_v,
            const std::vector<xpp::hyq::LegID>& step_sequence,
            double t_stance,
            double t_swing,
            double t_stance_initial,
            double t_stance_final,
            double dt = 0.1);




  static int Idx(int spline, int dim, int coeff);
  int GetTotalFreeCoeff() const;
  int GetTotalNodes(bool exclude_4ls_splines = false) const;
  int GetSplineID(int node) const;
  /**
   * Creates a Vector whose scalar product the optimized coefficients (a,b,c,d)
   * has the same effect as the original e coefficient in the spline equation
   * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
   *
   * @param spline_info
   * @param k the number of spline for which the e coefficient should be represented
   * @param dim X=0, Y=1
   * @param start_v the initial velocity of the first spline
   * @param[out] Ek the vector to represent e through a,b,c,d
   * @param[out] non_dependent the influence of e that are not dependent on a,b,c,d
   */
  void DescribeEByPrev(int k, int dim, Eigen::VectorXd& Ek, double& non_dependent) const;

  /**
   * Creates a Vector whose scalar product the optimized coefficients (a,b,c,d)
   * has the same effect as the original f coefficient in the spline equation
   * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
   *
   * @param spline_info
   * @param k the number of spline for which the e coefficient should be represented
   * @param dim X=0, Y=1
   * @param start_p the initial position of the first spline
   * @param start_v the initial velocity of the first spline
   * @param[out] Ek the vector to represent e through a,b,c,d
   * @param[out] non_dependent the influence of f that are not dependent on a,b,c,d
   */
  void DescribeFByPrev(int k, int dim, Eigen::VectorXd& Ek, double& non_dependent) const;

  MatVec ExpressZmpThroughCoefficients(double h, int dim) const;

  void AddOptimizedCoefficients(
      const Eigen::VectorXd& optimized_coeff);

  double dt_; // This is needed for creating support triangle inequality constraints
private:
  static const int kFreeCoeffPerSpline = kCoeffCount-2;

  Eigen::Vector2d start_cog_p_;
  Eigen::Vector2d start_cog_v_;
  void SetInitialPosVel(const Eigen::Vector2d& start_cog_p,
                        const Eigen::Vector2d& start_cog_v);
  bool initialized_ = false;
  void CheckIfInitialized() const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONTINUOUS_SPLINE_CONTAINER_H_ */
