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

static const int kFreeCoeffPerSpline = kCoeffCount-2;
static const SplineCoeff FreeSplineCoeff[] = { A, B, C, D };

/**
 * This class represents a collection of fifth order polynomials
 * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f, that are
 * continuous in position and velocity at their borders. This means that the e
 * and f coefficients of all splines can be uniquely determined from the other
 * coefficients and the initial position/velocity.
 */
class ContinuousSplineContainer : public SplineContainer
{
public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::RowVector4d VecABCD;

  typedef xpp::utils::coords_wrapper::Coords3D Coords;

public:
  ContinuousSplineContainer () {};
  ContinuousSplineContainer (const Vector2d& start_cog_p,
                             const Vector2d& start_cog_v,
                             int step_count,
                             const SplineTimes& times);
  virtual ~ContinuousSplineContainer () {};

public:

  void Init(const Vector2d& start_cog_p,
            const Vector2d& start_cog_v,
            int step_count,
            const SplineTimes& times,
            bool insert_initial_stance = true,
            bool insert_final_stance = true);

  /**
   * The index number of the coefficient \c coeff, for dimension \c dim and
   * spline number \c spline.
   */
  static int Index(int spline, Coords dim, SplineCoeff coeff);
  int GetTotalFreeCoeff() const;

  void AddOptimizedCoefficients(const Eigen::VectorXd& optimized_coeff,
                                VecSpline& splines) const;
  void AddOptimizedCoefficients(const Eigen::VectorXd& optimized_coeff)
  {
    AddOptimizedCoefficients(optimized_coeff, splines_);
  }
  VecSpline BuildOptimizedSplines(const Eigen::VectorXd& optimized_coeff) const
  {
    VecSpline splines = splines_;
    AddOptimizedCoefficients(optimized_coeff, splines);
    return splines;
  }

  VecScalar GetECoefficient(int spline_id_k, Coords dim) const;
  VecScalar GetFCoefficient(int spline_id_k, Coords dim) const;

  /**
   * Produces a vector and scalar, that, multiplied with the spline coefficients
   * a,b,c,d of all splines returns the position of the CoG at time t_local
   * @param t_local @attention local time of spline. So t_local=0 returns CoG at beginning of this spline.
   * @param id id of current spline
   * @param dim dimension specifying if x or y coordinate of CoG should be calculated
   * @return
   */
  VecScalar ExpressCogPosThroughABCD(double t_local, int id, Coords dim) const;

  VecScalar ExpressCogAccThroughABCD(double t_local, int id, Coords dim) const;

private:
  VecABCD ExpressCogAccThroughABCD(double t_local) const;


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
  MatVec DescribeEByABCD(Coords Coords, double start_cog_v) const;
  MatVec DescribeFByABCD(Coords Coords, double start_cog_p, double start_cog_v) const;


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_CONTINUOUS_SPLINE_CONTAINER_H_ */
