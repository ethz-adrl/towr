/**
@file    com_spline4.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Declares ComSpline4, which realizes a ComSpline
 */

#ifndef _XPP_ZMP_COMSPLINE4_H_
#define _XPP_ZMP_COMSPLINE4_H_

#include "com_spline.h"

namespace xpp {
namespace zmp {

/** Represents the center of mass motion with 4 coefficients per polynomial
  *
  * This class represents a collection of fifth order polynomials
  * p(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f, that are
  * continuous in position and velocity at their borders. This means that the e
  * and f coefficients of all splines can be uniquely determined from the other
  * coefficients and the initial position/velocity.
  */
class ComSpline4 : public ComSpline {
public:
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::utils::VecScalar VecScalar;
  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::RowVector4d VecABCD;
  typedef Eigen::VectorXd VectorXd;

  ComSpline4 ();
  virtual ~ComSpline4 ();
  ComSpline4 (const Vector2d& start_cog_p,
                             const Vector2d& start_cog_v,
                             int step_count,
                             const SplineTimes& times);

  void Init(const Vector2d& start_cog_p,
            const Vector2d& start_cog_v,
            int step_count,
            const SplineTimes& times,
            bool insert_initial_stance = true);

  void SetCoefficients(const VectorXd& optimized_coeff) override;
  void SetEndAtStart();

  Derivatives GetInitialFreeMotions()  const;
  Derivatives GetJunctionFreeMotions() const;
  Derivatives GetFinalFreeMotions()    const;

private:
  int NumFreeCoeffPerSpline() const override { return 4; };
  std::vector<SplineCoeff> GetFreeCoeffPerSpline() const override { return {A,B,C,D}; };

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

  VecABCD ExpressCogAccThroughABCD(double t_local) const;
  VecScalar ExpressCogPosThroughABCD (double t_local, int id, Coords dim) const override;
  VecScalar ExpressCogVelThroughABCD (double t_local, int id, Coords dim) const override;
  VecScalar ExpressCogAccThroughABCD (double t_local, int id, Coords dim) const override;
  VecScalar ExpressCogJerkThroughABCD(double t_local, int id, Coords dim) const override;

  VecScalar GetECoefficient(int spline_id_k, Coords dim) const;
  VecScalar GetFCoefficient(int spline_id_k, Coords dim) const;
};

} /* namespace zmp */
} /* namespace xpp */

#endif // _XPP_ZMP_COMSPLINE4_H_
