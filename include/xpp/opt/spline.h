/**
@file    spline.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   For manipulation of multiple sequential polynomials.
 */

#ifndef _XPP_OPT_SPLINE_H_
#define _XPP_OPT_SPLINE_H_

#include <memory>
#include <vector>

#include <xpp/state.h>

#include "polynomial.h"

namespace xpp {
namespace opt {

/** @brief For manipulation of multiple sequential polynomials ("spline").
  */
class Spline {
public:
  using PointType      = StateLinXd;
  using PolynomialPtr  = std::shared_ptr<Polynomial>;
  using VecPolynomials = std::vector<PolynomialPtr>;

  PointType GetPoint(double t_globals) const;
  int GetPolynomialID(double t_global) const;
  double GetLocalTime(double t_global) const;
  double GetTotalTime() const;

  int GetFreeCoeffPerPoly() const;
  int GetTotalFreeCoeff() const;

  const VecPolynomials GetImpl() const { return polynomials_; };
  VecPolynomials& GetImpl()            { return polynomials_; };

private:
  VecPolynomials polynomials_;
};


} // namespace opt
} // namespace xpp

#endif // _XPP_OPT_SPLINE_H_
