/**
 @file    parametrization.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_

#include "bound.h"
#include <Eigen/Dense>
#include <string>
#include <memory>

namespace xpp {
namespace opt {

/** @brief Assigns a meaning to pure numeric optimization variables.
  *
  * This class serves as a base class to represent different constructs, e.g.
  * polynomials, piecewise-constant functions, through a finite number of
  * parameters.
  */
class Parametrization {
public:
  using VectorXd = Eigen::VectorXd;
  using Ptr      = std::shared_ptr<Parametrization>;

  /** @param the name of what these parameters represent.
   */
  Parametrization (const std::string& id);
  virtual ~Parametrization ();

  int GetOptVarCount() const;
  std::string GetId() const;

  virtual VectorXd GetVariables() const = 0;
  virtual void SetVariables(const VectorXd&) = 0;

  VecBound GetBounds() const;

protected:
  void SetAllBounds(const Bound&) const;
  mutable VecBound bounds_;

private:
  std::string id_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_ */
