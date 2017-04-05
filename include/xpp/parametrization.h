/**
 @file    parametrization.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_

#include <Eigen/Dense>
#include <string>

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

  /** @param the name of what these parameters represent.
   */
  Parametrization (const std::string& id);
  virtual ~Parametrization ();

  int GetOptVarCount() const;
  std::string GetID() const;

  virtual VectorXd GetOptimizationParameters() const = 0;
  virtual void SetOptimizationParameters(const VectorXd&) = 0;

private:
  std::string id_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_PARAMETRIZATION_H_ */
