/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 28, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPTIMIZATION_VARIABLES_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPTIMIZATION_VARIABLES_H_

#include <memory>
#include <string>
#include <Eigen/Dense>

#include "bound.h"

namespace xpp {
namespace opt {

/** @brief Assigns a meaning to pure numeric optimization variables.
  *
  * This class serves as a base class to represent different constructs, e.g.
  * polynomials, piecewise-constant functions, through a finite number of
  * parameters.
  */
class OptimizationVariables {
public:
  using VectorXd = Eigen::VectorXd;
  using Ptr      = std::shared_ptr<OptimizationVariables>;

  /** @param the name of what these parameters represent.
   */
  OptimizationVariables (const std::string& id);
  virtual ~OptimizationVariables ();

  // already present in component
  virtual VectorXd GetValues() const = 0;
  int GetRows() const;
  virtual VecBound GetBounds() const;
  std::string GetName() const;

  // optimization specific functions
  virtual void SetValues(const VectorXd&) = 0;


protected:
//  void SetAllBounds(const Bound&) const;

private:
  std::string name_;
};


} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPTIMIZATION_VARIABLES_H_ */
