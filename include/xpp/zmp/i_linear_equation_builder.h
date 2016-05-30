/**
 @file    i_linear_equation_builder.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_LINEAR_EQUATION_BUILDER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_LINEAR_EQUATION_BUILDER_H_

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

class ILinearEquationBuilder {
public:
  typedef xpp::utils::MatVec MatVec;

  ILinearEquationBuilder () {};
  virtual ~ILinearEquationBuilder () {};

  virtual MatVec BuildLinearEquation() const = 0;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_LINEAR_EQUATION_BUILDER_H_ */
