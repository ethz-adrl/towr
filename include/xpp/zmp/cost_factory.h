/**
 @file    cost_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FACTORY_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FACTORY_H_

#include <xpp/utils/geometric_structs.h>
#include <memory>

namespace xpp {
namespace zmp {

class ACost;
class ContinuousSplineContainer;
class OptimizationVariablesInterpreter;

class CostFactory {
public:
  typedef std::shared_ptr<ACost> CostPtr;
  typedef xpp::utils::Point2d State2d;

  CostFactory ();
  virtual ~CostFactory ();

  static CostPtr CreateAccelerationCost(const ContinuousSplineContainer&);

  static CostPtr CreateFinalCost(const State2d& final_state_xy, const ContinuousSplineContainer&);
  static CostPtr CreateRangeOfMotionCost(const OptimizationVariablesInterpreter&);
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_FACTORY_H_ */
