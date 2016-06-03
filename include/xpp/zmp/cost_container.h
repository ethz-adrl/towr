/**
 @file    cost_container.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides a class to hold all the different cost terms.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_

#include <xpp/zmp/a_cost.h>

#include <map>
#include <memory>

namespace xpp {
namespace zmp {

/** @brief Combines all the cost terms to a cost function and provides value.
  *
  * This class is responsible for knowing about all the different cost terms
  * and delivering the total cost for specific optimization variables.
  */
class CostContainer {
public:
  typedef std::shared_ptr<ACost> CostPtr;

  CostContainer ();
  virtual ~CostContainer ();

  ACost& GetCost(const std::string& name);

  void AddCost(CostPtr cost, const std::string& name);
  double EvaluateTotalCost () const;

private:
  std::map<std::string, CostPtr > costs_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_COST_CONTAINER_H_ */
