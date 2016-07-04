/**
 @file    snopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SNOPT_ADAPTER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SNOPT_ADAPTER_H_

#include <snoptProblem.hpp>

#include <xpp/zmp/nlp.h>

namespace xpp {
namespace zmp {

/** @brief Converts the NLP defined in the xpp interface to the SNOPT interface.
  *
  * This implements the Adapter pattern. This class should not add any functionality,
  * but merely delegate it to the Adaptee (the nlp class).
  */
class SnoptAdapter : public snoptProblemA {
public:
  SnoptAdapter (NLP& nlp);
  virtual ~SnoptAdapter ();

private:
  NLP& nlp_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_SNOPT_ADAPTER_H_ */
