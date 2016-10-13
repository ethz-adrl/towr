/**
 @file    snopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_

#include "nlp.h"
#include <snoptProblem.hpp>

namespace xpp {
namespace opt {

/** @brief Converts the NLP defined in the xpp interface to the SNOPT interface.
  *
  * This implements the Adapter pattern. This class should not add any functionality,
  * but merely delegate it to the Adaptee (the nlp class).
  */
class SnoptAdapter : public snoptProblemA {
public:
  typedef Eigen::VectorXd VectorXd;
  typedef SnoptAdapter* SelfPtr;
  typedef std::unique_ptr<NLP> NLPPtr;

  /** Only way to get an instance of this class.
    *
    * This implements the singleton pattern, to be sure that only one instance
    * of this class exists at all times.
    */
  static SelfPtr GetInstance();
  virtual ~SnoptAdapter ();


  void SetNLP(NLPPtr&);
  void Init();
  static void ObjectiveAndConstraintFct(int    *Status, int *n,    double x[],
                                        int    *needF,  int *neF,  double F[],
                                        int    *needG,  int *neG,  double G[],
                                        char      *cu,  int *lencu,
                                        int    iu[],    int *leniu,
                                        double ru[],    int *lenru);

  void SolveSQP(int start_type);

private:
  SnoptAdapter ();
  NLPPtr nlp_;
  static SelfPtr instance_; ///< to access member variables in static function
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_ */
