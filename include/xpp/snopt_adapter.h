/**
 @file    snopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Declares the SnoptAdapter class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_

#include "nlp.h"
#include <snoptProblem.hpp>

namespace xpp {
namespace opt {

/** @brief Converts the NLP defined in XPP to the SNOPT interface.
  *
  * http://web.stanford.edu/group/SOL/guides/sndoc7.pdf
  *
  * This implements the Adapter pattern. This class should not add any functionality,
  * but merely delegate it to the Adaptee (the NLP class).
  */
class SnoptAdapter : public snoptProblemA {
public:
  using SelfPtr = SnoptAdapter*;
  using NLPPtr  = std::shared_ptr<NLP>;

  /** Only way to get an instance of this class.
    *
    * This implements the singleton pattern, to be sure that only one instance
    * of this class exists at all times.
    */
  static SelfPtr GetInstance();
  virtual ~SnoptAdapter ();


  /** Assign SNOPT to a Nonlinear Program. Keeps reference to this NLP and
    *  modifies it!.
    */
  void SetNLP(NLPPtr&);
  void Init();
  static void ObjectiveAndConstraintFct(int    *Status, int *n,    double x[],
                                        int    *needF,  int *neF,  double F[],
                                        int    *needG,  int *neG,  double G[],
                                        char      *cu,  int *lencu,
                                        int      iu[],  int *leniu,
                                        double   ru[],  int *lenru);

  void SolveSQP(int start_type);

private:
  SnoptAdapter ();
  NLPPtr nlp_;
  static SelfPtr instance_; ///< to access member variables in static function
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_ */
