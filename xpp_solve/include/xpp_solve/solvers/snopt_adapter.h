/**
 @file    snopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Declares the SnoptAdapter class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_

#include <snoptProblem.hpp>

#include <xpp_solve/nlp.h>

namespace xpp {

/** @brief Converts the NLP defined in XPP to the SNOPT interface.
  *
  * http://web.stanford.edu/group/SOL/guides/sndoc7.pdf
  *
  * This implements the Adapter pattern. This class should not add any functionality,
  * but merely delegate it to the Adaptee (the NLP class).
  */
class SnoptAdapter : public snoptProblemA {
public:
  using NLPPtr  = NLP*;

  /** Keeps reference to this NLP and modifies it! */
  SnoptAdapter (NLP& ref);
  virtual ~SnoptAdapter ();

  static void Solve(NLP& ref);

private:
  void Init();
  static void ObjectiveAndConstraintFct(int   *Status, int *n,    double x[],
                                        int   *needF,  int *neF,  double F[],
                                        int   *needG,  int *neG,  double G[],
                                        char     *cu,  int *lencu,
                                        int     iu[],  int *leniu,
                                        double  ru[],  int *lenru);

  void SetVariables();

  static NLPPtr nlp_; // use raw pointer as SnoptAdapter doesn't own the nlp.

  static void SetOptions(SnoptAdapter&);
};

} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_SNOPT_ADAPTER_H_ */
