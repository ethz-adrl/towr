/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
@file    snopt_adapter.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 4, 2016
 @brief   Declares the SnoptAdapter class
 */

#ifndef OPT_SOLVE_INCLUDE_OPT_SNOPT_ADAPTER_H_
#define OPT_SOLVE_INCLUDE_OPT_SNOPT_ADAPTER_H_

#include <snoptProblem.hpp>

#include <opt_solve/nlp.h>

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

} /* namespace opt */

#endif /* OPT_SOLVE_INCLUDE_OPT_SNOPT_ADAPTER_H_ */
