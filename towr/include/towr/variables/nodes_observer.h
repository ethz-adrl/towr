/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_


namespace towr {

class NodesVariables;

/**
 * @brief Base class to receive up-to-date values of the NodeVariables.
 *
 * This class registers with the node variables and everytime the positions or
 * velocities of a node change, the subject updates this class by calling the
 * UpdatePolynomials() method.
 *
 * Used by spline.h
 *
 * This class implements the observer pattern:
 * https://sourcemaking.com/design_patterns/observer
 */
class NodesObserver {
public:
  using NodeSubjectPtr = NodesVariables*; // observer shouldn't own subject

  /**
   * @brief Registers this observer with the subject class to receive updates.
   * @param node_values  The subject holding the Hermite node values.
   */
  NodesObserver(NodeSubjectPtr node_values);
  virtual ~NodesObserver() = default;

  /**
   * @brief Callback method called every time the subject changes.
   */
  virtual void UpdateNodes() = 0;

protected:
  NodeSubjectPtr node_values_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_ */
