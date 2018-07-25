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

#ifndef TOWR_VARIABLES_NODE_VALUES_H_
#define TOWR_VARIABLES_NODE_VALUES_H_

#include <ifopt/variable_set.h>

#include "state.h"
#include "nodes_observer.h"

namespace towr {

/**
 * @defgroup Variables
 * @brief Variables of the trajectory optimization problem.
 *
 * These are the quantities through which the optimization problem is
 * parameterized.
 *
 * Folder: @ref include/towr/variables.
 */

/**
 * @brief Position and velocity of nodes used to generate a Hermite spline.
 *
 * #### Four nodes defining a single spline (e.g. foot position in x-direction)
 * \image html nodes.jpg
 *
 * Instead of setting the polynomial coefficients directly, a third-order
 * polynomial is also fully defined by the value and first-derivative of the
 * start and end of the polynomial as well as the duration. This class holds
 * these node values composed of position and velocity.
 *
 * In the above image the nodes are defined by the scalar position and velocity
 * values x0, x0d, ..., xT, xTd. By optimizing over these nodes, different
 * spline shapes are generated. It is important to note that **not all node
 * values must be optimized over**. We can fix specific node values in advance, or
 * _one_ optimization variables can represent _multiple_ nodes values in the
 * spline. This is exploited in the subclass NodesVariablesPhaseBased using
 * _Phase-based End-effector Parameterization_.
 *
 * @ingroup Variables
 */
class NodesVariables : public ifopt::VariableSet {
public:
  using Ptr          = std::shared_ptr<NodesVariables>;
  using VecDurations = std::vector<double>;
  using ObserverPtr  = NodesObserver*;

  /**
   * @brief Semantic information associated with a scalar node value.
   *
   * This includes all information except the actual value. This comes from
   * the vector of optimization variables.
   *
   * @sa GetNodeValuesInfo()
   */
  struct NodeValueInfo {
    int id_;   ///< ID of the associated node (0 =< id < number of nodes in spline).
    Dx deriv_; ///< Derivative (pos,vel) of the node with that ID.
    int dim_;  ///< Dimension (x,y,z) of that derivative.

    NodeValueInfo() = default;
    NodeValueInfo(int node_id, Dx deriv, int node_dim);
    int operator==(const NodeValueInfo& right) const;
  };

  /**
   * @brief Node values affected by one specific optimization variable.
   * @param opt_idx  The index (=row) of the optimization variable.
   * @return All node values affected by this optimization variable.
   *
   * This function determines which node values are optimized over, and which
   * nodes values are set by the same optimization variable.
   *
   * Reverse of GetOptIndex().
   */
  virtual std::vector<NodeValueInfo> GetNodeValuesInfo(int opt_idx) const = 0;

  /**
   * @brief Index in the optimization vector for a specific nodes' pos/vel.
   * @param nvi Description of node value we want to know the index for.
   * @return The position of this node value in the optimization variables.
   *
   * Reverse of GetNodeInfoAtOptIndex().
   */
  int GetOptIndex(const NodeValueInfo& nvi) const;
  static const int NodeValueNotOptimized = -1;

  /**
   * @brief Pure optimization variables that define the nodes.
   *
   * Not all node position and velocities are independent or optimized over, so
   * usually the number of optimization variables is less than all nodes' pos/vel.
   *
   * @sa GetNodeInfoAtOptIndex()
   */
  VectorXd GetValues () const override;

  /**
   * @brief Sets some node positions and velocity from the optimization variables.
   * @param x The optimization variables.
   *
   * Not all node position and velocities are independent or optimized over, so
   * usually the number of optimization variables is less than
   * all nodes pos/vel.
   *
   * @sa GetNodeValuesInfo()
   */
  void SetVariables (const VectorXd&x) override;

  /**
   * @returns the bounds on position and velocity of each node and dimension.
   */
  VecBound GetBounds () const override;

  /**
   * @returns All the nodes that can be used to reconstruct the spline.
   */
  const std::vector<Node> GetNodes() const;

  /**
   * @returns the number of polynomials that can be built with these nodes.
   */
  int GetPolynomialCount() const;

  /**
   * @returns the two nodes that make up polynomial with "poly_id".
   */
  const std::vector<Node> GetBoundaryNodes(int poly_id) const;

  enum Side {Start=0, End};
  /**
   * @brief The node ID that belongs to a specific side of a specific polynomial.
   * @param poly_id The ID of the polynomial within the spline.
   * @param side The side from which the node ID is required.
   */
  static int GetNodeId(int poly_id, Side side);

  /**
   * @brief Adds a dependent observer that gets notified when the nodes change.
   * @param spline Usually a pointer to a spline which uses the node values.
   */
  void AddObserver(ObserverPtr const spline);

  /**
   * @returns  The dimensions (x,y,z) of every node.
   */
  int GetDim() const;

  /**
   * @brief Sets nodes pos/vel equally spaced from initial to final position.
   * @param initial_val  value of the first node.
   * @param final_val  value of the final node.
   * @param t_total  The total duration to reach final node (to set velocities).
   */
  void SetByLinearInterpolation(const VectorXd& initial_val,
                                const VectorXd& final_val,
                                double t_total);

  /**
   * @brief Restricts the first node in the spline.
   * @param deriv Which derivative (pos,vel,...) should be restricted.
   * @param dimensions Which dimensions (x,y,z) should be restricted.
   * @param val The values the fist node should be set to.
   */
  void AddStartBound (Dx deriv, const std::vector<int>& dimensions,
                      const VectorXd& val);

  /**
   * @brief Restricts the last node in the spline.
   * @param deriv Which derivative (pos,vel,...) should be restricted.
   * @param dimensions Which dimensions (x,y,z) should be restricted.
   * @param val The values the last node should be set to.
   */
  void AddFinalBound(Dx deriv, const std::vector<int>& dimensions,
                     const VectorXd& val);

protected:
  /**
   * @param n_dim  The number of dimensions (x,y,..) each node has.
   * @param variable_name  The name of the variables in the optimization problem.
   */
  NodesVariables (const std::string& variable_name);
  virtual ~NodesVariables () = default;

  VecBound bounds_; ///< the bounds on the node values.
  std::vector<Node> nodes_;
  int n_dim_;

private:
  /**
   * @brief Notifies the subscribed observers that the node values changes.
   */
  void UpdateObservers() const;
  std::vector<ObserverPtr> observers_;

  /**
   * @brief Bounds a specific node variables.
   * @param node_id  The ID of the node to bound.
   * @param deriv    The derivative of the node to set.
   * @param dim      The dimension of the node to bound.
   * @param values   The values to set the bounds to.
   */
  void AddBounds(int node_id, Dx deriv, const std::vector<int>& dim,
                 const VectorXd& values);
  /**
   * @brief Restricts a specific optimization variables.
   * @param node_info The specs of the optimization variables to restrict.
   * @param value     The value to set the bounds to.
   */
  void AddBound(const NodeValueInfo& node_info, double value);
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
