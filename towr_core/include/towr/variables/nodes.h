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
 * @brief Position and velocity of nodes used to generate a Hermite spline.
 *
 * Instead of setting the polynomial coefficients directly, a third-order
 * polynomial is also fully defined by the value and first-derivative of the
 * start and end of the polynomial as well as the duration. This way of
 * specifying a polynomial is called "Hermite". These are the node values
 * of position and velocity.
 *
 * @sa class CubicHermitePolynomial
 */
class Nodes : public ifopt::VariableSet {
public:
  using Ptr          = std::shared_ptr<Nodes>;
  using VecDurations = std::vector<double>;
  using ObserverPtr  = NodesObserver*;

  /**
   * @brief Holds information about the node the optimization index represents.
   */
  struct IndexInfo {
    int node_id_;   ///< The ID of the node of the optimization index.
    Dx node_deriv_; ///< The derivative (pos,vel) of that optimziation index.
    int node_dim_;  ///< the dimension (x,y,z) of that optimization index.

    IndexInfo() = default;
    IndexInfo(int node_id, Dx deriv, int node_dim);
    int operator==(const IndexInfo& right) const;
  };

  /**
   * @brief The node information that the optimization index represents.
   * @param idx  The index (=row) of the node optimization variable.
   * @return Semantic information on what that optimization variables means.
   *
   * One optimization variables can also represent multiple nodes in the spline
   * if they are always equal (e.g. constant phases). This is why this function
   * returns a vector.
   */
  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const = 0;

  /**
   * @brief The index at which a specific node variable is stored.
   * @param node_info The node variable we want to know the index for.
   * @return The position of this node value in the optimization variables.
   */
  int Index(const IndexInfo& node_info) const;
  int Index(int node_id, Dx deriv, int node_dim) const;

  /**
   * @brief Sets nodes pos/vel equally spaced from initial to final position.
   * @param initial_pos  The position of the first node.
   * @param final_pos  The position of the final node.
   * @param t_total  The total duration to reach final node (to set velocities).
   */
  void InitializeNodesTowardsGoal(const VectorXd& initial_pos,
                                  const VectorXd& final_pos,
                                  double t_total);
  /**
   * @returns the stacked node position and velocity values.
   */
  VectorXd GetValues () const override;

  /**
   * @brief Sets the node position and velocity optimization variables.
   * @param x The stacked variables.
   */
  void SetVariables (const VectorXd&x) override;

  /**
   * @returns the bounds on position and velocity of each node and dimension.
   */
  virtual VecBound GetBounds () const override;

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
  Nodes (int n_dim, const std::string& variable_name);
  virtual ~Nodes () = default;

  VecBound bounds_; ///< the bounds on the node values.

  /**
   * @brief initializes the member variables.
   * @param n_nodes  The number of nodes composing the spline.
   * @param n_variables  The number of variables being optimized over.
   *
   * Not every node value must be optimized, so n_variables can be different
   * than 2*n_nodes*n_dim.
   */
  void InitMembers(int n_nodes, int n_variables);

private:
  std::vector<Node> nodes_;
  int n_dim_;

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
  void AddBound(const IndexInfo& node_info, double value);
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_NODE_VALUES_H_ */
