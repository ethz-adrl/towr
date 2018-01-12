/*
 * SplineObserver.h
 *
 *  Created on: Jan 9, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_

#include <memory>


namespace towr {


class NodeVariables;

class NodesObserver {
public:
  using SubjectPtr = NodeVariables*;

  NodesObserver(SubjectPtr node_values);

  virtual ~NodesObserver() = default;

  virtual void UpdatePolynomials() = 0;

protected:
  SubjectPtr node_values_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODES_OBSERVER_H_ */
