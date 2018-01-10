/*
 * SplineObserver.h
 *
 *  Created on: Jan 9, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODE_OBSERVER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODE_OBSERVER_H_

#include <memory>

namespace towr {

class NodeObserver {
public:
  using Ptr = std::shared_ptr<NodeObserver>;

  NodeObserver() = default;
  virtual ~NodeObserver() = default;

  virtual void UpdatePolynomials() = 0;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_NODE_OBSERVER_H_ */
