/*
 * base_nodes.h
 *
 *  Created on: Jan 12, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_

#include "node_variables.h"

namespace towr {


class BaseNodes : public NodeVariables {
public:
  BaseNodes (int n_dim, int n_nodes, const std::string& name);
  virtual ~BaseNodes () = default;

  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const override;

  // these should probably go into phase class
  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const override;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const override;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const override;
  virtual bool IsInConstantPhase(int polynomial_id) const override;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_ */
