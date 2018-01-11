/**
 @file    phase_nodes1.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#ifndef TOWR_VARIABLES_PHASE_NODES_H_
#define TOWR_VARIABLES_PHASE_NODES_H_

#include <string>
#include <vector>

#include <ifopt/composite.h>

#include "node_values.h"


namespace towr {

class PhaseNodes : public NodeValues {
protected:
  using ContactVector = std::vector<bool>;
  using Ptr = std::shared_ptr<PhaseNodes>;

  enum Type {Force, Motion};
  PhaseNodes (int n_dim,
              int phase_count,
              bool is_in_contact_at_start,
              const std::string& name,
              int n_polys_in_changing_phase,
              Type type);
  virtual ~PhaseNodes() = default;

public:

  Eigen::Vector3d GetValueAtStartOfPhase(int phase) const;
  int GetNodeIDAtStartOfPhase(int phase) const;

  virtual std::vector<NodeInfo> GetNodeInfoAtOptIndex(int idx) const override;



protected:
  bool IsConstantNode(int node_id) const;

  int GetPolyIDAtStartOfPhase(int phase) const;


private:
  PolyInfoVec BuildPolyInfos(int phase_count,
                             bool is_in_contact_at_start,
                             int n_polys_in_changing_phase,
                             Type type) const;


  void SetNodeMappings();
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;

  // this could be removed i feel like
  // maps from the nodes that are actually optimized over
  // to all the nodes. Optimized nodes are sometimes used
  // twice in a constant phase.
  std::map<OptNodeIs, NodeIds > optnode_to_node_; // lookup

};


class EEMotionNodes : public PhaseNodes {
public:
  using Ptr = std::shared_ptr<EEMotionNodes>;

  EEMotionNodes (int phase_count,
                 bool is_in_contact_at_start,
                 const std::string& name,
                 int n_polys_in_changing_phase);
  virtual ~EEMotionNodes() = default;

  bool IsContactNode(int node_id) const;

  virtual VecBound GetBounds() const override;
};


class EEForceNodes : public PhaseNodes {
public:
  using Ptr = std::shared_ptr<EEForceNodes>;

  EEForceNodes (int phase_count,
                bool is_in_contact_at_start,
                const std::string& name,
                int n_polys_in_changing_phase);
  virtual ~EEForceNodes() = default;

  bool IsStanceNode(int node_id) const;

  int GetPhase(int node_id) const;

  virtual VecBound GetBounds() const override;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_PHASE_NODES_H_ */
