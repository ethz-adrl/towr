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

#include "node_values.h"

namespace towr {

class PhaseNodes : public NodeValues {
public:
  using Ptr = std::shared_ptr<PhaseNodes>;
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;


  enum Type {Force, Motion};
  PhaseNodes (int phase_count,
              bool is_in_contact_at_start,
              const std::string& name,
              int n_polys_in_changing_phase,
              Type type);

  virtual ~PhaseNodes() = default;



  Eigen::Vector3d GetValueAtStartOfPhase(int phase) const;
  int GetNodeIDAtStartOfPhase(int phase) const;

  // because one node soemtimes represents two
  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const override;
  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const override;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const override;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const override;

  int GetPhase(int node_id) const;

  // node is considered constant if either left or right polynomial
  // belongs to a constant phase.
  bool IsConstantNode(int node_id) const;
  // those that are not fixed by bounds
  NodeIds GetIndicesOfNonConstantNodes() const;


private:
  int GetPolyIDAtStartOfPhase(int phase) const;

  struct PolyInfo {
    int phase_;
    int poly_id_in_phase_;
    int num_polys_in_phase_;
    bool is_constant_;
    PolyInfo(int phase, int poly_id_in_phase, int n_polys_in_phase, bool is_constant);
  };
  std::vector<PolyInfo> polynomial_info_;
  static std::vector<PolyInfo> BuildPolyInfos(int phase_count,
                             bool is_in_contact_at_start,
                             int n_polys_in_changing_phase,
                             Type type);

  static std::map<OptNodeIs, NodeIds> SetNodeMappings(const std::vector<PolyInfo>&);
  std::vector<int> GetAdjacentPolyIds(int node_id) const;


  // maps from the nodes that are actually optimized over
  // to all the nodes. Optimized nodes are sometimes used
  // twice in a constant phase.
  std::map<OptNodeIs, NodeIds > optnode_to_node_; // lookup

  void SetBoundsEEMotion();
  void SetBoundsEEForce();

};
} /* namespace towr */

#endif /* TOWR_VARIABLES_PHASE_NODES_H_ */
