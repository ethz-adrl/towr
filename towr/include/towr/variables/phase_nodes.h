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
public:
  using Ptr = std::shared_ptr<PhaseNodes>;


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
  virtual std::vector<NodeInfo> GetNodeInfoAtOptIndex(int idx) const override;
  virtual VecDurations ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const override;
  virtual double GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const override;
  virtual int GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const override;

  int GetPhase(int node_id) const;

  // smell invert this, because doing it everywhere anyway
  bool IsConstantNode(int node_id) const;


private:
  int GetPolyIDAtStartOfPhase(int phase) const;

  struct PolyInfo {
    PolyInfo() {};
    PolyInfo(int phase, int poly_id_in_phase,
             int num_polys_in_phase, bool is_constant)
        :phase_(phase), poly_id_in_phase_(poly_id_in_phase),
         num_polys_in_phase_(num_polys_in_phase), is_constant_(is_constant) {}

    int phase_;
    int poly_id_in_phase_;
    int num_polys_in_phase_;
    bool is_constant_; // zmp_ this shouldn't be here, has to do with phases
  };
  std::vector<PolyInfo> polynomial_info_;
  std::vector<PolyInfo> BuildPolyInfos(int phase_count,
                             bool is_in_contact_at_start,
                             int n_polys_in_changing_phase,
                             Type type) const;

  void SetBoundsEEMotion();
  void SetBoundsEEForce();

  void SetNodeMappings();
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;

  // maps from the nodes that are actually optimized over
  // to all the nodes. Optimized nodes are sometimes used
  // twice in a constant phase.
  std::map<OptNodeIs, NodeIds > optnode_to_node_; // lookup


};
} /* namespace towr */

#endif /* TOWR_VARIABLES_PHASE_NODES_H_ */
