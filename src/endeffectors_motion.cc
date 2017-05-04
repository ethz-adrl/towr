/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/endeffectors_motion.h>

#include <iostream>
#include <vector>
#include <Eigen/Sparse>

namespace xpp {
namespace opt {

EndeffectorsMotion::EndeffectorsMotion (const EndeffectorsPos& initial_pos,
                                        const ContactSchedule& contact_schedule)
    :Composite("endeffectors_motion", true)
{
  auto ee = EndeffectorsMotion::BuildEndeffectors(initial_pos, contact_schedule);
  AddEndffectors(ee);
  ee_ordered_ = initial_pos.GetEEsOrdered();
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
}

EndeffectorsMotion::ComponentVec
EndeffectorsMotion::BuildEndeffectors (const EndeffectorsPos& initial_pos,
                                       const ContactSchedule& contact_schedule)
{
  ComponentVec endeffectors;

  for (auto ee : initial_pos.GetEEsOrdered()) {
    auto endeffector = std::make_shared<EEMotion>();
    endeffector->SetInitialPos(initial_pos.At(ee), ee);

    for (auto phase : contact_schedule.GetPhases(ee)) {
      auto is_contact = phase.first;
      auto duration   = phase.second;

      double lift_height = is_contact? 0.0 : 0.03; // 0.3
      endeffector->AddPhase(duration, lift_height, is_contact);
    }
    endeffectors.push_back(endeffector);
  }

  return endeffectors;
}

void
EndeffectorsMotion::AddEndffectors (const ComponentVec& endeffectors)
{
  for (auto& ee : endeffectors) {
    AddComponent(ee);
    // retain derived class pointer to access ee specific functions
    endeffectors_.push_back(ee);
  }
}

EndeffectorsMotion::EEState
EndeffectorsMotion::GetEndeffectors (double t_global) const
{
  EEState ee_state(GetNumberOfEndeffectors());

  for (auto ee : ee_ordered_)
    ee_state.At(ee) = endeffectors_.at(ee)->GetState(t_global);

  return ee_state;
}

int
EndeffectorsMotion::GetNumberOfEndeffectors () const
{
  return GetComponentCount();
}

JacobianRow
EndeffectorsMotion::GetJacobianPos (double t_global,
                                    EndeffectorID ee,
                                    d2::Coords dim) const
{
  JacobianRow jac_row(GetRows());
  JacobianRow jac_ee = endeffectors_.at(ee)->GetJacobianPos(t_global, dim);

  // insert single ee-Jacobian into Jacobian representing all endeffectors
  for (JacobianRow::InnerIterator it(jac_ee); it; ++it)
    jac_row.coeffRef(IndexStart(ee)+it.col()) = it.value();

  return jac_row;
}

int
EndeffectorsMotion::IndexStart (EndeffectorID ee) const
{
  int idx = 0;

  for (int e=E0; e<ee; ++e)
    idx += endeffectors_.at(static_cast<EndeffectorID>(e))->GetRows();

  return idx;
}



} /* namespace opt */
} /* namespace xpp */
