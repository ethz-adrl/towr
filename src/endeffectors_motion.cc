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
    :Component(1, "endeffectors_motion")
{
  // spring_clean_ replace by AddComponent(..)
  for (int i=0; i<initial_pos.GetCount(); ++i)
    endeffectors_.push_back(std::make_shared<EEMotion>());

  ee_ordered_ = initial_pos.GetEEsOrdered();

  SetInitialPos(initial_pos);
  SetParameterStructure(contact_schedule);

  // spring_clean_ this should be taken care by Composite class
  int count = 0;
  for (const auto& ee : endeffectors_)
    count += ee->GetRows();
  SetRows(count);
}

void
EndeffectorsMotion::SetInitialPos (const EndeffectorsPos& initial_pos)
{
  for (auto ee : initial_pos.GetEEsOrdered())
    endeffectors_.at(ee)->SetInitialPos(initial_pos.At(ee), ee);
}

void
EndeffectorsMotion::SetParameterStructure (const ContactSchedule& contact_schedule)
{
  for (auto ee : ee_ordered_) {

    for (auto phase : contact_schedule.GetPhases(ee)) {
      auto is_contact = phase.first;
      auto duration   = phase.second;

      double lift_height = is_contact? 0.0 : 0.03;
      endeffectors_.at(ee)->AddPhase(duration, lift_height, is_contact);
    }
  }
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
}

EndeffectorsMotion::EEState
EndeffectorsMotion::GetEndeffectors (double t_global) const
{
  EEState ee_state(GetNumberOfEndeffectors());

  for (auto ee : ee_ordered_)
    ee_state.At(ee) = endeffectors_.at(ee)->GetState(t_global);

  return ee_state;
}

EndeffectorsPos
EndeffectorsMotion::GetEndeffectorsPos (double t_global) const
{
  // zmp_ DRY with above -.-
  EndeffectorsPos pos(GetNumberOfEndeffectors());

  for (auto ee : ee_ordered_)
    pos.At(ee) = endeffectors_.at(ee)->GetState(t_global).p;

  return pos;
}

EndeffectorsMotion::VectorXd
EndeffectorsMotion::GetValues () const
{
  VectorXd x = VectorXd::Zero(GetRows());

  int row = 0;
  for (const auto& ee : endeffectors_) {
    int n = ee->GetRows();
    x.middleRows(row, n) = ee->GetValues();
    row += n;
  }

  return x;
}

// must be analog to the above
void
EndeffectorsMotion::SetValues (const VectorXd& x)
{
  int row = 0;

  for (auto ee : ee_ordered_) {
    int n = endeffectors_.at(ee)->GetRows();
    endeffectors_.at(ee)->SetValues(x.middleRows(row, n));
    row += n;
  }
}

int
EndeffectorsMotion::GetNumberOfEndeffectors () const
{
  return endeffectors_.size();
}

JacobianRow
EndeffectorsMotion::GetJacobianWrtOptParams (double t_global,
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

EndeffectorsMotion::EEState::Container
EndeffectorsMotion::GetEndeffectorsVec (double t_global) const
{
  return GetEndeffectors(t_global).ToImpl();
}

} /* namespace opt */
} /* namespace xpp */
