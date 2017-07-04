/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/variables/endeffectors_motion.h>

#include <Eigen/Sparse>
#include <string>

#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

// zmp_ this class seems unneccessary
EndeffectorsMotion::EndeffectorsMotion (const EndeffectorsPos& initial_pos,
                                        const ContactSchedule& contact_schedule)
    :Composite(id::endeffectors_motion, true)
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
      bool is_contact = phase.first;
      double duration = phase.second;

      double lift_height = is_contact? 0.0 : 0.04; // 0.03

//      // zmp_ hack to nicely show limping
//      if (!is_contact && (ee == E2  /*|| ee == E3*/  )) // LH=E0, RF=E3
//        lift_height += 0.1;

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
  // zmp_ this could be avoided, by adding each endeffector separately
  for (JacobianRow::InnerIterator it(jac_ee); it; ++it)
    jac_row.coeffRef(IndexStart(ee)+it.col()) = it.value();

  return jac_row;
}

int
EndeffectorsMotion::IndexStart (EndeffectorID ee) const
{
  int idx = 0;

  for (int i=E0; i<ee; ++i)
    idx += endeffectors_.at(i)->GetRows();

  return idx;
}



} /* namespace opt */
} /* namespace xpp */
