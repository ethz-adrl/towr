/**
 @file    hyq_motion_params.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#include <xpp/opt/quadruped_motion_parameters.h>

#include <initializer_list>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <xpp/endeffectors4.h>

namespace xpp {
namespace opt {
namespace quad{

QuadrupedMotionParameters::QuadrupedMotionParameters ()
{
  duration_polynomial_ = 0.05; //s
  load_dt_ = 0.01;
//  offset_geom_to_com_ << -0.02230, -0.00010, 0.03870;
//  offset_geom_to_com_ << -0.03, 0.02, 0.0;
  offset_geom_to_com_ << 0,0,0;
  max_dev_xy_ = {0.1, 0.1, 0.2};
  robot_ee_ = { EEID::E0, EEID::E1, EEID::E2, EEID::E3 };


  constraints_ = { InitCom,
                   FinalCom,
                   JunctionCom,
                   Dynamic,
                   Stance,
                   RomBox, // usually enforced as soft-constraint/cost
  };

//  cost_weights_[RangOfMotionCostID] = 1.0;
//  cost_weights_[ComCostID]          = 1.0;

  const double x_nominal_b = 0.28; //34
  const double y_nominal_b = 0.28;
  const double z_nominal_b = -0.58;
  nominal_stance_.SetCount(robot_ee_.size());
  nominal_stance_.At(kMapQuadToOpt.at(LF)) = PosXYZ( x_nominal_b,   y_nominal_b, z_nominal_b);
  nominal_stance_.At(kMapQuadToOpt.at(RF)) = PosXYZ( x_nominal_b,  -y_nominal_b, z_nominal_b);
  nominal_stance_.At(kMapQuadToOpt.at(LH)) = PosXYZ(-x_nominal_b,   y_nominal_b, z_nominal_b);
  nominal_stance_.At(kMapQuadToOpt.at(RH)) = PosXYZ(-x_nominal_b,  -y_nominal_b, z_nominal_b);

  II_.SetCount(robot_ee_.size()); II_.SetAll(false);
  PI_.SetCount(robot_ee_.size()); PI_.SetAll(false);
  bI_.SetCount(robot_ee_.size()); bI_.SetAll(false);
  IP_.SetCount(robot_ee_.size()); IP_.SetAll(false);
  Ib_.SetCount(robot_ee_.size()); Ib_.SetAll(false);
  // 2 swinglegs
  Pb_.SetCount(robot_ee_.size()); Pb_.SetAll(false);
  bP_.SetCount(robot_ee_.size()); bP_.SetAll(false);
  BI_.SetCount(robot_ee_.size()); BI_.SetAll(false);
  IB_.SetCount(robot_ee_.size()); IB_.SetAll(false);
  PP_.SetCount(robot_ee_.size()); PP_.SetAll(false);
  bb_.SetCount(robot_ee_.size()); bb_.SetAll(false);
  // 3 swinglegs
  Bb_.SetCount(robot_ee_.size()); Bb_.SetAll(false);
  BP_.SetCount(robot_ee_.size()); BP_.SetAll(false);
  bB_.SetCount(robot_ee_.size()); bB_.SetAll(false);
  PB_.SetCount(robot_ee_.size()); PB_.SetAll(false);
  // flight-phase
  BB_.SetCount(robot_ee_.size()); BB_.SetAll(true);

  PI_.At(kMapQuadToOpt.at(LH)) = true;
  bI_.At(kMapQuadToOpt.at(RH)) = true;
  IP_.At(kMapQuadToOpt.at(LF)) = true;
  Ib_.At(kMapQuadToOpt.at(RF)) = true;
  // two swinglegs
  Pb_.At(kMapQuadToOpt.at(LH)) = true; Pb_.At(kMapQuadToOpt.at(RF)) = true;
  bP_.At(kMapQuadToOpt.at(RH)) = true; bP_.At(kMapQuadToOpt.at(LF)) = true;
  BI_.At(kMapQuadToOpt.at(LH)) = true; BI_.At(kMapQuadToOpt.at(RH)) = true;
  IB_.At(kMapQuadToOpt.at(LF)) = true; IB_.At(kMapQuadToOpt.at(RF)) = true;
  PP_.At(kMapQuadToOpt.at(LH)) = true; PP_.At(kMapQuadToOpt.at(LF)) = true;
  bb_.At(kMapQuadToOpt.at(RH)) = true; bb_.At(kMapQuadToOpt.at(RF)) = true;
  // three swinglegs
  Bb_.At(kMapQuadToOpt.at(LH)) = true; Bb_.At(kMapQuadToOpt.at(RH)) = true;  Bb_.At(kMapQuadToOpt.at(RF))= true;
  BP_.At(kMapQuadToOpt.at(LH)) = true; BP_.At(kMapQuadToOpt.at(RH)) = true;  BP_.At(kMapQuadToOpt.at(LF))= true;
  bB_.At(kMapQuadToOpt.at(RH)) = true; bB_.At(kMapQuadToOpt.at(LF)) = true;  bB_.At(kMapQuadToOpt.at(RF))= true;
  PB_.At(kMapQuadToOpt.at(LH)) = true; PB_.At(kMapQuadToOpt.at(LF)) = true;  PB_.At(kMapQuadToOpt.at(RF))= true;
}

QuadrupedMotionParameters::MotionTypePtr
QuadrupedMotionParameters::MakeMotion (opt::MotionTypeID id)
{
  switch (id) {
    case WalkID:
      return std::make_shared<Walk>();
    case TrotID:
      return std::make_shared<Trot>();
    case BoundID:
      return std::make_shared<Bound>();
    case PaceID:
      return std::make_shared<Pace>();
    case PushRecID:
      return std::make_shared<PushRecovery>();
    default:
      throw std::runtime_error("MotionTypeID not defined");
  }
}

Walk::Walk()
{
//  max_dev_xy_ = {0.15, 0.15, 0.1};
  id_ = opt::WalkID;

  double t_phase = 0.3;
  double t_trans = 0.1;
  contact_timings_ =
  {
      0.3,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      0.2,
  };
  contact_sequence_ =
  {
      II_,
      PI_, PP_, IP_, bI_, bb_, Ib_,
      PI_, PP_, IP_, bI_, bb_, Ib_,
//      PI_, PP_, IP_, bI_, bb_, Ib_,
      II_,
  };


//  double t_step = 0.4;
//  contact_timings_ =
//  {
//      0.4,
//      t_step, t_step,t_step,t_step,
//      t_step, t_step,t_step,t_step,
//      0.2,
//  };
//
//  contact_sequence_ =
//  {
//      II_,
//      PI_, IP_, bI_, Ib_,
//      PI_, IP_, bI_, Ib_,
//      II_,
//  };


//  constraints_ = { InitCom,
//                   FinalCom,
//                   JunctionCom,
//                   Dynamic,
////                   RomBox,
//                   Stance,
//  };
//
//
//  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[PolyCenterCostID]   = 1.0;
//  cost_weights_[FinalComCostID] = 1000.0;
}

Trot::Trot()
{
//  max_dev_xy_ = {0.15, 0.15, 0.1};
  id_ = opt::TrotID;

  double t_phase = 0.3;
  double t_trans = 0.1;

  contact_timings_ =
  {   0.3,
      t_phase, t_phase, t_phase, t_phase, // trot
      0.5, // flight_phase
//      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase, // walk
      t_phase, t_phase, t_phase, t_phase, // trot
      0.2
  };

  contact_sequence_ =
  {
      II_,
      bP_, Pb_, bP_, Pb_, // trot
      BB_, // flight-phase
//      PI_, PP_, IP_, bI_, bb_, Ib_, // walk
      bP_, Pb_, bP_, Pb_, // trot
      II_
  };


//  constraints_ = {
//                   InitCom,
//                   FinalCom,
//                   JunctionCom,
//                   Dynamic,
////                   RomBox,
//                   Stance,
//                   };
//
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[ComCostID]      = 1.0;

//  cost_weights_[FinalComCostID] = 1.0;
//  cost_weights_[PolyCenterCostID]   = 50.0;
}

Pace::Pace()
{
//  max_dev_xy_ = {0.20, 0.20, 0.1};
  id_ = opt::PaceID;

  contact_timings_ =
  {
      0.3,
      0.3, 0.3, 0.3, 0.3,
      0.3
  };
  contact_sequence_ =
  {
      II_,
      PP_, bb_, PP_, bb_, // pace
      II_,
  };

//  constraints_ = { InitCom,
//                   FinalCom,
//                   JunctionCom,
//                   Dynamic,
////                   RomBox,
//                   Stance
//  };
//
  constraints_.push_back(RomBox);
  cost_weights_.clear();
//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[ComCostID]          = 1.0;

//  cost_weights_[PolyCenterCostID]   = 0.0;
}

Bound::Bound()
{
  max_dev_xy_ = {0.25, 0.21, 0.01};
  id_ = opt::BoundID;

  contact_timings_ =
  {
      0.8,
      0.4, 0.3, 0.4, 0.3,
      0.4, 0.3, 0.4, 0.3,
      0.4, 0.3, 0.4, 0.3,
      0.3
  };
  /*
  // sequence for normal bound
  contact_sequence_ =
  {
      II_,
      BI_, IB_, BI_, IB_, // bound
      II_
  };





    // sequence 3-1
  contact_sequence_ =
  {
      II_,
      BP_, Ib_, BP_, Ib_,
      BP_, Ib_, BP_, Ib_,
      BP_, Ib_, BP_, Ib_,
      II_
  };


  // sequence for limping, keeping left hind up all the time
  contact_sequence_ =
  {
      II_,
      Bb_, bP_, Bb_, bP_, // right hind leg stationary
      Bb_, bP_, Bb_, bP_, // right hind leg stationary
      Bb_, bP_, Bb_, bP_, // right hind leg stationary
      II_
  };
  */

  // biped walk
  contact_sequence_ =
  {
      II_,
      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
      Bb_, PB_, Bb_, PB_, // left hind and right front stationary
      II_
  };




  constraints_.push_back(RomBox);
  cost_weights_.clear();

//  cost_weights_[RangOfMotionCostID] = 10.0;
//  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

PushRecovery::PushRecovery ()
{
//  max_dev_xy_ = {0.15, 0.15, 0.1};
  id_ = opt::PushRecID;

  SetContactSequence(0.0, 0.0);

  constraints_.push_back(RomBox);
  cost_weights_.clear();
//  cost_weights_[ComCostID] = 1.0;

//  constraints_ = { InitCom,
//                   Stance,
//                   JunctionCom,
//                   Dynamic,
//  };

//  cost_weights_[ComCostID]          = 1.0;
//  cost_weights_[RangOfMotionCostID] = 100.0;
//  cost_weights_[FinalComCostID] = 1000.0;
//  cost_weights_[PolyCenterCostID]   = 0.0;
}

void
QuadrupedMotionParameters::SetContactSequence (double p_overlap_lateral,
                                               double p_overlap_opposite)
{
  // distribution of swing motion to different phases
  double p_single_leg  = 1.0-p_overlap_lateral-p_overlap_opposite;

  double t = 0.3;
  double ts = t*p_single_leg;
  double tl = t*p_overlap_lateral;
  double to = t*p_overlap_opposite;


  ContactSequence lateral  = {PI_, PP_, IP_, bP_, bI_, bb_, Ib_, Pb_};
  ContactSequence circular = {PI_, PP_, IP_, IB_, Ib_, bb_, bI_, BI_};
  ContactTimings  timings = { ts,  tl,  ts,  to,  ts,  tl,  ts,  to};


  ContactSequence sequence = lateral;

  contact_timings_.clear();
  contact_sequence_.clear();

  // beginning of motion
  contact_timings_.push_back(0.3);
  contact_sequence_.push_back(II_);

  contact_timings_.push_back(to);
  contact_sequence_.push_back(PI_);

  // during the motion
  double sign = +1;
  for (int i=0; i<12; ++i) {

    for (int j=0; j<sequence.size(); ++j) {

      double duration = timings.at(j);
      if (duration > 1e-10) { // skip phases with zero duration
        contact_sequence_.push_back(sequence.at(j));
        contact_timings_.push_back(duration);
      }


      //    contact_sequence_.insert(contact_sequence_.end(), sequence.begin(), sequence.end());
      //    contact_timings_.insert(contact_timings_.end(), timings.begin(), timings.end());

      // adapt timings

//      p_overlap_lateral+= sign*0.01;

//      double r1 = ((double) rand() / (RAND_MAX));
//      double r2 = ((double) rand() / (RAND_MAX));

//      p_overlap_opposite = r1/0.5 + 0.25;
//      p_overlap_lateral  = r2/(1-p_overlap_opposite);

      int n_overlaps = 1;

      p_overlap_opposite+= sign*0.025/n_overlaps;
//      p_overlap_lateral += sign*0.025/n_overlaps;

      if (p_overlap_opposite > 1.3/n_overlaps) {
        p_overlap_opposite = 1.3/n_overlaps;
//        p_overlap_lateral = 1.3/n_overlaps;
        sign = -1;
      }


      p_single_leg  = 1.0-p_overlap_lateral-p_overlap_opposite;

      ts = t*p_single_leg;
      tl = t*p_overlap_lateral;
      to = t*p_overlap_opposite;
      timings = { ts,  tl,  ts,  to,  ts,  tl,  ts,  to};
    }
  }

  // end of motion
  // remove last transition
  contact_timings_.pop_back();
  contact_sequence_.pop_back();

  contact_timings_.push_back(0.4);
  contact_sequence_.push_back(sequence.end()[-2]);

  contact_timings_.push_back(0.6);
  contact_sequence_.push_back(II_);

//  contact_sequence_ =
//  {
//      II_,
//      PI_,
//      //
//      PI_, PP_, IP_, // left side
//      bP_,           // transition left-right
//      bI_, bb_, Ib_, // right side
//      Pb_,           // transition right-left
//      //
//      PI_, PP_, IP_, // left side
//      bP_,           // transition left-right
//      bI_, bb_, Ib_, // right side
//      Pb_,           // transition right-left
//      //
//      PI_, PP_, IP_, // left side
//      bP_,           // transition left-right
//      bI_, bb_, Ib_, // right side
//      //
//      Ib_,
//      II_
//  };
}

} // namespace quad
} // namespace opt
} // namespace xpp

