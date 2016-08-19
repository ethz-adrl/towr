/*
 * 	rewrite of original SL inverse kinematics functions
 *
 *  Created on: Jan, 2016
 *      Author: Manuel Lussi <mlussi@student.ethz.ch>
 */

#include <xpp/hyq/hyq_inverse_kinematics.h>
#include <xpp/hyq/leg_data_map.h>

namespace xpp {
namespace hyq {

#define BASE2HAAX  0.3735  //!< center of base distance from leg hip in x direction
#define BASE2HAAY  0.207   //!< center of base distance from leg hip in y direction
#define HFE2KFEZ  (-0.35)  //!< distance of HFE to KFE in z direction  [is this used? is this at HFE=0rad?]
#define HAA2HFEZ  (-0.08)  //!< distance of HFE to HAA in z direction
#define ZOFFSET    0.0

#define LUPPER      0.35    //!< length of upper leg
#define LLOWER      0.33    //!< length of lower leg to the midpoint of the foot

//just indexes!!!!
#define LF_LEG 0
#define RF_LEG 1
#define LH_LEG 2
#define RH_LEG 3

#define LF_HAA 0
#define LF_HFE 1
#define LF_KFE 2
#define DOF_PER_LEG 3

#define MIN_THETA_INDEX 0
#define MAX_THETA_INDEX 1

#define NR_OF_CONSTRAINTS 2

#define X 0
#define Y 1
#define Z 2

// real joint angle limits
const static double haa_min_real = -90;
const static double haa_max_real = +30;

const static double hfe_min_real = -50;
const static double hfe_max_real =  70;

const static double kfe_min_real = -140;
const static double kfe_max_real =  -20;

// reduced joint angles for optimization
// for left front leg
const double joint_range[DOF_PER_LEG][NR_OF_CONSTRAINTS] = {
 {haa_min_real/180.0*M_PI, haa_max_real/180.0*M_PI},
 {hfe_min_real/180.0*M_PI, hfe_max_real/180.0*M_PI},
 {kfe_min_real/180.0*M_PI, kfe_max_real/180.0*M_PI}
};

// these are for the left front leg
//static double haa_min = haa_min_real;
//static double haa_max = haa_max_real;
const static double haa_min = -20;
const static double haa_max = -10;

//static double hfe_min = hfe_min_real;
//static double hfe_max = hfe_max_real;
const static double hfe_min = 10;
const static double hfe_max = 50;

//static double kfe_min = kfe_min_real;
//static double kfe_max = kfe_max_real;
const static double kfe_min = -90 - 20;
const static double kfe_max = -60;



HyqInverseKinematics::HyqInverseKinematics ()
{
}

HyqInverseKinematics::~HyqInverseKinematics ()
{
}

HyqInverseKinematics::JointAngles
HyqInverseKinematics::GetJointAngles(const EEPosition& pos_b, size_t ee) const
{
  Eigen::Vector3d q;
  q.setZero();

  int error_code;
  bool successFlag = compute(ee, pos_b, q, error_code);

  if(!successFlag) {
    std::cout << "not successfull in calculating q. q: " << q.transpose() << std::endl;
//    throw  std::runtime_error(std::string("computing joint position resulted in error! err_code: " + std::to_string(error_code)) );
  }

  return q;
}

HyqInverseKinematics::JointAngles
HyqInverseKinematics::GetUpperJointLimits (size_t ee) const
{
  // make sure to follow structure of GetJointAngles(). In this case
  // HAA, HFE, KFE
  JointAngles q_max(3);
  switch (ee) {
    case LF:
      q_max << haa_max, hfe_max, kfe_max;
      break;
    case RF:
      q_max << haa_max, hfe_max, kfe_max;
      break;
    case LH:
      q_max << haa_max, -hfe_min, -kfe_min;
      break;
    case RH:
      q_max << haa_max, -hfe_min, -kfe_min;
      break;
    default:
      assert(false);
      break;
  }

  return q_max/180.0*M_PI; // convert to radians
}

HyqInverseKinematics::JointAngles
HyqInverseKinematics::GetLowerJointLimits (size_t ee) const
{
  // make sure to follow structure of GetJointAngles(). In this case
  // HAA, HFE, KFE
  JointAngles q_min(3);
  switch (ee) {
    case LF:
      q_min << haa_min, hfe_min, kfe_min;
      break;
    case RF:
      q_min << haa_min, hfe_min, kfe_min;
      break;
    case LH:
      q_min << haa_min, -hfe_max, -kfe_max;
      break;
    case RH:
      q_min << haa_min, -hfe_max, -kfe_max;
      break;
    default:
      assert(false);
      break;
  }

  return q_min/180.0*M_PI; // convert to radians
}

bool
HyqInverseKinematics::compute(size_t leg, const EEPosition& x, Eigen::Vector3d& q, int &rc) const
{
	int i;
	double q_HAA_bf, q_HAA_br, q_HFE_br, q_HFE_bf, q_KFE_br, q_KFE_bf, q_HAA_temp;
	double tmp1;
	double alpha,beta,gamma;
	double ll,lu;
	bool br_exceeds_joint_limit = false;
	bool bf_exceeds_joint_limit = false;

	Eigen::Vector3d xr;
	Eigen::Vector3d xrd;
	Eigen::Vector3d xrdd;
	Eigen::Vector3d temp;
	Eigen::Matrix3d R;

	rc = 0;

	// translate to the local coordinate of the attachment of the leg
	// and flip coordinate signs such that all computations can be done
	// for the front-left leg
	if (leg == LF_LEG ) {
		xr = x + Eigen::Vector3d(-BASE2HAAX,-BASE2HAAY,-ZOFFSET);
	}
	else if (leg == RF_LEG) {
		xr = x.cwiseProduct(Eigen::Vector3d(1,-1,1)) + Eigen::Vector3d(-BASE2HAAX,-BASE2HAAY,-ZOFFSET);
	}
	else if (leg == LH_LEG) {
		xr = x.cwiseProduct(Eigen::Vector3d(-1,1,1)) + Eigen::Vector3d(-BASE2HAAX,-BASE2HAAY,-ZOFFSET);
	}
	else if (leg == RH_LEG) {
		xr = x.cwiseProduct(Eigen::Vector3d(-1,-1,1)) + Eigen::Vector3d(-BASE2HAAX,-BASE2HAAY,-ZOFFSET);
	}
	else {
		rc = 1; // leg doesn't exist
		return false;
	}

	// compute the HAA angle
	q_HAA_bf = q_HAA_br = -atan2(xr[Y],-xr[Z]);

	// rotate into the HFE coordinate system (rot around X)
	R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA_bf), -sin(q_HAA_bf), 0.0, sin(q_HAA_bf), cos(q_HAA_bf);

	xr = (R * xr).eval();

	// translate into the HFE coordinate system (along Z axis)
	xr[Z] -= HAA2HFEZ;

	// compute square of length from HFE to foot
	tmp1 = pow(xr[X],2)+pow(xr[Z],2);


	// compute temporary angles (with reachability check)
	lu = LUPPER; // length of upper leg
	ll = LLOWER;  // length of lower leg
	alpha = atan2(-xr[Z],xr[X]) - 0.5*M_PI;  //  flip and rotate to match HyQ joint definition


	double some_random_value_for_beta = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1)); // this must be between -1 and 1
	if (some_random_value_for_beta > 1) {
		// printf("Error in  for HFE in %s\n",cart_names[leg]);
		rc = 7;
		some_random_value_for_beta = 1;
//		return false;
	}
  if (some_random_value_for_beta < -1) {
    // printf("Error in  for HFE in %s\n",cart_names[leg]);
    rc = 7;
    some_random_value_for_beta = -1;
//    return false;
  }
	beta = acos(some_random_value_for_beta);
//	printf("debug beta: %f\n",beta);

	// compute Hip FE angle
	q_HFE_bf = q_HFE_br = alpha + beta;


	double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
	// law of cosines give the knee angle
	if (some_random_value_for_gamma > 1) {
		// printf("Error in computeFootIKVel for KFE in %s\n",cart_names[leg]);
		rc = 5;
		some_random_value_for_gamma = 1;
//		return false;
	}
  if (some_random_value_for_gamma < -1) {
    // printf("Error in computeFootIKVel for KFE in %s\n",cart_names[leg]);
    rc = 5;
    some_random_value_for_gamma = -1;
//    return false;
  }
	gamma  = acos(some_random_value_for_gamma);


//	printf("debug gamma: %f\n",gamma);
	q_KFE_bf = q_KFE_br = gamma - M_PI;  // -(PI - gamma);

	// check if joint angle lies in the correct range
	bf_exceeds_joint_limit = false;
	if (q_HAA_bf > joint_range[LF_HAA][MAX_THETA_INDEX]) {
		q_HAA_bf = joint_range[LF_HAA][MAX_THETA_INDEX];
		bf_exceeds_joint_limit = true;
	}
	if (q_HAA_bf < joint_range[LF_HAA][MIN_THETA_INDEX]) {
		q_HAA_bf = joint_range[LF_HAA][MIN_THETA_INDEX];;
		bf_exceeds_joint_limit = true;
	}
	if (q_HFE_bf > joint_range[LF_HFE][MAX_THETA_INDEX]) {
		q_HFE_bf = joint_range[LF_HFE][MAX_THETA_INDEX];0;
		bf_exceeds_joint_limit = true;
	}
	if (q_HFE_bf < joint_range[LF_HFE][MIN_THETA_INDEX]) {
		q_HFE_bf = joint_range[LF_HFE][MIN_THETA_INDEX];
		bf_exceeds_joint_limit = true;
	}
	if (q_KFE_bf > joint_range[LF_KFE][MAX_THETA_INDEX]) {
		q_KFE_bf = joint_range[LF_KFE][MAX_THETA_INDEX];
		bf_exceeds_joint_limit = true;
	}
	if (q_KFE_bf < joint_range[LF_KFE][MIN_THETA_INDEX]) {
		q_KFE_bf = joint_range[LF_KFE][MIN_THETA_INDEX];
		bf_exceeds_joint_limit = true;
	}

	// check joint angle range br
	br_exceeds_joint_limit = false;
	if (q_HAA_br > joint_range[LF_HAA][MAX_THETA_INDEX]) {
		q_HAA_br = joint_range[LF_HAA][MAX_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}
	if (q_HAA_br < joint_range[LF_HAA][MIN_THETA_INDEX]) {
		q_HAA_br = joint_range[LF_HAA][MIN_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}
	if (q_HFE_br > joint_range[LF_HFE][MAX_THETA_INDEX]) {
		q_HFE_br = joint_range[LF_HFE][MAX_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}
	if (q_HFE_br < joint_range[LF_HFE][MIN_THETA_INDEX]) {
		q_HFE_br = joint_range[LF_HFE][MIN_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}
	if (q_KFE_br > joint_range[LF_KFE][MAX_THETA_INDEX]) {
		q_KFE_br = joint_range[LF_KFE][MAX_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}
	if (q_KFE_br < joint_range[LF_KFE][MIN_THETA_INDEX]) {
		q_KFE_br = joint_range[LF_KFE][MIN_THETA_INDEX];
		br_exceeds_joint_limit = true;
	}

	// assign to output arrays
	if (leg==RH_LEG || leg==LH_LEG) { // for hind legs swap the forward bent/rear bent
		q << q_HAA_br, -q_HFE_br, -q_KFE_br;
	}
	else {
		q << q_HAA_bf, q_HFE_bf, q_KFE_bf;
	}


	return true;
}

} /* namespace xpp */
} /* namespace hyq */


