#include "cpgen.h"


namespace cp {

// init_leg_pos: 0: right, 1: left, world coodinate(leg end link)
void cpgen::initialize(const Vector3& com, const Quat& waist_r,
             const Affine3d init_leg_pose[], const Quat i_base2leg[],
             const double endcpoff[],
             double t, double sst, double dst, double cogh, double legh) {
  // init variable setup
  swingleg = right;
  base2leg[0] = i_base2leg[0];  base2leg[1] = i_base2leg[1];

  setup(t, sst, dst, cogh, legh);
  comtrack.init_setup(dt, single_sup_time, double_sup_time, cog_h, com);
  legtrack.init_setup(dt, single_sup_time, double_sup_time,
                      leg_h, init_leg_pose, waist_r);
  // pf.init_setup(init_leg_pose, waist_r, com, endcpoff);
  init_com_pose.set(com, waist_r);
  land_pos = Vector3::Zero();
  land_pos_mod = Vector3::Zero();
  end_cp_offset[0] = endcpoff[0];  end_cp_offset[1] = endcpoff[1];

  wstate = stopped;

  for (int i = 0; i < 2; ++i) {
    Vector3 trans = init_leg_pose[i].translation();
    Quat q(init_leg_pose[i].rotation());
    init_feet_pose[i].set(trans, q);
    // ref_land_pose[i].set(init_feet_pose[i]);
    dist_body2foot[i] << trans;  // TODO! subtraction to body position
  }

  std::cout << "[cpgen] initialize finish" << std::endl;
}

// always can change these value
// @param t: sampling time
// @param sst: single support time
// @param dst: double support time
// @param cogh: height center of gravity
// @param legh: height of up leg
void cpgen::setup(double t, double sst, double dst, double cogh, double legh) {
  dt = t;
  single_sup_time = sst;
  double_sup_time = dst;
  cog_h = cogh;
  leg_h = legh;

  comtrack.setup(dt, single_sup_time, double_sup_time, cog_h);
  legtrack.setup(dt, single_sup_time, double_sup_time, leg_h);
}

void cpgen::start() {
  if (wstate == stopped) {
    wstate = starting1;
    std::cout << "[cpgen] Start Walking" << std::endl;
  }
}

void cpgen::stop() {
  if (wstate == walk || wstate == step) {
    wstate = stop_next;
    std::cout << "[cpgen] Stop Walking" << std::endl;
  }
}

void cpgen::estop() {
  wstate = stopped;
  std::cout << "[cpgen] Emergency Stop" << std::endl;
}

void cpgen::setLandPos(const Vector3& pose) {
  land_pos = pose;  // TODO! Round down to about millimeter
  land_pos.z() = deg2rad(land_pos.z());
}

void cpgen::setLandPosModification(const Vector3& pos) {
  land_pos_mod = pos;
}

void cpgen::getWalkingPattern(Vector3* com_pos, Quat* waist_r,
                              Pose* right_leg_pose, Pose* left_leg_pose) {

  static double step_delta_time = double_sup_time + single_sup_time + 1.0;
  static Vector2 end_cp(init_com_pose.p().x(), init_com_pose.p().y());
  static Pose ref_waist_pose_toec = init_com_pose;
  static Pose ref_land_pose_toec[2] = {init_feet_pose[0], init_feet_pose[1]};
  static Pose ref_waist_pose = init_com_pose;
  static Pose ref_land_pose[2] = {init_feet_pose[0], init_feet_pose[1]};

  if (wstate == stopped) return;

  // if finished a step, calc leg track and reference ZMP.
  if (step_delta_time >= double_sup_time + single_sup_time) {
    // calc end_cp and ref_zmp
    calcNextFootprint(land_pos, land_pos.z(), ref_waist_pose_toec, ref_land_pose_toec);
    end_cp = calcEndCP(ref_land_pose_toec);
    comtrack.calcRefZMP(end_cp);

    // to calc legtrack
    calcNextFootprint(land_pos+land_pos_mod, land_pos.z(), ref_waist_pose, ref_land_pose);
    legtrack.setStepVar(ref_land_pose, ref_waist_pose.q(), swingleg, wstate);
    step_delta_time = 0.0;
  }

  // push walking pattern
  static Pose leg_pose[2];
  *com_pos = comtrack.getCoMTrack(end_cp, step_delta_time);
  *waist_r = legtrack.getWaistTrack(step_delta_time);
  legtrack.getLegTrack(step_delta_time, leg_pose);
  *right_leg_pose = leg_pose[0];
  *left_leg_pose  = leg_pose[1];

  // setting flag and time if finished a step
  step_delta_time += dt;
  if (step_delta_time >= double_sup_time + single_sup_time) {
    swingleg = swingleg == right ? left : right;
    if (wstate == starting1) {
      wstate = starting2;
    } else if (wstate == starting2) {
      // if (whichwalk == step) {
      //   wstate = step;
      // } else {
      //   wstate = walk;
      // }
      wstate = walk;
    } else if (wstate == stop_next) {
      wstate = stopping1;
    } else if (wstate == stopping1) {
      wstate = stopping2;
    } else if (wstate == stopping2) {
      wstate = stopped;
      std::cout << "[cpgen] Stopped" << std::endl;
    } else if (wstate == walk2step) {
      wstate = step;
    } else if (wstate == step2walk) {
      wstate = walk;
    }
  }
}


// @brief calc footprints of next step
// @param[in] step_vector: <X direction step distance, Y direction, no use>
// @param[in] step_angle: amount of rotation
// @param[in, out] ref_waist_pose:: in: now waist pose, out: reference of waist pose
// @param[out] ref_land_pose[2]: reference of footprints
void cpgen::calcNextFootprint(const Vector3& step_vector, double step_angle,
    Pose& ref_waist_pose, Pose ref_land_pose[]) {

  // calc next waist pose
  Quat waist_r = ref_waist_pose.q() * rpy2q(0.0, 0.0, step_angle);
  Vector3 waist_pos = ref_waist_pose.p() + waist_r * step_vector;
  waist_pos.z() = 0.0;
  ref_waist_pose.set(waist_pos, waist_r);

  // calc footprints
  Vector3 ref_land_pose_p = ref_waist_pose.p() +
                            ref_waist_pose.q() * dist_body2foot[swingleg];
  Quat ref_land_pose_q    = ref_waist_pose.q() * init_feet_pose[0].q();
  ref_land_pose[swingleg].set(ref_land_pose_p, ref_land_pose_q);
}


// @brief calc End Capture Point
// @param[in] ref_land_pose
// @return: end cp
Vector2 cpgen::calcEndCP(const Pose ref_land_pose[]) {

  Vector2 end_cp = Vector2::Zero();
  if (wstate == stopping2 || wstate == stopping1) {
    end_cp[0] = ref_land_pose[swingleg].p().x() + end_cp_offset[0];
    end_cp[1] = (ref_land_pose[0].p().y() + ref_land_pose[1].p().y()) * 0.5;
  } else {
    end_cp[0] = ref_land_pose[swingleg].p().x() + end_cp_offset[0];
    if (swingleg == right) {
      end_cp[1] = ref_land_pose[swingleg].p().y() + end_cp_offset[1];
    } else {
      end_cp[1] = ref_land_pose[swingleg].p().y() - end_cp_offset[1];
    }
  }
  return end_cp;
}

}  // namespace cp