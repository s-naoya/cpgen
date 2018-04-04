#include "cpgen.h"


namespace cp {

// init_leg_pos: 0: right, 1: left, world coodinate(leg end link)
void cpgen::initialize(const Vector3& com, const Quat& waist_r,
             const Affine3d init_leg_pose[], const Quat i_base2leg[],
             const double endcpoff[],
             double t, double sst, double dst, double cogh, double legh) {
  // init variable setup
  swingleg = left;
  base2leg[0] = i_base2leg[0];  base2leg[1] = i_base2leg[1];

  setup(t, sst, dst, cogh, legh);
  comtrack.init_setup(dt, single_sup_time, double_sup_time, cog_h, com);
  legtrack.init_setup(dt, single_sup_time, double_sup_time,
                      leg_h, init_leg_pose, waist_r);
  pf.init_setup(init_leg_pose, waist_r, com, endcpoff);
  ref_zmp = comtrack.calcRefZMP(pf.end_cp);
  land_pos = Vector3::Zero();

  wstate = stopped;

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

void cpgen::getWalkingPattern(Vector3* com_pos, Quat* waist_r,
                              Pose* right_leg_pose, Pose* left_leg_pose) {
  static double step_delta_time = double_sup_time + single_sup_time + 1.0;

  if (wstate == stopped) return;

  // if finished a step, calc leg track and reference ZMP.
  if (step_delta_time >= double_sup_time + single_sup_time) {
    swingleg = swingleg == right ? left : right;
    pf.setValues(wstate, swingleg, land_pos);
    pf.update();
    ref_zmp = comtrack.calcRefZMP(pf.end_cp);

    legtrack.setStepVar(pf.ref_land_pose, pf.ref_waist_r, swingleg, wstate);
    step_delta_time = 0.0;
  }

  // push walking pattern
  static Pose leg_pose[2];
  *com_pos = comtrack.getCoMTrack(pf.end_cp, step_delta_time);
  *waist_r = legtrack.getWaistTrack(step_delta_time);
  legtrack.getLegTrack(step_delta_time, leg_pose);
  *right_leg_pose = leg_pose[0];
  *left_leg_pose  = leg_pose[1];

  // setting flag and time if finished a step
  step_delta_time += dt;
  if (step_delta_time >= double_sup_time + single_sup_time) {
    if (wstate == starting1) {
      wstate = starting2;
    } else if (wstate == starting2) {
      if (pf.whichwalk == step) {
        wstate = step;
      } else {
        wstate = walk;
      }
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

}  // namespace cp