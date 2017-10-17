#include "cpgen.h"

namespace cp {

// init_leg_pos: 0: right, 1: left, world coodinate(leg end link)
cpgen::cpgen(const Vector3& com, const Affine3d init_leg_pos[],
             const Quaternion i_base2leg[], double t, double sst, double dst,
             double cogh, double legh) {
  // init variable setup
  swingleg = left;
  end_cp_offset[0] = 0.001;
  end_cp_offset[1] = 0.02;
  setInitLandPos(init_leg_pos);
  base2leg[0] = i_base2leg[0];
  base2leg[1] = i_base2leg[1];

  setup(t, sst, dst, cogh, legh);
  comtrack.init_setup(dt, single_sup_time, double_sup_time, cog_h, com);
  legtrack.init_setup(dt, single_sup_time, double_sup_time, leg_h,
                      land_pos_leg_w);
  // init Capture Point
  end_cp[0] = com[0];
  end_cp[1] = com[1];

  land_pos = Vector3::Zero();

  wstate = stopped;

  std::cout << "[cpgen] activate" << std::endl;
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
    wstate = starting;
    std::cout << "[cpgen] Start Walking" << std::endl;
  }
}

void cpgen::stop() {
  if (wstate == walk || wstate == step) {
    wstate = stop_next;
    std::cout << "[cpgen] stopped Walking" << std::endl;
  }
}

void cpgen::restart() {
  wstate = starting;
  std::cout << "[cpgen] Restart Walking" << std::endl;
}

void cpgen::setLandPos(const Vector3& pose) { land_pos = pose; }

void cpgen::getWalkingPattern(Vector3* com_pos, Pose* right_leg_pos,
                              Pose* left_leg_pos) {
  static double step_delta_time = 0.0;

  if (wstate == stopped) {
    return;
  }

  // if finished a step, calc leg track and reference ZMP.
  if (designed_leg_track[0].empty()) {
    swingleg = swingleg == right ? left : right;
    calcEndCP();
    ref_zmp = comtrack.calcRefZMP(end_cp);

    legtrack.getLegTrack(swingleg, land_pos_leg_w, designed_leg_track);
    step_delta_time = 0.0;
  }

  // push walking pattern
  *com_pos = comtrack.getCoMTrack(end_cp, step_delta_time);
  *right_leg_pos = designed_leg_track[right].front();
  *left_leg_pos = designed_leg_track[left].front();
  designed_leg_track[right].pop_front();
  designed_leg_track[left].pop_front();

  // setting flag and time if finished a step
  if (designed_leg_track[0].empty()) {
    if (wstate == starting) {
      if (whichwalk == step) {
        wstate = step;
      } else {
        wstate = walk;
      }
    } else if (wstate == stopping) {
      wstate = stopped;
    } else if (wstate == walk2step) {
      wstate = step;
    } else if (wstate == step2walk) {
      wstate = walk;
    }
  }
  step_delta_time += dt;
}

void cpgen::calcEndCP() {
  calcLandPos();
  if (wstate == stopping) {
    end_cp[0] = land_pos_leg_w[swingleg].p().x();
    end_cp[1] = (land_pos_leg_w[0].p().y() + land_pos_leg_w[1].p().y()) / 2;
  } else {
    end_cp[0] = land_pos_leg_w[swingleg].p().x() + end_cp_offset[0];
    if (swingleg == right) {
      end_cp[1] = land_pos_leg_w[swingleg].p().y() + end_cp_offset[1];
    } else {
      end_cp[1] = land_pos_leg_w[swingleg].p().y() - end_cp_offset[1];
    }
  }
}

// neccesary call this if change "land_pos"
void cpgen::calcLandPos() {
  whichWalkOrStep();
  static Vector3 before_land_pos(land_pos.x(), land_pos.y(), 0.0);

  // calc next step land position
  Vector2 next_land_distance(0.0, 0.0);
  if (wstate == starting || wstate == step2walk) {  // walking start
    next_land_distance[0] = land_pos.x();
    next_land_distance[1] = isCollisionLegs(land_pos.y()) ? 0.0 : land_pos.y();
  } else if (wstate == stop_next) {  // walking stop
    next_land_distance[0] = land_pos.x();
    next_land_distance[1] =
        isCollisionLegs(before_land_pos.y()) ? before_land_pos.y() : 0.0;
    wstate = stopping;
  } else if (wstate == walk2step) {  // walk -> step
    next_land_distance[0] = before_land_pos.x();
    next_land_distance[1] =
        isCollisionLegs(before_land_pos.y()) ? before_land_pos.y() : 0.0;
  } else if (wstate == walk) {
    // correspond accelate
    next_land_distance[0] =
        before_land_pos.x() * 2 + (land_pos.x() - before_land_pos.x());
    next_land_distance[1] =
        // isCollisionLegs(land_pos.y(), before_land_pos.y()) ?
        // before_land_pos.y() : land_pos.y();
        isCollisionLegs(land_pos.y()) ? before_land_pos.y() : land_pos.y();
  } else {
    next_land_distance[0] = 0.0;
    next_land_distance[1] = 0.0;
  }

  // calc next step land rotation
  Quaternion next_swing_q, next_sup_q;
  if (land_pos.z() > 0.0) {
    if (swingleg == right) {
      next_swing_q = rpy2q(0, 0, -land_pos.z() / 2);
      next_sup_q = rpy2q(0, 0, -land_pos.z() / 2);
    } else {
      next_swing_q = rpy2q(0, 0, before_land_pos.z() / 2);
      next_sup_q = rpy2q(0, 0, before_land_pos.z() / 2);
    }
  } else {
    if (swingleg == left) {
      next_swing_q = rpy2q(0, 0, land_pos.z() / 2);
      next_sup_q = rpy2q(0, 0, land_pos.z() / 2);
    } else {
      next_swing_q = rpy2q(0, 0, -before_land_pos.z() / 2);
      next_sup_q = rpy2q(0, 0, -before_land_pos.z() / 2);
    }
  }

  // set next landing position
  // swing leg
  Vector3 swing_p(land_pos_leg_w[swingleg].p().x() + next_land_distance[0],
                  land_pos_leg_w[swingleg].p().y() + next_land_distance[1],
                  0.0);
  Quaternion swing_q = land_pos_leg_w[swingleg].q() * next_swing_q;
  // support leg
  rl supleg = swingleg == right ? left : right;
  Vector3 sup_p(land_pos_leg_w[supleg].p().x(), land_pos_leg_w[supleg].p().y(),
                0.0);
  Quaternion sup_q = land_pos_leg_w[supleg].q() * next_sup_q;  // union angle

  // set next landing position
  land_pos_leg_w[swingleg].set(swing_p, swing_q);
  land_pos_leg_w[supleg].set(sup_p, sup_q);

  before_land_pos.x() = land_pos.x();
  before_land_pos.y() = land_pos.y();
  before_land_pos.z() = land_pos.z();
}

void cpgen::whichWalkOrStep() {
  if (land_pos.x() != 0.0 || land_pos.y() != 0.0 || land_pos.z() != 0.0) {
    if (whichwalk == step) {
      wstate = step2walk;
    }
    whichwalk = walk;
  } else {
    if (whichwalk == walk) {
      wstate = walk2step;
    }
    whichwalk = step;
  }
}

bool cpgen::isCollisionLegs(double y) {
  if ((swingleg == right && y > 0) || (swingleg == left && y < 0)) {
    return true;
  } else {
    return false;
  }
}

bool cpgen::isCollisionLegs(double yn, double yb) {
  if ((swingleg == right && yn > 0 && yn >= yb) ||
      (swingleg == left && yn <= 0 && yn <= yb)) {
    return true;
  } else {
    return false;
  }
}

// initialze "land_pos_leg_w"
void cpgen::setInitLandPos(const Affine3d init_leg_pos[]) {
  for (int i = 0; i < 2; ++i) {
    Vector3 trans = init_leg_pos[i].translation();
    Quaternion q = Quaternion(init_leg_pos[i].rotation());

    land_pos_leg_w[i].set(trans, q);
  }
}

Pose cpgen::setInitLandPos(const Affine3d& init_leg_pos) {
  Vector3 trans = init_leg_pos.translation();
  Quaternion q = Quaternion(init_leg_pos.rotation());
  cp::Pose pose;
  pose.set(trans, q);
  return pose;
}

}  // namespace cp
