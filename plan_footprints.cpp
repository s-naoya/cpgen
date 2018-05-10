#include "plan_footprints.h"

namespace cp{

void PlanFootprints::init_setup(const Affine3d init_leg_pose[],
                                const Quat& waist_q, const Vector3& com,
                                const double endcpoff[]) {
  for (int i = 0; i < 2; ++i) {
    Vector3 trans = init_leg_pose[i].translation();
    Quat q(init_leg_pose[i].rotation());
    init_feet_pose[i].set(trans, q);
    ref_land_pose[i].set(init_feet_pose[i]);
    dist_body2foot[i] << trans;  // TODO! subtraction to body position
  }
  ref_waist_pose.set(com, waist_q);
  // ref_waist_r = waist_r;
  end_cp << com[0], com[1];
  end_cp_offset[0] = endcpoff[0];  end_cp_offset[1] = endcpoff[1];
  step_vector << 0.0, 0.0, 0.0;
}

void PlanFootprints::setValues(walking_state wstate, rl swingleg,
                               Vector3 land_pos) {
  this->wstate = wstate;
  this->swingleg = swingleg;
  step_vector << land_pos.x(), land_pos.y(), 0.0;
  step_angle = land_pos.z();
}

void PlanFootprints::update() {
  calcNextWaistPose();
  calcNextFootprint();
  calcEndCP();
}

void PlanFootprints::calcNextWaistPose() {
  Quat waist_r = ref_waist_pose.q() * rpy2q(0.0, 0.0, step_angle);
  Vector3 waist_pos = ref_waist_pose.p() + waist_r * step_vector;
  ref_waist_pose.set(waist_pos, waist_r);
}

void PlanFootprints::calcNextFootprint() {
  Vector3 ref_land_pose_p = ref_waist_pose.p() +
                            ref_waist_pose.q() * dist_body2foot[swingleg];
  Quat ref_land_pose_q    = ref_waist_pose.q() * init_feet_pose[0].q();
  ref_land_pose[swingleg].set(ref_land_pose_p, ref_land_pose_q);
  ref_waist_r = ref_waist_pose.q();
}

void PlanFootprints::calcEndCP() {
  if (wstate == stopping1 || wstate == stopping2 || wstate == stop_next) {
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
}


//////  old style  //////
void PlanFootprints::calcNextFootprintOld() {
  whichWalkOrStep();
  static Vector3 before_land_pos(step_vector.x(), step_vector.y(), 0.0);
  static Vector3 before_land_dis(step_vector.x(), step_vector.y(), 0.0);

  // calc next step land position
  Vector3 next_land_distance(0.0, 0.0, 0.0);
  if (wstate == starting1 || wstate == starting2 || wstate == step2walk) {
    next_land_distance[0] = step_vector.x();
    next_land_distance[1] = isCollisionLegs(step_vector.y()) ? step_vector.y() : 0.0;
  } else if (wstate == stop_next) {  // walking stop
    next_land_distance[0] = step_vector.x();
    next_land_distance[1] =
        isCollisionLegs(before_land_pos.y()) ? before_land_pos.y() : 0.0;
  } else if (wstate == walk2step) {  // walk -> step
    next_land_distance[0] = before_land_pos.x();
    next_land_distance[1] =
        isCollisionLegs(before_land_pos.y()) ? 0.0 : before_land_pos.y();
  } else if (wstate == walk) {
    next_land_distance[0] =
        before_land_pos.x() * 2 + (step_vector.x() - before_land_pos.x());
    next_land_distance[1] = isCollisionLegs(step_vector.y(), before_land_dis.y());
  } else {
    next_land_distance[0] = 0.0;
    next_land_distance[1] = 0.0;
  }

  // calc next step land rotation
  Quat foot_rot = rpy2q(0.0, 0.0, step_angle;
  Quat waist_rot = rpy2q(0.0, 0.0, step_angle * 0.5);

  // swing leg
  Vector3 swing_p(
      ref_land_pose[swingleg].p().x() + next_land_distance[0],
      ref_land_pose[swingleg].p().y() + next_land_distance[1],
      init_feet_pose[swingleg].p().z());
  Quat swing_q = ref_land_pose[swingleg].q() * foot_rot;

  // set next landing position
  ref_land_pose[swingleg].set(swing_p, swing_q);
  ref_waist_r = ref_waist_r * waist_rot;

  before_land_pos = step_vector;
  before_land_dis = next_land_distance;
}

void PlanFootprints::whichWalkOrStep() {
  if (wstate != walk && wstate != step) {return;}
  if (step_vector.x() != 0.0 || step_vector.y() != 0.0) {
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

bool PlanFootprints::isCollisionLegs(double y) {
  if ((swingleg == right && y < 0) || (swingleg == left && y >= 0)) {
    return true;
  } else {
    return false;
  }
}

double PlanFootprints::isCollisionLegs(double yn, double yb) {
  if (swingleg == right && yn < 0) {
    if (yb > 0) return 0.0;
    else return yn;
  } else if (swingleg == left && yn >= 0) {
    if (yb < 0) return 0.0;
    else return yn;
  } else {
    return yb;
  }
}



}; // namespace cp