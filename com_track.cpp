#include "com_track.h"

namespace cp {

// setting initial value
// necesarry call this before call getCoMTrack
void CoMTrack::init_setup(double t, double sst, double dst, double cogh,
                     const Vector3& com) {
  setup(t, sst, dst, cogh);
  now_cp << com[0], com[1];
  ref_zmp << com[0], com[1];
  ref_com << com[0], com[1], cog_h;
}

// always can change these value
void CoMTrack::setup(double t, double sst, double dst, double cogh) {
  dt = t;
  single_sup_time = sst;
  double_sup_time = dst;
  cog_h = cogh;
  step_time = single_sup_time + double_sup_time;

  w = sqrt(9.806 / cog_h);
}

// calc CoM track of walking pattern every cycle
// @param com : now CoM  --- now no use ---
// @param end_cp : end CP of this step
// @param step_delta_time : dT of this step
// @return : CoM track
Vector3 CoMTrack::getCoMTrack(const Vector3& com, const Vector2& end_cp,
                              double step_delta_time) {
  Vector2 ref_cp = calcCPTrack(step_delta_time);
  calcCoMTrack(ref_cp, com);
  return ref_com;
}

// call only changed swing leg (same timing of "LegTrack::getLegTrack")
// @param end_cp : end CP of this step
// @return : reference ZMP point of this step
Vector2 CoMTrack::calcRefZMP(const Vector2& end_cp) {
  setStepVariable();
  double b = exp(w * step_time_s);
  now_cp = calcCPTrack(step_time_s);
  ref_zmp = (end_cp - b * now_cp) / (1 - b);

  return ref_zmp;
}

void CoMTrack::setStepVariable() {
  step_time_s = step_time;
  dt_s = dt;
  w_s = w;
}

Vector2 CoMTrack::calcCPTrack(double step_delta_time) {
  Vector2 ref_cp = ref_zmp + exp(w_s * step_delta_time) * (now_cp - ref_zmp);
  return ref_cp;
}

void CoMTrack::calcCoMTrack(const Vector2& ref_cp, const Vector3& com) {
  Vector2 now_com_pos, com_vel, com_pos;
  now_com_pos << ref_com[0], ref_com[1];
  // now_com_pos << com[0], com[1]; // TODO! local CoM to World coodinate
  com_vel = w_s * (ref_cp - now_com_pos);
  com_pos = now_com_pos + com_vel * dt_s;
  ref_com[0] = com_pos[0];
  ref_com[1] = com_pos[1];
}

}  // namespace cp