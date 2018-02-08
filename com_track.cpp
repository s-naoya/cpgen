#include "com_track.h"

namespace cp {

// setting initial value
// necesarry call this before call getCoMTrack
void CoMTrack::init_setup(double sampling_time, double single_sup_time,
                          double double_sup_time, double cog_h,
                          const Vector3& com) {
  setup(sampling_time, single_sup_time, double_sup_time, cog_h);
  now_cp << com[0], com[1];
  ref_zmp << com[0], com[1];
  ref_com << com[0], com[1], cogh;
}

// always can change these value
void CoMTrack::setup(double t, double single_sup_time,
                     double double_sup_time, double cog_h) {
  dt = t;
  sst = single_sup_time;
  dst = double_sup_time;
  cogh = cog_h;
  st = single_sup_time + double_sup_time;

  w = sqrt(9.806 / cogh);
}

// calc CoM track of walking pattern every cycle
// @param end_cp : end CP of this step
// @param step_delta_time : dT of this step
// @return : CoM track
Vector3 CoMTrack::getCoMTrack(const Vector2& end_cp, double step_delta_time) {
  Vector2 ref_cp = calcCPTrack(step_delta_time);
  calcCoMTrack(ref_cp);
  return ref_com;
}

// call only changed swing leg
// @param end_cp : end CP of this step
// @return : reference ZMP point of this step
Vector2 CoMTrack::calcRefZMP(const Vector2& end_cp) {
  setStepVariable();
  double b = exp(w * st_s);
  now_cp = calcCPTrack(st_s);
  ref_zmp = (end_cp - b * now_cp) / (1 - b);

  return ref_zmp;
}

void CoMTrack::setStepVariable() {
  st_s = st;
  dt_s = dt;
  w_s = w;
}

Vector2 CoMTrack::calcCPTrack(double step_delta_time) {
  Vector2 ref_cp = ref_zmp + exp(w_s * step_delta_time) * (now_cp - ref_zmp);
  return ref_cp;
}

void CoMTrack::calcCoMTrack(const Vector2& ref_cp) {
  Vector2 now_com_pos, com_vel, com_pos;
  now_com_pos << ref_com[0], ref_com[1];
  com_vel = w_s * (ref_cp - now_com_pos);
  com_pos = now_com_pos + com_vel * dt_s;
  ref_com[0] = com_pos[0];
  ref_com[1] = com_pos[1];
}

}  // namespace cp
