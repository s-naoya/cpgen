#include "leg_track.h"

namespace cp {

// setting initial value
// necesarry call this before call getLegTrack
void LegTrack::init_setup(double t, double sst, double dst, double legh,
                     std::array<Pose, 2>& now_leg_pose) {
  setup(t, sst, dst, legh);
  init_pose[0] = now_leg_pose[0];
  init_pose[1] = now_leg_pose[1];
  before_landpos[0] = now_leg_pose[0];
  before_landpos[1] = now_leg_pose[1];
}

// always can change these value
void LegTrack::setup(double t, double sst, double dst, double legh) {
  dt = t;
  single_sup_time = sst;
  double_sup_time = dst;
  leg_h = legh;
}

// calc both leg track and push to "r_leg_pos" every step
// @param swingleg : swing leg flag
// @param ref_landpos_leg_w[2] : Reference landing position. world coodinate.
// @return return: r_leg_pos[2] : Designed both leg track. world coodinate.
void LegTrack::getLegTrack(const rl swingleg,
                           const std::array<Pose, 2>& ref_landpos_leg_w,
                           std::array<std::deque<Pose>, 2>& r_leg_pos) {
  std::deque<Pose> swing_leg_pos;

  std::deque<double> leg_pos_z;
  std::deque<Quaternion> leg_pos_quat;
  std::deque<Quaternion> sup_leg_quat;
  rl supleg = swingleg == right ? left : right;

  // x, y
  lerp_pose(before_landpos[swingleg], before_landpos[swingleg],
            double_sup_time * 0.25, &swing_leg_pos);
  lerp_pose(before_landpos[swingleg], ref_landpos_leg_w[swingleg],
            single_sup_time, &swing_leg_pos);
  lerp_pose(ref_landpos_leg_w[swingleg], ref_landpos_leg_w[swingleg],
            double_sup_time * 0.25, &swing_leg_pos);

  // z
  const double grand_h = init_pose[0].p().z();
  lerp_same(grand_h, double_sup_time * 0.25, &leg_pos_z);
  lerp_d(grand_h, grand_h + leg_h, single_sup_time * 0.5, &leg_pos_z);
  lerp_d(grand_h + leg_h, grand_h, single_sup_time * 0.5, &leg_pos_z);
  lerp_same(grand_h, double_sup_time * 0.25, &leg_pos_z);

  // yaw
  // swing leg
  Quaternion start_quat = before_landpos[swingleg].q();
  Quaternion finish_quat = ref_landpos_leg_w[swingleg].q();
  lerp_same(start_quat, double_sup_time * 0.25, &leg_pos_quat);
  lerp_q(start_quat, finish_quat, single_sup_time, &leg_pos_quat);
  lerp_same(finish_quat, double_sup_time * 0.25, &leg_pos_quat);
  // support leg
  start_quat = before_landpos[supleg].q();
  finish_quat = ref_landpos_leg_w[supleg].q();
  lerp_same(start_quat, double_sup_time * 0.25, &sup_leg_quat);
  lerp_q(start_quat, finish_quat, single_sup_time, &sup_leg_quat);
  lerp_same(finish_quat, double_sup_time * 0.25, &sup_leg_quat);

  // union
  Pose sup_leg_pose;
  for (int i = 0, n = swing_leg_pos.size(); i < n; ++i) {
    // swing leg
    Vector3 tmp_p(swing_leg_pos[i].p().x(),
                  swing_leg_pos[i].p().y(), leg_pos_z[i]);
    swing_leg_pos[i].set(tmp_p, leg_pos_quat[i]);
    r_leg_pos[swingleg].push_back(swing_leg_pos[i]);

    // support leg
    sup_leg_pose.set(before_landpos[supleg].p(), sup_leg_quat[i]);
    r_leg_pos[supleg].push_back(sup_leg_pose);
  }

  before_landpos[swingleg] = swing_leg_pos.back();
  before_landpos[supleg] = sup_leg_pose;
}

void LegTrack::lerp_pose(const Pose start, const Pose finish, double tf,
                         std::deque<Pose>* input) {
  const int num = (int)(tf / dt + 1e-8);
  Pose tmp;
  for (int i = 1; i < num + 1; ++i) {
    tmp.p().x() = start.p().x() + (finish.p().x() - start.p().x()) * i / num;
    tmp.p().y() = start.p().y() + (finish.p().y() - start.p().y()) * i / num;
    input->push_back(tmp);
  }
}

void LegTrack::lerp_d(const double start, const double finish, double tf,
                      std::deque<double>* input) {
  const int num = (int)(tf / dt + 1e-8);
  double tmp;
  for (int i = 1; i < num + 1; i++) {
    tmp = start + (finish - start) * i / num;
    input->push_back(tmp);
  }
}

void LegTrack::lerp_q(const Quaternion start, const Quaternion finish,
                      double tf, std::deque<Quaternion>* input) {
  const int num = (int)(tf / dt + 1e-8) + 1;
  Quaternion tmp;
  for (int i = 1; i < num + 1; i++) {
    double t = 1 * i / num;
    input->push_back(start.slerp(t, finish));
  }
}

}  // namespace cp