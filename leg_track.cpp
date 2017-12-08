#include "leg_track.h"

namespace cp {

// setting initial value
// necesarry call this before call getLegTrack
void LegTrack::init_setup(double sampling_time, double single_sup_time,
                          double double_sup_time, double legh,
                          Pose now_leg_pose[2]) {
  setup(sampling_time, single_sup_time, double_sup_time, legh);
  init_pose[0] = now_leg_pose[0];
  init_pose[1] = now_leg_pose[1];
  bfr_landpose[0] = now_leg_pose[0];
  bfr_landpose[1] = now_leg_pose[1];
}

// always can change these value
void LegTrack::setup(double sampling_time, double single_sup_time,
                     double double_sup_time, double legh) {
  dt  = sampling_time;
  sst = single_sup_time;
  dst = double_sup_time;
  st  = single_sup_time + double_sup_time;
  leg_h = legh;
}

// set variable of a step
// call only switch swing leg
void LegTrack::setStepVar(
    const Pose ref_landpose_leg_w[], rl swingleg, walking_state wstate) {
  // set time var of a step
  sst_s = sst;
  dst_s = dst;
  dt_s  = dt;
  st_s = sst_s + dst_s;
  swl = swingleg;
  ws = wstate;
  // set next landing pos
  bfr_landpose[0] = ref_landpose[0];
  bfr_landpose[1] = ref_landpose[1];
  ref_landpose[0] = ref_landpose_leg_w[0];
  ref_landpose[1] = ref_landpose_leg_w[1];

  bfr << bfr_landpose[swl].p().x(), bfr_landpose[swl].p().y();
  ref << ref_landpose[swl].p().x(), ref_landpose[swl].p().y();

  inter_z_1.setInter5(0.0, 0.0, 0.0, leg_h, 0.0, 0.0, sst*0.5);
  inter_z_2.setInter5(leg_h, 0.0, 0.0, 0.0, 0.0, 0.0, sst*0.5);
}

void LegTrack::getLegTrack(double t, Pose r_leg_pose[]) {
  rl spl = swl == right ? left : right;
  if (ws == starting1 || ws == stopping2) {
      r_leg_pose[right] = bfr_landpose[right];
      r_leg_pose[left]  = bfr_landpose[left];
  } else {
    if (t < dst_s*0.5) {
        r_leg_pose[swl].p() = bfr_landpose[swl].p();
    } else if (t < sst_s*0.5) {
        double sst_s_t = t - dst_s;
        Vector2 nex = inter_vec2.lerp(bfr, ref, sst_s, sst_s_t);
        r_leg_pose[swl].p().x() = nex.x();
        r_leg_pose[swl].p().y() = nex.y();
        r_leg_pose[swl].p().z() = inter_z_1.inter5(sst_s_t);
    } else if (t < sst_s) {
        double sst_s_t = t - dst_s;
        double sst_s_ht = t - dst_s - sst_s*0.5;
        Vector2 nex = inter_vec2.lerp(bfr, ref, sst_s, sst_s_t);
        r_leg_pose[swl].p().x() = nex.x();
        r_leg_pose[swl].p().y() = nex.y();
        r_leg_pose[swl].p().z() = inter_z_2.inter5(sst_s_ht);
    } else if (t <= st_s) {
        r_leg_pose[swl].p() = ref_landpose[swl].p();
    }
    r_leg_pose[spl] = bfr_landpose[spl];
  }
}

//   if (wstate == starting1 ||  wstate == stopping2) {
//     lerp_pose(before_landpos[swingleg],
//         before_landpos[swingleg], step_time, &swing_leg_pos);
//     lerp_same(init_pose[0].p().z(), step_time, &leg_pos_z);
//     lerp_same(before_landpos[swingleg].q(), step_time, &leg_pos_quat);
//     lerp_same(before_landpos[supleg].q(), step_time, &sup_leg_quat);
//   } else {
//     // x, y
//     lerp_pose(before_landpos[swingleg], before_landpos[swingleg],
//               double_sup_time * 0.5, &swing_leg_pos);
//     lerp_pose(before_landpos[swingleg], ref_landpos_leg_w[swingleg],
//               single_sup_time, &swing_leg_pos);
//     lerp_pose(ref_landpos_leg_w[swingleg], ref_landpos_leg_w[swingleg],
//               double_sup_time * 0.5, &swing_leg_pos);
// 
//     // z
//     const double grand_h = init_pose[0].p().z();
//     lerp_same(grand_h, double_sup_time * 0.5, &leg_pos_z);
//     lerp_d(grand_h, grand_h + leg_h, single_sup_time * 0.5, &leg_pos_z);
//     lerp_d(grand_h + leg_h, grand_h, single_sup_time * 0.5, &leg_pos_z);
//     lerp_same(grand_h, double_sup_time * 0.5, &leg_pos_z);
// 
//     // yaw
//     // swing leg
//     Quaternion start_quat = before_landpos[swingleg].q();
//     Quaternion finish_quat = ref_landpos_leg_w[swingleg].q();
//     lerp_same(start_quat, double_sup_time * 0.5, &leg_pos_quat);
//     lerp_q(start_quat, finish_quat, single_sup_time, &leg_pos_quat);
//     lerp_same(finish_quat, double_sup_time * 0.5, &leg_pos_quat);
//     // support leg
//     start_quat = before_landpos[supleg].q();
//     finish_quat = ref_landpos_leg_w[supleg].q();
//     lerp_same(start_quat, double_sup_time * 0.5, &sup_leg_quat);
//     lerp_q(start_quat, finish_quat, single_sup_time, &sup_leg_quat);
//     lerp_same(finish_quat, double_sup_time * 0.5, &sup_leg_quat);
//   }
// 
//   // union
//   Pose sup_leg_pose;
//   for (int i = 0, n = swing_leg_pos.size(); i < n; ++i) {
//     // swing leg
//     Vector3 tmp_p(swing_leg_pos[i].p().x(), swing_leg_pos[i].p().y(),
//                   leg_pos_z[i]);
//     swing_leg_pos[i].set(tmp_p, leg_pos_quat[i]);
//     r_leg_pos[swingleg].push_back(swing_leg_pos[i]);
// 
//     // support leg
//     sup_leg_pose.set(before_landpos[supleg].p(), sup_leg_quat[i]);
//     r_leg_pos[supleg].push_back(sup_leg_pose);
//     // std::cout << i << std::endl;
//   }
// 
//   before_landpos[swingleg] = swing_leg_pos.back();
//   before_landpos[supleg] = sup_leg_pose;
// }


Quat LegTrack::lerp_q(Quat start, Quat finish, double normt) {
    return start.slerp(normt, finish);
}
}  // namespace cp
