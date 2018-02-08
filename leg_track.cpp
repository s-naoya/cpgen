#include "leg_track.h"

namespace cp {

// setting initial value
// need to call this before call getLegTrack
void LegTrack::init_setup(double sampling_time, double single_sup_time,
                          double double_sup_time, double legh,
                          Pose now_leg_pose[2], const Quat& waist_r) {
  init_pose[right].set(now_leg_pose[right]);
  init_pose[left].set(now_leg_pose[left]);
  bfr_landpose[right].set(now_leg_pose[right]);
  bfr_landpose[left].set(now_leg_pose[left]);
  ref_landpose[right].set(now_leg_pose[right]);
  ref_landpose[left].set(now_leg_pose[left]);

  ground_h = init_pose[0].p().z();
  waist = waist_r;
  setup(sampling_time, single_sup_time, double_sup_time, legh);
}

// always can change these value
void LegTrack::setup(double sampling_time, double single_sup_time,
                     double double_sup_time, double legh) {
  dt  = sampling_time;
  sst = single_sup_time;
  dst = double_sup_time;
  st  = single_sup_time + double_sup_time;
  leg_h = legh + ground_h;
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
  bfr_landpose[right].set(ref_landpose[right]);
  bfr_landpose[left].set(ref_landpose[left]);
  ref_landpose[right].set(ref_landpose_leg_w[right]);
  ref_landpose[left].set(ref_landpose_leg_w[left]);

  // std::cout << "[cpgen] bfr right p: " << bfr_landpose[right].p() << std::endl;
  // std::cout << "[cpgen] bfr left  p: " << bfr_landpose[left].p() << std::endl;
  // std::cout << "[cpgen] ref right p: " << ref_landpose[right].p() << std::endl;
  // std::cout << "[cpgen] ref left  p: " << ref_landpose[left].p() << std::endl;

  // for (x, y) lerp
  bfr << bfr_landpose[swl].p().x(), bfr_landpose[swl].p().y();
  ref << ref_landpose[swl].p().x(), ref_landpose[swl].p().y();
  // for z lerp
  inter_z_1.setInter5(ground_h, 0.0, 0.0, leg_h, 0.0, 0.0, sst*0.5);
  inter_z_2.setInter5(leg_h, 0.0, 0.0, ground_h, 0.0, 0.0, sst*0.5);
}

void LegTrack::getLegTrack(double t, Pose r_leg_pose[]) {
  rl spl = swl == right ? left : right;
  if (ws == starting1 || ws == stopping2) {
      r_leg_pose[right].set(bfr_landpose[right]);
      r_leg_pose[left].set(bfr_landpose[left]);
  } else {
    if (t < dst_s*0.5) {
        r_leg_pose[swl].set(bfr_landpose[swl]);
        r_leg_pose[spl].set(bfr_landpose[spl].q());
    } else if (t < dst_s*0.5 + sst_s*0.5) {
        double sst_s_t = t - dst_s*0.5;
        Vector2 nex = inter_vec2.lerp(bfr, ref, sst_s, sst_s_t);
        r_leg_pose[swl].set(Vector3(nex.x(), nex.y(), inter_z_1.inter5(sst_s_t)));
        r_leg_pose[swl].set(inter_q.lerp(bfr_landpose[swl].q(), ref_landpose[swl].q(), sst_s, sst_s_t));
        r_leg_pose[spl].set(inter_q.lerp(bfr_landpose[spl].q(), ref_landpose[spl].q(), sst_s, sst_s_t));
    } else if (t < dst_s*0.5 + sst_s) {
        double sst_s_t = t - dst_s*0.5;
        double sst_s_ht = sst_s_t - sst_s*0.5;
        Vector2 nex = inter_vec2.lerp(bfr, ref, sst_s, sst_s_t);
        r_leg_pose[swl].set(Vector3(nex.x(), nex.y(), inter_z_2.inter5(sst_s_ht)));
        r_leg_pose[swl].set(inter_q.lerp(bfr_landpose[swl].q(), ref_landpose[swl].q(), sst_s, sst_s_t));
        r_leg_pose[spl].set(inter_q.lerp(bfr_landpose[spl].q(), ref_landpose[spl].q(), sst_s, sst_s_t));
    } else if (t <= st_s) {
        r_leg_pose[swl].set(ref_landpose[swl]);
        r_leg_pose[spl].set(ref_landpose[spl]);
    }
    r_leg_pose[spl].set(bfr_landpose[spl].p());
  }
}

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

}  // namespace cp
