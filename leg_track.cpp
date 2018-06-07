#include "leg_track.h"

namespace cp {

// setting initial value
// need to call this before call getLegTrack
void LegTrack::init_setup(double sampling_time, double single_sup_time,
                          double double_sup_time, double legh,
                          const Affine3d now_leg_pose[2], const Quat& waist_r) {
  init_pose[right].set(now_leg_pose[right]);
  init_pose[left].set(now_leg_pose[left]);
  bfr_landpose[right].set(now_leg_pose[right]);
  bfr_landpose[left].set(now_leg_pose[left]);
  ref_landpose[right].set(now_leg_pose[right]);
  ref_landpose[left].set(now_leg_pose[left]);

  ground_h = init_pose[0].p().z();
  waist = waist_r;  ref_waist_r = waist_r;
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

// @brief set variable of a step. call only switch swing leg
// @param[in] ref_landpose_leg_w[2]: reference landing pose(world coodinate)
// @param[in] ref_waist: reference waist rotation
// @param[in] swingleg: next step swing leg
// @param[in] wstate: next step walking state
void LegTrack::setStepVar(const Pose ref_landpose_leg_w[],
     const Quat &ref_waist, rl swingleg, walking_state wstate) {
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
  bfr_waist_r = ref_waist_r;
  ref_waist_r = ref_waist;

  // for (x, y) lerp
  bfr << bfr_landpose[swl].p().x(), bfr_landpose[swl].p().y();
  ref << ref_landpose[swl].p().x(), ref_landpose[swl].p().y();
  // for z lerp
  inter_z_1.setInter5(ground_h, 0.0, 0.0, leg_h, 0.0, 0.0, sst*0.5);
  inter_z_2.setInter5(leg_h, 0.0, 0.0, ground_h, 0.0, 0.0, sst*0.5);
}

// @brief calculate next roop leg pose
// @param[in] t: delta step time.  0 <= t < single support time + double support time
// @param[out] r_leg_pose: return next roop leg pose
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
        waist = inter_q.lerp(bfr_waist_r, ref_waist_r, sst_s, sst_s_t);
    } else if (t < dst_s*0.5 + sst_s) {
        double sst_s_t = t - dst_s*0.5;
        double sst_s_ht = sst_s_t - sst_s*0.5;
        Vector2 nex = inter_vec2.lerp(bfr, ref, sst_s, sst_s_t);
        r_leg_pose[swl].set(Vector3(nex.x(), nex.y(), inter_z_2.inter5(sst_s_ht)));
        r_leg_pose[swl].set(inter_q.lerp(bfr_landpose[swl].q(), ref_landpose[swl].q(), sst_s, sst_s_t));
        r_leg_pose[spl].set(inter_q.lerp(bfr_landpose[spl].q(), ref_landpose[spl].q(), sst_s, sst_s_t));
        waist = inter_q.lerp(bfr_waist_r, ref_waist_r, sst_s, sst_s_t);
    } else if (t <= st_s) {
        r_leg_pose[swl].set(ref_landpose[swl]);
        r_leg_pose[spl].set(ref_landpose[spl]);
    }
    r_leg_pose[spl].set(bfr_landpose[spl].p());
  }
}

}  // namespace cp
