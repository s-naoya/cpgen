#ifndef CPGEN_LEG_TRACK_H_
#define CPGEN_LEG_TRACK_H_

#include <iostream>

#include "interpolation.h"
#include "eigen_types.h"

namespace cp {

// Calc leg track class.
// It used by cpgen class only.
class LegTrack {
 public:
  LegTrack() {}
  ~LegTrack() {}

  void init_setup(double sampling_time, double single_sup_time,
                  double double_sup_time, double legh,
                  const Affine3d now_leg_pose[], const Quat& waist_r);
  void setup(double sampling_time, double single_sup_time,
             double double_sup_time, double legh);
  void setStepVar(const Pose ref_land_pose[], const Quat &ref_waist,
                  rl swingleg, walking_state wstate);
  void getLegTrack(double t, Pose r_leg_pose[]);
  Quat getWaistTrack(double step_delta_time) {return waist;}
  // void getLegTrack(const rl swingleg, const walking_state wstate,
  //                  const Pose ref_landpos_leg_w[],
  //                  std::deque<Pose, Eigen::aligned_allocator<Pose> > r_leg_pos[]);

 private:
  interpolation<double> inter_z_1, inter_z_2, inter_d;
  interpolation<Vector2> inter_vec2;
  interpolation<Quat> inter_q;
  double dt;     // sampling time [s]
  double sst;    // single support time [s]
  double dst;    // double support time [s]
  double st;     // step time = dst + sst
  double leg_h;  // height of up leg [m]
  double ground_h;

  // use this step
  double sst_s, dst_s, dt_s, st_s;

  Quat waist, ref_waist_r, bfr_waist_r;
  Pose ref_landpose[2];
  Vector2 bfr, ref;
  rl swl;  // swing leg
  walking_state ws;

  Pose init_pose[2];
  Pose bfr_landpose[2];
};

}  // namespace cp

#endif  // CPGEN_LEG_TRACK_H_
