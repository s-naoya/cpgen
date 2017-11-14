#ifndef CPGEN_LEG_TRACK_H_
#define CPGEN_LEG_TRACK_H_

#include <deque>
#include <iostream>

#include "eigen_types.h"

namespace cp {

// Calc leg track class.
// It used by cpgen class only.
class LegTrack {
 public:
  LegTrack() {}
  ~LegTrack() {}

  void init_setup(double t, double sst, double dst, double legh,
                  Pose now_leg_pose[]);
  void setup(double t, double sst, double dst, double legh);

  void getLegTrack(const rl swingleg, const Pose ref_landpos_leg_w[],
                   std::deque<Pose, Eigen::aligned_allocator<Pose> > r_leg_pos[]);

 private:
  void lerp_pose(const Pose start, const Pose finish, double tf,
                 std::deque<Pose, Eigen::aligned_allocator<Pose> >* input);

  void lerp_d(const double start, const double finish, double tf,
              std::deque<double>* input);

  void lerp_q(const Quaternion start, const Quaternion finish, double tf,
              std::deque<Quaternion, Eigen::aligned_allocator<Quaternion> >* input);
  void lerp_same(double x, double tf, std::deque<double>* input);
  void lerp_same(Quaternion x, double tf, std::deque<Quaternion,Eigen::aligned_allocator<Quaternion> >* input);

  double dt;               // sampling time [s]
  double single_sup_time;  // [s]
  double double_sup_time;  // [s]
  double leg_h;            // height of up leg [m]

  Pose init_pose[2];
  Pose before_landpos[2];
};

}  // namespace cp

#endif  // CPGEN_LEG_TRACK_H_