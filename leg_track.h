#ifndef ROBOTIS_OP_CPWALK_LEG_TRACK_H_
#define ROBOTIS_OP_CPWALK_LEG_TRACK_H_

#include <deque>

#include "eigen_types.h"

namespace cp {

// Calc leg track class.
// It used by cpgen class only.
class LegTrack {
 public:
  LegTrack() {}
  ~LegTrack() {}

  void init_setup(double t, double sst, double dst, double legh,
             std::array<Pose, 2>& now_leg_pose);
  void setup(double t, double sst, double dst, double legh);

  void getLegTrack(const rl swingleg,
                   const std::array<Pose, 2>& ref_landpos_leg_w,
                   std::array<std::deque<Pose>, 2>& r_leg_pos);

 private:
  void lerp_pose(const Pose start, const Pose finish, double tf,
                 std::deque<Pose>* input);

  void lerp_d(const double start, const double finish, double tf,
              std::deque<double>* input);

  void lerp_q(const Quaternion start, const Quaternion finish, double tf,
              std::deque<Quaternion>* input);

  template <class T>
  void lerp_same(T x, double tf, std::deque<T>* input) {
    int num = (int)(tf / dt + 1e-8);
    for (int i = 1; i < num + 1; i++) {
      input->push_back(x);
    }
  }

  double dt;               // sampling time [s]
  double single_sup_time;  // [s]
  double double_sup_time;  // [s]
  double leg_h;            // height of up leg [m]

  std::array<Pose, 2> init_pose;
  std::array<Pose, 2> before_landpos;
};

}  // namespace cp

#endif  // ROBOTIS_OP_CPWALK_LEG_TRACK_H_