#ifndef CPGEN_CPGEN_H_
#define CPGEN_CPGEN_H_

#include "com_track.h"
#include "leg_track.h"

namespace cp {

class cpgen {
 public:
  cpgen(const Vector3& com, const std::array<Affine3d, 2>& init_leg_pos,
        const std::array<cp::Quaternion, 2>& i_base2leg,
        double t, double sst, double dst, double cogh, double legh);
  ~cpgen() {}

  void setup(double t, double sst, double dst, double cogh, double legh);

  void start();
  void stop();
  void restart();
  void getWalkingPattern(const Vector3& com, Vector3* com_pos,
                         Pose* right_leg_pos, Pose* left_leg_pos);

  Pose setInitLandPos(const Affine3d& init_leg_pos);
  void setLandPos(const Vector3& pose);

 private:
  void setInitLandPos(const std::array<Affine3d, 2>& init_leg_pos);
  void calcLandPos();
  void calcEndCP();
  void whichWalkOrStep();
  bool isCollisionLegs(double y);

  CoMTrack comtrack;
  LegTrack legtrack;

  double dt;               // sampling time [s]
  double single_sup_time;  // [s]
  double double_sup_time;  // [s]
  double cog_h;            // height of center of gravity [m]
  double leg_h;            // height of up leg [m]

  std::array<cp::Quaternion, 2> base2leg;

  Vector3 land_pos;  // landing position x[m], y[m], theta[rad]
  rl swingleg;
  walking_state whichwalk;
  walking_state wstate;
  Vector2 end_cp;  // world coodinate
  Vector2 ref_zmp;
  double end_cp_offset[2];  // offset  x[m], y[m]

  // world coodinate reference land position
  std::array<Pose, 2> land_pos_leg_w;
  // designed leg track [right, left] world
  std::array<std::deque<Pose>, 2> designed_leg_track;
};

}  // namespace cp

#endif  // CPGEN_CPGEN_H_