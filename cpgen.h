#ifndef CPGEN_CPGEN_H_
#define CPGEN_CPGEN_H_

#include <cmath>

#include "com_track.h"
#include "leg_track.h"

namespace cp {

class cpgen {
 public:
  cpgen() {}
  ~cpgen() {}

  void initialize(const Vector3& com, const Affine3d init_leg_pos[],
        const Quaternion i_base2leg[], const double endcpoff[],
        double t, double sst, double dst,
        double cogh, double legh);
  void setup(double t, double sst, double dst, double cogh, double legh);

  void start();
  void stop();
  void estop();
  void changeSpeed(double scale);
  void getWalkingPattern(Vector3* com_pos, Pose* right_leg_pos, Pose* left_leg_pos);

  Pose setInitLandPos(const Affine3d& init_leg_pos);
  void setLandPos(const Vector3& pose);

  rl getSwingleg() {return swingleg;}
  Vector2 getRefZMP() {return ref_zmp;}
  Vector2 getEndCP() {return end_cp;}
  int getWstate() {return wstate;}

 private:
  void setInitLandPos(const Affine3d init_leg_pos[]);
  void calcLandPos();
  void calcEndCP();
  void whichWalkOrStep();
  bool isCollisionLegs(double y);
  double isCollisionLegs(double yn, double yb);

  CoMTrack comtrack;
  LegTrack legtrack;

  // parameter
  double dt;               // sampling time [s]
  double single_sup_time;  // single support time [s]
  double double_sup_time;  // double support time [s]
  double cog_h;            // height of center of gravity [m]
  double leg_h;            // height of up leg [m]
  double foot_wd_size;     // foot size of width [m]
  double feet_dist;        // distance of feet [m]
  double end_cp_offset[2]; // end-of-CP offset  (x, y)[m]
  double both_end_scale;

  Quaternion base2leg[2];

  Vector3 land_pos;         // landing position x[m], y[m], theta[rad]
  rl swingleg;              // which swing leg(0: right, 1: left)
  walking_state whichwalk;  // now walk or step
  walking_state wstate;     // now walking state (definition is eigen_types.h)
  Vector2 end_cp;           // end-of-CP world coodinate
  Vector2 ref_zmp;          // reference ZMP (calc by CoMTrack class)

  Pose land_pos_leg_w[2];   // world coodinate reference land position
  // designed leg track [right, left] world
  std::deque<Pose, Eigen::aligned_allocator<Pose> > designed_leg_track[2];
};

}  // namespace cp

#endif  // CPGEN_CPGEN_H_
