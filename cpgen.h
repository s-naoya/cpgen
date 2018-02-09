#ifndef CPGEN_CPGEN_H_
#define CPGEN_CPGEN_H_

#include <cmath>

#include "com_track.h"
#include "leg_track.h"
#include "plan_footprints.h"

namespace cp {

class cpgen {
 public:
  cpgen() {}
  ~cpgen() {}

  void initialize(
      const Vector3& com, const Quat& waist_r, const Affine3d init_leg_pose[],
      const Quat i_base2leg[], const double endcpoff[],
      double t, double sst, double dst, double cogh, double legh);
  void setup(double t, double sst, double dst, double cogh, double legh);

  void start();
  void stop();
  void estop();
  void getWalkingPattern(Vector3* com_pos, Quat* waist_r,
                         Pose* right_leg_pose, Pose* left_leg_pose);

  void setLandPos(const Vector3& pos);

  rl getSwingleg() {return swingleg;}
  Vector2 getRefZMP() {return ref_zmp;}
  // Vector2 getEndCP() {return end_cp;}
  int getWstate() {return wstate;}

 private:
  void calcLandPos();
  void whichWalkOrStep();
  bool isCollisionLegs(double y);
  double isCollisionLegs(double yn, double yb);

  CoMTrack comtrack;
  LegTrack legtrack;
  PlanFootprints pf;

  // parameter
  double dt;               // sampling time [s]
  double single_sup_time;  // single support time [s]
  double double_sup_time;  // double support time [s]
  double cog_h;            // height of center of gravity [m]
  double leg_h;            // height of up leg [m]

  Quat base2leg[2];

  Vector3 land_pos;         // landing position x[m], y[m], theta[rad]
  rl swingleg;              // which swing leg(0: right, 1: left)
  walking_state wstate;     // now walking state (definition is eigen_types.h)
  Vector2 ref_zmp;          // reference ZMP (calc by CoMTrack class)

  Vector3 dist_body_foot[2];
};

}  // namespace cp

#endif  // CPGEN_CPGEN_H_
