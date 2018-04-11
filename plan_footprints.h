#ifndef CPGEN_PLAN_FOOTPRINTS_H_
#define CPGEN_PLAN_FOOTPRINTS_H_

#include <iostream>
// #include <cmath>

#include "eigen_types.h"

namespace cp {

class PlanFootprints {
 public:
  PlanFootprints() {}
  ~PlanFootprints() {}

  void init_setup(const Affine3d init_leg_pose[], const Quat& waist_q,
                  const Vector3& com, const double endcpoff[]);

  void setValues(walking_state wstate, rl swingleg, Vector3 step_length);
  void update();

  walking_state whichwalk;  // now walk or step

  Pose ref_land_pose[2];    // world coodinate reference land position
  Pose ref_waist_pose;
  Quat ref_waist_r;
  Vector2 end_cp;

 private:
  void calcNextWaistPose();
  void calcNextFootprint();
  void calcNextFootprintOld();
  void calcEndCP();

  void whichWalkOrStep();
  bool isCollisionLegs(double y);
  double isCollisionLegs(double yn, double yb);

  rl swingleg;
  Vector3 step_length;
  walking_state wstate;
  Pose init_feet_pose[2];

  Vector3 dist_body2foot[2];
  double end_cp_offset[2]; // end-of-CP offset  (x, y)[m]

};

}  // namespace cp

#endif  // CPGEN_PLAN_FOOTPRINTS_H_
