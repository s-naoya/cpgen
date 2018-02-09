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

  void init_setup(const Affine3d init_leg_pose[], const Quat& waist_r,
                  const Vector3& com, const double endcpoff[]);

  void setValues(walking_state ws, rl swing_leg, Vector3 landpos);
  void update();

  walking_state whichwalk;  // now walk or step

  Pose ref_land_pose[2];    // world coodinate reference land position
  Quat ref_waist_r;
  Vector2 end_cp;

 private:
  void calcNextFootprint();
  void calcEndCP();

  void whichWalkOrStep();
  bool isCollisionLegs(double y);
  double isCollisionLegs(double yn, double yb);

  rl swingleg;
  Vector3 land_pos;
  walking_state wstate;
  Pose init_feet_pose[2];

  double end_cp_offset[2]; // end-of-CP offset  (x, y)[m]

};

}  // namespace cp

#endif  // CPGEN_PLAN_FOOTPRINTS_H_
