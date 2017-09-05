#ifndef ROBOTIS_OP_CPWALK_COM_TRACK_H_
#define ROBOTIS_OP_CPWALK_COM_TRACK_H_

#include <iostream>
#include <cmath>

#include "eigen_types.h"

namespace cp {

// Calc CoM track class.
// It used by cpgen class only.
class CoMTrack {
 public:
  CoMTrack() {}
  ~CoMTrack() {}


  void init_setup(double t, double sst, double dst, double cogh, const Vector3& com);
  void setup(double t, double sst, double dst, double cogh);

  Vector3 getCoMTrack(const Vector3& com, const Vector2& end_cp,
                      double step_delta_time);
  Vector2 calcRefZMP(const Vector2& end_cp);

 private:
  Vector2 calcCPTrack(double step_delta_time);
  void calcCoMTrack(const Vector2& ref_cp, const Vector3& com);

  double step_time;        // single + double [s]
  double dt;               // sampling time [s]
  double single_sup_time;  // [s]
  double double_sup_time;  // [s]
  double cog_h;
  double w;

  Vector3 ref_com;
  Vector2 now_cp;
  Vector2 ref_zmp;
};
}  // namespace cp

#endif  // ROBOTIS_OP_CPWALK_COM_TRACK_H_
