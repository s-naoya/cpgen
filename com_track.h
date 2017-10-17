#ifndef CPGEN_COM_TRACK_H_
#define CPGEN_COM_TRACK_H_

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

  Vector3 getCoMTrack(const Vector2& end_cp, double step_delta_time);
  Vector2 calcRefZMP(const Vector2& end_cp);

 private:
  void setStepVariable();
  Vector2 calcCPTrack(double step_delta_time);
  void calcCoMTrack(const Vector2& ref_cp);

  double step_time;        // single + double [s]
  double dt;               // sampling time [s]
  double single_sup_time;  // [s]
  double double_sup_time;  // [s]
  double cog_h;            // [m]
  double w;

  // for calcurate a step
  double step_time_s;
  double dt_s;
  double w_s;

  Vector3 ref_com;
  Vector2 now_cp;
  Vector2 ref_zmp;
};
}  // namespace cp

#endif  // CPGEN_COM_TRACK_H_
