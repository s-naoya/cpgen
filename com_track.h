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


  void init_setup(double sampling_time, double single_sup_time,
                  double double_sup_time, double cog_h,
                  const Vector3& com);
  void setup(double t, double single_sup_time,
             double double_sup_time, double cog_h);

  Vector3 getCoMTrack(const Vector2& end_cp, double step_delta_time);
  void calcRefZMP(const Vector2& end_cp);
  Vector2 getRefZMP() {return ref_zmp;}

 private:
  void setStepVariable();
  Vector2 calcCPTrack(double step_delta_time);
  void calcCoMTrack(const Vector2& ref_cp);

  double dt;    // sampling time [s]
  double sst;   // single support time [s]
  double dst;   // double support time [s]
  double st;    // step_time = single + double [s]
  double cogh;  // center of gravity height [m]
  double w;

  // for calcurate a step
  double st_s;
  double dt_s;
  double w_s;

  Vector3 ref_com;
  Vector2 now_cp;
  Vector2 ref_zmp;
};
}  // namespace cp

#endif  // CPGEN_COM_TRACK_H_
