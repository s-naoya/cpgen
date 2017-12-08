#ifndef CPGEN_INTERPOLATION_H
#define CPGEN_INTERPOLATION_H

#include "eigen_types.h"


namespace cp {

template <typename T>
class interpolation {
public:
  T lerp(T begin, T end, double lent, double nowt);

  void setInter5(T xb, T dxb, T ddxb, T xe, T dxe, T ddxe, double t);
  T inter5(double t);

private:
  T a[6];
};
}  // namespace cp
#endif // CPGEN_INTERPOLATION_H
