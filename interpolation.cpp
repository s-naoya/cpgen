#include "interpolation.h"
#include <iostream>


namespace cp {

template <typename T>
T interpolation<T>::lerp(T begin, T end, double lent, double nowt) {
    if (lent == 0.0) return begin;
    double normt = nowt/lent;
    return (begin + (end - begin)*normt);
}

template<>
Quat interpolation<Quat>::lerp(Quat begin, Quat end, double lent, double nowt) {
    if (lent == 0.0) return begin;
    double normt = nowt/lent;
    return (begin.slerp(normt, end));
}


template <typename T>
void interpolation<T>::setInter5(T xb, T dxb, T ddxb, T xe, T dxe, T ddxe, double t) {
    a[0] = xb;
    a[1] = dxb;
    a[2] = ddxb * 0.5;
    a[3] = (20.0*xe - 20.0*xb - ( 8.0*dxe + 12.0*dxb)*t - (3.0*ddxb -     ddxe)*pow(t, 2)) / (2.0*pow(t, 3));
    a[4] = (30.0*xb - 30.0*xe + (14.0*dxe + 16.0*dxb)*t + (3.0*ddxb - 2.0*ddxe)*pow(t, 2)) / (2.0*pow(t, 4));
    a[5] = (12.0*xe - 12.0*xb - ( 6.0*dxe +  6.0*dxb)*t - (    ddxb -     ddxe)*pow(t, 2)) / (2.0*pow(t, 5));
}

template<>
void interpolation<Quat>::setInter5(Quat xb, Quat dxb, Quat ddxb, Quat xe, Quat dxe, Quat ddxe, double t) {
    std::cout << "[cpgen] inter5 is not correspond Quaternion" << std::endl;
    return;
}

template <typename T>
T interpolation<T>::inter5(double t) {
    return (a[0] + a[1]*t + a[2]*pow(t, 2) + a[3]*pow(t, 3) + a[4]*pow(t, 4) + a[5]*pow(t, 5));
}

template<>
Quat interpolation<Quat>::inter5(double t) {
    std::cout << "[cpgen] inter5 is not correspond Quaternion" << std::endl;
    return Quat();
}


template class interpolation<float>;
template class interpolation<double>;
template class interpolation<Vector2>;
template class interpolation<Vector3>;
template class interpolation<Quat>;
} // namespace cp
