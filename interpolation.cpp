#include "interpolation.h"


namespace cp {

template <typename T>
T interpolation<T>::lerp(T begin, T end, double normt) {
    return (begin + (end - begin)*normt);
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

template <typename T>
T interpolation<T>::inter5(double t) {
    return (a[0] + a[1]*t + a[2]*pow(t, 2) + a[3]*pow(t, 3) + a[4]*pow(t, 4) + a[5]*pow(t, 5));
}



template class interpolation<float>;
template class interpolation<double>;
template class interpolation<Vector2>;
template class interpolation<Vector3>;
} // namespace cp
