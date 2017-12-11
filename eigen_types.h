// original is "choreonoid"

#ifndef ROBOTIS_OP_CPWALK_EIGEN_TYPES_H_
#define ROBOTIS_OP_CPWALK_EIGEN_TYPES_H_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <Eigen/AlignedVector3>

namespace cp {

using Eigen::Vector2i;
using Eigen::Matrix2f;
using Eigen::Vector2f;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Array2i;
using Eigen::Array2f;
using Eigen::Array2d;

using Eigen::Vector3i;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Array3i;
using Eigen::Array3f;
using Eigen::Array3d;

using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Array4i;
using Eigen::Array4f;
using Eigen::Array4d;

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;

using Eigen::Affine3f;
using Eigen::Affine3d;
using Eigen::Translation3f;
using Eigen::Translation3d;

typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Vector3d Vector3;
// typedef Eigen::AlignedVector3<double> Vector3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Vector4d Vector4;
typedef Eigen::VectorXd VectorX;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Affine3d Affine3;
typedef Eigen::Translation3d Translation3;
typedef Eigen::AngleAxisd AngleAxis;

//! \deprecated
typedef Eigen::Quaterniond Quat;

typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Transform<double, 3, Eigen::AffineCompact> EPosition;

// The followings should be removed later
using Eigen::Isometry3f;
using Eigen::Isometry3d;
typedef Eigen::Isometry3d EIsometry3;

// constexpr double PI = 3.14159265358979323846;
// constexpr double PI_2 = 1.57079632679489661923;
const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;

// constexpr double TO_DEGREE = 180.0 / PI;
// constexpr double TO_RADIAN = PI / 180.0;
const double TO_DEGREE = 180.0 / PI;
const double TO_RADIAN = PI / 180.0;

inline double rad2deg(double rad) { return TO_DEGREE * rad; }
inline double deg2rad(double deg) { return TO_RADIAN * deg; }
inline double deg2rad(int deg) { return TO_RADIAN * deg; }

inline Quaternion rpy2q(double roll, double pitch, double yaw) {
  AngleAxisd rollAngle(roll, Vector3d::UnitZ());
  AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
  AngleAxisd yawAngle(yaw, Vector3d::UnitX());

  Quaternion q = rollAngle * yawAngle * pitchAngle;
  return q;
}

inline Matrix3 rpy2mat(double roll, double pitch, double yaw) {
  AngleAxisd rollAngle(roll, Vector3d::UnitZ());
  AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
  AngleAxisd yawAngle(yaw, Vector3d::UnitX());

  Quaternion q = rollAngle * yawAngle * pitchAngle;
  Matrix3 rotationMatrix = q.matrix();
  return rotationMatrix;
}

inline Vector3 mat2rqy(Matrix3 mat) { return mat.eulerAngles(0, 1, 2); }
inline Vector3 q2rpy(Quaternion q) {
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}
inline Quat mat2q(Matrix3 mat) {
  Quat q(mat); return q;
}

enum rl { right, left, both };
// enum walking_state { stopped, starting, walk, next_stop, stopping, stepping
// };
enum walking_state {
  // state
  stopped = 0,
  walk = 1,
  step = 2,

  // transitions
  stopping1 = 3,   // (walk or step) -> stop
  stopping2 = 4,   // (walk or step) -> stop
  stop_next = 5,  // stopping when switch next step

  starting1 = 6,  // stop -> (walk or step)
  starting2 = 7,

  walk2step = 8,  // walk -> step
  step2walk = 9   // step -> walk
};

class Pose {
  Vector3 pp;
  Quat qq;
  Affine3 af;

 public:
  Pose() {}
  Pose(const Vector3& translation, const Quat& rotation)
      : pp(translation), qq(rotation) {}
  Pose(const Vector3& translation, const Matrix3& rotation)
      : pp(translation), qq(rotation) {}

  void set(const Vector3& translation, const Quat& rotation) {
    this->pp = translation;
    this->qq = rotation;
  }
  void set(const Vector3& translation, const Matrix3& R) {
    this->pp = translation;
    this->qq = R;
  }
  void set(const Vector3& translation) {this->pp = translation;}
  void set(const Quat& rotation) {this->qq = rotation;}
  void set(const Matrix3& R) {this->qq = R;}
  void set(const Pose& pose) {this->pp = pose.p();  this->qq = pose.q();}
  Vector3& p() { return pp; }
  const Vector3& p() const { return pp; }
  Quat& q() { return qq; }
  const Quat& q() const { return qq; }

  Affine3& affine() {
    Translation3 tr = Translation3(pp.x(), pp.y(), pp.z());
    af = tr * qq;
    return af;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
inline Pose affine2pose(const Affine3d& init_leg_pose) {
  Vector3 trans = init_leg_pose.translation();
  Quaternion q = Quaternion(init_leg_pose.rotation());
  cp::Pose pose(trans, q);
  return pose;
}
}

#endif
