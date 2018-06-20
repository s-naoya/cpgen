# cpgen
"cpgen" is walking pattern generator of bipedal humanoid robot by Capture Point.

## simple exsample
```c++
#include <cpgen/cpgen.h>

int main(int argc, char** argv) {
  const cp::Vector3 init_com = now_CoM_position;
  const cp::Affine3d init_waist_pose = now_waist_pose;
  const cp::Affine3d init_leg_pose[2] = {now_right_leg_pose, now_left_leg_pose};
  const cp::Quat base_to_leg[2];  // no use
  double end_cp_offset[2] = {0.0, 0.0};
  double sampling_time = 5e-3;
  double single_support_time = 0.5;
  double double_support_time = 0.2;
  double cog_height = init_com.z();
  double leg_height = 0.02;

  // initialize cpgen
  cp::cpgen cpgen();
  cpgen.initialize(
      init_com,
      init_waist_pose,
      init_leg_pose,
      base_to_leg,
      end_cp_offset,
      sampling_time,
      single_support_time,
      double_support_time,
      cog_height,
      leg_height
  );

  // set next landing position(x[m], y[m], theta[rad])
  // always can change this parameter(The change will be reflected next step)
  cp::Vector3 land_pos(0.0, 0.0, cp::deg2rad(0.0));

  // walking pattern value
  cp::Vector3 wp_com;
  cp::Pose wp_right_leg_pose, wp_left_leg_pose;
  cp::Quat wp_waist;

  // walking start
  cpgen.start();
  while (true) {
    cpgen.setLandPos(land_pos);
    // get leg track and com track every cycle
    cpgen.getWalkingPattern(&wp_com, &wp_waist, &wp_right_leg_pose, &wp_left_leg_pose);

    // can calculate inverse kinematics by wp_com and wp_(right and left)_leg_pos
  }
  // walking stop
  cpgen.stop();
  return 0;
}
```


## how to install
```sh
$ cmake .
$ make
$ make install
```


## necessary library
This library needs "Eigen3".


## LICENSE
This software is licensed under the MIT License.