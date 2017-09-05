# cpgen
"cpgen" is walking pattern generator of bipedal humanoid robot by Capture Point.

## simple exsample
```c++
#include <cpgen/cpgen.h>

int main(int argc, char** argv) {
  double dt = 5e-3;
  double single_sup_time = 0.5;
  double double_sup_time = 0.2;
  double cog_h = com[2];
  double leg_h = 0.02;

  // initialize cpgen
  // com : now center of mass
  // init_leg_pos : now both leg position
  // base2leg : both leg rotation matrix(now no use)
  cp::cpgen cpgen(com, init_leg_pos, base2leg, dt,
                  single_sup_time, double_sup_time, cog_h, leg_h);

  // set next landing position(x[m], y[m], theta[rad])
  // always can change this parameter(The change will be reflected next step)
  cp::Vector3 land_pos(0.0, 0.0, cp::deg2rad(0.0));

  // walking start
  cpgen.start();
  while (true) {
    cpgen.setLandPos(land_pos);
    // get leg track and com track every cycle
    cpgen.getWalkingPattern(com, &wp_com, &wp_right_leg_pos, &wp_left_leg_pos);

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