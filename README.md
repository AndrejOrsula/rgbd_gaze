# RGB-D Gaze
![OS](https://img.shields.io/badge/OS-Ubuntu_18.04-orange.svg) ![ROS_2](https://img.shields.io/badge/ROS_2-Eloquent-brightgreen.svg)

This repository implements gaze estimation by the use of 3D model method and RGB-D camera. The pupil centre localisation is implemented with the gradients based method proposed in [1]. Early tests suggest gaze accuracy of ~5.14°, where most inaccuracies originate from calibration of eyeball centre (issue for enhancement is opened and contributions are welcome).


## Dependencies

- [ros2_openface](https://github.com/AndrejOrsula/ros2_openface)
- [openface_rgbd_gaze_bridge](https://github.com/AndrejOrsula/openface_rgbd_gaze_bridge)


## Building

First, clone this repository into your favourite workspace. Then build all packages with [colcon](https://colcon.readthedocs.io/en/released/user/installation.html).
```bash
mkdir -p <awesome_ws>/src && cd <awesome_ws>/src
git clone https://github.com/AndrejOrsula/rgbd_gaze
cd ..
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

## Usage

First, source the ROS 2 global installation (if not done before).
```bash
source /opt/ros/eloquent/setup.bash
```

Then source the ROS 2 workspace overlay (if not done before).
```bash
source <awesome_ws>/install/local_setup.bash
```

Finally, you can run the personal calibration and the gaze estimation itself.
```bash
# Personal calibration
ros2 launch rgbd_gaze calibration_eyeball.launch.py
ros2 launch rgbd_gaze calibration_kappa.launch.py
# Gaze estimation
ros2 launch rgbd_gaze rgbd_gaze.launch.py
```

## License
This project is licensed under [BSD 3-Clause License](LICENSE).


## References

[1] F. Timm and E. Barth, “Accurate eye centre localisation by means of gradients.” VISAPP 2011 - Proceedings of the International Conferenceon Computer Vision Theory and Application, pp. 125–130, 2011.

