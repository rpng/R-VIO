# R-VIO

R-VIO is an efficient, lightweight, **robocentric** visual-inertial navigation algorithm for consistent 3D motion tracking using only a monocular camera and a single IMU. Different from the standard world-centric algorithms which directly estimate absolute motion of the mobile platform with respect to a fixed, gravity-aligned, global frame of reference, R-VIO i) estimates relative motion of higher accuracy with respect to a moving, local frame (the IMU frame here), and ii) incrementally updates global pose (orientation and position) through a composition step. This code is developed with the robocentric sliding-window filtering-based VIO framework that was originally proposed in our *IROS2018* paper and further extended in our recent *IJRR* paper:

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *The International Journal of Robotics Research (IJRR)*, July 2019: [download](https://journals.sagepub.com/doi/10.1177/0278364919853361).

```
@article{huai2019robocentric,
  title     = {Robocentric visual-inertial odometry},
  author    = {Huai, Zheng and Huang, Guoquan},
  journal   = {The International Journal of Robotics Research},
  publisher = {SAGE Publications Sage UK: London, England},
  year      = {2019},
  url       = {https://journals.sagepub.com/doi/10.1177/0278364919853361}
}
```

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Madrid, Spain, Oct 1-5, 2018: [download](https://ieeexplore.ieee.org/document/8593643).

```
@inproceedings{huai2018robocentric,
  title     = {Robocentric visual-inertial odometry},
  author    = {Huai, Zheng and Huang, Guoquan},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages     = {6319--6326},
  year      = {2018},
  address   = {Madrid, Spain}
}
```

IROS video (ETH **EuRoC MAV** dataset): [YouTube](https://www.youtube.com/watch?v=UtiZ0EKa55M).
![](https://media.giphy.com/media/RMecOYlfxEcy4T8JdS/giphy.gif)

IJRR video (Our 9.8km **Urban Driving** dataset): [YouTube](https://www.youtube.com/watch?v=l9IC2ddBEYQ).
![](rvio.gif)

## 1. Prerequisites

We have tested this code under Ubuntu **16.04** and ROS **Kinetic**.

### ROS
Download and install instructions can be found at: http://wiki.ros.org/kinetic/Installation/Ubuntu.

Additional ROS packages needed: tf, sensor_msgs, geometry_msgs, nav_msgs, cv_bridge, eigen_conversions.

### Eigen
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### OpenCV
Download and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3**. **Tested with 2.4.11 and 3.3.1**.

## 2. Build and Run
First, `git clone` the repository and `catkin_make` it. Then, to run `rvio` with single camera/IMU inputs from the ROS topics `/camera/image_raw` and `/imu`, a config file in *config* folder and the corresponding launch file in *launch* folder (for example, `rvio_euroc.yaml` and `euroc.launch` are for [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset) are needed, and to visualize the outputs of R-VIO please use `rviz` with the settings file `rvio_rviz.rviz` in *config* folder.
  ```
  Terminal 1: roscore
  ```
  ```
  Terminal 2: rviz (AND OPEN rvio_rviz.rviz IN THE CONFIG FOLDER)
  ```
  ```
  Terminal 3: roslaunch rvio euroc.launch
  ```
  ```
  Terminal 4: rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/image_raw /imu0:=/imu
  ```

Note that when testing the `Machine Hall` sequences, you should skip the data in the first few seconds (e.g., 40s for `MH_01_easy`) which are used for initializing the map for SLAM-based algorithms.

You can also run R-VIO with your own sensors (data) by creating a config file `rvio_NAME_OF_YOUR_DATA.yaml` in *config* folder and the corresponding launch file `NAME_OF_YOUR_DATA.launch` in *launch* folder, referring to our EuRoC example.

## 3. License

This code is released under [GNU General Public License v3 (GPL-3.0)](https://www.gnu.org/licenses/gpl-3.0.en.html).
