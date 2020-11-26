# webots_ros

The `webots_manipulation` package contains training code of Webots with MoveIt!.

## Getting started

- Build a dockerfile from `webots_docker`. Instructions in that repo.
- Organize the project like this:

```no-lang
worskpace
|
|___webots_docker
|___ws
    |___src
        |___robotiq
        |___universal_robot
        |___webots_manipulation
```

Clone the following dependencies into your workspace:

- [`robotiq`](https://github.com/ros-industrial/robotiq)
- [`universal_robot`](https://github.com/eborghi10/universal_robot)
- [`ur5_e_moveit_config`](https://github.com/eborghi10/ur5_e_moveit_config)

## Instructions

### Minimal testing

```bash
roslaunch webots_manipulation minimal.launch
```

### Complete

```bash
roslaunch webots_manipulation complete.launch
```

### [Old] Pick and place

```bash
rosrun webots_manipulation pick_and_place
```

### Grasping generator and pipeline

```bash
roslaunch webots_manipulation grasp_pipeline.launch
```

### Motion planning pipeline

```bash
roslaunch webots_manipulation motion_planning_pipeline.launch
```

---

## Tips

- Getting `move_group`s. Open the MoveIt! Command Line Tool:

```bash
$ rosrun moveit_commander moveit_commander_cmdline.py
> use [TAB]
```

<!-- GPD remove -O3 and set PCL + OpenCv versions -->
