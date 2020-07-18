# webots_ros

The `webots_manipulation` package contains training code of Webots with MoveIt!.

## Getting started

- Build a dockerfile from `webots_docker`. Instructions in that repo.
- Organize the project like this:

```
worskpace
|
|___webots_docker
|___ws
    |___src
        |___universal_robot
        |___webots_manipulation
```

- Download `universal_robot` from [here](https://github.com/eborghi10/universal_robot).

## Instructions

### Minimal testing

```bash
roslaunch webots_manipulation minimal.launch
```

### Complete

```bash
roslaunch webots_manipulation complete.launch
```
