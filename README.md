# Webots manipulator demo

## Getting started

To copy a demo world from Webots repo, I had to do this:

1. Open for example the "WEBOTS_HOME/projects/robots/universal_robots/worlds/ure.wbt" world file.
2. If the simulation was running, stop it and revert it.
3. Then, open one of the robot nodes in the scene tree and change its controller field from `ure_can_grasper` to <extern>.
4. Save the simulation, restart it and run it.

Reference: https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?version=R2019b-rev1#example-usage

## Run Webots

```bash
# Run world
webots universal_robots/worlds/ure.wbt
# Run external controller
python3 universal_robots/controllers/my_controller/my_controller.py
```

## To do

* Test OpenVLA: https://github.com/openvla/openvla?tab=readme-ov-file#getting-started
* Test LeRobot: https://github.com/huggingface/lerobot/blob/main/examples/11_use_lekiwi.md#j-train-a-policy
