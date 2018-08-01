This project contains a gazebo model and ros plugin for simulating the Bitcraze MultiRanger deck for Crazyflie 2.0.
MultiRanger is currently unreleased and this project is a work in progress.

# Dependencies
Currently this project requires Gazebo 9

# Installation
Clone this repository to a catkin workspace and build from source using
`catkin_make multiranger_deck_gazebo_ros`
This will put the gazebo plugin library and model in the devel/share folder of the workspace.

# How to use
Relative to your catkin workspace location, add devel/lib to CATKIN_PLUGIN_PATH
and devel/share/multiranger_deck_gazebo_ros to CATKIN_MODEL_PATH.
Then, start a roscore and Gazebo with
`roscore & rosrun gazebo_ros gazebo`
Finally, just insert the multiranger model into the Gazebo world.

# Acknowledgements
This project benefited from the open-source Hector Gazebo Plugins repository.
In particular, the sonar gazebo plugin in that repository was used as a reference.

# TODO
- Remake the collision and visual elements of the model to match the dimensions of the sensor
- Add an inertia / mass model
- Parameterize the model to implement the various operating modes of the VL53L1X (angle spread, range, update rate etc)
- Add parameter to disable visual rendering of the sensor rays