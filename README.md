# avoi_dance
This project demonstrates two iRobot Create-3 robots autonomously navigating without collision, using ROS2 for intelligent path planning and obstacle avoidance in a shared space.

## Requirements
* [`ROS 2 Humble`](https://docs.ros.org/en/ros2_documentation/humble/) on `Ubuntu 22.04 LTS`
* [`Gazebo11`](https://classic.gazebosim.org/https://classic.gazebosim.org/)
* `Python >= 3.10`
* `C++ >= 17`
* Submodule [`aws-robomaker-small-house-world`](git@github.com:aws-robotics/aws-robomaker-small-house-world.git)
* Submodule [`create3_sim`](git@github.com:iRobotEducation/create3_sim.git)

## Setup Instructions
* Create a workspace:
    ```bash
    $ cd ~
    $ mkdir -p create3_ws/src
    ```
* Clone the repo:
    ```bash
    $ cd ~/create3_ws/src
    $ git clone git@github.com:DJacquemont/avoi_dance.git
    ```
* Setup the submodules:
    ```bash
    $ cd ~/create3_ws/src
    $ git submodule update --init --recursive
    ```

## Build and Run Instructions
* Build the workspace:
    ```bash
    $ cd ~/create3_ws
    $ colcon build
    ```
* Run the ROS2 packages and Gazebo simulation:
    ```bash
    $ cd ~/create3_ws
    $ source install/local_setup.bash
    $ ros2 launch avoi_dance avoi_dance.launch.py
    ```

