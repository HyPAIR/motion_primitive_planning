# Motion Primitives Planning Package
A ROS package for simulation. Given a graph and traversal order, and a set of motion primitives, compute the optimal trajectory satisfying the kinematic constraints and visualized in gazebo and rviz.

## Requirements

 - ROS Kinetic or later
 - Ubuntu 16.04 or later

## Installation

1. Create a new workspace:

```shell
mkdir -p ~/motion_primitive_planning/src
cd ~/motion_primitive_planning/src
catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
git clone https://github.com/HyPAIR/motion_primitive_planning.git
```

3. Build the workspace:
```shell
cd ..
catkin_make
source devel/setup.bash
```

4. If fail to build the workspace, install dependencies:

```shell
cd src/motion_primitive_planning
rosdep install motion_primitive_planner
cd ..
```
Then, back to step3 and try again.

## Usage
### Compute the optimal trajectory
1. Save the ```data.json``` file to ```~/motion_primitive_planning/src/motion_primitive_planning/data/```.

2. Launch the node to compute the optimal trajectory:

    ```shell
    roslaunch motion_primitive_planning compute_optimal_traj.launch
    ```
3. Launch the node to visulize the trajectory in simulation:

    ```shell
   roslaunch motion_primitive_planning visualize.launch
    ```
    
![kf_trajectory](https://github.com/HyPAIR/motion_primitive_planning/blob/main/figure/kf_trajectory.png)

### Test the real robot in rviz and gazebo
1. Launch the node to visulize the robot in gazebo and rviz:

    ```shell
   roslaunch mir_examples single_mir_100.launch 
    ```
    
2. Launch the node to visualize the map

    ```shell
   roslaunch motion_primitive_planning visualize_real.launch 
    ```
    
3. Launch the node to let the robot track the given trajectory

    ```shell
   roslaunch motion_primitive_planning sarah_traj_tracking.launch 
    ```
![real_robot_sim](https://github.com/HyPAIR/motion_primitive_planning/blob/main/figure/real_robot_sim.png)
