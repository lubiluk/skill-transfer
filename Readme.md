### IROS 2018
# Skill Transfer

ROS package that realises transfer of manipulation skills from known objects and situations to new, unseen objects and their setups.

## Requirements

This package is **Developed and Tested on ROS Kinetic**.
For use with Gazebo simulator additional Gazebo models and plugins are needed. These can be found here: https://github.com/lubiluk/iros2018-gazebo
At it's core, the system makes use of Giskard library for robot control: https://github.com/SemRoCo/giskard_core

## Architecture

The package consists of multiple ROS nodes that work collectively for achieving the desired effects. They communicate in roughly following manner:

```
[EdgeDetector][ToolPointDetector] <--> [TaskExecutive] --> [ConstraintController] --> <Actuators>
```

*TaskExecutive* is the main node that supervises the whole process and sends requests to all other nodes.

*EdgeDetector* finds object edges and desired contact points on them.

~~*ToolPointDetector* finds important contact points on tools.~~ (Not implemented yet)

*ConstraintController* uses Giskard internally, translates motion description files into desired joint velocities.

### The Process

The whole process begins with *TaskExecutive* reading an experiment YAML file. 
Checks what points of the objects should be perceived, e.g container edge and asks vision nodes (*EdgeDetector*) for those.
Afterwards it the moiton sequence begins. *TaksExecutive* loads robot template file and in each task phase combines that template with motion constraints file. Also adds there grasping transformations and object points obtained from vision nodes. 
Such prepared motion phase file is then sent to *ConstraintController* for execution. While that happens *TaskExecutive* observes the state of the robot and decides when to finish one phase and begin the next one according to the task specification file.
When all motion phases are done the task is considered as finished.

### Configuration files

There are configuration files that describe different levels of the system: motions, tasks, experiments. All files are YAML.

*robot template* specifies the kinematic chain of a robot.

*motion phase* specifies motion in terms of constraints that should be satified.

*tasks* contains a sequence of motion phases and appropriate stop conditions that toghether form a full task description.

*experiments* specifies a task to be executed, objects that take part in the task, callibrated grasp transformations and visual features that should be resolved.

### Supported tasks

1. Scraping butter off a tool into a container
2. Scooping a substance (e.g. grains) from a container
3. Cutting an object on a flat object/surface

## Installation
* First, follow the instructions of iros2018-gazebo repository:
  https://github.com/lubiluk/iros2018-gazebo
* Install ROS, then:
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  cd src
  wstool init
  wstool merge https://raw.githubusercontent.com/lubiluk/iros2018/master/rosinstall/catkin.rosinstall
  wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_core/master/rosinstall/catkin.rosinstall
  wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_examples/master/rosinstall/catkin.rosinstall
  wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_pr2/master/rosinstall/catkin_indigo.rosinstall
  wstool update
  rosdep install --ignore-src --from-paths .
  cd ..
  catkin build
  source ~/catkin_ws/devel/setup.bash  
  ```
* Install Matlab executable from here:
  https://github.com/pauloabelha/enzymes/blob/master/Bremen/edge_detector/for_redistribution/edge_detector_installer.install
  ```
  sudo edge_detector/edge_detector.install
  ```
  Add edge_detector application directory to your *PATH*, so you can run it with only following command:
  ```
  run_edge_detector.sh
  ```

## Running

### Running with Gazebo simulator

1. Launch the Gazebo world and keep it running
   ```
   roslaunch skill_transfer simulation.launch world:=scraping_velocity_controlled
   ```

2. In a new terminal, launch the experiment
   ```
   roslaunch skill_transfer experiment.launch pr2:=false experiment:=scraping_1
   ```

### Running with Gazebo and iai_naive_kinematics PR2 simulator
...

### Running with real robot
...

