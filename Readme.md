### IROS 2018
# Skill Transfer

ROS package that realises transfer of manipulation skills from known objects and situations to new, unseen objects and their setups.

## Requirements

This package is **Developed and Tested on ROS Kinetic**.
At it's core, the system makes use of Giskard library for robot control: https://github.com/SemRoCo/giskard_core

## Architecture

The package consists of multiple ROS nodes that work collectively for achieving the desired effects. They communicate in roughly following manner:

```
[FeatureDetector] <--> [KnowledgeManager] <--> [TaskExecutive] --> [ConstraintController] --> <Actuators>
```

*KnowledgeManager* manages all specs needed for the task.

*TaskExecutive* is the main node that supervises the whole process and sends requests to all other nodes.

*FeatureDetector* finds desired object features (edge-point, ...)

*ConstraintController* uses Giskard internally, translates motion description files into desired joint velocities.

### The Process

The whole process begins with *KnowledgeManager* reading task and setup YAML files. It decides what visual features are missing
from the description and asks *FeatureDetector* for them. Once the specs are ready *TaskExecutive* asks for them and the moiton sequence begins. 
*KnowledgeManager* provides individual motion specs to the *TaskExecutive* previously combining them with appropriate motion template.
Such prepared motion phase file is then sent to *ConstraintController* for execution. While that happens *TaskExecutive* observes 
the state of the robot and decides when to finish one phase and begin the next one according to the task specification file.
When all motion phases are done the task is considered as finished.

### Configuration files

There are configuration files that describe different levels of the system: motions, tasks, setups. All files are YAML.

*robot template* specifies the kinematic chain of a robot.

*motion phase* specifies motion in terms of constraints that should be satified.

*tasks* contains a sequence of motion phases and appropriate stop conditions as well as required visual features that should be resolved. Those elements toghether form a full task description.

*setups* specifies objects that take part in the task, callibrated grasp transformations and handcoded visual features.

### Supported tasks

1. Scraping butter off a tool into a container
2. Scooping a substance (e.g. grains) from a container
3. Cutting an object on a flat object/surface

## Installation
* Install ROS, then:
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  cd src
  wstool init
  wstool merge https://raw.githubusercontent.com/lubiluk/iros2018/master/rosinstall/catkin.rosinstall
  wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_core/master/rosinstall/catkin.rosinstall
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

Worlds with `_v` prefix are for free end effectors simulation only, `_p` for PR2 simulation.

Experiment launch file can be run for freely flying end effectors simulation (argument `robot:=free_ees`) or simulated or real PR2 (`robot:=pr2`).

### Running with Gazebo simulator

1. Launch the Gazebo world and keep it running
   ```
   roslaunch skill_transfer simulation.launch world:=big_bowl_spatula_v
   ```

2. In a new terminal, launch the experiment
   ```
   roslaunch skill_transfer experiment.launch task:=scraping robot:=free_ees setup:=big_bowl_spatula
   ```

### Running with Gazebo and iai_naive_kinematics PR2 simulator

1. Launch PR2 simulator, keep it running
  ```
  roslaunch skill_transfer pr2.launch
  ```
2. Launch the Gazebo world, keep it running
  ```
  roslaunch skill_transfer simulation.launch world:=big_bowl_spatula_p
  ```

3. In a new terminal, launch the experiment.
   ```
   roslaunch skill_transfer experiment.launch task:=scraping robot:=pr2 setup:=big_bowl_spatula
   ```

### Running with real robot

1. Prepare the robot.

2. Launch the experiment.
   ```
   roslaunch skill_transfer experiment.launch task:=scraping robot:=pr2 setup:=big_bowl_spatula
   ```
