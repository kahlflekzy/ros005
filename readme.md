- [ROBOT with multiple poses published to it](#robot-with-multiple-poses-published-to-it)
  - [Approach 1](#approach-1)
    - [Steps](#steps)
      - [Install BehaviorTree](#install-behaviortree)
      - [Install Turtlebot3 Navigation Simulation](#install-turtlebot3-navigation-simulation)
      - [Create Package](#create-package)
    - [BehaviorTree Navigation](#behaviortree-navigation)
      - [Introduction](#introduction)
      - [Create Actions](#create-actions)
        - [Step 1](#step-1)
        - [Step 2](#step-2)
        - [Step 3](#step-3)
        - [Step 4](#step-4)
      - [Create Client (BehaviorTree)](#create-client-behaviortree)
      - [Create Sample Publisher](#create-sample-publisher)
      - [Create Server (Publishers, Subscribers, and Workers)](#create-server-publishers-subscribers-and-workers)
      - [Execution](#execution)
        - [1](#1)
        - [2](#2)
        - [3](#3)
  - [Approach 2](#approach-2)
- [REFERENCE](#reference)

# Robot with tasks published to it
In this work, a task is defined as a `PoseArray`. A `PoseArray` is made up of a collection of different `Pose`. Our 
goal is to be able to publish multiple poses to a robot in the form of a `PoseArray` then have the robot move to those 
poses and await further task (`PoseArray`). 

There are two ways we can go about it.

## Approach 1
We use the server - client architecture with the BehaviorTree being the client.

In this approach, I will create two action servers, one listens for goals, the other executes goals.

### Steps
1. Create workspace and run `catkin_make`
2. Install BehaviorTree
3. Install Turtlebot3 Navigation Simulation
4. Create Package

#### Install BehaviorTree
`cd` into `src` and run the code
```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
```
Go back to the workspace folder and run `catkin_make`

#### Install Turtlebot3 Navigation Simulation
Again, `cd` into `src` and run the code
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Go back to the workspace folder and run `catkin_make`

#### Create Package
`cd` into `src` and create a package using the code below with the shown dependencies.
```
catkin_create_pkg behavior_tree_navigation_v1 rospy roscpp behaviortree_cpp_v3 \
std_msgs geometry_msgs genmsg actionlib actionlib_msgs
```
`actionlib_msgs` depends on `genmsg` for building Action Messages.

Go back to the workspace folder and run `catkin_make`

### BehaviorTree Navigation
#### Introduction
I created an XML representation of a tree. It is based on a sequence node which has two children. The first child 
listens for tasks while the second child executes tasks.

A `BT::SequenceNode` keeps ticking the root as long as one child returns `FAILURE` or `RUNNING`. 

If a child returns `FAILURE`, the tree is restarted, while, 

If a child returns `RUNNING`, previous siblings are skipped while the child itself is `tick`ed again. 

I used a decorator node to make the child which executes tasks to return `FAILURE` (if it's task was 
successfully completed) this will make the tree to RESTART. Effectively going back to tick the first child which 
listens for tasks.

If the task execution fails at any point, the node will return `FAILURE` which the `DECORATOR` will convert to 
`SUCCESS` and this will make the tree to then stop executing.

#### Create Actions
We will create two Actions. One for the Tasks Listener and the other for Tasks Executioner. The executioner will just 
use a Move Robot Action.

##### Step 1
Create an _action_ folder in the package.

##### Step 2
Create a `.action` file for the two actions. See the two files for details.

##### Step 3
Then in `CMakeLists.txt`, add/edit the block below. I didn't change _actions_ directory, so there is no need for the 
`[DIRECTORY]` option of `add_action_files`. And the only dependency is on `actionlib_msgs`. The other message types I use
are `std_msgs`.

```
add_action_files(FILES TasksListener.action MoveRobot.action)
generate_messages(DEPENDENCIES actionlib_msgs)
```

##### Step 4
Then run `catkin_make` from the workspace.

#### Create Client (BehaviorTree)
We create the two action nodes, the listener has an output port which it sets to the number of goals in the task.

The executioner executes the number of goals set by the listener. We will use blackboards fot this communication.

In the Listener, I created a callback which gets task from the server and then sets in on the blackboard for the 
goal executor. 

In the TaskListener, when a task of size 0 is gotten, it notifies the tree to stop running.

I ran into some silly logical errors with the tree ticking when it's supposed to shut down, after a long time debugging, 
I was finally able to note and eliminate the bug.

Also, the root node keeps ticking as far the status is SUCCESS or RUNNING. 

I had to search the codes on `GitHub` for how to tell a tree to halt and eventually got the method. `tree.haltTree()`.

I also found out I'm not allowed to implement the `halt()` method of the `StatefulActionNode`. I can only implement 
`onHalted()`.  

See the files `header_nodes.h` and `behaviortree_node.cpp` for details.

#### Create Sample Publisher
I created a node which publishes `PoseArray` messages to my specified topic. It just reads goals from the goals file I 
previously created. It then parses them into poses. Then runs a loop, which waits for user input and then selects some 
poses based on the user input, and constructs a `PoseArray` which is then published over the topic. See the file 
`task_publisher.py` for details. 

#### Create Server (Publishers, Subscribers, and Workers)
The server basically creates the action pipeline, gets request from the actionlib client and performs the action.
Two servers are created. I also created subscribers for listening for messages which moves the robot to specific points.

When `PoseArray` messages are gotten, they are queued until a previous task has been executed and a request is made to 
get a new task, then the oldest task is popped.

I also created methods which creates `Pose` messages, initialises the pose for `amcl` and a worker that listens to 
`GoalStatusArray` messages.

I only had challenges with shutting down action server. It at times requires a SIGKILL which I want to avoid.

Everything ran successfully.

See the file `actions_server.py` for details.

#### Execution
##### 1
Open a terminal, and run the codes below. If conda is active, deactivate it via `conda deactivate`
```
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch behavior_tree_navigation_v1 launch_nav_sim.launch
```
##### 2
Open another terminal, and run the codes below
```
source devel/setup.bash
roslaunch behavior_tree_navigation_v1 behaviortree_navigation.launch init_base:=1 [or 0]
```
Note that actions_server depends on an `init_pose` file that is external to this project.

##### 3
Open a third terminal, and run the codes below
```
source devel/setup.bash
rosrun behavior_tree_navigation_v1 task_publisher.py
```
Note that the publisher depends on a goals file that is external to this project.

## Approach 2
We run everything in the BehaviorTree. I ultimately didn't implement this, because it goes against the design principles
of the `BehaviorTree`.

# REFERENCE
1. [Message Definition](http://wiki.ros.org/action/show/msg?action=show&redirect=ROS%2FMessage_Description_Language)
2. [ROS-CPP Initialization and Shutdown](https://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown)
3. [ROSPY Initialization and Shutdown](http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node)
4. [Actionlib](http://wiki.ros.org/actionlib)
5. [ActionLib API](https://docs.ros.org/en/api/actionlib/html/namespaceactionlib.html)