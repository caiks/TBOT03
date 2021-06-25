# TBOT03 - TurtleBot3 slice topology controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we extend the developments in [TBOT01](https://github.com/caiks/TBOT01#readme) and  [TBOT02](https://github.com/caiks/TBOT02#readme) to examine *slice* topologies and motor actions.

The implementation of *slice* topologies in `TBOT02` only had a low action success rate. In `TBOT03` we will seek to improve this performance by considering the factors that increase  *model likelihood*, *slice* topology connectivity and completeness, and action responsiveness. To do this we will control the timings of scans and actions. This will allow us to manually control the turtlebot so that we can trace the action decisions that would have been taken in the various modes. In this way `TBOT03` will debug `TBOT02`.

As well as the `location` goal mode of `TBOT02`, we will consider two more modes in `TBOT03`, both of which will be implemented using *slice* topologies. 

First, 'random' mode acts to match the `motor` *histogram* for each *slice* to a desired uniform distribution. In this way the turtlebot will systematically explore the physical configuration, so improving the *slice* topology's representation. 'Random' mode is used in the configuration *level*, which is defined as the *level* that acts on the motor *variables*.

Second, 'interest' mode acts to move towards the *slice* with the *size* nearest to the *induction* threshold. The turtlebot therefore will spend more time in the *slices* with the highest potential for *modelling* so far undetected *alignments*, thus improving the *model likelihood* per active *history size*. This is similar to static *induction* in which the largest *slice* is *induced* first. 'Interest' mode can be used at any *level*. The goal *slice* of a higher *level* implies a set of goal *slices* in the configuration *level*, recursing through any intermediate *underlying* *levels*. The interests of different actives may sometimes be contradictory, so some method of coordinating between them will be needed.


## Sections

[Download, build and run main executable](#main)

[Download, build and run TurtleBot3 nodes](#controller)

[Discussion](#Discussion)

<a name="main"></a>

## Download, build and run main executable

To run the non-ROS main executable it is only necessary to install the [AlignmentActive repository](https://github.com/caiks/AlignmentActive), the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC) and the underlying repositories. The `AlignmentActive` and the `AlignmentRepaC` modules require [modern C++](https://en.cppreference.com/w/) version 17 or later to be installed.

For example, in Ubuntu bionic (18.04),
```
sudo apt-get update -y && sudo apt install -y git g++ cmake

```
Then download the zip files or use git to get the `TBOT03` repository and the underlying `rapidjson`, `AlignmentC`, `AlignmentRepaC`and `AlignmentActive`  repositories -
```
cd
git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/AlignmentActive.git
git clone https://github.com/caiks/TBOT03.git

```
Then download the [TBOT03 workspace repository](https://github.com/caiks/TBOT03_ws) -
```
git clone https://github.com/caiks/TBOT03_ws.git

```
Then build -
```
cd
cp TBOT03/CMakeLists_noros.txt TBOT03/CMakeLists.txt
mkdir -p TBOT03_build
cd TBOT03_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../TBOT03
make

```
The `main` executable has various modes,
```
cd ../TBOT03_ws
ln -s ../TBOT03_build/main main

 ./main

```
<a name = "controller"></a>

## Download, build and run TurtleBot3 nodes

To run the turtlebot it is necessary to install [ROS2](https://index.ros.org/doc/ros2/), [Gazebo](http://gazebosim.org/tutorials?cat=install) and [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#simulation) on a machine with a GPU and at least 4GB of memory.

[AWS EC2 instance](https://github.com/caiks/TBOT01#AWS)

[Windows 10 WSL2 instance](https://github.com/caiks/TBOT01#Windows)

[Installation](#Installation)
 
<a name = "Installation"></a>

### Installation

Now install Gazebo9,
```
sudo apt-get install -y gazebo9 libgazebo9-dev

gazebo -v

```
Install ROS2 Eloquent,
```
sudo apt install -y curl gnupg2 lsb-release

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update

sudo apt install -y ros-eloquent-desktop ros-eloquent-gazebo-* ros-eloquent-cartographer ros-eloquent-cartographer-ros

echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
Test by running these nodes in separate shells,
```
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_py listener

```
Install TurtleBot3,

```
sudo apt install -y python3-argcomplete python3-colcon-common-extensions google-mock libceres-dev liblua5.3-dev libboost-dev libboost-iostreams-dev libprotobuf-dev protobuf-compiler libcairo2-dev libpcl-dev python3-sphinx python3-vcstool

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos
colcon build --symlink-install

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc

export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

```
To test launch one of these worlds,
```
ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```
Then in a separate shell,
```
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

ros2 run turtlebot3_teleop teleop_keyboard

```
Check that you can steer the turtlebot using the w/x and a/d keys.

Now download and build the `TBOT03` repository and the underlying `rapidjson`, `AlignmentC`, `AlignmentRepaC` and `AlignmentActive` repositories -
```
cd ~/turtlebot3_ws/src

git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/AlignmentActive.git
git clone https://github.com/caiks/TBOT03.git
git clone https://github.com/caiks/TBOT03_ws.git

cd ~/turtlebot3_ws/src
mkdir -p AlignmentC_build AlignmentRepaC_build AlignmentActive_build
cd ~/turtlebot3_ws/src/AlignmentActive_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../AlignmentActive
make AlignmentC AlignmentRepaC AlignmentActive

cd ~/turtlebot3_ws/src/TBOT03
cp CMakeLists_ros.txt CMakeLists.txt

cd ~/turtlebot3_ws
colcon build --packages-select TBOT03

source ~/.bashrc
```

The simulation can be started in paused mode,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env001.model -s libgazebo_ros_init.so

```
In a separate shell,
```
cd ~/turtlebot3_ws/src/TBOT03_ws
echo '{}' > actor.json
ros2 run TBOT03 actor

```
Press play in `gazebo` and the turtlebot3 will start moving.

To run the non-ros `main` executable, create a link,

```
cd ~/turtlebot3_ws/src/TBOT03_ws
ln -s ~/TBOT03_build/main main

```

<a name = "Discussion"></a>

## Discussion

Now let us investigate various turtlebot *slice* topologies and goals. 

[Active discontinuities and physical configuration](#Active)

[Actor node](#Actor)

[Conclusion](#Conclusion)


<a name = "Active"></a>

### Active discontinuities and physical configuration

Like the `TBOT02` actor node, the `TBOT03` actor node is implemented with the active framework defined in the [AlignmentActive repository](https://github.com/caiks/AlignmentActive). An active updates a *history* and induces a *model* from a stream of incoming *events* in real time. In a simulator such as Gazebo we are able to keep information not known to the robot such as its position and orientation. In `TBOT03` we will record the x and y coorinates and the yaw. This will enable us to determine the label *entropy* of the map from the *slices* to the physical configuration space. To keep the configuration record list synchronised with the active *history* we must handle discontinuities in the stream of *events*. In addition, if we wish to look back or forward from a particular *event* to past or future *events*, we must not roll past discontinuities of *event* update such as before the restart of an active given an initial *model*, or before the first *event* in the *history* which depends on overflow. 

So before moving on to `TBOT03`, we added a map from discontinuous *events* to the *event* id in the stream - 
```
cd ~/TBOT02_build
make
```

```
cd ~/TBOT02_ws
./main induce09 model027 data009 
...
activeB.continousIs: true
activeC.continousIs: true
activeB.continousHistoryEventsEvent: {(0,0),(1,1),(2,4),(3,6),(4,8),(5,10)}
activeC.continousHistoryEventsEvent: {(0,0),(1,1),(2,4),(3,6),(4,8),(5,10)}
...
```
The physical configuration records can be viewed thus -
```
cd ~/TBOT03_build
make
```

```
cd ~/TBOT03_ws
./main view_records model075 
*records: (-1.99993,1.5,0.00114709)
(-1.9997,1.5,0.0029681)
(-1.99955,1.5,0.00490264)
(-1.99942,1.5,0.0067631)
(-1.99928,1.5,0.00800717)
(-1.99915,1.5,0.00973616)
(-1.999,1.5,0.0114542)
(-1.99884,1.5,0.0134741)
(-1.99866,1.5,0.0154754)
(-1.99851,1.5,0.0174261)
(-1.99835,1.5,0.0193936)
(-1.9982,1.5,0.0212292)
(-1.99803,1.5,0.0232272)
(-1.99787,1.5,0.0252249)
(-1.99771,1.5,0.027225)
(-1.99756,1.5,0.0290707)
(-1.9974,1.5,0.0310904)
(-1.99729,1.5,0.0324365)
(-1.99716,1.5,0.0341257)
(-1.99699,1.5,0.036157)
```
When an actor starts with an initial *model* it pushes an empty record into the list and creates a discontinuity on the actives where the *slices* map to configuration space, so that the dummy record does not appear in their *histories*.

<a name = "Actor"></a>

### Actor node

The `TBOT03` actor node has the same active structure - `struct001` which defines a two *level* active structure. There are 12 *level* one actives, each with a fixed non-overlapping field-of-view of 30 degrees, and 1 *level* two active. The *level* one active *history size* defaults to 10,000 *events*. The *level* two active *size* defaults to 1,000,000 *events*.

describe the act loop and the test modes 1-4

randomly distributed actions mode 5

turn randomly chosen from distribution with collision avoidance mode 6

turn randomly chosen from distribution with collision avoidance handling case of blocked ahead and blocked to the sides mode 7

mode 8 first room goal 

mode 9 manual with mode 8 


TBOT03 README - actions recorded on next event, actions disconnected from the performance (the acts)

TBOT03 readme TBOT02 had a small success rate but because it was in constant motion that was enough to attain the goals statistically significantly. In TBOT03 we are stopping between actions like a bird walking and so much more successful, but ironically this means that the turtlebot is more likely to be stuck in any loops caused by ambiguity wormholes. We must either have better maps to the configuration space or loop handling that artificially fails to go to the wormhole neighbour. Always better to have a good map because delooping is not guaranteed to be better than chance. It will help only if there are parts of the slice topology that is well mapped eg at doorways.

TBOT03  README in mode 9 we are essentially debugging the slice topology by manually manoeuvring the turtlebot with tracing what it would have done automatically in mode 8

TBOT03 In model 77, of the 38k slices only 19k have 5 or more events. Around 17K have low variance. Only around 10% of larger slices have low variance. So even if we prevent loops and ambiguous slices, the slice topology is likely to be patchy. We should seek to improve both the model and the effectiveness. README

TBOT03 mustn't make unfair anthropomorphic comparisons, except between TBOT03 and a newborn README

TBOT03 calculate room entropy - model 77 has considerably higher room label entropy than the similar TBOT02 model 65 - we will need slice-room transitions - add to README. 




TBOT03 use the change in pose rather than linear or angular speed. So for forward motion have a change in postion of 0.5m i.e. 4m lidar range / 8 valency. For turn a change in orientation of 30 deg with update rate of 5ms (odom is 3.3ms), i.e. 360/30 = 12 regional models in level 1. Forwards take 3000ms, turns 30ms. Then wait 220ms for the lidar before stop state. Add a wait state before the stop state. Run at a higher real_time_factor.

TBOT03 measure successful transition actions

TBOT03 we can model configuration space clustering by making a slice tree for each dimension and then inducing. The resultant configuration model slices are the clusters. Of course this is just the same as having the odometry as substrate instead of LiDAR. No doubt we could have a very good slice topology in that case. We can compare models by rel ent so could compare the substrate induced model to the configuration induced model (with same threshold) to get a measure of the quality of the slice topology. Then can run the model without odometry knowing whether it's slice topology was better or worse than some other substrate induced model.

TBOT03 NB 202103081030 - essentially, we can measure the quality induced model's slice topology by inducing a model of the configuration space from the odometry, and then the clusterivity of the slice topology is the negative label entropy wrt the induced slice where the label is the configuration model slice (partition variable). The connectivity of the slice topology is the label entropy wrt the motor action where the label is the induced slice. Of course, in many cases a configuration model is infeasible. TBOT03 is also complicated by the goal depending on room. Relative entropy of motor actions between nearest and not nearest - configuration space is too large. Can compute the total motor relative entropy for each model for each goal as a comparison for the slice topology.So we should add this as a comment to the README, rather than spend much more time 'solving' the maze problem - we possibly could add orientation or odometry or dynamics, but we might still have wormholes or loops due to collision avoidance, jumping over doorways in one step, probabilistic motor actions, and so on. I think to navigate using a slice topology with these motor actions would require wide doorways and plenty of paths.

TBOT02 does not know the grammar of room navigation. We of course have high level models of paths, obstructions, doorways, escalators, etc. We look around to get our bearings when we walk into an unfamiliar room. TBOT02 has none of this. It is trapped in the present instant and so needs a large history and model for every location in the house.

TBOT02 constrain each slice so that it spans a reasonable physical space ie the distance between events is a few metres. Doorways provide a potential barrier if sometimes the TBOT02 jumps across so have neighbours in same direction with differing steps. Slice topology necessarily poor because the slices are too large we need complete sets for each room. With vision and relative goals it would be better. Rather than crossing with room we need to cross with local region to keep the slices very small but with a good set of transitions. Add orientation/room to substrate instead of crossing slice topologies. Avoids the slices from becoming too small.

TBOT02 a lot of the problem, especially in small rooms, is that we are looking at actions and not requests. TBOT03 motor actions will be carefully sycnhronised with the senses. Ideally we would have a slice per act and vice-versa i.e. isomorphism. 

TBOT03 we are no longer interested in model likelihood, but assume that the inducer is always reasonably good. Interested in the temporal relationships between slices  or components of the model.

TBOT03 short term is underlying frames, medium term is active history, long term is the model and the motor/label history - what about reflexive?

TBOT03 control is like a duck paddling requires individual strokes experiences increasing resistance

In the TBOT02 discussion we said that we can think of each *slice* as a set of *events* that corresponds roughly to a particular bounded and oriented area or region of the turtlebot house. Now, having, manually navigated in TBOT03's *slice* topology we can see that sometimes a *slice* corresponds to several different clusters of *events*. Each *event* has a coordinate in physical configuration space of `x`,`y` and `yaw`. Each cluster will have a  different mean coordinate and variance or deviation. If a *slice* has more than one cluster the overall mean won't correspond to any of the cluster means and the overall deviation will be greater than the deviation of any of the clusters.

These areas arrange themselves contiguously to cover the turtlebot's explorations of the house. So a slice topology is a sort of map of the pathways through the physical environment, formed by the past movements of the agent. 



The aim of TBOT03 is to debug the slice topologies created in TBOT02. 
The best topologies approximate closely to the physical configuration space and affordances, with diverse neighbourhoods and complete connectivity. In TBOT02 the topologies were far from optimal!

A disadvantage of a *slice* topology is that the *slices* are sometimes ambiguous with respect to goal label. The *events* of a *slice* may have occured in more than one physical area. Generally, however, the *slice* will consist of *events* that form clusters in which the turtlebot has similar x and y coordinate and orientation. The 'view' from the *slice* always looks the same, though, whichever cluster the turtlebot is in. The trick is to have only one cluster per *slice*. To do that, the turtlebot has to be able to distinguish between the clusters and that means a better resolution for the lidar scan, or landmarks so that the clusters have different views, or perhaps a compass to distinguish between orientations. For example, when the turtlebot is in the large central room looking 'north' at the doorway, it sometimes thinks it is looking 'south' at the other doorway, even though we can easily tell at a glance which doorway is which because of their different widths. If the turtlebot cannot distiguish exactly where it is within a room, or which way it is pointing, it clearly cannot make the decision to turn left or right or proceed ahead. So the key thing is to have a good map between the turtlebot's *model* and the physical configuration space.

In addition the action was poorly defined. Another disadvantage is that the *slices* are sometimes spread out and so the transitions are sensitive to the timing of the actions within them. We have fixed this problem so well that now the turtlebot can get stuck in loops when there are ambiguous *slices*. However, we can detect when it has gone around a loop and choose a different action to break out of it - somewhat as we do when we see that we have gone around in a circle in a maze.

The ironic thing about TBOT03 is that its slice topology is so complete and connected that it doesn't accidentally miss any wormholes or inconsistencies with physical space and reliably gets lost, ending up in loops! It is so good it is bad. TBOT02 hurtled along so quickly, almost out of control, that its slice topology was no more than a bias or tendency, so that it usually did better than random. Given that there will always be some ambiguity in the TBOT03 model, especially when starting in a new environment, we must add some loop avoidance, so that if it finds that it is back to where it started it does not repeat the past erroneous action, but tries some other action. I think we do the same thing when we are lost in a maze and we recognise that we have gone in a circle and then try a different turn.


<a name = "Conclusion"></a>

### Conclusion








