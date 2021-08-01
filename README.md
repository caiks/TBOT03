# TBOT03 - TurtleBot3 slice topology controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we extend the developments in [TBOT01](https://github.com/caiks/TBOT01#readme) and  [TBOT02](https://github.com/caiks/TBOT02#readme) to examine *slice* topologies and motor actions.

The implementation of *slice* topologies in `TBOT02` only had a low action success rate. In `TBOT03` we will seek to improve this performance by considering the factors that increase  *model likelihood*, *slice* topology connectivity and completeness, and action responsiveness. To do this we will control the timings of scans and actions. This will allow us to manually control the turtlebot so that we can trace the action decisions that would have been taken in the various modes. In this way `TBOT03` will debug `TBOT02`.

As well as the `location` goal mode of `TBOT02`, we will consider two more modes in `TBOT03`, both of which will be implemented using *slice* topologies. 

First, 'random' mode acts to match the `motor` *histogram* for each *slice* to a desired uniform distribution. In this way the turtlebot will systematically explore the physical configuration, so improving the *slice* topology's representation. 'Random' mode is used in the action *level*, which is defined as the *level* that acts on the motor *variables*.

Second, 'interest' mode acts to move towards the *slice* with the *size* nearest to the *induction* threshold. The turtlebot therefore will spend more time in the *slices* with the highest potential for *modelling* so far undetected *alignments*, thus improving the *model likelihood* per active *history size*. This is similar to static *induction* in which the largest *slice* is *induced* first. 

'Interest' mode can be used at any *level*. In a action *level*, i.e. one that has motor *variables* in an *underlying substrate*, the active chooses actions that transition to the subset of neighbouring *slices* which have the fewest transitions to the goal *slice*. In higher *levels* which are not also action *levels*, i.e. they do not have actions in an *underlying substrate*, the goal *slice* of a higher *level* implies a set of goal *slices* in an *underlying* action *level*, recursing through any intermediate *underlying* *levels*. The interests of different actives may sometimes be contradictory, so some method of coordinating between them will be needed.

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

cd ~/TBOT03_ws
cat model083_2a* >model083_2.ac
cat model084_2a* >model084_2.ac
cat model085_2a* >model085_2.ac
cat model086_2a* >model086_2.ac
cat model087_2a* >model087_2.ac
cat model089_2a* >model089_2.ac

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

cd ~/turtlebot3_ws/src/TBOT03_ws
cat model083_2a* >model083_2.ac
cat model084_2a* >model084_2.ac
cat model085_2a* >model085_2.ac
cat model086_2a* >model086_2.ac
cat model087_2a* >model087_2.ac
cat model089_2a* >model089_2.ac

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

[Physical configuration](#Physical)

[Active discontinuities](#Active)

[Actor node](#Actor)

[Conclusion](#Conclusion)


<a name = "Physical"></a>

### Physical configuration

In a simulator such as Gazebo we are able to gather information not known to the robot such as its exact position and orientation. Although the physical configuration could include many other dynamic measures of the world, such as the poses of limbs, other agents, or movable objects, in `TBOT03` we will record just the turtlebot's x and y coordinates and yaw. This configuration will be recorded separately from its active *history*. 

This is an example, where the turtlebot moves from its starting position in room 4 into the corridor to room 2 -
```
cd ~/TBOT03_ws
./main view_records model078 
*records: (-1.99981,1.49997,0.0253202)
(-1.44517,1.49196,-1.26718)
(-1.44015,1.48738,-32.0543)
(-0.983593,1.17426,-36.6514)
(-0.98557,1.17272,-62.3233)
(-0.722155,0.688435,-60.0746)
(-0.720621,0.689003,-33.1571)
(-0.719856,0.686763,-56.9441)
(-0.718682,0.68733,-29.7472)
(-0.242669,0.407572,-30.4445)
(-0.242399,0.408778,-3.77064)
(0.307508,0.367783,-4.17933)
(0.859555,0.32512,-4.44195)
(0.859711,0.32612,21.3517)
(0.859928,0.3279,50.352)
(0.862455,0.329382,26.2096)
(0.864997,0.329421,-1.49013)
(1.41928,0.319821,-0.533272)
(1.41856,0.320849,26.0159)
(1.41867,0.321568,54.0519)
(1.42144,0.32312,30.0234)
(1.42398,0.323476,1.41045)
(1.9765,0.335904,1.70144)
(2.53027,0.352991,1.8135)
(3.08338,0.369931,1.83139)
(3.63614,0.386508,1.79969)
```
Of course, the physical configuration cannot be obtained when the turtlebot is operating in the real world, so we will only use this simulator information to help us choose its sensors, *induction* parameters, and motors, and also to help us design and debug its modes of operation.

Ultimately any agent's model of the world is key to its ability to act in that world. So the map between the turtlebot's *slices* and the physical configuration must be at least as accurate as is required to accomplish its goals, whether they are imperative goals, such as navigating to a room, or cognitive/interest goals which aim to maximise the *model likelihood* given finite *history size*. In the case of `TBOT03` we will start with the room goal of `TBOT01` and `TBOT02` before moving on to consider various 'interest' modes. 

In order to navigate to another room, the turtlebot must be able to determine not only which room it is in, but also its position and orientation within the room. At any point in time it knows the current *slice*. By examining the *slice* transitions from this *slice* it can determine its *slice* neighbourhood. Then the turtlebot can select the subset of its neighbours which have the fewest *slice* transitions to the goal *slice* and choose the action accordingly. If all of its *slices* are such that each *slice's* *events* are closely clustered in configuration space, then turtlebot's choice of action will usually be correct. If, however, the *slices* sometimes contain more than one cluster of configurations, each cluster with a different average position and orientation, or if the clusters are rather ill-defined and spread out, then the map between the *slice* topology and the environment can become blurry or have worm-holes. In this case the turtlebot may end up going around in loops. It is therefore important to minimise the label *entropy* of the map from the *slices* to the physical configuration space. 

In `TBOT01` and `TBOT02` we calculated `location` *entropy* rather than configuration  *entropy*. We can do the same for `TBOT03` in *models* 76 and 77  (described below), for example -

```
./main location_entropy model076_2
model: model076_2
model076_2      load    file name: model076_2.ac        time 0.0395493s
activeA.historyOverflow: false
sizeA: 90178
activeA.decomp->fuds.size(): 851
activeA.decomp->fudRepasSize: 15983
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.943689
entropyA: 100571
entropyA/sizeA: 1.11525
(double)sizeA * std::log(sizeA): 1.02889e+06
std::log(sizeA): 11.4095

./main location_entropy model077_2
model: model077_2
model077_2      load    file name: model077_2.ac        time 0.231134s
activeA.historyOverflow: false
sizeA: 395073
activeA.decomp->fuds.size(): 3925
activeA.decomp->fudRepasSize: 67049
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.993487
entropyA: 330471
entropyA/sizeA: 0.83648
(double)sizeA * std::log(sizeA): 5.09124e+06
std::log(sizeA): 12.8868
```

Adding these 2 *models* to the earlier ones confirms that, although the `location` *entropy* decreased with *size*, these *induced models* are far from being able to to reduce it to zero -

model|size|label entropy/size
---|---|---
76|100,571|1.11525
77|395,073|0.83648
61|258,581|0.71099
65|370,181|0.668522
73|946,931|0.651033
69|428,154|0.615338
67|525,414|0.609749
66|540,699|0.595765
63|562,969|0.597108
68|684,004|0.557499
70|580,398|0.554539
71|1,000,000|0.383128
72|1,000,000|0.349623

The physical configuration consists of continuous measures rather than discrete so configuration *entropy* is more difficult to define than `location` *entropy*. Instead of entropy we will calculate the standard deviation of the Euclidean distance of the *event* configuration coordinates from the *slice's* mean configuration coordinate. The configurations are normalised so that the position and orientation measures are commensurate.

Here we calculate the overall standard deviation for models 76 and 77 -
```
./main configuration_deviation_all model076 
model: model076
model076_2      load    file name: model076_2.ac        time 0.0353962s
activeA.historyOverflow: false
sizeA: 90178
activeA.decomp->fuds.size(): 851
activeA.decomp->fudRepasSize: 15983
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.943689
records->size(): 90178
size: 90178
slice_count: 3689
slice_size_mean: 24.4451
deviation: 0.4548
size: 90178
slice_location_count: 11872
slice_location_size_mean: 7.59586
deviation_location: 0.220935

./main configuration_deviation_all model077 
model: model077
model077_2      load    file name: model077_2.ac        time 0.232236s
activeA.historyOverflow: false
sizeA: 395073
activeA.decomp->fuds.size(): 3925
activeA.decomp->fudRepasSize: 67049
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.993487
records->size(): 395074
size: 395073
slice_count: 15008
slice_size_mean: 26.3242
deviation: 0.409984
size: 395073
slice_location_count: 38008
slice_location_size_mean: 10.3945
deviation_location: 0.191815
```
In fact we calculate both the configuration deviation for the *slices* and for the cross between *slice* and `location`. In the second case, we are assuming that the room or doorway is known to the turtlebot. That, of course, probably would not be the case in practice, but it is a requirement to identify room goal *slices*.

Now that we have a measure of the map between *models* and the physical environments, we will experiment with various active structures and random modes, before going on to look at the goal modes.

<a name = "Active"></a>

### Active discontinuities and physical configuration

Like the `TBOT02` actor node, the `TBOT03` actor node is implemented with the active framework defined in the [AlignmentActive repository](https://github.com/caiks/AlignmentActive). An active updates a *history* and induces a *model* from a stream of incoming *events* in real time. To keep the physical configuration record list synchronised with the active *history* we must handle discontinuities in the stream of *events*. In addition, if we wish to look backward or forward from a particular *event* to past or future *events*, we must not roll past the discontinuities of *event* update, such as before the first *event* in the *history*, or before the restart of an active given an initial *model*. 

So, before moving on to `TBOT03`, we added a map from the discontinuous *events* to the *event* id in the stream - 
```
cd ~/TBOT02_build
make
```

```
cd ~/TBOT02_ws
./main induce09 model027 data009 
...
activeB.continousIs: true
...
activeB.continousHistoryEventsEvent: {(0,0),(1,1),(2,4),(3,6),(4,8),(5,10)}
...
```
When an actor starts with an initial *model*, it pushes an empty record into the list to create a discontinuity in the active's map between the *history* and the configuration space.

<a name = "Actor"></a>

### Actor node

The `TBOT03` actor node has similar active structures to `TBOT02`. Structure `struct001` defines a two *level* active structure. There are 12 *level* one actives, each with a fixed non-overlapping field-of-view of 30 degrees, and 1 *level* two active. The *level* one active *history size* defaults to 10,000 *events*. The *level* two active *size* defaults to 1,000,000 *events*. By default each active's induce operation will be called at regular intervals, unless *induction* is explicity disabled.

The principal differences between `TBOT02` and `TBOT03` are not in the active structures, however, but in the actor's update and its modes of action.

The actor's update has been simplified in `TBOT03`. The update no longer does record collection, collision avoidance or navigation, but instead it has a simple set of state transitions. If it's current state is an action state, `LEFT`, `AHEAD` or `RIGHT`, the turtlebot publishes a velocity or twist request. While moving it monitors its odometry. When the turtlebot has moved ahead the required distance, or rotated through the required angle, it publishes a stop request (zero velocity and twist), and sets its state to `WAIT_ODOM`. While in this state it continues to check its pose until it has stopped moving. It then changes its state to `WAIT_SCAN`. In this state it waits until there has been a complete scan by the lidar. This usually takes around 200 milliseconds - the lidar spins at a frequency of 5 hertz. The turtlebot then transitions to the `STOP` state where it stays until it is given a new action state. 

So having started by being given an action state, the turtlebot finishes having (a) performed the action, and (b) taken a full scan while completely stationary. The time required for a complete action cycle varies but each cycle always yields only one configuration record and one active *event*. This is a change from `TBOT02` where the *events* were captured at regular intervals, regardless of the action requested or what state the actor was in. Now motor actions and sensor data are atomic and synchronised.

By contrast, although `TBOT02` sometimes did find *slices* that spanned a small physical space, the *slices* were often blurry or ill-defined partly due to the smearing of the lidar data when in motion. Although the distortion of the scan while moving forward at a top speed of 30 cm per second is only a linear error of around +/- 6 cm compared to 50 cm per *event*, while rotating at 40 degrees per second there is an angular error of around +/- 8 degrees compared to 30 degrees per *event*. In `TBOT03` the turtlebot is stationary between actions. This is similar to the jerky motion of a pigeon's head as it walks. Another analogy would be a duck's motion if it were to paddle through syrup. This should improve the map between *slice* and configuration space.

The update only takes a short time to process the state transitions, and so it can be called frequently. It defaults to 10 milliseconds. In this way, the turtlebot moves as quickly as possible.

All of the rest of turtlebot's behaviour is controlled by the act operation, defined according to the mode. Regardless of mode, the turtlebot only acts if the current actor state is `STOP`. When it first finds that it has stopped after an action it  (a) records the configuration, (b) constructs the *event* from the lidar scan, (c) sets the the *event's* motor *variables* to the previous action, i.e. the action that led to this *event*, and then (d) updates the active *levels* with the new *event*. The *levels* are updated in sequence from lowest to highest, running the active updates within a *level* in parallel threads. At this point the current *slices* are defined for each active. The actor then processes the mode if it is defined.

#### Test modes 1-4

In mode 1 the turtlebot's only action is to set the actor state to `AHEAD`. This is the JSON configuration in `actor.json` -
```json
{
	"structure" : "struct001",
	"mode" : "mode001",
	"logging_update" : true
}
```
Run the simulation -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env009.model -s libgazebo_ros_init.so

```
Run the actor -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

```
When the simulator is started the turtlebot moves ahead and then crashes into the wall -
```
actor       START
actor       WAIT_ODOM       time 10.277s
actor       WAIT_SCAN       time 0.000s     x: -2   y: 1.5  yaw: 0.00115
actor       STOP    time 0.210s
actor       AHEAD   time 0.008s
actor       WAIT_ODOM       time 1.951s     distance: 0.501
actor       WAIT_SCAN       time 0.210s     x: -1.45        y: 1.5  yaw: 0.0113
actor       STOP    time 0.350s
actor       AHEAD   time 0.007s
actor       WAIT_ODOM       time 1.823s     distance: 0.508
actor       WAIT_SCAN       time 0.200s     x: -0.903       y: 1.5  yaw: 0.0215
actor       STOP    time 0.170s
actor       AHEAD   time 0.007s
actor       WAIT_ODOM       time 1.833s     distance: 0.508
actor       WAIT_SCAN       time 0.210s     x: -0.351       y: 1.5  yaw: 0.0327
actor       STOP    time 0.360s
actor       AHEAD   time 0.010s
actor       CRASH   time 20.2777s
```
The default distance travelled before the brake is applied is 0.5 metres. The turtlebot decelerates for around another 5 cm before coming to a complete halt. This distance is chosen to be equal to the 4 metre lidar range divided by a *valency* of 8.

In mode 2 the turtlebot's only action is to set the actor state to `LEFT`. This is the JSON configuration in `actor.json` -
```json
{
	"angular_maximum_lag" : 2.0,
	"structure" : "struct001",
	"mode" : "mode002",
	"logging_update" : true
}
```
The turtlebot rotates anti-clockwise approximately 30 degrees,
```
actor       START
actor       WAIT_ODOM       time 7.383s
actor       WAIT_SCAN       time 0.000s     x: -2   y: 1.5  yaw: 0.000996
actor       STOP    time 0.220s
actor       LEFT    time 0.006s
actor       WAIT_ODOM       time 0.814s     angle: 29.2
actor       WAIT_SCAN       time 0.100s     x: -2   y: 1.5  yaw: 31.2
actor       STOP    time 0.310s
actor       LEFT    time 0.006s
actor       WAIT_ODOM       time 0.774s     angle: 29.2
actor       WAIT_SCAN       time 0.110s     x: -2   y: 1.5  yaw: 62.4
actor       STOP    time 0.310s
actor       LEFT    time 0.006s
actor       WAIT_ODOM       time 0.774s     angle: 29.2
actor       WAIT_SCAN       time 0.100s     x: -2   y: 1.5  yaw: 93.6
actor       STOP    time 0.330s
actor       LEFT    time 0.005s
actor       WAIT_ODOM       time 0.784s     angle: 29.2
actor       WAIT_SCAN       time 0.100s     x: -2   y: 1.5  yaw: 125
actor       STOP    time 0.310s
```
Note that the default rotation is 30 degrees, but a lag has to be subtracted depending on the simulator real time factor and update frequency. For example, when running at 10x the lag should be set to 6.0 -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env015.model -s libgazebo_ros_init.so

```
Run the actor -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

```

```json
{
	"update_interval" : 1,
	"angular_maximum_lag" : 6.0,
	"structure" : "struct001",
	"mode" : "mode002",
	"logging_update" : true
}
```
```
actor       START
actor       WAIT_ODOM       time 2.712s
actor       WAIT_SCAN       time 0.000s     x: -2   y: 1.5  yaw: 0.0187
actor       STOP    time 0.020s
actor       LEFT    time 0.006s
actor       WAIT_ODOM       time 0.074s     angle: 24.5
actor       WAIT_SCAN       time 0.032s     x: -2   y: 1.5  yaw: 30.6
actor       STOP    time 0.033s
actor       LEFT    time 0.012s
actor       WAIT_ODOM       time 0.075s     angle: 24.9
actor       WAIT_SCAN       time 0.023s     x: -2   y: 1.51 yaw: 60.3
actor       STOP    time 0.034s
actor       LEFT    time 0.011s
actor       WAIT_ODOM       time 0.075s     angle: 25.3
actor       WAIT_SCAN       time 0.025s     x: -2   y: 1.51 yaw: 90.5
actor       STOP    time 0.034s
actor       LEFT    time 0.007s
actor       WAIT_ODOM       time 0.075s     angle: 25.3
actor       WAIT_SCAN       time 0.025s     x: -2   y: 1.51 yaw: 120
actor       STOP    time 0.033s
```
Again the 30 degrees rotation is chosen to be equal to the default field of view of the *level* one actives in structure 1. In general we are aiming to have a *slice* topology resolution such that there is generally a *slice* transition for each action. Ideally we would have a *slice* per act as well as an *event* per act. In this way the indeterminacy of the `TBOT02` *slice* topology will be improved, not only by a better map between *slice* and configuration space, but also by a better map between action and *slice* transition.

Mode 3 is the same as mode 2 except that the turtlebot rotates clockwise, i.e. an actor state of `RIGHT`. 

Mode 4 tests the `WAIT_ODOM` state.

#### Random modes 5-7

Now, having dealt with the new actor update state transitions, we can consider active *modelling* for the room goal. To do this we will acquire active *history* in various 'random' modes. In mode 5, the turtlebot chooses an action at random from a probability distribution. By default, for every turn `LEFT` or `RIGHT`, the turtlebot will move `AHEAD` 5 times -
```json
{
	"structure" : "struct001",
	"mode" : "mode005",
	"distribution_LEFT" : 1.0,
	"distribution_AHEAD" : 5.0,
	"distribution_RIGHT" : 1.0,
	"logging_update" : true
}
```
Run the simulation -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env009.model -s libgazebo_ros_init.so

```
Run the actor -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

```
The turtlebot moves ahead, then rotates clockwise, and then moves ahead twice more before crashing -
```
actor       START
actor       WAIT_ODOM       time 4.364s
actor       WAIT_SCAN       time 0.000s     x: -2   y: 1.5  yaw: 0.000996
actor       STOP    time 0.220s
actor       AHEAD   time 0.017s
actor       WAIT_ODOM       time 1.862s     distance: 0.502
actor       WAIT_SCAN       time 0.200s     x: -1.45        y: 1.5  yaw: 0.0113
actor       STOP    time 0.360s
actor       RIGHT   time 0.013s
actor       WAIT_ODOM       time 0.647s     angle: -23.8
actor       WAIT_SCAN       time 0.110s     x: -1.45        y: 1.5  yaw: -25.7
actor       STOP    time 0.230s
actor       AHEAD   time 0.009s
actor       WAIT_ODOM       time 1.831s     distance: 0.501
actor       WAIT_SCAN       time 0.210s     x: -0.957       y: 1.26 yaw: -25.9
actor       STOP    time 0.350s
actor       AHEAD   time 0.007s
actor       WAIT_ODOM       time 1.823s     distance: 0.508
actor       WAIT_SCAN       time 0.200s     x: -0.46        y: 1.02 yaw: -26
actor       STOP    time 0.170s
actor       AHEAD   time 0.005s
actor       CRASH   time 14.2228s
```
There is no collision avoidance in mode 5. 

In mode 6 we add a simple check before moving forward to see if anything in the field of view directly ahead (defaulting to +/- 20 degrees) is within a certain range (defaulting to 1 metre). If the turtlebot is blocked ahead it rotates left or right according to the given distribution. 

In mode 7 we add a further check. When the turtlebot is blocked ahead it looks to the left and right at the rotation angle (defaulting to 30 degrees), so that if the turtlebot is also blocked to the left it turns right, and vice-versa. Thus, in mode 7 the turtlebot spends less time oscillating in corners.

Now we run *model* 76, which was discussed above in [Physical configuration](#Physical). The `model076.json` file is
```json
{
	"update_interval" : 1,
	"act_interval" : 10,
	"structure" : "struct001",
	"model" : "model076",
	"mode" : "mode007",
	"logging_update" : false,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
Run the simulation at 20x -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env016.model -s libgazebo_ros_init.so

```
Run the actor -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model076.json

```
This is the log before it crashes -
```
...
model076_1_10       induce summary  slice: 1441937  diagonal: 33.2406       fud cardinality: 112    model cardinality: 1811 fuds per threshold: 1.12
model076_1_11       induce summary  slice: 1573023  diagonal: 33.9256       fud cardinality: 129    model cardinality: 2190 fuds per threshold: 1.29
...
model076_2  induce summary  slice: 1704951  diagonal: 29.6998       fud cardinality: 850    model cardinality: 15972        fuds per threshold: 0.944245
model076_2  induce summary  slice: 1706485  diagonal: 14.0399       fud cardinality: 851    model cardinality: 15983        fuds per threshold: 0.944265
...
```
We can take *model* 76 as the initial *model* to create *model* 77, run at 4x to avoid crashes -

```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env012.model -s libgazebo_ros_init.so

ros2 run TBOT03 actor model077.json

{
	"update_interval" : 2,
	"act_interval" : 2,
	"structure" : "struct001",
	"model_initial" : "model076",
	"structure_initial" : "struct001",
	"model" : "model077",
	"mode" : "mode007",
	"summary_level1" : true,
	"summary_level2" : true
}
```
This is the tail end of the log -
```
model077_2  induce summary  slice: 1706675  diagonal: 29.3814       fud cardinality: 3925   model cardinality: 67049        fuds per threshold: 0.993641
```
*Model* 76 has an active *size* of 90,178. The *level* two active has a *fuds* per threshold per *size* of 0.944265, which is lower than similar *models* in `TBOT02`. *Model* 77 has an active *size* of 395,073. Its *level* two active has a *fuds* per threshold per *size* of 0.993641, which is higher than that of *model* 76 but still lower than similar *models* in `TBOT02`.

The lower *model* measures might be due to the fact that a bug in `TBOT02` meant that it used a `WMAX` of 9 instead of 18 for the *level* two active, inadvertently improving the *likelihood*. 

More likely, it is due to the time it spends oscillating in corners. If we analyse the *substrate* we can see that the actual distribution of the actor actions suggests that it travels forward far less often than it should -
```
cd ~/TBOT03_ws
./main substrate_analyse model076_2

hr->dimension: 2
hr->size: 90178
({(motor,0)},27837 % 1)
({(motor,1)},35473 % 1)
({(motor,2)},26868 % 1)

({(location,door12)},1036 % 1)
({(location,door13)},1412 % 1)
({(location,door14)},1013 % 1)
({(location,door45)},502 % 1)
({(location,door56)},867 % 1)
({(location,room1)},23366 % 1)
({(location,room2)},12752 % 1)
({(location,room3)},17196 % 1)
({(location,room4)},16792 % 1)
({(location,room5)},7013 % 1)
({(location,room6)},8229 % 1)

cd ~/TBOT03_ws
./main substrate_analyse model077_2
hr->dimension: 2
hr->size: 395073
({(motor,0)},128717 % 1)
({(motor,1)},139604 % 1)
({(motor,2)},126752 % 1)

({(location,door12)},4762 % 1)
({(location,door13)},5044 % 1)
({(location,door14)},4840 % 1)
({(location,door45)},2048 % 1)
({(location,door56)},4209 % 1)
({(location,room1)},97649 % 1)
({(location,room2)},51097 % 1)
({(location,room3)},58539 % 1)
({(location,room4)},86744 % 1)
({(location,room5)},38283 % 1)
({(location,room6)},41858 % 1)
```
We can compare this to a `TBOT01` dataset -
```
./main analyse data009
hr->dimension: 363
hr->size: 172301
...
({(motor,0)},17809 % 1)
({(motor,1)},136432 % 1)
({(motor,2)},18060 % 1)

({(location,door12)},2067 % 1)
({(location,door13)},2365 % 1)
({(location,door14)},2012 % 1)
({(location,door45)},1288 % 1)
({(location,door56)},2314 % 1)
({(location,room1)},42708 % 1)
({(location,room2)},19975 % 1)
({(location,room3)},17110 % 1)
({(location,room4)},45058 % 1)
({(location,room5)},16658 % 1)
({(location,room6)},20746 % 1)
...
```
In `TBOT01` and `TBOT02` the actor turned even if the way ahead was not blocked but the side opposite to its turn bias was blocked. Also the turn direction depended on the bias which rarely changed during the actual turning. 

This might also partly explain why `location` *entropies* of these two *models* is so much higher than similar *sized models* in `TBOT02`. Before going on to consider improvements to the random mode, however, let us see how well *model* 77 fares with navigating to a goal room.

#### Room goal modes 8 and 9

`TBOT03` mode 8 is very similar to [`TBOT02` mode 4 ](https://github.com/caiks/TBOT02#Actor_mode_4). Firstly, the topology is cached at startup rather than being recalculated for each action. Secondly, instead of a *slice* topology, mode 8 crosses with the `location` to create a *slice*-`location` topology. The count of transitions in the shortest path from local *slice*-`location` to the global goal *slice*-`location` set is more realistic than without `location` because of the high label *entropy*. The corresponding shortest-path neighbourhood should, in theory, provide a useful basis for the actions -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env009.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

```
```
{
	"update_interval" : 2,
	"act_interval" : 2,
	"no_induce" : true,
	"structure" : "struct001",
	"model_initial" : "model077",
	"structure_initial" : "struct001",
	"mode" : "mode008",
	"logging_update" : true,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : false,
	"summary_level2" : false
}
```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 commander room5 60 17 50

```
This is the log after 2 hours -
```
transition_success_rate: 12.3143
transition_expected_success_rate: 8.33337
transition_null_rate: 63.1134

room_initial: room5
TBOT03 commander node has been initialised
goal: room2     n: 1    mean: 892       std dev: 0      std err: 0      running mean: 892       running std dev: 0      running std err: 0
goal: room6     n: 2    mean: 2243      std dev: 1351   std err: 955.301        running mean: 2243      running std dev: 1351   running std err: 955.301
goal: room2     n: 3    mean: 12585     std dev: 14667.3        std err: 8468.19        running mean: 12585     running std dev: 14667.3        running std err: 8468.19
goal: room6     n: 4    mean: 13041     std dev: 12726.8        std err: 6363.41        running mean: 13041     running std dev: 12726.8        running std err: 6363.41
goal: room2     n: 5    mean: 13517     std dev: 11423  std err: 5108.5 running mean: 13517     running std dev: 11423  running std err: 5108.5
goal: room4     n: 6    mean: 13314.8   std dev: 10437.5        std err: 4261.08        running mean: 13314.8   running std dev: 10437.5        running std err: 4261.08
goal: room6     n: 7    mean: 13342.1   std dev: 9663.45        std err: 3652.44        running mean: 13342.1   running std dev: 9663.45        running std err: 3652.44
goal: room2     n: 8    mean: 12430.2   std dev: 9355.77        std err: 3307.76        running mean: 12430.2   running std dev: 9355.77        running std err: 3307.76
goal: room4     n: 9    mean: 12087.6   std dev: 8873.8 std err: 2957.93        running mean: 12087.6   running std dev: 8873.8 running std err: 2957.93
```
While its transition success rate is (12.3-8.3)/(1.0-0.631) = 10.8%, which is better than that of `TBOT02`, the navigation performance is very poor. This is because it frequently becomes stuck in loops or in corners. The problem is that the *model* does not have a sufficient number of low deviation *slices* always to provide a path to goal even from room 4 to room 5. 

We can demonstrate that this is the case in mode 9. In this mode, the choices that would have been made are calculated and traced but the control is now done manually. A modification to the commander allows us to set the action -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env009.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

```
```
{
	"update_interval" : 2,
	"act_interval" : 2,
	"no_induce" : true,
	"structure" : "struct001",
	"model_initial" : "model077",
	"structure_initial" : "struct001",
	"mode" : "mode009",
	"logging_update" : true,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : false,
	"summary_level2" : false
}
```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 commander LEFT
ros2 run TBOT03 commander AHEAD
ros2 run TBOT03 commander RIGHT

```
In this sequence we begin 3 steps away from room 5 approaching from right. All previous slices are ambiguous -
```
actor       goal:AHEAD
actor       AHEAD   time 3.705s
actor       WAIT_ODOM       time 1.841s     distance: 0.509
actor       WAIT_SCAN       time 0.202s     x: -4.41        y: 2.81 yaw: 151
actor       STOP    time 0.272s
locations[locA]: room4
sliceLocA: 18746890
actor.eventsRecord(historyEventA): (-4.41126,2.81218,151.223)
neighbours: {(18746901,2),(18750487,2),(18912336,1),(18928209,1),(18932444,1),(18947899,1),(18947910,2)}
least: 1
neighbourLeasts: {18912336,18928209,18932444,18947899}
transition_success_rate: 27.2727
transition_expected_success_rate: 14.7509
transition_null_rate: 36.3636
record: (-4.9616,2.54394,154.439)
record: (-4.864,2.60797,149.98)
record: (-4.75085,2.43268,148.37)
record: (-4.74081,2.47845,142.994)
record: (-4.72679,2.75988,159.749)
record: (-4.70993,2.38943,141.813)
record: (-4.64723,2.63207,155.35)
record: (-4.60178,3.38246,161.111)
record: (-4.51526,3.1513,-158.949)
record: (-4.41126,2.81218,151.223)
record: (-4.17517,2.79346,140.08)
record: (-2.98415,1.79346,-11.8733)
recordsMean(recordStandards).config(): (-4.5074,2.64811,111.19)
recordsDeviation(recordStandards): 0.264161
actionsCount: {(1,8)}
actor       goal:AHEAD
actor       AHEAD   time 31.260s
actor       WAIT_ODOM       time 1.804s     distance: 0.509
actor       WAIT_SCAN       time 0.206s     x: -4.88        y: 3.09 yaw: 149
actor       STOP    time 0.244s
locations[locA]: door45
sliceLocA: 18908530
actor.eventsRecord(historyEventA): (-4.88708,3.09318,149.362)
neighbours: {(18763110,1),(18800378,1),(18811334,1),(18838031,2),(18838097,2),(18838317,1),(18862715,2),(18866103,1),(18874837,1),(18881965,2),(18882702,2),(18883175,1),(18894736,1),(18894747,1),(18954730,1)}
least: 1
neighbourLeasts: {18763110,18800378,18811334,18838317,18866103,18874837,18883175,18894736,18894747,18954730}
transition_success_rate: 25
transition_expected_success_rate: 13.5216
transition_null_rate: 41.6667
record: (-5.06587,2.8607,142.779)
record: (-5.02026,2.94188,142.548)
record: (-5.01515,2.82079,143.733)
record: (-5.01153,2.9356,142.433)
record: (-5.00167,2.66534,143.85)
record: (-4.99744,2.82138,136.398)
record: (-4.90548,2.918,131.172)
record: (-4.88708,3.09318,149.362)
record: (-4.88625,2.98187,134.417)
record: (-4.87977,2.91927,137.666)
record: (-4.86373,3.11077,140.484)
record: (-4.84094,3.16154,147.353)
record: (-4.83975,2.74191,139.886)
record: (-4.83742,3.05049,150.42)
record: (-4.77453,2.79094,138.483)
record: (-4.77402,2.79014,138.351)
record: (-4.77009,2.85562,140.978)
record: (-4.75207,2.8483,137.7)
record: (-4.72749,3.02889,136.994)
record: (-4.72241,3.34354,129.167)
record: (-4.72136,3.01224,136.367)
record: (-4.68858,3.11888,138.666)
record: (-4.67847,3.21472,142.1)
recordsMean(recordStandards).config(): (-4.85484,2.95765,140.057)
recordsDeviation(recordStandards): 0.022714
actionsCount: {(1,15)}
actor       goal:AHEAD
actor       AHEAD   time 168.113s
actor       WAIT_ODOM       time 1.600s     distance: 0.501
actor       WAIT_SCAN       time 0.204s     x: -5.33        y: 3.4  yaw: 145
actor       STOP    time 0.328s
locations[locA]: door45
sliceLocA: 18883175
actor.eventsRecord(historyEventA): (-5.33604,3.40131,145.413)
neighbours: {(18766377,1),(18807490,0),(18857672,0),(18881998,2),(18894247,0),(18894742,0),(18932620,1),(18947531,0)}
least: 0
neighbourLeasts: {18807490,18857672,18894247,18894742,18947531}
transition_success_rate: 30.7692
transition_expected_success_rate: 17.6097
transition_null_rate: 38.4615
record: (-5.53493,3.29021,149.403)
record: (-5.52074,3.29973,148.474)
record: (-5.52069,3.30005,148.27)
record: (-5.33604,3.40131,145.413)
record: (-5.30758,3.54669,148.385)
record: (-5.30076,3.46075,146.468)
record: (-5.28061,3.53057,144.279)
record: (-5.21639,3.52913,137.401)
record: (-5.21624,3.48592,137.731)
record: (-5.12453,3.57356,135.499)
record: (-5.0935,3.48735,136.702)
record: (-5.07736,3.45355,139.628)
record: (-5.01815,3.4727,140.735)
recordsMean(recordStandards).config(): (-5.27289,3.44858,142.953)
recordsDeviation(recordStandards): 0.0210105
actionsCount: {(1,8)}
actor       goal:AHEAD
actor       AHEAD   time 51.940s
actor       WAIT_ODOM       time 1.758s     distance: 0.508
actor       WAIT_SCAN       time 0.204s     x: -5.78        y: 3.71 yaw: 145
actor       STOP    time 0.302s
locations[locA]: room5
sliceLocA: 18894742
actor.eventsRecord(historyEventA): (-5.78985,3.71544,145.306)
neighbours: {(18745868,0),(18790506,0),(18805213,0),(18825816,0),(18840633,0),(18857672,0),(18919492,0),(18927918,0),(18956199,0)}
least: 0
neighbourLeasts: {18745868,18790506,18805213,18825816,18840633,18857672,18919492,18927918,18956199}
transition_success_rate: 35.7143
transition_expected_success_rate: 20.8161
transition_null_rate: 35.7143
actionsCount: {}
```
We can see that the first step is ambiguous but happens to be correct, the last two steps are correct. That is, there are sometimes low deviation 'tunnels' to goal in certain *slices*.

This is not always the case, though - this approach from right has no luck -
```
actor       goal:LEFT
actor       LEFT    time 6.366s
actor       WAIT_ODOM       time 0.628s     angle: 23.1
actor       WAIT_SCAN       time 0.102s     x: -4.14        y: 4.45 yaw: -162
actor       STOP    time 0.358s
locations[locA]: room4
sliceLocA: 18949928
actor.eventsRecord(historyEventA): (-4.13752,4.45259,-161.997)
neighbours: {}
least: 0
neighbourLeasts: {}
transition_success_rate: 5.26316
transition_expected_success_rate: 4.36042
transition_null_rate: 68.4211
actionsCount: {}
actor       goal:AHEAD
actor       AHEAD   time 18.651s
actor       WAIT_ODOM       time 1.847s     distance: 0.51
actor       WAIT_SCAN       time 0.210s     x: -4.66        y: 4.28 yaw: -162
actor       STOP    time 0.240s
locations[locA]: room4
sliceLocA: 18939632
actor.eventsRecord(historyEventA): (-4.67051,4.28111,-162.164)
neighbours: {(18790890,2),(18790934,3),(18809172,3),(18820282,3),(18820337,2),(18835143,1),(18835144,0),(18837376,2),(18855801,3),(18870640,1),(18873412,3),(18888845,2),(18893026,0),(18904520,2),(18923913,1),(18927290,2),(18932911,2),(18939699,0),(18953349,2),(18954449,3)}
least: 0
neighbourLeasts: {18835144,18893026,18939699}
transition_success_rate: 5
transition_expected_success_rate: 4.14239
transition_null_rate: 70
record: (-4.75839,4.23672,-167.648)
record: (-4.7514,4.2066,-168.771)
record: (-4.70615,4.23509,-163.59)
record: (-4.68102,4.31272,-159.481)
record: (-4.67051,4.28111,-162.164)
record: (-4.63861,4.349,-154.719)
record: (-4.59832,4.26167,-140.724)
record: (-4.59609,4.31813,-157.781)
record: (-4.54589,4.44682,-142.85)
record: (-4.51442,4.36654,-136.012)
record: (-3.67404,4.72455,-160.923)
record: (-3.5936,4.70378,-157.822)
record: (-3.53164,4.73006,-159.069)
record: (-3.50688,4.71231,-163.984)
record: (-3.48899,4.70431,-158.165)
record: (-3.48463,0.813982,-28.9957)
record: (-3.40722,0.817243,-31.9204)
record: (-3.3381,4.74963,-162.584)
record: (-3.29562,4.73266,-162.444)
record: (-3.25783,4.72927,-167.59)
record: (-1.86244,4.24274,156.766)
record: (-1.75437,4.27639,147.845)
record: (-1.73335,0.348791,17.5367)
record: (-1.01467,1.62824,69.544)
record: (-0.951391,1.66583,70.701)
record: (-0.612843,3.75314,109.939)
record: (-0.608354,3.95653,116.129)
record: (-0.607435,3.78828,117.274)
record: (-0.597169,3.85509,112.111)
record: (-0.596998,3.83145,112.563)
record: (-0.57529,3.17496,101.561)
recordsMean(recordStandards).config(): (-2.96625,3.7727,-57.2667)
recordsDeviation(recordStandards): 0.387261
actionsCount: {(1,4)}
actor       goal:AHEAD
actor       AHEAD   time 10.796s
actor       WAIT_ODOM       time 1.790s     distance: 0.502
actor       WAIT_SCAN       time 0.210s     x: -5.19        y: 4.12 yaw: -162
actor       STOP    time 0.290s
locations[locA]: room5
sliceLocA: 18880926
actor.eventsRecord(historyEventA): (-5.19127,4.11592,-162.409)
neighbours: {(18762291,0),(18780166,0),(18793993,0),(18863238,0),(18865020,0),(18894137,0),(18921329,0),(18939699,0)}
least: 0
neighbourLeasts: {18762291,18780166,18793993,18863238,18865020,18894137,18921329,18939699}
transition_success_rate: 4.7619
transition_expected_success_rate: 3.94514
transition_null_rate: 71.4286
actionsCount: {}
```
Here is a successful transition -
```
locations[locA]: door56
sliceLocA: 18754916
actor.eventsRecord(historyEventA): (-6.3306,1.29055,84.4717)
record: (-6.38131,1.03265,92.0128)
record: (-6.34975,0.948405,86.993)
record: (-6.33608,0.949404,89.9757)
record: (-6.3306,1.29055,84.4717)
record: (-6.31727,1.03887,85.9343)
record: (-6.30239,1.05015,93.0726)
record: (-6.29992,1.06378,83.2033)
record: (-6.29662,1.13977,91.9588)
record: (-6.27197,1.04943,93.219)
record: (-6.24911,1.10854,94.9866)
record: (-6.23914,1.05122,79.4263)
recordsMean(recordStandards).config(): (-6.30674,1.06571,88.6595)
recordsDeviation(recordStandards): 0.0159766
neighbours: {(18749295,1),(18763391,0),(18810966,0),(18816560,1),(18855505,0),(18888912,0),(18894940,0)}
least: 0
neighbourLeasts: {18763391,18810966,18855505,18888912,18894940}
transition_success_rate: 4.25532
transition_expected_success_rate: 3.41392
transition_null_rate: 80.8511
actionsCount: {(1,8)}
actor       goal:AHEAD
actor       AHEAD   time 18.274s
actor       WAIT_ODOM       time 1.778s     distance: 0.505
actor       WAIT_SCAN       time 0.204s     x: -6.28        y: 1.83 yaw: 84.5
actor       STOP    time 0.316s
locations[locA]: room5
sliceLocA: 18855505
actor.eventsRecord(historyEventA): (-6.27756,1.83661,84.4508)
```
We can see from the experiments above that the *slices* often have several configuration clusters. If the *model* had more *events* in the *slices* away from the walls and corners, then the *model* might resolve into single cluster *slices*, i.e. low deviation *slices*. 

We will attempt to address this problem by reverting the random mode 7 back to the obstruction handling in `TBOT01` and `TBOT02`.

#### Random modes 10 - 12

Modes 10, 11 and 12 are variations on the obstruction handling in `TBOT01` and `TBOT02`. They all use a turn bias in order to prevent the turtlebot from oscillating before obstructions. After experimenting with the various modes and parameters, it was found that the mode 12 prevented both crashing and the time spent in indecision. Its operation is simple - if there is any obstruction within the collision range and within the collision field of view then it turns in the direction of the turn bias only. The turn bias alternates between left and right at random intervals that tend to be longer than needed to escape from the current obstruction. That is, if the turtlebot is blocked it turns randomly to the either left or right, and then usually sticks to that decision until it is free to move at random again. 

This is the configuration for *model* 79 running in mode 12 -

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env015.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model079.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model079",
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
Now we find that now it does, in fact, spend a larger proportion of its time moving ahead
```
cd ~/turtlebot3_ws/src/TBOT03_ws
./main substrate_analyse model079_2

hr->dimension: 2
hr->size: 347920
({(motor,0)},93325 % 1)
({(motor,1)},161054 % 1)
({(motor,2)},93541 % 1)

({(location,door12)},6035 % 1)
({(location,door13)},4060 % 1)
({(location,door14)},3895 % 1)
({(location,door45)},1776 % 1)
({(location,door56)},4451 % 1)
({(location,room1)},94882 % 1)
({(location,room2)},55812 % 1)
({(location,room3)},30103 % 1)
({(location,room4)},72314 % 1)
({(location,room5)},30709 % 1)
({(location,room6)},43883 % 1)
```
(Note that, although the fraction of *events* moving ahead is still lower than in `TBOT01`, the distance moved per *event* is greater.)

However, the reduction in the time spent avoiding obstructions does not translate into lower `location` *entropy* at 0.856429 -
```
./main location_entropy model079_2
model: model079_2
model079_2      load    file name: model079_2.ac        time 0.205295s
activeA.historyOverflow: false
sizeA: 347920
activeA.decomp->fuds.size(): 3393
activeA.decomp->fudRepasSize: 58541
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.975224
entropyA: 297969
entropyA/sizeA: 0.856429
(double)sizeA * std::log(sizeA): 4.43936e+06
std::log(sizeA): 12.7597
```
Nor does it improve the configuration deviation at 0.202962, which is very similar to *models* 76 and 77 - 
```
./main configuration_deviation_all model079 
model: model079
model079_2      load    file name: model079_2.ac        time 0.191438s
activeA.historyOverflow: false
sizeA: 347920
activeA.decomp->fuds.size(): 3393
activeA.decomp->fudRepasSize: 58541
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 0.975224
records->size(): 347920
size: 347920
slice_count: 14015
slice_size_mean: 24.8248
deviation: 0.422196
size: 347920
slice_location_count: 35811
slice_location_size_mean: 9.71545
deviation_location: 0.202962
```
Having determined that obstructions are not a major factor in the high deviation, we will go on to consider whether the *model* itself might be the reason. There are only 12 *underlying models* in *level* 1, so it may be the case that features or details with an angular resolution of less than 30 degrees might sometimes have no *alignment* at *level* 2. This might explain why the turtlebot does not always distinguish between narrow and wide doorways, especially at a distance. Inability to distinguish landmarks could certainly increase configuration deviation. So, in *model* 80, we increased the number of *underlying* to 36, i.e. the angular resolution is decreased to 10 degrees -

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env015.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model080.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model080",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
The mode 12 `motor` *values* distribution is similar to that of *model* 79, as we would expect -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
./main substrate_analyse model080_2
hr->dimension: 2
hr->size: 222012
({(motor,0)},59977 % 1)
({(motor,1)},101997 % 1)
({(motor,2)},60038 % 1)

({(location,door12)},2888 % 1)
({(location,door13)},2446 % 1)
({(location,door14)},2204 % 1)
({(location,door45)},1569 % 1)
({(location,door56)},3755 % 1)
({(location,room1)},47491 % 1)
({(location,room2)},25840 % 1)
({(location,room3)},19586 % 1)
({(location,room4)},57088 % 1)
({(location,room5)},25111 % 1)
({(location,room6)},34034 % 1)
```
The `location` *entropy* at 0.895414 and configuration deviation at 0.204491 are little different, however, -
```
./main location_entropy model080_2
model: model080_2
model080_2      load    file name: model080_2.ac        time 0.112666s
activeA.historyOverflow: false
sizeA: 222012
activeA.decomp->fuds.size(): 2426
activeA.decomp->fudRepasSize: 37306
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.09273
entropyA: 198793
entropyA/sizeA: 0.895414
(double)sizeA * std::log(sizeA): 2.73308e+06
std::log(sizeA): 12.3105

./main configuration_deviation_all model080 
model: model080
model080_2      load    file name: model080_2.ac        time 0.116825s
activeA.historyOverflow: false
sizeA: 222012
activeA.decomp->fuds.size(): 2426
activeA.decomp->fudRepasSize: 37306
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.09273
records->size(): 222012
size: 222012
slice_count: 7814
slice_size_mean: 28.4121
deviation: 0.427935
size: 222012
slice_location_count: 21223
slice_location_size_mean: 10.4609
deviation_location: 0.204491
```
Note, though, that the *fuds* per *size* per threshold has improved from 0.975224 to 1.09273, which is similar to the `TBOT02` case. We might speculate that this is because the increased angular resolution counteracts the more gridlike distribution of the turtlebot's poses in `TBOT03`.

Let us continue to focus on the *model* by improving the *level* 1 *models*. In *model* 81 we increase the *level* one active *size*, increase the *induce* threshold and also increase the *induction* parameters `XMAX` and `WMAX`. We also alter the *level* two initial threshold and parameter `XMAX`,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env015.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model081.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model081",
	"valency_scan" : 16,
	"level1Count" : 36,
	"activeSizeLevel1" : 90000,
	"induceThresholdLevel1" : 300,
	"induceThresholdInitialLevel1" : 900,
	"induceParametersLevel1.xmax" : 1024,
	"induceParametersLevel1.wmax" : 18,
	"induceThresholdInitial" : 1000,
	"induceParameters.xmax" : 1024,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```

```
./main location_entropy model081_2
model: model081_2
model081_2      load    file name: model081_2.ac        time 0.183945s
activeA.historyOverflow: false
sizeA: 275496
activeA.decomp->fuds.size(): 2836
activeA.decomp->fudRepasSize: 54419
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.02942
entropyA: 276339
entropyA/sizeA: 1.00306
(double)sizeA * std::log(sizeA): 3.45095e+06
std::log(sizeA): 12.5263

./main configuration_deviation_all model081 
model: model081
model081_2      load    file name: model081_2.ac        time 0.168261s
activeA.historyOverflow: false
sizeA: 275496
activeA.decomp->fuds.size(): 2836
activeA.decomp->fudRepasSize: 54419
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.02942
records->size(): 275496
size: 275496
slice_count: 8194
slice_size_mean: 33.6217
deviation: 0.450795
size: 275496
slice_location_count: 25524
slice_location_size_mean: 10.7936
deviation_location: 0.21413

```
Both the `location` *entropy* and configuration deviation increase.

Note that the *fuds* per *size* per threshold are not in fact as low as 1.02942. This is an artefact of the slower induce performance, which causes the *model* to lag behind the available *history*. In fact, when the turtlebot eventually crashed, there were 119 *slices* that had exceeded the induce threshold but were yet to be processed.

In *model* 82 we run a very similar configuration but with lower `WMAX`,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env015.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model082.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model082",
	"valency_scan" : 16,
	"level1Count" : 36,
	"activeSizeLevel1" : 90000,
	"induceThresholdLevel1" : 300,
	"induceThresholdInitialLevel1" : 900,
	"induceParametersLevel1.xmax" : 1024,
	"induceParametersLevel1.wmax" : 9,
	"induceThresholdInitial" : 1000,
	"induceParameters.xmax" : 1024,
	"induceParameters.wmax" : 9,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
Again, these changes increase the `location` *entropy* and configuration deviation -
```
./main location_entropy model082_2
model: model082_2
model082_2      load    file name: model082_2.ac        time 0.0730772s
activeA.historyOverflow: false
sizeA: 90073
activeA.decomp->fuds.size(): 992
activeA.decomp->fudRepasSize: 16870
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.10133
entropyA: 100869
entropyA/sizeA: 1.11986
(double)sizeA * std::log(sizeA): 1.02759e+06
std::log(sizeA): 11.4084

./main configuration_deviation_all model082 
model: model082
model082_2      load    file name: model082_2.ac        time 0.0672517s
activeA.historyOverflow: false
sizeA: 90073
activeA.decomp->fuds.size(): 992
activeA.decomp->fudRepasSize: 16870
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.10133
records->size(): 90073
size: 90073
slice_count: 2361
slice_size_mean: 38.1504
deviation: 0.471576
size: 90073
slice_location_count: 8877
slice_location_size_mean: 10.1468
deviation_location: 0.235734
```
In *model* 83 we go back to the original *induce* parameterisation of *model* 80, but add new landmarks to the environment, to see if this can reduce ambiguity in physical configuration,

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model083.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model083",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
```
./main location_entropy model083_2
model: model083_2
model083_2      load    file name: model083_2.ac        time 0.286235s
activeA.historyOverflow: false
sizeA: 365483
activeA.decomp->fuds.size(): 3924
activeA.decomp->fudRepasSize: 60527
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07365
entropyA: 406947
entropyA/sizeA: 1.11345
(double)sizeA * std::log(sizeA): 4.68146e+06
std::log(sizeA): 12.809

./main configuration_deviation_all model083 
model: model083
model083_2      load    file name: model083_2.ac        time 0.280011s
activeA.historyOverflow: false
sizeA: 365483
activeA.decomp->fuds.size(): 3924
activeA.decomp->fudRepasSize: 60527
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07365
records->size(): 365483
size: 365483
slice_count: 13043
slice_size_mean: 28.0214
deviation: 0.442595
size: 365483
slice_location_count: 42663
slice_location_size_mean: 8.56674
deviation_location: 0.188369
```
Interesingly, we see an increase in `location` *entropy* but a decrease configuration deviation to 0.188369. We may conjecture that we have added some strong *alignments* within the rooms where the landmarks have been added, at the cost of configuration deviation relevant *alignments* between the rooms. Now we run the same *model* for longer to create *model* 84 -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model084.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure_initial" : "struct001",
	"model_initial" : "model083",
	"structure" : "struct001",
	"model" : "model084",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
After 929471 *events* the configuration deviation has decreased again to 0.167629 -
```
./main location_entropy model084_2
model: model084_2
model084_2      load    file name: model084_2.ac        time 1.28805s
activeA.historyOverflow: false
sizeA: 929471
activeA.decomp->fuds.size(): 10288
activeA.decomp->fudRepasSize: 157302
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.10687
entropyA: 836547
entropyA/sizeA: 0.900025
(double)sizeA * std::log(sizeA): 1.27731e+07
std::log(sizeA): 13.7424

./main configuration_deviation_all model084 
model: model084
model084_2      load    file name: model084_2.ac        time 1.29636s
activeA.historyOverflow: false
sizeA: 929471
activeA.decomp->fuds.size(): 10288
activeA.decomp->fudRepasSize: 157302
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.10687
records->size(): 929472
size: 929471
slice_count: 30632
slice_size_mean: 30.3431
deviation: 0.399805
size: 929471
slice_location_count: 88665
slice_location_size_mean: 10.483
deviation_location: 0.167629
```
Although we have seen an improvement, the configuration deviation is nowhere near what would be needed to obtain an average deviation of the scale that we see in *slices* that have only one cluster such as this one in *model* 77 when run in manual mode 9 (also shown above) -
```
sliceLocA: 18754916
actor.eventsRecord(historyEventA): (-6.3306,1.29055,84.4717)
record: (-6.38131,1.03265,92.0128)
record: (-6.34975,0.948405,86.993)
record: (-6.33608,0.949404,89.9757)
record: (-6.3306,1.29055,84.4717)
record: (-6.31727,1.03887,85.9343)
record: (-6.30239,1.05015,93.0726)
record: (-6.29992,1.06378,83.2033)
record: (-6.29662,1.13977,91.9588)
record: (-6.27197,1.04943,93.219)
record: (-6.24911,1.10854,94.9866)
record: (-6.23914,1.05122,79.4263)
recordsMean(recordStandards).config(): (-6.30674,1.06571,88.6595)
recordsDeviation(recordStandards): 0.0159766
```
Next, let us try adding a `direction` *variable* to the *substrate* (a magnetometer in practice) to see if there are *alignments* with it that can distringuish between clusters. In structure `struct003` the `direction` has a *valency* of 12. *Model* 85 is a copy of *model* 83 with structure `struct003` -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model085.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct003",
	"model" : "model085",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}
```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
./main location_entropy model085_2
model: model085_2
model085_2      load    file name: model085_2.ac        time 0.374053s
activeA.historyOverflow: false
sizeA: 324795
activeA.decomp->fuds.size(): 3495
activeA.decomp->fudRepasSize: 55642
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07606
entropyA: 356515
entropyA/sizeA: 1.09766
(double)sizeA * std::log(sizeA): 4.12196e+06
std::log(sizeA): 12.6909

./main configuration_deviation_all model085 
model: model085
model085_2      load    file name: model085_2.ac        time 0.328507s
activeA.historyOverflow: false
sizeA: 324795
activeA.decomp->fuds.size(): 3495
activeA.decomp->fudRepasSize: 55642
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07606
records->size(): 324795
size: 324795
slice_count: 11700
slice_size_mean: 27.7603
deviation: 0.434392
size: 324795
slice_location_count: 37943
slice_location_size_mean: 8.56008
deviation_location: 0.185767
```
We can see that the configuration deviation has decreased a little from 0.188369 to 0.185767. If we run *model* 85 in manual mode 9 we can step ahead once to view a *slice* with two large clusters and a few unclustered *events* -
```
locations[locA]: room4
sliceLocA: 53355277
actor.eventsRecord(historyEventA): (-0.990066,1.47734,-3.1242)
record: (-4.62833,2.18997,-147.263)
record: (-4.18553,1.23923,169.797)
record: (-4.16153,3.68723,-24.6138)
record: (-4.13195,3.70864,-24.7941)
record: (-4.12985,3.73495,-25.6054)
record: (-4.04945,3.65882,-21.4776)
record: (-4.01907,3.96418,-40.2642)
record: (-3.99751,3.88248,-36.9523)
record: (-3.98583,3.88344,-35.0906)
record: (-3.98557,3.51418,-26.7673)
record: (-3.97122,3.4146,-16.2444)
record: (-3.93743,3.38519,-22.0811)
record: (-3.92838,3.3992,-26.6156)
record: (-3.89309,3.87912,-36.2889)
record: (-3.8342,3.16802,-16.6457)
record: (-3.00994,0.726074,-86.2496)
record: (-1.51163,1.03998,132.138)
record: (-1.46686,1.28999,148.558)
record: (-1.31869,0.812944,-100.048)
record: (-1.11409,0.806501,-100.163)
record: (-1.03749,0.817748,-101.157)
record: (-1.00139,0.805619,-98.4984)
record: (-0.990066,1.47734,-3.1242)
record: (-0.945078,0.79466,-98.3732)
record: (-0.916815,0.758091,-94.0738)
record: (-0.907985,0.800604,-95.2947)
record: (-0.856944,0.784582,-98.7587)
record: (-0.689069,0.711402,-85.5383)
record: (-0.676035,0.77519,-95.3396)
record: (-0.668564,0.79086,-95.6515)
record: (-0.664544,0.754784,-90.4891)
recordsMean(recordStandards).config(): (-2.53594,2.08567,-41.7087)
recordsDeviation(recordStandards): 0.268739
```
Now let us run on until there is sufficient *history* to compare to the earlier *models* -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model086.json

{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure_initial" : "struct003",
	"model_initial" : "model085",
	"structure" : "struct003",
	"model" : "model086",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}

cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model087.json

{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure_initial" : "struct003",
	"model_initial" : "model086",
	"structure" : "struct003",
	"model" : "model087",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true
}

./main location_entropy model086_2
model: model086_2
model086_2      load    file name: model086_2.ac        time 0.950687s
activeA.historyOverflow: false
sizeA: 633806
activeA.decomp->fuds.size(): 6849
activeA.decomp->fudRepasSize: 106742
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.08061
entropyA: 605275
entropyA/sizeA: 0.954984
(double)sizeA * std::log(sizeA): 8.46733e+06
std::log(sizeA): 13.3595

./main configuration_deviation_all model086 
model: model086
model086_2      load    file name: model086_2.ac        time 0.851379s
activeA.historyOverflow: false
sizeA: 633806
activeA.decomp->fuds.size(): 6849
activeA.decomp->fudRepasSize: 106742
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.08061
records->size(): 633807
size: 633806
slice_count: 21574
slice_size_mean: 29.3782
deviation: 0.406912
size: 633806
slice_location_count: 64548
slice_location_size_mean: 9.81914
deviation_location: 0.170962

./main location_entropy model087_2
model: model087_2
model087_2      load    file name: model087_2.ac        time 1.45364s
activeA.historyOverflow: false
sizeA: 927748
activeA.decomp->fuds.size(): 10194
activeA.decomp->fudRepasSize: 157019
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.09879
entropyA: 805245
entropyA/sizeA: 0.867956
(double)sizeA * std::log(sizeA): 1.27477e+07
std::log(sizeA): 13.7405

./main configuration_deviation_all model087 
model: model087
model087_2      load    file name: model087_2.ac        time 1.23499s
activeA.historyOverflow: false
sizeA: 927748
activeA.decomp->fuds.size(): 10194
activeA.decomp->fudRepasSize: 157019
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.09879
records->size(): 927750
size: 927748
slice_count: 30779
slice_size_mean: 30.1422
deviation: 0.388352
size: 927748
slice_location_count: 86902
slice_location_size_mean: 10.6758
deviation_location: 0.162262
```
*Model* 87 is a small improvement over *model* 84 with a configuration deviation of 0.162262 instead of 0.167629. Of course, the improvement is far too small to justify the modification of the *substrate*.

Now let us return to the original *substrate*, i.e. without `direction`, but instead add a third *level* in structure `struct002`. In this *level* we add two *models*, the first will have *underlying frames* 0,1,2,3,4,6,8 and 10, the second has *underlying frames* 0,1,2,3 and 4, and *self frames* 5 and 10. *Model* 88 is otherwise a copy of *model* 83 -
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model088.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct002",
	"model" : "model088",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true,
	"summary_level3" : true
}
```
This run crashes after 72,400 *events* so we continue on in *model* 89 -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model089.json

```
```
{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure_initial" : "struct002",
	"model_initial" : "model088",
	"structure" : "struct002",
	"model" : "model089",
	"level1Count" : 36,
	"mode" : "mode012",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : true,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : true,
	"summary_level2" : true,
	"summary_level3" : true
}
```
First let's check the *level* two *model* -
```
cd ~/turtlebot3_ws/src/TBOT03_ws

./main location_entropy model089_2
model: model089_2
model089_2      load    file name: model089_2.ac        time 0.601311s
activeA.historyOverflow: false
sizeA: 536347
activeA.decomp->fuds.size(): 5822
activeA.decomp->fudRepasSize: 90656
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.08549
entropyA: 540269
entropyA/sizeA: 1.00731
(double)sizeA * std::log(sizeA): 7.07578e+06
std::log(sizeA): 13.1925

./main configuration_deviation_all model089 
model: model089
model089_2      load    file name: model089_2.ac        time 0.54833s
activeA.historyOverflow: false
sizeA: 536347
activeA.decomp->fuds.size(): 5822
activeA.decomp->fudRepasSize: 90656
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.08549
records->size(): 536348
size: 536347
slice_count: 18946
slice_size_mean: 28.3092
deviation: 0.420615
size: 536347
slice_location_count: 57844
slice_location_size_mean: 9.2723
deviation_location: 0.177661
```
As expected, *level* 2 *model* 89, with 536347 *events*, is intermediate between *model* 83 (365483 *events*) and *model* 84 (929471 *events*).

Let us examine the configuration deviation of each of the *level* 3 *models* -
```
cd ~/turtlebot3_ws/src/TBOT03_ws

./main configuration_deviation_all model089 model089_3_00
active: model089
model089_3_00   load    file name: model089_3_00.ac     time 0.586668s
activeA.historyOverflow: false
sizeA: 536347
activeA.decomp->fuds.size(): 5585
activeA.decomp->fudRepasSize: 121552
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.0413
records->size(): 536348
size: 536347
slice_count: 23906
slice_size_mean: 22.4357
deviation: 0.486294
size: 536347
slice_location_count: 96697
slice_location_size_mean: 5.54668
deviation_location: 0.222139

./main configuration_deviation_all model089 model089_3_01
active: model089
model089_3_01   load    file name: model089_3_01.ac     time 0.592444s
activeA.historyOverflow: false
sizeA: 536347
activeA.decomp->fuds.size(): 5729
activeA.decomp->fudRepasSize: 124982
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.06815
records->size(): 536348
size: 536347
slice_count: 24116
slice_size_mean: 22.2403
deviation: 0.484602
size: 536347
slice_location_count: 96073
slice_location_size_mean: 5.5827
deviation_location: 0.21878
```
They turn out to be more ambiguous, although not much more, presumably because the zeroth *frame alignments* are less prominent than those of the middle *frames*. The second *model*, with two *reflexive frames*, has and more *fuds* and so is a little more *likely* than the first *model*, which has only *underlying frames*. It also has a slightly better configuration deviation. Both *level* three *models* have smaller mean *slice sizes* than the *level* two *model*, with only 5-6 *events* against 9-10 *events*.

These *slice sizes* are already very small, so we cannot expect that crosssing the *level* two *model* with a *level* three *model* will produce a very useful *slice* topology -
```
cd ~/turtlebot3_ws/src/TBOT03_ws
./main configuration_deviation_all_3level model089 model089_2 model089_3_00
active: model089
model089_2      load    file name: model089_2.ac        time 0.494546s
model089_3_00   load    file name: model089_3_00.ac     time 0.491431s
records->size(): 536348
size: 536347
slice_count: 463822
slice_size_mean: 1.15636
deviation: 0.10275
size: 536347
slice_location_count: 478766
slice_location_size_mean: 1.12027
deviation_location: 0.0341585

./main configuration_deviation_all_3level model089 model089_2 model089_3_01
active: model089
active: model089
model089_2      load    file name: model089_2.ac        time 0.499151s
model089_3_01   load    file name: model089_3_01.ac     time 0.503101s
records->size(): 536348
size: 536347
slice_count: 440249
slice_size_mean: 1.21828
deviation: 0.122966
size: 536347
slice_location_count: 460388
slice_location_size_mean: 1.16499
deviation_location: 0.0411679
```
The `location` configuration deviation is now very small, but that is obviously because the mean *slice size* is now only between one and two *events*. That is, the *models* of the different *levels* seem to be rather orthogonal - so much so, in fact, that crossing gives us little benefit. The map of *slice* to configuration is far too over-fitted.



problems with the topology - measure of deviation rather than configuration entropy

calculate room entropy - model 77 has considerably higher room label entropy than the similar TBOT02 model 65 - we will need slice-room transitions - add to README. 

In model 77, of the 38k slices only 19k have 5 or more events. Around 17K have low variance. Only around 10% of larger slices have low variance. So even if we prevent loops and ambiguous slices, the slice topology is likely to be patchy. We should seek to improve both the model and the effectiveness. README

In TBOT03 we are stopping between actions like a bird walking and so much more successful, but ironically this means that the turtlebot is more likely to be stuck in any loops caused by ambiguity wormholes. We must either have better maps to the configuration space or loop handling that artificially fails to go to the wormhole neighbour. Always better to have a good map because delooping is not guaranteed to be better than chance. It will help only if there are parts of the slice topology that is well mapped eg at doorways.

can't really expect too much - need landmarks, magnetic compass, etc

mustn't make unfair anthropomorphic comparisons, except between TBOT03 and a newborn

TBOT03 measure successful transition actions

debugging the topology

mode 9 manual with mode 8 

TBOT03  README in mode 9 we are essentially debugging the slice topology by manually manoeuvring the turtlebot with tracing what it would have done automatically in mode 8


TBOT03 bigger gain from forcing an action per request mapping to a change in configuration?

problem with structure 1 - if a detail is not visible across at least two the underlying actives then there are no alignments in the overlying active, and so, although, the underlying can 'see' the feature, it does not appear in the slice topology. One solution is to cross the overlying and underlying slices. 


TBOT03 we can model configuration space clustering by making a slice tree for each dimension and then inducing. The resultant configuration model slices are the clusters. Of course this is just the same as having the odometry as substrate instead of LiDAR. No doubt we could have a very good slice topology in that case. We can compare models by rel ent so could compare the substrate induced model to the configuration induced model (with same threshold) to get a measure of the quality of the slice topology. Then can run the model without odometry knowing whether it's slice topology was better or worse than some other substrate induced model.

TBOT03 NB 202103081030 - essentially, we can measure the quality induced model's slice topology by inducing a model of the configuration space from the odometry, and then the clusterivity of the slice topology is the negative label entropy wrt the induced slice where the label is the configuration model slice (partition variable). The connectivity of the slice topology is the label entropy wrt the motor action where the label is the induced slice. Of course, in many cases a configuration model is infeasible. TBOT03 is also complicated by the goal depending on room. Relative entropy of motor actions between nearest and not nearest - configuration space is too large. Can compute the total motor relative entropy for each model for each goal as a comparison for the slice topology.So we should add this as a comment to the README, rather than spend much more time 'solving' the maze problem - we possibly could add orientation or odometry or dynamics, but we might still have wormholes or loops due to collision avoidance, jumping over doorways in one step, probabilistic motor actions, and so on. I think to navigate using a slice topology with these motor actions would require wide doorways and plenty of paths.

TBOT02 does not know the grammar of room navigation. We of course have high level models of paths, obstructions, doorways, escalators, etc. We look around to get our bearings when we walk into an unfamiliar room. TBOT02 has none of this. It is trapped in the present instant and so needs a large history and model for every location in the house.

TBOT03 we are no longer interested in model likelihood, but assume that the inducer is always reasonably good. Interested in the temporal relationships between slices  or components of the model.

TBOT03 short term is underlying frames, medium term is active history, long term is the model and the motor/label history - what about reflexive?

In the TBOT02 discussion we said that we can think of each *slice* as a set of *events* that corresponds roughly to a particular bounded and oriented area or region of the turtlebot house. Now, having, manually navigated in TBOT03's *slice* topology we can see that sometimes a *slice* corresponds to several different clusters of *events*. Each *event* has a coordinate in physical configuration space of `x`,`y` and `yaw`. Each cluster will have a  different mean coordinate and variance or deviation. If a *slice* has more than one cluster the overall mean won't correspond to any of the cluster means and the overall deviation will be greater than the deviation of any of the clusters.

These areas arrange themselves contiguously to cover the turtlebot's explorations of the house. So a slice topology is a sort of map of the pathways through the physical environment, formed by the past movements of the agent. 

The aim of TBOT03 is to debug the slice topologies created in TBOT02. 
The best topologies approximate closely to the physical configuration space and affordances, with diverse neighbourhoods and complete connectivity. In TBOT02 the topologies were far from optimal!

A disadvantage of a *slice* topology is that the *slices* are sometimes ambiguous with respect to goal label. The *events* of a *slice* may have occured in more than one physical area. Generally, however, the *slice* will consist of *events* that form clusters in which the turtlebot has similar x and y coordinate and orientation. The 'view' from the *slice* always looks the same, though, whichever cluster the turtlebot is in. The trick is to have only one cluster per *slice*. To do that, the turtlebot has to be able to distinguish between the clusters and that means a better resolution for the lidar scan, or landmarks so that the clusters have different views, or perhaps a compass to distinguish between orientations. For example, when the turtlebot is in the large central room looking 'north' at the doorway, it sometimes thinks it is looking 'south' at the other doorway, even though we can easily tell at a glance which doorway is which because of their different widths. If the turtlebot cannot distiguish exactly where it is within a room, or which way it is pointing, it clearly cannot make the decision to turn left or right or proceed ahead. So the key thing is to have a good map between the turtlebot's *model* and the physical configuration space.

In addition the action was poorly defined. Another disadvantage is that the *slices* are sometimes spread out and so the transitions are sensitive to the timing of the actions within them. We have fixed this problem so well that now the turtlebot can get stuck in loops when there are ambiguous *slices*. However, we can detect when it has gone around a loop and choose a different action to break out of it - somewhat as we do when we see that we have gone around in a circle in a maze.

The ironic thing about TBOT03 is that its slice topology is so complete and connected that it doesn't accidentally miss any wormholes or inconsistencies with physical space and reliably gets lost, ending up in loops! It is so good it is bad. TBOT02 hurtled along so quickly, almost out of control, that its slice topology was no more than a bias or tendency, so that it usually did better than random. Given that there will always be some ambiguity in the TBOT03 model, especially when starting in a new environment, we must add some loop avoidance, so that if it finds that it is back to where it started it does not repeat the past erroneous action, but tries some other action. I think we do the same thing when we are lost in a maze and we recognise that we have gone in a circle and then try a different turn.


<a name = "Conclusion"></a>

### Conclusion
