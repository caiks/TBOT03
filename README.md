# TBOT03 - TurtleBot3 slice topology controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we extend the developments in [TBOT01](https://github.com/caiks/TBOT01#readme) and  [TBOT02](https://github.com/caiks/TBOT02#readme) to examine *slice* topologies and motor actions.

The implementation of *slice* topologies in `TBOT02` only had a low action success rate. The best topologies approximate closely to the physical configuration space and affordances, with diverse neighbourhoods and complete connectivity. In `TBOT02` the topologies were far from optimal! In `TBOT03` we will seek to improve this performance by considering the factors that increase *model likelihood*, *slice* topology connectivity and completeness, and action responsiveness. To do this we will control the timings of scans and actions. This will allow us to manually control the turtlebot so that we can trace the action decisions that would have been taken in the various modes. In this way `TBOT03` will debug `TBOT02`.

As well as the `location` goal mode of `TBOT02`, we will consider two more modes in `TBOT03`, in order to investigate *slice* topologies. 

First, in 'random' mode the turtlebot acts to match the `motor` *histogram* for each *slice* to a desired uniform distribution. In this way the turtlebot will stochastically explore the physical configuration, so improving the *slice* topology's representation. 'Random' mode is used in the action *level*, which is defined as the *level* that acts on the motor *variables*.

Second, in 'interest' mode the turtlebot acts to move towards the *slice* with the greatest *size* per parent *slice size*. The turtlebot therefore will tend to spend more time in the *slices* with the highest potential for *modelling* so far undetected *alignments*, thus improving the *model likelihood* per active *history size*. This is similar to static *induction* in which the largest *slice* is *induced* first, but avoids large *slices* with *independent substrates*. 

'Interest' mode can be used at any *level*. In an action *level*, i.e. one that has motor *variables* in an *underlying substrate*, the active chooses actions that transition to the subset of neighbouring *slices* which have the fewest transitions to the goal *slice*. We will be examining only action *levels* for the moment.

In higher *levels* which are not also action *levels*, i.e. they do not have actions in an *underlying substrate*, but consist of a set of *underlying frames* of an action *level*, the goal *slice* of a higher *level* implies a set of goal *slices* in the *underlying* action *level*. So higher *levels* could operate in 'interest' mode indirectly.

In general, the interests of different actives may sometimes be contradictory, so some method of coordinating between them will be needed. In this investigation, however, we will only be examining the case where there is one action *level* active in 'interest' mode.

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
cat model090_2a* >model090_2.ac
cat model092_2a* >model092_2.ac

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
cat model090_2a* >model090_2.ac
cat model092_2a* >model092_2.ac

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

Ultimately any agent's model of the world is key to its ability to act in that world. So the map between the turtlebot's *slices* and the physical configuration must be at least as accurate as is required to accomplish its goals, whether they are imperative goals, such as navigating to a room, or cognitive/interest goals which aim to maximise the *model likelihood* given finite *history size*. In the case of `TBOT03` we will start with the same room goal as that of `TBOT01` and `TBOT02` before moving on to consider various 'interest' modes. 

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

Adding these 2 *models* to the earlier ones of `TBOT02` confirms that, although the `location` *entropy* decreased with *size*, these *induced models* are far from being able to to reduce it to zero -

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

By contrast, although `TBOT02` sometimes did find *slices* that spanned a small physical space, the *slices* were often blurry or ill-defined partly due to the smearing of the lidar data when in motion. Although the distortion of the scan while moving forward at a top speed of 30 cm per second is only a linear error of around +/- 6 cm compared to 50 cm per *event*, while rotating at 40 degrees per second there is an angular error of around +/- 8 degrees compared to 30 degrees per *event*. In `TBOT03` the turtlebot is stationary between actions. This is similar to the jerky motion of a pigeon's head as it walks. Another analogy would be a duck's motion if it were to paddle through syrup. The new update operation should improve the map between *slice* and configuration space.

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

We can demonstrate that this is the case in mode 9. In this mode, the choices that would have been made are calculated and traced but the control is now done manually. In this way we can debug the *slice* topology. A modification to the commander allows us to set the action -
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
In this sequence we begin 3 steps away from room 5 approaching from the right. All previous slices are ambiguous -
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

This is not always the case, though - this approach from the right has no luck -
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
In the `TBOT02` discussion we said that we can think of each *slice* as a set of *events* that corresponds roughly to a particular bounded and oriented area or region of the turtlebot house. Now we can see from the experiments above that the *slices* often have several configuration clusters. Each *event* has a coordinate in physical configuration space of `x`,`y` and `yaw`. Each cluster will have a  different mean coordinate and variance or deviation. If a *slice* has more than one cluster the overall mean will not necessarily correspond to any of the cluster means and the overall deviation will be greater than the deviation of any of the clusters. If the *model* had more *events* in the *slices* away from the walls and corners, however, then the *model* might resolve into single cluster *slices*, i.e. low deviation *slices*. 

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
Now we find that it spends a larger proportion of its time moving ahead,
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
Having determined that obstructions are not a major factor causing the high deviation, we will go on to consider whether the *model* itself might be the reason. In the default structure 1 configuration there are only 12 *underlying models* in *level* 1, so it may be the case that features or details with an angular resolution of less than 30 degrees might sometimes have no *alignment* at *level* 2. This might explain why the turtlebot does not always distinguish between narrow and wide doorways, especially at a distance. Inability to identify landmarks could certainly increase configuration deviation. So, in *model* 80, we increased the number of *underlying* to 36, i.e. the angular resolution is decreased to 10 degrees -

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

Let us continue to focus on the *model* by improving the *level* 1 *models*. In *model* 81 we increase the *level* 1 active *size*, increase the *induce* threshold and also increase the *induction* parameters `XMAX` and `WMAX`. We also alter the *level* 2 initial threshold and parameter `XMAX`,
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
Interesingly, we see an increase in `location` *entropy* and general configuration deviation, but a decrease in `location`-configuration deviation to 0.188369. We may conjecture that we have added some strong *alignments* within the rooms where the landmarks have been added, at the cost of configuration deviation relevant *alignments* between the rooms. Now we run the same *model* for longer to create *model* 84 -
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
After 929,471 *events* the `location`-configuration deviation has decreased further to 0.167629 -
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
Although we have seen an improvement, the configuration deviation is nowhere near what would be needed to obtain an average deviation of the scale that we see in *slices* that have only one cluster such as this one from *model* 77, seen when run in manual mode 9 (also shown above) -
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
Next, let us try adding a `direction` *variable* to the *substrate* (a magnetometer, in practice) to see if there exist *alignments* with it that are strong enough to disambiguate between clusters. In structure `struct003` the `direction` has a *valency* of 12. *Model* 85 is a copy of *model* 83 with structure `struct003` -
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
*Model* 87 is a small improvement over *model* 84 with a configuration deviation of 0.162262 instead of 0.167629. Of course, the improvement is far too small to justify the modification of the *substrate*. It does suggest, however, that a richer sensorium does improve the map between the *model* and the configuration and environment. This is what we would expect intuitively.

Now let us return to the original *substrate*, i.e. without `direction`, but instead add a third *level* defined in structure `struct002`. In this *level* we add two *models*, the first has *underlying frames* 0,1,2,3,4,6,8 and 10, the second has *underlying frames* 0,1,2,3 and 4, and *self frames* 5 and 10. *Model* 88 is otherwise a copy of *model* 83 -
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
As expected, *level* 2 *model* 89, with 536,347 *events*, is intermediate between *model* 83 (365,483 *events*) and *model* 84 (929,471 *events*).

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
They both turn out to be more ambiguous, although not much more, presumably because the zeroth *frame alignments* are less prominent than those of the middle *frames*. The second *model*, with two *reflexive frames*, has more *fuds* and so is a little more *likely* than the first *model*, which has only *underlying frames*. It also has a slightly better configuration deviation. Both *level* 3 *models*, however, have smaller mean *slice sizes* than the *level* 2 *model*, with only 5-6 *events* against 9-10 *events*. 

The mean *slice sizes* of the *level* 3 *models* are already small, so we cannot expect that crossing the *level* 2 *model* with a *level* 3 *model* will produce a very useful *slice* topology -
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
The `location`-configuration deviation is now very small, but that is obviously because the mean *slice size* is now only between one and two *events*. That suggests that the *models* of the different *levels* are rather orthogonal - so much so, in fact, that crossing gives us little benefit. In other words, crossing the *levels* causes the map of *slice* to configuration to be far too over-fitted.

#### Effective mode 13

We have attempted to reduce the configuration deviation in various ways. We did this in the hope that the turtlebot's *model* would map to the physical location closely enough that the turtlebot would reliably navigate it's environment. It is clear by now, however, that future efforts are likely to result in only marginal gains, because the largest *alignments* in the sensor *substrate* do not appear to capture the information that we need. So, instead of focussing on our interests, we will aim now to focus on the turtlebot's interests. That is, we will program the turtlebot to actively seek the highest nearby potential *alignments* and so increase the rate of growth of its *model*. In this way we aim to maximise the *model likelihood* that can be yielded from the sensors given the environment and the compute resources. We will assume that the parameterisation of the *induction* is reasonably good and consider only the physical search algorithm. To demonstrate that we have succeeded, we will demonstrate that the interest search mode produces a larger *model* per *history size* than a purely random search mode.

To make these modes reasonably comparable we will need similar *slice* topologies in both cases, at least for similar *model* sizes. To do this we must first create a random search mode that ensures that all possible actions have been tried by the turtlebot for its current *slice*. In that way the whole immediate neighbourhood will be accessible and the topology will maximise its connectivity or completeness in both cases.

Mode 13 first determines the past set of actions in the *events* of the current *slice*. If every possible action has occurred in the *slice*, all of the actions said to be effective and the next action is taken randomly from the given distribution. If there are any ineffective actions, i.e. those that have not been taken before in this *slice*, the effective actions are removed from the distribution. The action is then selected at random from the normalised remainder of the distribution. In this way, the *slices* are made effective with respect to actions as quickly as possible.

*Model* 92 is a copy of *model* 83, which has the new landmarks, in mode 13 effective random -

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor model092.json

{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"model" : "model092",
	"level1Count" : 36,
	"mode" : "mode013",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"turn_bias_factor" : 10,
	"logging_update" : false,
	"logging_action" : false,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : false,
	"summary_level2" : false,
	"logging_mode013" : true,
	"logging_mode013_factor" : 1000	
}
```

```
cd ~/turtlebot3_ws/src/TBOT03_ws
./main location_entropy model092_2
model: model092_2
model092_2      load    file name: model092_2.ac        time 0.451896s
activeA.historyOverflow: false
sizeA: 386379
activeA.decomp->fuds.size(): 4135
activeA.decomp->fudRepasSize: 65611
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07019
entropyA: 388707
entropyA/sizeA: 1.00603
(double)sizeA * std::log(sizeA): 4.9706e+06
std::log(sizeA): 12.8646

./main configuration_deviation_all model092 
active: model092
model092_2      load    file name: model092_2.ac        time 0.378641s
activeA.historyOverflow: false
sizeA: 386379
activeA.decomp->fuds.size(): 4135
activeA.decomp->fudRepasSize: 65611
(double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA: 1.07019
records->size(): 386379
size: 386379
slice_count: 13676
slice_size_mean: 28.2523
deviation: 0.422107
size: 386379
slice_location_count: 41275
slice_location_size_mean: 9.36109
deviation_location: 0.178327
```
*Model* 92 is a slightly less *likely model* than *model* 83, but it has lower `location` entropy and configuration deviation. It appears that the effective action slightly improves the map to the environment without affecting the *likelihood* of the *model* significantly.

#### Interest modes 14-16

Interest modes 14-16 behave in the same way as random effective mode 13, except in the case where the actions of the current *slice* are all effective, i.e. all have been tried at least once. In this case mode 13 simply chooses another action randomly according to the given distribution, but the interest modes 14-16 first look to see if there is a decidable action that can be chosen instead. To determine if a *slice* is decidable, the actor first looks at the forward *slice* transitions from its current *slice* to find its immediate neighbourhood *slice* set. It then repeats the search to find the neighbours of the immediate neighbourhood. That is, to find the set of *slices* connected by two *slice* transitions from the current *slice*. The actor recurses in the *slice* topology until some limit is reached, such as the total number of connected *slices*, or the number of *slice* transitions. Once it has found the set of forward connected *slices*, it then determines the connected *slice* which maximises some measure of interest. The actor then works back from this goal *slice*, recursively reversing the *slice* transitions until it finds the subset of the actor's immediate neighbourhood, i.e. those *slices* only one transition away from the current *slice*. Each of this 'nearest' subset will have the same least number of transitions to the goal *slice*. If the nearest subset of the neighbourhood is a proper subset, i.e. it is smaller than the neighbourhood itself, then the current *slice* is defined as decidable and the distribution of actions that only lead to the nearest neighbourhood is used instead of the given effective distribution. When the turtlebot has transitioned to a new *slice* (whether it has successfully transitioned to one of the shortest path neighbours, or to one of its other known neighbours, or to a newly discovered neighbour), the whole process is repeated. That is, the actor chooses a new goal *slice* (which may be different from the previous goal *slice*) and then finds out whether its new current *slice* is decidable. In this way, the turtlebot will tend to take the shortest routes towards the goal *slices*. If goals are defined by interest, or potential *likelihood*, the turtlebot's *model* should grow more quickly than if the turtlebot acts randomly.

The differences between modes 14, 15 and 16 are differences in implementation. They all have similar functionality and the changes in implementation were made in response to the results of experimentation. In mode 14 the *slice* topology is notional and is searched by following the sequences of *events* that start at the *events* in the current *slice*. That is, the *slice* transitions are recomputed as needed. This is laborious and repetitive, and the performance decreases exponentially with transition count. In mode 15, the *slice* transitions are cached on the actor and so do not need to be recomputed for the current *slice*. This is considerably faster than mode 14, but it requires recomputing the entire *slice* transition cache whenever the *model* changes. In mode 16, the actor makes use of *slice* transition structures that are maintained by the active during update and induce. That is, the *slice* topology is cached on the active rather than the actor.

In addition to simplifying the actor implementation and improving the performance of the cache, the active *slice* topology can optionally be cumulative so that when the active *history* overflows the transitions of the rolled off *events* are retained. (Of course if there are no longer any *events* corresponding to a transition, the actor will not know which action it must take to go to a shortest route neighbour, but at least the actor knows that there exists a route to the interest goal and so can modify its behaviour accordingly, e.g. by a systematic search for a path, or by looking in sibling *slices* for *events*. When we ride a bicycle for the first time in years, it requires a few wobbles, if not crashes, before we regain our former proficiency.)

The three interest modes are all able to run as though they are in random effective mode 13 by means of a `random_override` flag set in the configuration. They still calculate the goal *slices* and the statistics, but ignore the decidable action distribution if it exists.

To begin with, in mode 14, the measure of interest, which determines the goal *slice*, is merely the *size* of the *slice*. The idea is that larger *slices* are more probably *on-diagonal*, and hence have greater potential for future *alignments*. In this way we hope to increase the rate of *model* growth. Later on in mode 15, as we shall see, a better measure of interest is used.

Much of the first work in mode 14 was to debug the frequent crashing. Various changes were made to such parameters as (a) the simulation frames per second, (b) the collision field of view, (c) a rectangular or wedge field of view, (d) the bias mechanism when blocked, and (e) the goal search limits such as the maximum number of open *slices*. Eventually a degree of stability was obtained. 

In this run, the mode 14 was first set to use random effective mode -

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT03_ws/gazebo_models
cd ~/turtlebot3_ws/src/TBOT03_ws
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT03_ws/env017.model -s libgazebo_ros_init.so

```
```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"level1Count" : 36,
	"mode" : "mode014",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"collision_rectangular" : false,
	"turn_bias_factor" : 10,
	"open_slices_maximum" : 100,
	"goal_size_maximum" : 99,
	"random_max_slice" : true,
	"bias_if_blocked" : true,
	"random_override" : true,
	"logging_update" : false,
	"logging_action" : false,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : false,
	"summary_level2" : false,
	"logging_mode" : true,
	"logging_mode_factor" : 100,
	"tracing_mode" : false,
	"logging_hit" : true
}
```
Now in mode 14 proper -

```
cd ~/turtlebot3_ws/src/TBOT03_ws
ros2 run TBOT03 actor actor.json

{
	"update_interval" : 1,
	"linear_maximum" : 0.45,
	"angular_maximum_lag" : 6.0,
	"act_interval" : 1,
	"structure" : "struct001",
	"level1Count" : 36,
	"mode" : "mode014",
	"distribution_AHEAD" : 10.0,
	"collision_range" : 0.85,
	"collision_field_of_view" : 20,
	"collision_rectangular" : false,
	"turn_bias_factor" : 10,
	"open_slices_maximum" : 100,
	"goal_size_maximum" : 99,
	"random_max_slice" : true,
	"bias_if_blocked" : true,
	"random_override" : false,
	"logging_update" : false,
	"logging_action" : false,
	"logging_action_factor" : 100,
	"logging_level1" : false,
	"logging_level2" : false,
	"summary_level1" : false,
	"summary_level2" : false,
	"logging_mode" : true,
	"logging_mode_factor" : 100,
	"tracing_mode" : false,
	"logging_hit" : true
}
```
The turtlebot logs various statistics at regular intervals of *events*. We can compare the outputs of the two runs after 52,700 *events*,

```
model_2     ev: 52700       fuds: 535       fuds/sz/thrshld: 1.01516        eff: 96.26      dec: 46.19      succ: 6.08      expt: 5.41      null: 50.61	marg: 1.37       live: 402       goals: 1415     hits: 1013      len: 262.52     sz: 56.16       par: 648.90     pos: 0.617875   neg: -0.326120

model_2     ev: 52700       fuds: 544       fuds/sz/thrshld: 1.03224        eff: 96.00      dec: 61.84      succ: 12.36     expt: 6.29      null: 43.31	marg: 10.71      live: 388       goals: 2020     hits: 1632      len: 165.82     sz: 55.95       par: 534.67     pos: 0.593775   neg: -0.307027
```

In tabular form this looks like -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|margin|live|goals|hits|hit length|slice size|parent size|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|model_2|52700|535|1.01516|96.26|46.19|6.08|5.41|50.61|1.37|402|1415|1013|262.52|56.16|648.90|0.617875|-0.326120
interest|model_2|52700|544|1.03224|96.00|61.84|12.36|6.29|43.31|10.71|388|2020|1632|165.82|55.95|534.67|0.593775|-0.307027

The less self-explanatory columns are as follows:

The 'effective' is the percentage of *slices* that have at least one *event* for each possible action (turn left, turn right or move ahead). The 'decidable' is the percentage of *slices* for which the turtlebot could act differently from the default action distribution to better obtain its goals. 

The 'null' is the proportion, as a percentage, of *slice* transitions which are to a previously unknown neighbour. The 'successful' is the proportion of *slice* transitions which are to decidable shortest route neighbours. The 'expected' is the average of the proportion of neighbours which are shortest route neighbours, for non-null *slice* transitions. That is, the 'expected' is the success rate that might be expected by chance. The 'margin' is the successful less the expected given non-null. So 'margin' is the measure of how well the turtlebot's actions obtain a transition to a shortest route neighbour.

The 'goals' is the number of goal *slices* ever set. The 'live' is the current number of goal *slices* that have not been hit. The 'hits' is the current number of goal *slices* that have been hit. 'Hit length' is the count of *events* between a goal *slice* being set and then hit. 

The '*slice size*' column is the average *size* of the current *slice*. 'Parent *size*' is the same for the parent of the current *slice*. 

Columns '+ve likelihood' and '-ve likelihood' depend on the following *likelihood* measure: `(ln(slice_size) - ln(parent_size) + ln(WMAX)) / ln(WMAX)`, where `WMAX` is the maximum *derived volume*. By this measure *on-diagonal* or frequent child *slices*, i.e. those which are a large proportion of their parent's *size*, tend to one. Far *off-diagonal* or rare child *slices*, i.e. with very few *events*, are less than zero. Uninteresting child *slices* have a *likelihood* measure of around zero, i.e. where the ratio of parent *size* to child *size* is approximately `WMAX`, which is the case the when the *histogram* is *independent* or weakly *aligned*. So interesting child *slices* on this measure are either extremely positive or extremely negative. The positive column shows the average positive, and the negative column shows the average negative. For most of the runs below we will be looking at the positive extreme as our measure for interesting goal, e.g. by maximising the goal *slice size* or the fraction of its parent's *size*. The idea of using this measure of *likelihood* is that highly *aligned* descendent *histograms* tend to have highly *aligned* ancestor *histograms*, and so are potential sources of new *likelihood* in further descendents. Note that an alternative measure might be to use the recalculated *derived alignment* of the parent *slice*, but that would treat all its children *slices* as equally interesting. The alternative method, however, would avoid the phenomenon of unusual child *slices* initially becoming uninteresting as attention is turned towards them and their *count* increases.

Clearly in this run of mode 14 the turtlebot is behaving differently between random effective mode and interest mode. We can see that the interest mode is more decidable at 61.84% versus 46.19%, while being similarly effective. That means the turtlebot is more often in *slices* where there is a preferred direction to the nearest goal. Once the turtlebot begins to follow a path to a goal, it tends to stay on or near the path. Each *slice* on the path is also more likely to be decidable, so overall interest mode tends to have higher decidability than random effective mode.

Also, we can see that the interest mode marginal success rate is higher at 10.71% against 1.37%. That is, the turtlebot succeeds in reaching its desired shortest route neighbours at least 9% more often than chance. 

We can see that the *model* is a little larger in interest mode with the *fuds* per *size* per threshold at 1.03224 versus 1.01516. The hit length is much shorter 165.82 versus 262.52. The fraction of hits per goal is higher at 80% versus 71%. Although a margin of around 10% is not a very high action success rate, the continuous pressure does appear to have an effect. At this stage, however, there is not enough evidence to conclude that interest mode is definitely increasing *model* growth.
 
If we run for longer we often find that the effect has disappeared, for example -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|model_2|398500|4276|1.07302|93.39|49.22|2.12|2.16|52.61|-0.08|118|259|141|2150.52|57.61|368.25|0.634770|-0.258065
interest|model_2|398500|4274|1.07252|92.63|56.62|5.02|1.95|57.50|7.22|119|289|170|2101.32|55.95|379.68|0.621137|-0.250982

Now at 398,500 *events* the *model* size, at 4,274 versus 4,276, is very similar, and the hit length is similar, although the hits is a little higher. The margin difference of 7.3% is not much less than at 52,700 *events*, so we can guess that the random mode has caught up with the interest mode, rather than interest mode falling behind.

We then added two more *likelihood* measure statistics - the 'average *likelihood*', which is not split into positive or negative, and the 'hit *likelihood*', which shows the average *likelihood* for the goal *slices* as they are hit.

As we continued testing we found that the interest runs are highly variable, e.g. if we compare at 32,600 *events* in these runs,

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|model_2|32600|332|1.01837|96.29|45.24|7.65|6.75|46.83|1.70|254|942|688|159.94|56.09|738.17|||0.618442|-0.345272
interest|model_2|32600|340|1.04291|96.10|60.33|14.30|7.65|39.55|10.99|243|1354|1111|98.23|56.17|624.86|||0.595199|-0.327428
interest|model_2|32600|311|0.953958|95.99|63.88|13.08|7.56|41.31|9.40|213|1329|1116|113.58|54.42|655.84|0.351294|0.632307|0.578092|-0.329129
interest|model_2|32600|339|1.03985|96.20|57.56|12.72|7.91|38.65|7.85|199|1272|1073|117.97|56.26|562.06|0.387543|0.627318|0.581911|-0.326883

For example, the second interest run in the table has a considerably smaller *model* at 311 *fuds* than the random effective run at 332 *fuds* and the first interest run at 340 *fuds*. This seems odd when we examine the other statistics - the larger margin, shorter hit length and higher hits compared to random effective. The hit *likelihood* at 0.632307 is much higher than the average *likelihood* of 0.351294. The third interest run, by contrast, has a *model* at 339 *fuds* that is nearly as large as the first interest run.

We also experimented with the two *level* 3 actives in mode 14, but found that the results were even less conclusive than for *level* 2. 

After some investigation we found that when the turtlebot is in interest mode it moves along the corridor between rooms 1,2,3 and 4,5,6 much less often than for random effective mode. Sometimes it remains trapped in one set of rooms for the entire run. We can conjecture that requiring no more than one *event* in each action, i.e. effective *slice*, is sometimes insufficient for the turtlebot to discover anything interesting along the corridor. This means that the *models* has not explored enough and so are rather incomplete. In order to make the modes comparable we eventually restricted the turtlebot to rooms 4,5 and 6 by blocking the corridor in environment 19. We did this from mode 15 onwards. Ultimately, of course, a greater weight will have to be placed on random exploration, but for the moment this restriction will do.

At this point we moved on from mode 14 to mode 15. Here the *slice* topology is cached on the actor, improving performance and well as enabling greater functionality. Now we could more easily keep track of the parent *slice size* and so we could move from a goal of maximum *slice size* to one of maximum *slice size* per parent *slice size*. With maximum *slice size* we saw large gryrations in the *modelling* rate, although possibly this was also because of the corridor problem mentioned above. Initially we picked off the highest *alignments* but later we ended up accidentally boosting *slices* with low *alignments* because these older *slices* tended to be larger than newer *slices*, simply because of the accumulation of *events*. Also, because of the induce threshold, there is sometimes more than one goal *slice* to choose from. If one of these goals is chosen arbitrarily at each *event*, then the turtlebot can start off in one direction and then change its mind to go in another direction and so waste time oscillating in lower *alignments*. If we move to maximum *slice size* per parent *slice size* instead, we are maximising the *likelihood* measure described above. That is, we will tend to find the most *diagonalised* or *aligned* goal *slices* more quickly, and so improve our chances of finding potential new *alignments*. 

During mode 14 and 15 *level* 3 runs we observed that sometimes the *modelling* rate, measured in *fuds* per *size* per induce threshold, would be abnormally large. After some investigation we found that the induce appeared to be recursing down a single *decomposition* path creating a *model* that was very deep but highly lopsided with many near *singletons*. We conjectured that perhaps the induce threshold of only 100 was producing quite a lot of spurious *shuffle alignment* with the result that the later parts of the *model* consisted of essentially random *partitions*. We traced the *alignments* that were obtained during the induce and found that there were quite a lot of very small *alignments*. The distribution was 'U'-shaped with a minimum at around a 6% *diagonal*. We assumed that below that minimum most of the *alignments* were caused by the coarse *shuffle*. So we added a *diagonal* threshold to the active so that it does not add *model* below that threshold, but will wait and retry later. To prevent constant retries at incremental *events* we also added a stepped sequence of thresholds on fail, e.g. this is `actor.json` with the additional configuration,
```
{
...
	"induceParametersLevel1.diagonalMin" : 6.0,
	"induceParametersLevel1.induceThresholds" : [110,120,150,180,200,300,400,500,800,1000],
	"induceParameters.diagonalMin" : 6.0,
	"induceParameters.induceThresholds" : [110,120,150,180,200,300,400,500,800,1000],
...
}
```
With the minimum *diagonal* threshold set to 6% and the turtlebot limited to rooms 4, 5 and 6, we can compare some mode 15 runs at 36,000 *events* -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|	actor_2|36,000|360|0.999972|96.26|46.41|8.03|7.79|46.04|0.45|19|437|418|184.60|56.66|680.04|0.380548|0.862271|0.615651|-0.323553
random|	actor_2|36,000|358|0.994417|96.19|46.69|8.83|8.32|45.24|0.93|11|442|431|155.04|55.48|549.81|0.390565|0.878057|0.615325|-0.286318
random|	actor_2|36,000|358|0.994417|96.21|46.66|8.37|7.96|46.28|0.76|13|397|384|195.52|55.80|528.34|0.390294|0.873051|0.608259|-0.312380
likely|	actor_2|36,000|401|1.11386|95.89|70.19|14.24|8.21|33.40|9.06|17|555|538|140.16|56.60|579.13|0.383769|0.897542|0.592533|-0.325611
likely|	actor_2|36,000|372|1.0333|96.11|60.03|14.69|8.67|36.41|9.46|27|618|591|112.68|56.55|579.68|0.386420|0.892925|0.596425|-0.325404
likely|	actor_2|36,000|374|1.03886|96.01|59.88|13.52|7.94|37.93|8.99|20|677|657|117.17|56.10|615.06|0.384956|0.896843|0.594560|-0.332610

These three runs in random effective mode and then interest mode generally seem to confirm that maximising goal *slice size* per parent *slice size* does slightly increase the *model* growth rate. We can see that the average *likelihood* and hit *likelihood* are fairly constant in both modes, with hit *likelihood* considerably higher at around 0.89 versus around 0.38. In interest mode, however, the marginal success rate of around 9% is producing more decidable *slices*, higher hits, lower hit length and hence higher *fuds* per *size* per threshold.

The effect continues until 69,000 *events* at least -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|69,000|695|1.00723|96.34|48.19|7.99|7.42|47.96|1.09|17|660|643|308.09|56.09|481.49|0.410795|0.889171|0.617150|-0.275622
random|actor_2|69,000|713|1.03332|96.29|47.74|7.62|7.24|48.76|0.73|22|595|573|347.46|56.36|480.07|0.412921|0.884487|0.617248|-0.295943
likely|actor_2|69,000|751|1.08839|95.99|61.69|13.28|7.86|39.51|8.97|37|830|793|223.59|57.13|561.76|0.410579|0.901542|0.602529|-0.312157
likely|actor_2|69,000|737|1.0681|95.82|62.57|12.30|7.25|40.49|8.49|30|881|851|244.61|56.31|554.80|0.408571|0.904610|0.598199|-0.312692

But from 100,000 *events* the random and *likely* runs are beginning to converge -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|100,000|1063|1.06299|96.34|48.66|6.87|6.49|49.74|0.77|29|790|761|522.39|56.67|465.68|0.422140|0.894239|0.621712|-0.286731
likely|actor_2|100,000|1070|1.06999|95.86|65.68|11.92|6.85|41.57|8.68|36|1102|1066|374.30|56.48|532.94|0.413223|0.907592|0.598660|-0.301512

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|155,000|1663|1.0729|96.37|49.28|6.16|5.76|50.26|0.80|36|1170|1134|839.99|56.96|445.41|0.428965|0.902739|0.628176|-0.278908
likely|actor_2|155,000|1679|1.08322|96.07|64.63|11.28|6.49|42.28|8.30|54|1459|1405|738.22|57.08|467.72|0.427589|0.906392|0.602969|-0.280159
likely|actor_2|155,000|1651|1.06515|95.90|65.93|10.71|6.10|43.61|8.17|64|1505|1441|706.97|56.93|490.78|0.420760|0.910780|0.601186|-0.289404

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|232,000|2486|1.07155|96.45|49.85|5.62|5.36|49.88|0.52|43|1644|1601|1276.38|57.34|445.15|0.434260|0.907416|0.632389|-0.275920
likely|actor_2|232,000|2498|1.07672|95.99|67.49|9.87|5.55|44.32|7.77|72|2025|1953|1240.00|57.31|490.07|0.428419|0.914166|0.606430|-0.282565

It appears that random effective search is not much less efficient at *model* discovery than interest search over time because the motor action *volume* is small and so the *slice* topologies in both cases are as complete and connected as the sensor *alignments* permit.

Having observed a small difference in growth rates, it is interesting to note at this point that random and interesting modes look qualitatively different. In random mode, the turtlebot tends to run in a straight line until it is obstructed, with occasional turns to the left or right. This accords with the action distribution where a move ahead is 10 times more probable than either turn. The behaviour in *likely* mode appears quite different. In this mode, the turtlebot does one or two steps ahead and then oscillates, turning left and right repeatedly, for a few actions. This is probably because the uncertainty of the turtlebot's position causes wormholes for interest goals as well as for room goals. In cases where there is little configuration deviation we would expect there to be less vacillation.

As mentioned above, mode 16 places the cached *slice* topology on the active. In addition, the topology information can be cumulative, retaining the transitions after the *history* overflows and *events* roll off. The mode 16 runs confirm the findings in mode 15, as we would expect -

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|36,000|360|0.999972|96.26|46.41|8.03|7.79|46.04|0.45|19|437|418|184.60|56.66|680.04|0.380548|0.862271|0.615651|-0.323553
random|actor_2|36,000|358|0.994417|96.19|46.69|8.83|8.32|45.24|0.93|11|442|431|155.04|55.48|549.81|0.390565|0.878057|0.615325|-0.286318
random|actor_2|36,000|358|0.994417|96.21|46.66|8.37|7.96|46.28|0.76|13|397|384|195.52|55.80|528.34|0.390294|0.873051|0.608259|-0.312380
random|model100_2|36,000|371|1.03053|96.23|46.22|11.44|10.77|46.30|1.25|11|450|439|168.88|55.92|588.18|0.382392|0.887521|0.611288|-0.304849
random|model102_2|36,000|362|1.00553|96.24|46.20|10.50|9.94|47.65|1.07|18|417|399|182.57|55.44|469.41|0.401853|0.888562|0.607212|-0.284820
likely|actor_2|36,000|401|1.11386|95.89|70.19|14.24|8.21|33.40|9.06|17|555|538|140.16|56.60|579.13|0.383769|0.897542|0.592533|-0.325611
likely|actor_2|36,000|372|1.0333|96.11|60.03|14.69|8.67|36.41|9.46|27|618|591|112.68|56.55|579.68|0.386420|0.892925|0.596425|-0.325404
likely|actor_2|36,000|374|1.03886|96.01|59.88|13.52|7.94|37.93|8.99|20|677|657|117.17|56.10|615.06|0.384956|0.896843|0.594560|-0.332610
likely|model101_2|36,000|372|1.0333|95.99|61.81|16.16|10.42|38.41|9.31|17|681|664|138.50|55.65|552.00|0.378556|0.890606|0.585895|-0.296460
likely|model103_2|36,000|361|1.00275|96.09|58.05|15.56|10.80|39.31|7.85|23|641|618|143.81|55.41|583.36|0.374962|0.900021|0.591339|-0.331747

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|69,000|695|1.00723|96.34|48.19|7.99|7.42|47.96|1.09|17|660|643|308.09|56.09|481.49|0.410795|0.889171|0.617150|-0.275622
random|actor_2|69,000|713|1.03332|96.29|47.74|7.62|7.24|48.76|0.73|22|595|573|347.46|56.36|480.07|0.412921|0.884487|0.617248|-0.295943
random|model100_2|69,000|716|1.03767|96.24|48.27|10.50|9.91|49.55|1.17|15|595|580|279.56|56.23|523.63|0.408577|0.898077|0.618627|-0.284617
random|model102_2|69,000|696|1.00868|96.31|47.87|10.17|9.82|50.33|0.71|19|546|527|301.03|55.81|481.50|0.411925|0.898892|0.611391|-0.274062
likely|actor_2|69,000|751|1.08839|95.99|61.69|13.28|7.86|39.51|8.97|37|830|793|223.59|57.13|561.76|0.410579|0.901542|0.602529|-0.312157
likely|actor_2|69,000|737|1.0681|95.82|62.57|12.30|7.25|40.49|8.49|30|881|851|244.61|56.31|554.80|0.408571|0.904610|0.598199|-0.312692
likely|model101_2|69,000|712|1.03187|95.93|62.15|15.07|10.31|42.35|8.25|23|856|833|223.28|56.24|524.30|0.402010|0.895195|0.591345|-0.290725
likely|model103_2|69,000|711|1.03042|96.06|61.95|14.98|10.35|42.39|8.02|35|886|851|202.73|55.94|552.03|0.392625|0.906884|0.591122|-0.312349

type|model|events|fuds|fuds/sz/thrshld|effective|decidable|successful|expected|null|marg|live|goals|hits|hit length|slice size|parent size|likelihood|hit likelihood|+ve likelihood|-ve likelihood
---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---
random|actor_2|100,000|1063|1.06299|96.34|48.66|6.87|6.49|49.74|0.77|29|790|761|522.39|56.67|465.68|0.422140|0.894239|0.621712|-0.286731
random|model100_2|100,000|1066|1.06599|96.26|48.95|10.14|9.63|50.77|1.04|19|697|678|387.83|56.60|481.98|0.419998|0.902292|0.621421|-0.276846
random|model102_2|100,000|1014|1.01399|96.37|48.41|9.89|9.73|51.42|0.32|28|649|621|418.98|56.10|448.83|0.418957|0.904255|0.615044|-0.267376
likely|actor_2|100,000|1070|1.06999|95.86|65.68|11.92|6.85|41.57|8.68|36|1102|1066|374.30|56.48|532.94|0.413223|0.907592|0.598660|-0.301512
likely|model101_2|100,000|1043|1.04299|95.89|65.92|14.27|9.76|43.70|8.01|28|1010|982|309.18|56.63|504.96|0.414278|0.899931|0.596064|-0.283803
likely|model103_2|100,000|1059|1.05899|96.06|62.93|14.52|10.20|43.86|7.70|38|1007|969|304.78|56.49|518.10|0.407378|0.909914|0.595356|-0.306074


<!--

Usually the likely performs better for fuds/sz/thrshld than the random, but not always. Presumably it depends on chance to some extent in a well explored space. The decidability, hits, hit len, margin likely is consistently better, however. So there is good evidence that it works well README.

Note that the prev transitions count is lower for likely, as expected, because the choice of transition is deliberate. So, counterintuitively, the slice topology is actually smaller with likely goal, although the model is larger.

restricted history size

restricted active size enhances likely over random, so we have conclusive evidence of the practicability of agent modelling or the play principle that models can be optimised by actively searching for likelihood

Conversely for rare choose most off-diagonal ie min size/parent-size. If cannot find any of sufficient fraction in interest mode, flip to rare mode, until cannot find any with small enough fractions and then flip back. lopsided models with low alignment might have high slice/parent fraction in the large slice. Really should record the parent alignment or implied diagonal in the decomp and use that for any of its child slices - although I suppose that we are really only interested in the largest slices. Probably lopsided models are wasteful but the slice/parent fraction will still tend to accelerate model induction

unusual slices - likely but rare. Consider other modes such as repelled by unaligned



we only have a few actions so random effective does explore pretty well and we cannot expect much advantage from likelihood selection

note on why the expected is not zero

check to see if remaining actions are effective

if any ineffective choose randomly from the ineffective, setting the turn bias if blocked

else if goal slice choose action by neighbourhood or turn by neighbourhood if blocked

else choose randomly or by turn bias if blocked

if no least neighbours, choose random, ignoring ahead if blocked, case where goal is neighbour, case of one or no neighbours, choose nearest of same max size slice goals


Improved margin rate over TBOT02 - varies from room to room, depending on wormholes.

TBOT03 short term is underlying frames, medium term is active history, long term is the model and the motor/label history - what about reflexive?

The fact that this advantage of interest mode over random is not very large partly explains why it has been so difficult to prove conclusively that the interest goal can increase the rate of *model* growth. 

it may be the case that we will not be able to demonstrate likelihood goal conclusively because of poor configuration mapping, low action success, medium decidability and propensity to become stuck on one side of the corridor.

TBOT03 evidence - behaviour looks different from random, margins, nulls, hit length, decidability, parent? auto turning at walls is another reason why the margins are low

-->



<a name = "Conclusion"></a>

### Conclusion

In `TBOT02` we successfully implemented active *modelling*, and found that it was no worse than static *induced modelling*, although not as good as static *conditional modelling*. We experimented with a *slice* topology rather than requiring long continuously experienced sequences of *events* to make decisions, but found that the resultant configuration deviation was too poor to avoid the wormholes and ambiguities that plague the paths, often resulting in looping behaviour and performances worse than random. 

There were several explanations that were considered. First capture of sense data was often taken while the turtlebot was in motion. In addition the action was poorly defined. Another disadvantage is that the *slices* are sometimes spread out and so the transitions were sensitive to the timing of the actions within them. 

In `TBOT03` the actor was rewritten to address these issues. This helped us to make more successful action decisions in interest mode, and trace the turtlebot's behaviour, but this wasn't sufficient to improve the configuration deviation enough to navigate from room to room.

This is because the *alignments* are sometimes such that the *events* of a *slice* may have occured in more than one physical area. The *events* may even be spread out over a very large area. In general, however, the *slice* will tend to consist of *events* that form one or more clusters in which the turtlebot has similar x and y coordinate and orientation. The 'view' from the *slice* always looks the same to the turtlebot, though, whichever cluster the turtlebot is in. For example, when the turtlebot is in the large central room looking 'north' at the doorway, it sometimes thinks it is looking 'south' at the other doorway, even though we can easily tell at a glance which doorway is which because of their different widths. The trick is to have only one cluster per *slice*. If the turtlebot cannot distinguish exactly where it is within a room, or which way it is pointing, it clearly cannot make the decision to turn left or right or proceed ahead. In order to have a good map between the turtlebot's *model* and the physical configuration space, the turtlebot has to be able to distinguish between the clusters. To do this we tried a better resolution for the lidar scan, adding landmarks so that the clusters have different views, adding a compass to distinguish between orientations, and considering the dynmaic *level* 3 *model*.

None of these hints to the turtlebot were sufficient to reduce the configuration deviation to a low enough level. This is not very surprising, however, if we do not make unfair anthropomorphic comparisons. `TBOT03` does not know the general grammar of room navigation. We human beings with our episodic memories have high level models of paths, obstructions, doorways, escalators, etc. We look around to get our bearings when we walk into an unfamiliar room. `TBOT03` has none of this, and it appears that it so happens that the largest *alignments* are not those that help it distinguish between similar views in different locations.

The ironic thing about `TBOT03` is that its *slice* topology is now so complete and connected that it doesn't accidentally miss any wormholes or inconsistencies with physical space and so it reliably becomes lost, ending up in loops! `TBOT02` hurtled along so quickly, almost out of control, that its *slice* topology was no more than a bias or tendency, so that it usually did slighty better than random though its very inefficiency. Given that the *alignments* are such that there will always be some ambiguity in the `TBOT03` *model*, we abandoned the room navigation goal and aimed instead to explore the *slice* topology and whether it could help the turtlebot learn more quickly. 


It turned out, after (a) restricting the min diagonal, (b) using only rooms 4,5,6 to improve the reliability of the signal, (c) calculating interest *likelihood* by comparing the *slice* size to its parent, and (d) caching the topology on the active, that there was a marginal action success and other statistics (decidability, hit length for comparable *fud* counts, total hits, hit *likelihood*) that in general demonstrated an advantage of interest goal over random goal in the rate of *fud* increase in the *level* 2 active, although this difference disappears over time. The random happens to be a fairly effective form of exploration, and the interest mode decision action success rate is so low (only ~10%), that the advantage is fleeting and often not visible at all. However, if we restricted the active *history size*, we found that interest mode had a substantial advantage over random, although, of course, it could not compensate for the missing *events* with such a low margin. This signal is very strong and so is definite proof that play works as a strategy. In practice it might be the case that we are not particularly restricted memory-wise (at around 300 bytes or 30 bytes compressed per *event*, it takes a long time to run out at 4 FPS and 30m seconds/annum), but the *volume* of the action *substrate* would be much bigger than it is for turtlebots and so random can only probe a little way into the future - so little, in fact, that it might be difficult to even attain an action success margin of 10%. We might need to evolve quite a tight parameterisation of the motor *variables* to obtain a critical success rate.

<!--

Future directions - 


the problem with TBOT03 is that random/effective is pretty good and slice map to physical configuration is pretty bad. In future try to ensure that the reverse is true, then the likelihood goal, whether gradient or topmost, should produce behaviour that we can intuit as intelligent, or at least cannot produce in any other way.


TBOT04 WOTBOT we really want a test case that has high margin success and goals that are less easy to hit by chance. Interaction with another agent might be a better choice than with a fixed environment. The goal of conversing cannot be hit by chance. ELIZA? or text interface. Want an inaccessible but fruitful set of alignments, eg walking, talking, playing. 

Principle of evolution by likelihood selection, or principle of most interest, or principle of sufficient interest. The big idea is that we act to maximise the model by moving to the largest slices. This cognitive goal acceleration of modelling mirrors the sexual swapping of genes which accelerates natural selection. That is, we can speed up the modelling rate in the case of CAIKS, and the rate of complex niche finding in the case of evolution. In CAIKS the technique resembles the golf ball method of iterative corrections.

dynamic frames



-->


