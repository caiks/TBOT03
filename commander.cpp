// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2-devel/turtlebot3_gazebo/src

#include "commander.h"

#include <stdlib.h>
#include <cmath>
#include <sstream>

using namespace Alignment;
using namespace TBOT03;
using namespace std;
using namespace std::chrono_literals;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Commander::Commander(const std::string& goal, std::chrono::milliseconds command_interval, std::size_t room_seed, std::size_t running)
: Node("TBOT03_commander_node")
{
	srand(room_seed);
	
	_goal = goal;
	
	_locations = vector<string>{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
	_room = _locations.size()-1;
	for (size_t i = 0; i < _locations.size()-1; i++)
		if (_locations[i] == goal)
			_room = i;
		
	_counts.push_back(0);
	_running = running;
				
	_publisherGoal = this->create_publisher<std_msgs::msg::String>("goal", rclcpp::QoS(rclcpp::KeepLast(10)));

	_subscriptionScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Commander::callbackScan, this, std::placeholders::_1));
	_subscriptionOdom = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Commander::callbackOdom, this, std::placeholders::_1));
		
	_timerCommand = this->create_wall_timer(command_interval, std::bind(&Commander::callbackCommand, this));

	EVAL(goal);
	RCLCPP_INFO(this->get_logger(), "TBOT03 commander node has been initialised");		
}

Commander::~Commander()
{
	RCLCPP_INFO(this->get_logger(), "TBOT03 commander node has been terminated");
}

void Commander::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	_pose[0] = msg->pose.pose.position.x;
	_pose[1] = msg->pose.pose.position.y;
	_pose[2] = msg->pose.pose.position.z;
	_pose[3] = msg->pose.pose.orientation.x;
	_pose[4] = msg->pose.pose.orientation.y;
	_pose[5] = msg->pose.pose.orientation.z;
	_pose[6] = msg->pose.pose.orientation.w;	
	_poseTimestamp = Clock::now();
}

void Commander::callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	for (std::size_t i = 0; i < _scan.size(); i++)
	{
		if (std::isinf(msg->ranges.at(i)))
			_scan[i] = msg->range_max;
		else
			_scan[i] = msg->ranges.at(i);
	}
	_scanTimestamp = Clock::now();
}

void Commander::callbackCommand()
{
	if (_goal.substr(0,4)!="room")
	{
		std_msgs::msg::String msg;
		msg.data = _goal;
		_publisherGoal->publish(msg);	
		rclcpp::shutdown();
	}
	else if (_poseTimestamp != TimePoint() && _scanTimestamp != TimePoint())
	{	
		auto xx = posesScansHistoryRepa(8, _pose, _scan);
		auto ur = std::move(std::get<1>(xx));
		auto hr = std::move(std::get<2>(xx));
		auto& vvi = ur->mapVarSize();
		auto rr = hr->arr;
		auto& mvv = hr->mapVarInt();
		auto p = mvv[vvi[Variable("location")]];	
		auto u = rr[p];
		if ((size_t)u == _room)
		{
			auto n = _counts.size();
			// EVAL(n);
			double mean = 0.0;
			for (auto a : _counts)
				mean += a;
			mean /= n;
			// EVAL(mean);
			double variance = 0.0;
			for (auto a : _counts)
				variance += ((double)a - mean)*((double)a - mean);
			variance /= n;
			// EVAL(sqrt(variance));		
			// EVAL(sqrt(variance/n));					
			auto running_n = std::min(n,_running);
			double running_mean = mean;
			double running_variance = variance;
			if (running_n < n)
			{
				running_mean = 0.0;
				for (std::size_t i = n - running_n; i < n; i++)
					running_mean += _counts[i];
				running_mean /= running_n;
				running_variance = 0.0;
				for (std::size_t i = n - running_n; i < n; i++)
					running_variance += ((double)_counts[i] - running_mean)*((double)_counts[i] - running_mean);
				running_variance /= running_n;
			}			
			_counts.push_back(0);
			while ((size_t)u == _room)
				_room = (rand() % 6) + 5; 
			// EVAL(_room);
			// EVAL(_locations[_room]);
			std_msgs::msg::String msg;
			msg.data = _locations[_room];
			_publisherGoal->publish(msg);
			std::ostringstream str; 
			str << "goal: " << msg.data 
				<< "\tn: " << n << "\tmean: " << mean << "\tstd dev: " << sqrt(variance) << "\tstd err: " << sqrt(variance/n);
			str << "\trunning mean: " << running_mean << "\trunning std dev: " << sqrt(running_variance) << "\trunning std err: " << sqrt(running_variance/running_n);
			
			RCLCPP_INFO(this->get_logger(), str.str());
		}
		_counts.back()++;
	}
}

int main(int argc, char** argv)
{
	std::string room_initial(argc >= 2 ? std::string(argv[1]) : "room1");
	std::chrono::milliseconds command_interval(argc >= 3 ? std::atol(argv[2]) : 250);
	std::size_t room_seed(argc >= 4 ? std::atoi(argv[3]) : 17);
	std::size_t running(argc >= 5 ? std::atoi(argv[4]) : 10);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Commander>(room_initial, command_interval, room_seed, running));
	rclcpp::shutdown();

	return 0;
}

