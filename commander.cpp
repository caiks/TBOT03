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

Commander::Commander(const std::string& room_initial, std::chrono::milliseconds command_interval, std::size_t room_seed, std::size_t running)
: Node("TBOT03_commander_node")
{
	srand(room_seed);
	
	_locations = vector<string>{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
	_room = _locations.size()-1;
	for (size_t i = 0; i < _locations.size()-1; i++)
		if (_locations[i] == room_initial)
			_room = i;
		
	_counts.push_back(0);
	_running = running;
			
	_pose_updated = false;
	_scan_updated = false;
	
	_goal_pub = this->create_publisher<std_msgs::msg::String>("goal", rclcpp::QoS(rclcpp::KeepLast(10)));

	_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Commander::scan_callback, this, std::placeholders::_1));
	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Commander::odom_callback, this, std::placeholders::_1));
		
	_command_timer = this->create_wall_timer(command_interval, std::bind(&Commander::command_callback, this));

	EVAL(room_initial);
	RCLCPP_INFO(this->get_logger(), "TBOT03 commander node has been initialised");
}

Commander::~Commander()
{
	RCLCPP_INFO(this->get_logger(), "TBOT03 commander node has been terminated");
}

void Commander::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	_record.sensor_pose[0] = msg->pose.pose.position.x;
	_record.sensor_pose[1] = msg->pose.pose.position.y;
	_record.sensor_pose[2] = msg->pose.pose.position.z;
	_record.sensor_pose[3] = msg->pose.pose.orientation.x;
	_record.sensor_pose[4] = msg->pose.pose.orientation.y;
	_record.sensor_pose[5] = msg->pose.pose.orientation.z;
	_record.sensor_pose[6] = msg->pose.pose.orientation.w;
	_pose_updated = true;		
}

void Commander::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	for (std::size_t i = 0; i < 360; i++)
	{
		if (std::isinf(msg->ranges.at(i)))
			_record.sensor_scan[i] = msg->range_max;
		else
			_record.sensor_scan[i] = msg->ranges.at(i);
	}
	_scan_updated = true;
}

void Commander::command_callback()
{
	if (_pose_updated && _scan_updated)
	{	
		auto xx = recordListsHistoryRepa_4(8, RecordList{ _record });
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
			_goal_pub->publish(msg);
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

