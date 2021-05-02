#ifndef COMMANDER_H
#define COMMANDER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

typedef std::pair<std::string, std::string> StringPair;
typedef std::vector<std::string> StringList;

typedef std::pair<double, double> Coord;

class Commander : public rclcpp::Node
{
public:
	Commander(const std::string&, std::chrono::milliseconds, std::size_t, std::size_t);
	~Commander();

private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _goal_pub;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::TimerBase::SharedPtr _command_timer;

	std::vector<std::string> _locations;
	std::size_t _room;
	std::vector<std::size_t> _counts;
	std::size_t _running;

	TBOT03::Record _record;
	bool _pose_updated;
	bool _scan_updated;

	void command_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // COMMANDER_H
