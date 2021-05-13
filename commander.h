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
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisherGoal;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscriptionScan;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriptionOdom;

	rclcpp::TimerBase::SharedPtr _timerCommand;

	std::vector<std::string> _locations;
	std::size_t _room;
	std::vector<std::size_t> _counts;
	std::size_t _running;

	std::array<double,7> _pose;
	TimePoint _poseTimestamp;
	std::array<double,360> _scan;
	TimePoint _scanTimestamp;

	void callbackCommand();
	void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // COMMANDER_H
