#ifndef ACTOR_H
#define ACTOR_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

class Actor : public rclcpp::Node
{
public:
	Actor(const std::string&);
	~Actor();
	
	bool _terminate;

	TBOT03::Record _record;
	bool _updateLogging;
	bool _pose_updated;
	bool _scan_updated;
	bool _update_updated;
	bool _crashed;
	
	double _robot_pose;
	double _prev_robot_pose;
	double _scan_data[3];

	bool _bias_right;
	int _bias_factor;
	int _turn_factor;
	int _acts_per_turn;
	
	std::string _turn_request;
	
	bool _actLogging;
	bool _actWarning;
	std::chrono::milliseconds _actInterval;
	std::string _struct;
	std::string _model;
	std::size_t _induceThreadCount;
	std::chrono::milliseconds _induceInterval;
	std::string _mode;
	
	std::string _goal;

	std::map<std::string, std::map<std::string,std::string>> _goalsLocationsNext;
	
	std::shared_ptr<Alignment::ActiveSystem> _system;
	std::shared_ptr<Alignment::ActiveEventsRepa> _events;
	std::vector<std::thread> _threads;
	std::vector<std::shared_ptr<Alignment::Active>> _level1;
	std::size_t _level1Count;
	std::vector<std::shared_ptr<Alignment::Active>> _level2;
	std::vector<std::shared_ptr<Alignment::Active>> _level3;
	Alignment::ActiveUpdateParameters _updateParameters;
	Alignment::ActiveInduceParameters _induceParametersLevel1;
	Alignment::ActiveInduceParameters _induceParameters;
	std::size_t _eventId;
	
	std::shared_ptr<Alignment::System> _uu;
	std::shared_ptr<Alignment::SystemRepa> _ur;
	
	bool _modeLogging;	
	double _mode1DiscountRate;	
	double _mode1Turnaway;	
	bool _mode1Probabilistic;	
	bool _mode1Shortest;	
	bool _mode1ExpectedPV;	
	bool _mode1Repulsive;	
	bool _mode1GuessLocation;	
	bool _modeProbabilistic;	
	std::unordered_map<std::size_t, Alignment::SizeSet> _mode3SlicesSliceSetNext;
	std::unordered_map<std::size_t, std::size_t> _mode3SlicesLocation;
	bool _modeMultipleTransition;	
	bool _mode4Caching;
	bool _mode4Stepwise;
	std::unordered_map<std::size_t, Alignment::SizeSet> _mode4SlicesSliceSetNext;
	std::map<std::size_t, std::unordered_map<std::size_t, std::size_t>> _mode4locationsSlicesStepCount;
	int _mode4Lag;
	std::set<std::size_t> _mode4Neighbours;
	std::set<std::size_t> _mode4NeighbourLeasts;
	std::size_t _mode4SliceLocA;
	std::size_t _mode4TransistionSuccessCount;
	std::size_t _mode4TransistionNullCount;
	double _mode4TransistionExpectedSuccessCount;
	std::size_t _mode4TransistionCount;
	
private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _goal_sub;

	rclcpp::TimerBase::SharedPtr _update_timer;
	
	void update_callback();
	void update_cmd_vel(double linear, double angular);
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void goal_callback(const std_msgs::msg::String::SharedPtr msg);
};
#endif // ACTOR_H
