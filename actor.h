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
	
	enum Status {START, WAIT_ODOM, WAIT_SCAN, AHEAD, LEFT, RIGHT, STOP, CRASH} _status;
	TimePoint _statusTimestamp;
	TimePoint _startTimestamp;

	std::array<double,7> _posePrevious;
	std::array<double,7> _pose;
	TimePoint _poseTimestampPrevious;
	TimePoint _poseTimestamp;
	std::array<double,7> _poseStop;
	double _linearMaximum;
	double _linearVelocity;
	double _angularMaximum;
	double _angularVelocity;
	std::array<double,360> _scan;
	TimePoint _scanTimestampPrevious;
	TimePoint _scanTimestamp;
	
	TBOT03::RecordList _records;
	
	bool _updateLogging;
	
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
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisherCmdVel;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscriptionScan;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriptionOdom;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriptionGoal;

	rclcpp::TimerBase::SharedPtr _timerUpdate;
	
	void callbackUpdate();
	void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
	void callbackGoal(const std_msgs::msg::String::SharedPtr msg);
};
#endif // ACTOR_H
