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
	
	enum Status {START, WAIT_ODOM, WAIT_SCAN, AHEAD, LEFT, RIGHT, STOP, CRASH};
	Status _status;
	TimePoint _startTimestamp;
	TimePoint _statusTimestamp;

	std::array<double,7> _posePrevious;
	std::array<double,7> _pose;
	TimePoint _poseTimestampPrevious;
	TimePoint _poseTimestamp;
	std::array<double,7> _poseStop;
	TimePoint _poseStopTimestamp;
	double _linearStopMaximum;
	double _linearMaximum;
	double _linearVelocity;
	double _angularStopMaximum;
	double _angularMaximum;
	double _angularMaximumLag;
	double _angularVelocity;
	std::array<double,360> _scan;
	TimePoint _scanTimestampPrevious;
	TimePoint _scanTimestamp;
	Status _actionPrevious;
	
	std::shared_ptr<TBOT03::RecordList> _records;
	TBOT03::Record eventsRecord(std::size_t);
	
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
	std::map<Status, double> _distribution;
	std::map<Status, double> _distributionTurn;
	double _collisionRange;
	std::size_t _collisionFOV;
	std::unordered_map<std::size_t, Alignment::SizeSet> _slicesSliceSetNext;
	std::map<std::size_t, std::unordered_map<std::size_t, std::size_t>> _locationsSlicesStepCount;
	std::set<std::size_t> _neighbours;
	std::set<std::size_t> _neighbourLeasts;
	std::size_t _sliceLocA;
	std::size_t _transistionSuccessCount;
	std::size_t _transistionNullCount;
	double _transistionExpectedSuccessCount;
	std::size_t _transistionCount;
	
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
