#include "actor.h"
#include <sstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Alignment;
using namespace TBOT03;
using namespace std;
using namespace std::chrono_literals;
namespace js = rapidjson;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(actor_this->get_logger(), str.str());}
#define EVALTIME(x) { std::ostringstream str; str << #x << ": " << std::fixed << std::setprecision(3) << ((Sec)(x - actor_this->_startTimestamp)).count() << std::defaultfloat << "s"; RCLCPP_INFO(actor_this->get_logger(), str.str());}
#define EVALDATETIME(x) { const std::time_t t_c = std::chrono::system_clock::to_time_t(x); std::ostringstream str; str << #x << ": " << std::put_time(std::localtime(&t_c), "%F %T"); RCLCPP_INFO(actor_this->get_logger(), str.str());}
#define TRUTH(x) { std::ostringstream str; str << #x << ": " << ((x) ? "true" : "false"); RCLCPP_INFO(actor_this->get_logger(), str.str());}

#define ARGS_STRING_DEF(x,y) args.HasMember(#x) && args[#x].IsString() ? args[#x].GetString() : y
#define ARGS_STRING(x) ARGS_STRING_DEF(x,"")
#define ARGS_INT_DEF(x,y) args.HasMember(#x) && args[#x].IsInt() ? args[#x].GetInt() : y
#define ARGS_INT(x) ARGS_INT_DEF(x,0)
#define ARGS_DOUBLE_DEF(x,y) args.HasMember(#x) && args[#x].IsDouble() ? args[#x].GetDouble() : y
#define ARGS_DOUBLE(x) ARGS_DOUBLE_DEF(x,0.0)
#define ARGS_BOOL_DEF(x,y) args.HasMember(#x) && args[#x].IsBool() ? args[#x].GetBool() : y
#define ARGS_BOOL(x) ARGS_BOOL_DEF(x,false)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

Actor* actor_this = 0;

#define UNLOG ; str.flush(); RCLCPP_INFO(actor_this->get_logger(), str.str());}
#define LOG { std::ostringstream str; str << 

typedef std::pair<double, double> Coord;
typedef std::pair<Coord, Coord> CoordP;

SystemHistoryRepaTuple TBOT03::posesScansHistoryRepa_2(int valencyScan, int valencyDirection, const std::array<double,7>& pose, const std::array<double,360>& scan)
{
	auto lluu = listsSystem_u;

	std::size_t n = 360 + 3;
	std::size_t z = 1;
	ValSet buckets;
	for (int i = 0; i < valencyScan; i++)
		buckets.insert(Value(i));
	ValSet actions;
	for (int i = 0; i < 3; i++)
		actions.insert(Value(i));
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room3"), Value("room4"), Value("room5"), Value("room6") };
	ValSet bucketsDirection;
	for (int i = 0; i < valencyDirection; i++)
		bucketsDirection.insert(Value(i));
	vector<Coord> doors{ Coord(6.2,-0.175), Coord(2.3,4.5), Coord(2.3,0.375), Coord(-5.15,3.1), Coord(-6.325,0.925) };
	vector<CoordP> rooms{
		CoordP(Coord(2.3,-0.175),Coord(7.5,5.27)),
		CoordP(Coord(4.9,-5.275),Coord(7.5,-0.175)),
		CoordP(Coord(-0.05,0.925),Coord(2.3,5.27)),
		CoordP(Coord(-5.15,-0.175),Coord(2.3,5.27)),
		CoordP(Coord(-7.5,0.925),Coord(-5.15,5.27)),
		CoordP(Coord(-7.5,-3.925),Coord(-5.15,0.925)) };
	vector<VarValSetPair> ll;
	auto vscan = std::make_shared<Variable>("scan");
	for (std::size_t i = 0; i < n - 3; i++)
		ll.push_back(VarValSetPair(Variable(vscan, std::make_shared<Variable>((int)i + 1)), buckets));
	ll.push_back(VarValSetPair(Variable("motor"), actions));
	ll.push_back(VarValSetPair(Variable("direction"), bucketsDirection));
	ll.push_back(VarValSetPair(Variable("location"), locations));
	auto uu = lluu(ll);
	auto ur = std::make_unique<SystemRepa>();
	auto& mm = ur->listVarSizePair;
	mm.reserve(ll.size());
	for (auto& vww : ll)
		mm.push_back(VarSizePair(std::make_shared<Variable>(vww.first), vww.second.size()));
	auto hr = make_unique<HistoryRepa>();
	hr->dimension = n;
	hr->vectorVar = new size_t[n];
	auto vv = hr->vectorVar;
	hr->shape = new size_t[n];
	auto sh = hr->shape;
	hr->size = z;
	hr->evient = true;
	hr->arr = new unsigned char[z*n];
	auto rr = hr->arr;
	for (size_t i = 0; i < n; i++)
		vv[i] = i;
	for (size_t i = 0; i < n - 3; i++)
		sh[i] = valencyScan;
	sh[n - 3] = actions.size();
	sh[n - 2] = bucketsDirection.size();
	sh[n - 1] = locations.size();
	double f = (double)valencyScan / 4.0;
	for (size_t i = 0; i < n - 3; i++)
		rr[i] = (unsigned char)(scan[i] * f);
	rr[n - 3] = 0;
	double yaw = 0.0;		
	{
		tf2::Quaternion q(
			pose[3],
			pose[4],
			pose[5],
			pose[6]);
		tf2::Matrix3x3 m(q);
		double roll, pitch;
		m.getRPY(roll, pitch, yaw);
		yaw *= RAD2DEG;	
		yaw += 180.0;
		if (yaw >= 360.0) yaw = 359.9;
		if (yaw < 0.0) yaw = 0.0;
	}
	rr[n - 2] = (unsigned char)(yaw * valencyDirection / 360.0);
	double x = pose[0];
	double y = pose[1];
	size_t k = 0;
	while (k < doors.size())
	{
		if ((doors[k].first - x)*(doors[k].first - x) + (doors[k].second - y)*(doors[k].second - y) <= 0.25)
			break;
		k++;
	}
	while (k >= doors.size() && k < doors.size() + rooms.size())
	{
		auto room = rooms[k - doors.size()];
		if (room.first.first <= x && x <= room.second.first && room.first.second <= y && y <= room.second.second)
			break;
		k++;
	}
	if (k >= doors.size() + rooms.size())
		k = 9;
	rr[n - 1] = (unsigned char)k;
	hr->transpose();
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}



void actor_log(Active& active, const std::string& str)
{
	if (actor_this)
	{
		RCLCPP_INFO(actor_this->get_logger(), str);
	}
	else
		std::cout << str << std::endl;
	return;
};

void layerer_actor_log(const std::string& str)
{
	if (actor_this)
	{
		RCLCPP_INFO(actor_this->get_logger(), str);
	}
	else
		std::cout << str << std::endl;
	return;
};
	
void run_induce(Actor& actor, Active& active, ActiveInduceParameters& param, std::size_t induceThresholdInitial, std::chrono::milliseconds induceInterval)
{
	while (!actor._terminate && !active.terminate)
	{
		if (actor._poseTimestamp != TimePoint() && actor._scanTimestamp != TimePoint() && actor._eventId >= induceThresholdInitial)
			active.induce(param);
		std::this_thread::sleep_for(std::chrono::milliseconds(induceInterval));
	}	
	return;
};

void run_update(Active& active, ActiveUpdateParameters ppu)
{
	if (!active.terminate)
		active.update(ppu);
	return;
};

void run_act(Actor& actor)
{
	while (!actor._terminate)
	{
		auto mark = Clock::now();
		if (actor._status == Actor::STOP 
			&& actor._system)
		{
			if (actor._actionPrevious != Actor::STOP)
			{
				std::unique_ptr<HistoryRepa> hr;
				{
					SystemHistoryRepaTuple xx;
					if (actor._struct=="struct003")
						xx = posesScansHistoryRepa_2(actor._valencyScan, actor._valencyDirection, actor._pose, actor._scan);	
					else
						xx = posesScansHistoryRepa(actor._valencyScan, actor._pose, actor._scan);
					hr = std::move(std::get<2>(xx));
					auto n = hr->dimension;
					auto rr = hr->arr;
					if (actor._actionPrevious == Actor::LEFT)
						rr[n - 3] = 0;
					else if (actor._actionPrevious == Actor::RIGHT)
						rr[n - 3] = 2;
					else
						rr[n - 3] = 1;
				}
				double x2 = actor._pose[0];
				double y2 = actor._pose[1];
				double yaw2 = 0.0;
				{			
					tf2::Quaternion q(
						actor._pose[3],
						actor._pose[4],
						actor._pose[5],
						actor._pose[6]);
					tf2::Matrix3x3 m(q);
					double roll, pitch;
					m.getRPY(roll, pitch, yaw2);
					yaw2 *= RAD2DEG;			
				}	
				actor._records->push_back(Record(x2,y2,yaw2));				
				actor._eventId = actor._records->size() - 1;
				actor._events->mapIdEvent[actor._eventId] = HistoryRepaPtrSizePair(std::move(hr),actor._events->references);			
				{		
					std::vector<std::thread> threadsLevel;
					threadsLevel.reserve(actor._level1.size());
					for (auto& activeA  : actor._level1)
					{
						threadsLevel.push_back(std::thread(run_update, std::ref(*activeA), actor._updateParameters));
					}
					for (auto& t : threadsLevel)
						t.join();			
				}
				for (auto& activeA  : actor._level2)
				{
					if (!activeA->terminate)		
						activeA->update(actor._updateParameters);
				}
				{		
					std::vector<std::thread> threadsLevel;
					threadsLevel.reserve(actor._level3.size());
					for (auto& activeA  : actor._level3)
					{
						threadsLevel.push_back(std::thread(run_update, std::ref(*activeA), actor._updateParameters));
					}
					for (auto& t : threadsLevel)
						t.join();			
				}
				if (actor._actLogging && (actor._actLoggingFactor <= 1 || actor._eventId % actor._actLoggingFactor == 0))
				{
					LOG "actor\tevent id: " << actor._eventId << "\ttime " << ((Sec)(Clock::now() - mark)).count() << "s" UNLOG
				}		
			}	
			if (actor._mode=="mode001")
			{
				actor._status = Actor::AHEAD;
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "AHEAD" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode002")
			{
				actor._status = Actor::LEFT;
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "LEFT" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode003")
			{
				actor._status = Actor::RIGHT;
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "RIGHT" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode004")
			{
				std::this_thread::sleep_for((std::chrono::milliseconds)250);
				actor._status = Actor::WAIT_ODOM;
				actor._actionPrevious = Actor::STOP;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "WAIT_ODOM" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode005")
			{
				// turn randomly chosen from distribution
				actor._status = Actor::AHEAD;
				{
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode006")
			{
				// turn randomly chosen from distribution with collision avoidance
				actor._status = Actor::AHEAD;
				{
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				actor._actionPrevious = actor._status;
				if (actor._status == Actor::AHEAD)
				{
					for (std::size_t i = 360 - actor._collisionFOV; i < 360 + actor._collisionFOV; i++)
						if (actor._scan[i%360] <= actor._collisionRange)
						{
							actor._status = Actor::WAIT_ODOM;
							break;					
						}					
				}
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
						case Actor::WAIT_ODOM   : statusString = "WAIT_ODOM";    break;
						default   : statusString = "UNDEFINED";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode007")
			{
				// turn randomly chosen from distribution with collision avoidance
				// handle case of blocked ahead and blocked to the sides
				actor._status = Actor::AHEAD;
				{
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				bool blockedAhead = false;
				if (actor._status == Actor::AHEAD)
				{
					for (std::size_t i = 360 - actor._collisionFOV; i < 360 + actor._collisionFOV; i++)
						if (actor._scan[i%360] <= actor._collisionRange)
						{
							blockedAhead = true;
							break;					
						}					
				}
				bool blockedLeft = actor._scan[(int)actor._angularMaximum] <= actor._collisionRange;
				bool blockedRight = actor._scan[360-(int)actor._angularMaximum] <= actor._collisionRange;
				if (blockedAhead && blockedLeft && !blockedRight)
					actor._status = Actor::RIGHT;
				else if (blockedAhead && !blockedLeft && blockedRight)
					actor._status = Actor::LEFT;
				else if (blockedAhead)
				{
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distributionTurn)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}				
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode008")
			{
				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate)	
				{			
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					if (ok)
					{
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						auto nloc = locations.size();
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	
						auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
						auto sliceA = activeA.historySparse->arr[historyEventA];
						std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
						auto& hs = *activeA.historySparse;
						std::size_t goal = locationsInt[actor._goal];
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						auto rs = hs.arr;
						auto locA = rr[historyEventA*n+location];
						auto sliceLocA = sliceA*nloc+locA;
						auto sliceCount = activeA.historySlicesSetEvent.size();
						actor._status = Actor::AHEAD;
						// EVAL(sliceA);							
						EVAL(locations[locA]);							
						EVAL(sliceLocA);	
						EVAL(actor.eventsRecord(historyEventA));						
						std::map<std::size_t, std::size_t> neighbours;
						{
							auto& slicesStepCount = actor._locationsSlicesStepCount[goal];
							for (auto sliceLocB : actor._slicesSliceSetNext[sliceLocA])
							{
								auto it = slicesStepCount.find(sliceLocB);
								if (it != slicesStepCount.end())
									neighbours[sliceLocB] = it->second;
							}
						}
						EVAL(neighbours);							
						std::set<std::size_t> neighbourLeasts;
						{
							bool found = false;
							std::size_t least = 0;
							for (auto& p : neighbours)	
								if (!found || least > p.second)
								{
									least = p.second;
									found = true;
								}
							for (auto& p : neighbours)	
								if (p.second == least)	
									neighbourLeasts.insert(p.first);
							EVAL(least);
						}
						EVAL(neighbourLeasts);
						if (sliceLocA != actor._sliceLocA)
						{
							if (!actor._neighbours.count(sliceLocA))
								actor._transistionNullCount++;
							else 
							{
								if (actor._neighbourLeasts.count(sliceLocA))
									actor._transistionSuccessCount++;
								actor._transistionExpectedSuccessCount += (double) actor._neighbourLeasts.size() / (double) actor._neighbours.size();
							}
							actor._transistionCount++;
							double transition_success_rate = (double) actor._transistionSuccessCount * 100.0 / (double) actor._transistionCount;
							double transition_expected_success_rate = (double) actor._transistionExpectedSuccessCount * 100.0 / (double) actor._transistionCount;
							double transition_null_rate = (double) actor._transistionNullCount * 100.0 / (double) actor._transistionCount;
							EVAL(transition_success_rate);
							EVAL(transition_expected_success_rate);
							EVAL(transition_null_rate);
							actor._sliceLocA = sliceLocA;
							actor._neighbourLeasts = neighbourLeasts;
							actor._neighbours.clear();
							for (auto& p : neighbours)
								actor._neighbours.insert(p.first);
						}
						std::map<std::size_t, std::size_t> actionsCount;
						if (neighbourLeasts.size() && neighbourLeasts.size() < neighbours.size())
						{
							RecordList recordStandards;
							std::vector<std::tuple<double, double, double>> records;
							for (auto ev : activeA.historySlicesSetEvent[sliceA])
							{
								if (rr[ev*n+location] == locA)
								{
									Record record = actor.eventsRecord(ev);
									records.push_back(std::make_tuple(record.x, record.y, record.yaw));
									recordStandards.push_back(record.standard());
									auto j = ev + (ev >= y ? 0 : z)  + 1;	
									if (j < y+z && (!activeA.continousIs || !activeA.continousHistoryEventsEvent.count(j%z)))
									{
										auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
										if (sliceLocB != sliceLocA)
										{
											if (neighbourLeasts.count(sliceLocB))
												actionsCount[rr[(j%z)*n+motor]]++;
										}
									}										
								}
							}
							std::sort(records.begin(),records.end());
							for (auto& recordA : records)
							{
								Record record(std::get<0>(recordA),std::get<1>(recordA),std::get<2>(recordA));
								EVAL(record);
							}
							EVAL(recordsMean(recordStandards).config());
							EVAL(recordsDeviation(recordStandards));
						}
						EVAL(actionsCount);						
						if (actionsCount.size())
						{
							char action = ahead;
							std::size_t total = 0;
							for (auto& p : actionsCount)	
								total += p.second;
							auto r = rand() % total;
							std::size_t accum = 0.0;
							for (auto& p : actionsCount)
							{
								accum += p.second;
								if (r < accum)
								{
									action = p.first;
									break;
								}
							}		
							if (action == turn_left)
								actor._status = Actor::LEFT;
							else if (action == turn_right)
								actor._status = Actor::RIGHT;					
						}
						else
						{					
							auto r = (double) rand() / (RAND_MAX);
							double accum = 0.0;
							for (auto& p : actor._distribution)
							{
								accum += p.second;
								if (r < accum)
								{
									actor._status = p.first;
									break;
								}
							}						
						}
						bool blockedAhead = false;
						{
							if (actor._status == Actor::AHEAD)
							{
								for (std::size_t i = 360 - actor._collisionFOV; i < 360 + actor._collisionFOV; i++)
									if (actor._scan[i%360] <= actor._collisionRange)
									{
										blockedAhead = true;
										break;					
									}					
							}
							bool blockedLeft = actor._scan[(int)actor._angularMaximum] <= actor._collisionRange;
							bool blockedRight = actor._scan[360-(int)actor._angularMaximum] <= actor._collisionRange;
							if (blockedAhead && blockedLeft && !blockedRight)
								actor._status = Actor::RIGHT;
							else if (blockedAhead && !blockedLeft && blockedRight)
								actor._status = Actor::LEFT;
							else if (blockedAhead)
							{
								auto r = (double) rand() / (RAND_MAX);
								double accum = 0.0;
								for (auto& p : actor._distributionTurn)
								{
									accum += p.second;
									if (r < accum)
									{
										actor._status = p.first;
										break;
									}
								}						
							}				
							actor._actionPrevious = actor._status;
						}
						TRUTH(blockedAhead);
						if (actor._updateLogging)
						{
							string statusString;
							switch(actor._status)
							{
								case Actor::AHEAD   : statusString = "AHEAD";    break;
								case Actor::LEFT   : statusString = "LEFT";    break;
								case Actor::RIGHT   : statusString = "RIGHT";    break;
							}
							LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
						}			
						actor._statusTimestamp = Clock::now();	
					}
				}		
			}
			else if (actor._mode=="mode009")
			{
				bool ok = true;		
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate && actor._actionPrevious != Actor::STOP)	
				{			
					std::lock_guard<std::mutex> guard(activeA.mutex);
					ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
					if (ok)
					{
						std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
						auto nloc = locations.size();
						std::map<std::string,std::size_t> locationsInt;
						for (std::size_t i = 0; i < locations.size(); i++)
							locationsInt[locations[i]] = i;	
						auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
						auto sliceA = activeA.historySparse->arr[historyEventA];
						std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
						auto& hs = *activeA.historySparse;
						std::size_t goal = locationsInt[actor._goal];
						const char turn_left = 0;
						const char ahead = 1;
						const char turn_right = 2;
						auto over = activeA.historyOverflow;
						auto& mm = actor._ur->mapVarSize();
						auto& mvv = hr->mapVarInt();
						auto motor = mvv[mm[Variable("motor")]];
						auto location = mvv[mm[Variable("location")]];
						auto n = hr->dimension;
						auto z = hr->size;
						auto y = activeA.historyEvent;
						auto rr = hr->arr;	
						auto rs = hs.arr;
						auto locA = rr[historyEventA*n+location];
						auto sliceLocA = sliceA*nloc+locA;
						auto sliceCount = activeA.historySlicesSetEvent.size();
						// EVAL(sliceA);							
						EVAL(locations[locA]);							
						EVAL(sliceLocA);	
						EVAL(actor.eventsRecord(historyEventA));	
						{
							RecordList recordStandards;
							std::vector<std::tuple<double, double, double>> records;
							for (auto ev : activeA.historySlicesSetEvent[sliceA])
							{
								if (rr[ev*n+location] == locA)
								{
									Record record = actor.eventsRecord(ev);
									records.push_back(std::make_tuple(record.x, record.y, record.yaw));
									recordStandards.push_back(record.standard());
								}
							}
							std::sort(records.begin(),records.end());
							for (auto& recordA : records)
							{
								Record record(std::get<0>(recordA),std::get<1>(recordA),std::get<2>(recordA));
								EVAL(record);
							}
							EVAL(recordsMean(recordStandards).config());
							EVAL(recordsDeviation(recordStandards));
						}						
						std::map<std::size_t, std::size_t> neighbours;
						{
							auto& slicesStepCount = actor._locationsSlicesStepCount[goal];
							for (auto sliceLocB : actor._slicesSliceSetNext[sliceLocA])
							{
								auto it = slicesStepCount.find(sliceLocB);
								if (it != slicesStepCount.end())
									neighbours[sliceLocB] = it->second;
							}
						}
						EVAL(neighbours);							
						std::set<std::size_t> neighbourLeasts;
						{
							bool found = false;
							std::size_t least = 0;
							for (auto& p : neighbours)	
								if (!found || least > p.second)
								{
									least = p.second;
									found = true;
								}
							for (auto& p : neighbours)	
								if (p.second == least)	
									neighbourLeasts.insert(p.first);
							EVAL(least);
						}
						EVAL(neighbourLeasts);
						if (sliceLocA != actor._sliceLocA)
						{
							if (!actor._neighbours.count(sliceLocA))
								actor._transistionNullCount++;
							else 
							{
								if (actor._neighbourLeasts.count(sliceLocA))
									actor._transistionSuccessCount++;
								actor._transistionExpectedSuccessCount += (double) actor._neighbourLeasts.size() / (double) actor._neighbours.size();
							}
							actor._transistionCount++;
							double transition_success_rate = (double) actor._transistionSuccessCount * 100.0 / (double) actor._transistionCount;
							double transition_expected_success_rate = (double) actor._transistionExpectedSuccessCount * 100.0 / (double) actor._transistionCount;
							double transition_null_rate = (double) actor._transistionNullCount * 100.0 / (double) actor._transistionCount;
							EVAL(transition_success_rate);
							EVAL(transition_expected_success_rate);
							EVAL(transition_null_rate);
							actor._sliceLocA = sliceLocA;
							actor._neighbourLeasts = neighbourLeasts;
							actor._neighbours.clear();
							for (auto& p : neighbours)
								actor._neighbours.insert(p.first);
						}
						std::map<std::size_t, std::size_t> actionsCount;
						if (neighbourLeasts.size() && neighbourLeasts.size() < neighbours.size())
						{
							for (auto ev : activeA.historySlicesSetEvent[sliceA])
							{
								if (rr[ev*n+location] == locA)
								{
									auto j = ev + (ev >= y ? 0 : z)  + 1;	
									if (j < y+z && (!activeA.continousIs || !activeA.continousHistoryEventsEvent.count(j%z)))
									{
										auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
										if (sliceLocB != sliceLocA)
										{
											if (neighbourLeasts.count(sliceLocB))
												actionsCount[rr[(j%z)*n+motor]]++;
										}
									}										
								}
							}
						}
						EVAL(actionsCount);		
						actor._actionPrevious = Actor::STOP;
					}	
				}
				if (actor._actionManual.size())
				{				
					if (actor._actionManual == "LEFT")
						actor._status = Actor::LEFT;
					else if (actor._actionManual == "AHEAD")
						actor._status = Actor::AHEAD;					
					else if (actor._actionManual == "RIGHT")
						actor._status = Actor::RIGHT;					
					actor._actionPrevious = actor._status;
					actor._actionManual = "";
					if (actor._updateLogging)
					{
						string statusString;
						switch(actor._status)
						{
							case Actor::AHEAD   : statusString = "AHEAD";    break;
							case Actor::LEFT   : statusString = "LEFT";    break;
							case Actor::RIGHT   : statusString = "RIGHT";    break;
						}
						LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
					}			
					actor._statusTimestamp = Clock::now();	
				}
			}
			else if (actor._mode=="mode010")
			{
				// turn randomly chosen from distribution after 
				// checking to the sides according to the turn bias
				// and checking ahead - similar to TBOT01/2
				if (actor._turnBiasFactor > 0 && (rand() % actor._turnBiasFactor) == 0)
					actor._turnBiasRight = !actor._turnBiasRight;
				bool blockedAhead = actor._scan[0] <= actor._collisionRange;
				bool blockedLeft = actor._scan[(int)actor._angularMaximum] <= actor._collisionRangeAngle;
				bool blockedRight = actor._scan[360-(int)actor._angularMaximum] <= actor._collisionRangeAngle;
				if (actor._turnBiasRight && blockedLeft)
					actor._status = Actor::RIGHT;
				else if (blockedRight)
					actor._status = Actor::LEFT;
				else if (blockedLeft)
					actor._status = Actor::RIGHT;
				else if (blockedAhead)
					actor._status = actor._turnBiasRight ? Actor::RIGHT : Actor::LEFT;
				else
				{
					actor._status = Actor::AHEAD;
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode011")
			{
				// turn randomly chosen from distribution after 
				// checking to the sides according to the turn bias
				// and checking ahead - similar to TBOT01/2 but less blocking
				if (actor._turnBiasFactor > 0 && (rand() % actor._turnBiasFactor) == 0)
					actor._turnBiasRight = !actor._turnBiasRight;
				bool blockedAhead = actor._scan[0] <= actor._collisionRange;
				bool blockedLeft = actor._scan[(int)actor._angularMaximum] <= actor._collisionRangeAngle;
				bool blockedRight = actor._scan[360-(int)actor._angularMaximum] <= actor._collisionRangeAngle;
				if (actor._turnBiasRight && blockedLeft)
					actor._status = Actor::RIGHT;
				else if (!actor._turnBiasRight && blockedRight)
					actor._status = Actor::LEFT;
				else if (blockedAhead)
					actor._status = actor._turnBiasRight ? Actor::RIGHT : Actor::LEFT;
				else
				{
					actor._status = Actor::AHEAD;
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode012")
			{
				// turn randomly chosen from distribution after 
				// checking the entire field of view
				if (actor._turnBiasFactor > 0 && (rand() % actor._turnBiasFactor) == 0)
					actor._turnBiasRight = !actor._turnBiasRight;
				bool blockedAhead = false;
				for (std::size_t i = 360 - actor._collisionFOV; i < 360 + actor._collisionFOV; i++)
					if (actor._scan[i%360] <= actor._collisionRange)
					{
						blockedAhead = true;
						break;					
					}		
				if (blockedAhead)
					actor._status = actor._turnBiasRight ? Actor::RIGHT : Actor::LEFT;
				else
				{
					actor._status = Actor::AHEAD;
					auto r = (double) rand() / (RAND_MAX);
					double accum = 0.0;
					for (auto& p : actor._distribution)
					{
						accum += p.second;
						if (r < accum)
						{
							actor._status = p.first;
							break;
						}
					}						
				}
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode013")
			{
				// check the entire field of view
				// check to see if remaining actions are effective
				// choose randomly from the ineffective, setting the turn bias if blocked
				// else choose randomly or by turn bias if blocked
				if (actor._turnBiasFactor > 0 && (rand() % actor._turnBiasFactor) == 0)
					actor._turnBiasRight = !actor._turnBiasRight;
				bool blockedAhead = false;
				for (std::size_t i = 360 - actor._collisionFOV; i < 360 + actor._collisionFOV; i++)
					if (actor._scan[i%360] <= actor._collisionRange)
					{
						blockedAhead = true;
						break;					
					}	
				const char turn_left = 0;
				const char ahead = 1;
				const char turn_right = 2;
				std::map<std::size_t, std::size_t> actionsCount;
				auto& activeA = *actor._level2.front();
				if (!activeA.terminate && (activeA.historyOverflow	|| activeA.historyEvent))	
				{			
					std::lock_guard<std::mutex> guard(activeA.mutex);
					auto historyEventA = activeA.historyEvent ? activeA.historyEvent - 1 : activeA.historySize - 1;
					auto sliceA = activeA.historySparse->arr[historyEventA];
					std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
					auto cont = activeA.continousIs;
					auto& disc = activeA.continousHistoryEventsEvent;
					auto& mm = actor._ur->mapVarSize();
					auto& mvv = hr->mapVarInt();
					auto motor = mvv[mm[Variable("motor")]];
					auto n = hr->dimension;
					auto z = hr->size;
					auto y = activeA.historyEvent;
					auto rr = hr->arr;	
					// EVAL(sliceA);							
					// EVAL(actor.eventsRecord(historyEventA));
					for (auto& ev : activeA.historySlicesSetEvent[sliceA])
					{
						auto j = ev + (ev >= y ? 0 : z)  + 1;	
						if (j < y+z && (!cont || !disc.count(j%z)))
							actionsCount[rr[(j%z)*n+motor]]++;
					}	
				}	
				// EVAL(actionsCount);
				if (blockedAhead)
				{
					if (!actionsCount[turn_left] && actionsCount[turn_right])
						actor._turnBiasRight = false;
					else if (actionsCount[turn_left] && !actionsCount[turn_right])
						actor._turnBiasRight = true;
					actor._status = actor._turnBiasRight ? Actor::RIGHT : Actor::LEFT;
				}					
				else
				{
					std::map<Actor::Status, double> distributionA;
					if (!actionsCount[turn_left])
						distributionA[Actor::LEFT] = actor._distribution[Actor::LEFT];
					if (!actionsCount[ahead])
						distributionA[Actor::AHEAD] = actor._distribution[Actor::AHEAD];
					if (!actionsCount[turn_right])
						distributionA[Actor::RIGHT] = actor._distribution[Actor::RIGHT];
					if (!actionsCount.size())
						distributionA = actor._distribution;
					{
						double norm = 0.0;
						for (auto& p : distributionA)
							norm += p.second;	
						for (auto& p : distributionA)
							distributionA[p.first] /= norm;	
					}
					actor._status = Actor::AHEAD;
					{
						auto r = (double) rand() / (RAND_MAX);
						double accum = 0.0;
						for (auto& p : distributionA)
						{
							accum += p.second;
							if (r < accum)
							{
								actor._status = p.first;
								break;
							}
						}						
					}
				}
				actor._actionPrevious = actor._status;
				if (actor._updateLogging)
				{
					string statusString;
					switch(actor._status)
					{
						case Actor::AHEAD   : statusString = "AHEAD";    break;
						case Actor::LEFT   : statusString = "LEFT";    break;
						case Actor::RIGHT   : statusString = "RIGHT";    break;
					}
					LOG "actor\t" << statusString << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
		}
		auto t = Clock::now() - mark;
		if (t < actor._actInterval)
		{
			std::this_thread::sleep_for(actor._actInterval-t);
		}
		else if (actor._actWarning)		
		{
			LOG "actor warning\ttime " << ((Sec)(Clock::now() - mark)).count() << "s" UNLOG
		}		
	}
};

Actor::Actor(const std::string& args_filename)
: Node("TBOT03_actor_node")
{
	actor_this = this;
	_terminate = false;
	_status = START;
	_startTimestamp = Clock::now();
	_statusTimestamp = Clock::now();
			
	js::Document args;
	if (!args_filename.empty())
	{
		std::ifstream in;
		try 
		{
			in.open(args_filename);
			js::IStreamWrapper isw(in);
			args.ParseStream(isw);
		}
		catch (const std::exception& e) 
		{
			LOG "actor\terror: failed to open arguments file " << args_filename UNLOG
			return;
		}	
		if (!args.IsObject())
		{
			LOG "actor\terror: failed to read arguments file " << args_filename UNLOG
			return;
		}
	}
	else
	{
		args.Parse("{}");
	}

	_updateLogging = ARGS_BOOL(logging_update);
	_actLogging = ARGS_BOOL(logging_action);
	_actLoggingFactor = ARGS_INT(logging_action_factor); 
	_actWarning = ARGS_BOOL(warning_action);
	std::chrono::milliseconds updateInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(update_interval,10));
	_linearStopMaximum = ARGS_DOUBLE_DEF(linear_stop,0.005);
	_linearMaximum = ARGS_DOUBLE_DEF(linear_maximum,0.5);
	_linearVelocity = ARGS_DOUBLE_DEF(linear_velocity,0.3);
	_angularStopMaximum = ARGS_DOUBLE_DEF(angular_stop, 0.1);
	_angularMaximum = ARGS_DOUBLE_DEF(angular_maximum, 30.0);
	_angularMaximumLag = ARGS_DOUBLE_DEF(angular_maximum_lag, 7.0);
	_angularVelocity = ARGS_DOUBLE_DEF(angular_velocity, 40.0);
	_records = std::make_shared<RecordList>();
	_eventId = 0;
	_actInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(act_interval,10));
	_actionPrevious = AHEAD;
	_goal = ARGS_STRING_DEF(goal_initial,"room5");
	_struct = ARGS_STRING(structure);
	_model = ARGS_STRING(model);
	std::string modelInitial = ARGS_STRING(model_initial);
	std::string structInitial = ARGS_STRING(structure_initial);
	_induceThreadCount = ARGS_INT_DEF(induceThreadCount,4);
	_valencyScan = ARGS_INT_DEF(valency_scan,8);
	_valencyDirection = ARGS_INT_DEF(valency_direction,12);
	_level1Count = ARGS_INT_DEF(level1Count,12);
	bool level1Logging = ARGS_BOOL(logging_level1);
	bool level1Summary = ARGS_BOOL(summary_level1);
	std::size_t activeSizeLevel1 = ARGS_INT_DEF(activeSizeLevel1,10000);
	std::size_t induceThresholdLevel1 = ARGS_INT_DEF(induceThresholdLevel1,100);
	std::size_t induceThresholdInitialLevel1 = ARGS_INT_DEF(induceThresholdInitialLevel1,500);
	std::chrono::milliseconds induceIntervalLevel1 = (std::chrono::milliseconds)(ARGS_INT_DEF(induceIntervalLevel1,10));
	bool level2Logging = ARGS_BOOL(logging_level2);
	bool level2Summary = ARGS_BOOL(summary_level2);
	std::size_t activeSize = ARGS_INT_DEF(activeSize,1000000);
	_records->reserve(activeSize);
	std::size_t induceThreshold = ARGS_INT_DEF(induceThreshold,100);
	std::size_t induceThresholdInitial = ARGS_INT_DEF(induceThresholdInitial,1000);
	std::size_t induceThresholdInitialLevel3 = ARGS_INT_DEF(induceThresholdInitialLevel3,1500);
	std::chrono::milliseconds induceInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(induce_interval,10));	
	bool level3Logging = ARGS_BOOL(logging_level3);
	bool level3Summary = ARGS_BOOL(summary_level3);
	bool induceNot = ARGS_BOOL(no_induce);
	_mode = ARGS_STRING_DEF(mode, "mode001");	
	_modeLogging = ARGS_BOOL(mode_logging);	
	srand(ARGS_INT_DEF(mode_seed,7));
	_distribution[LEFT] = ARGS_DOUBLE_DEF(distribution_LEFT,1.0);
	_distribution[AHEAD] = ARGS_DOUBLE_DEF(distribution_AHEAD,5.0);
	_distribution[RIGHT] = ARGS_DOUBLE_DEF(distribution_RIGHT,1.0);
	{
		double norm = 0.0;
		for (auto& p : _distribution)
			norm += p.second;	
		for (auto& p : _distribution)
			_distribution[p.first] /= norm;	
	}
	_distributionTurn[LEFT] = ARGS_DOUBLE_DEF(distribution_LEFT,1.0);
	_distributionTurn[RIGHT] = ARGS_DOUBLE_DEF(distribution_RIGHT,1.0);
	{
		double norm = 0.0;
		for (auto& p : _distributionTurn)
			norm += p.second;	
		for (auto& p : _distributionTurn)
			_distributionTurn[p.first] /= norm;	
	}
	_collisionRange = ARGS_DOUBLE_DEF(collision_range, 1.0);
	_collisionFOV = ARGS_INT_DEF(collision_field_of_view, 20);
	_collisionRangeAngle = ARGS_DOUBLE_DEF(collision_range_angle, 0.7);
	_turnBiasRight = ARGS_BOOL(turn_bias_right);	
	_turnBiasFactor = ARGS_INT_DEF(turn_bias_factor,20);
	_configDeviationMax = ARGS_DOUBLE(configuration_deviation_maximum);
	{
		_induceParametersLevel1.tint = _induceThreadCount;
		_induceParametersLevel1.wmax = ARGS_INT_DEF(induceParametersLevel1.wmax,9);
		_induceParametersLevel1.lmax = ARGS_INT_DEF(induceParametersLevel1.lmax,8);
		_induceParametersLevel1.xmax = ARGS_INT_DEF(induceParametersLevel1.xmax,128);
		_induceParametersLevel1.znnmax = 200000.0 * 2.0 * 300.0 * 300.0 * _induceThreadCount;
		_induceParametersLevel1.omax = ARGS_INT_DEF(induceParametersLevel1.omax,10);
		_induceParametersLevel1.bmax = ARGS_INT_DEF(induceParametersLevel1.bmax,10*3);
		_induceParametersLevel1.mmax = ARGS_INT_DEF(induceParametersLevel1.mmax,3);
		_induceParametersLevel1.umax = ARGS_INT_DEF(induceParametersLevel1.umax,128);
		_induceParametersLevel1.pmax = ARGS_INT_DEF(induceParametersLevel1.pmax,1);
		_induceParametersLevel1.mult = ARGS_INT_DEF(induceParametersLevel1.mult,1);
		_induceParametersLevel1.seed = ARGS_INT_DEF(induceParametersLevel1.seed,5);
		_induceParameters.tint = _induceThreadCount;		
		_induceParameters.wmax = ARGS_INT_DEF(induceParameters.wmax,18);
		_induceParameters.lmax = ARGS_INT_DEF(induceParameters.lmax,8);
		_induceParameters.xmax = ARGS_INT_DEF(induceParameters.xmax,128);
		_induceParameters.znnmax = 200000.0 * 2.0 * 300.0 * 300.0 * _induceThreadCount;
		_induceParameters.omax = ARGS_INT_DEF(induceParameters.omax,10);
		_induceParameters.bmax = ARGS_INT_DEF(induceParameters.bmax,10*3);
		_induceParameters.mmax = ARGS_INT_DEF(induceParameters.mmax,3);
		_induceParameters.umax = ARGS_INT_DEF(induceParameters.umax,128);
		_induceParameters.pmax = ARGS_INT_DEF(induceParameters.pmax,1);
		_induceParameters.mult = ARGS_INT_DEF(induceParameters.mult,1);
		_induceParameters.seed = ARGS_INT_DEF(induceParameters.seed,5);		
	}
		
	// EVAL(_goal);
	// EVAL(_struct);
	// EVAL(_model);
	// EVAL(_mode);
	
	if (modelInitial.size())
	{
		try
		{
			std::ifstream in(modelInitial + ".rec", std::ios::binary);
			if (in.is_open())
			{
				_records = std::move(persistentsRecordList(in));
				_records->push_back(Record());
				in.close();
			}
			else
			{
				LOG "actor\terror: failed to open records file" << modelInitial + ".rec" UNLOG
				return;
			}
		}
		catch (const exception&)
		{
			LOG "actor\terror: failed to read records file" << modelInitial + ".rec" UNLOG
			return;
		}
	}

	if (_struct=="struct001" || _struct=="struct002" || _struct=="struct003")
	{
		std::unique_ptr<HistoryRepa> hr;
		{
			SystemHistoryRepaTuple xx;
			if (_struct=="struct003")
				xx = posesScansHistoryRepa_2(_valencyScan, _valencyDirection, _pose, _scan);	
			else
				xx = posesScansHistoryRepa(_valencyScan, _pose, _scan);
			_uu = std::move(std::get<0>(xx));
			_ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		_system = std::make_shared<ActiveSystem>();
		_threads.reserve(1+_level1Count+1+6);
		_events = std::make_shared<ActiveEventsRepa>(_level1Count+1);
		for (std::size_t m = 0; m < _level1Count; m++)
			_level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < _level1Count; m++)
		{			
			auto& activeA = *_level1[m];
			activeA.log = actor_log;
			activeA.layerer_log = layerer_actor_log;
			activeA.system = _system;
			if (modelInitial.size())
			{
				ActiveIOParameters ppio;
				ppio.filename = modelInitial + "_1_" + (m<10 ? "0" : "") + std::to_string(m) +".ac";
				activeA.logging = true;
				if (!activeA.load(ppio))
				{
					LOG "actor\terror: failed to load model" << ppio.filename UNLOG				
					_system.reset();
					return;
				}								
				_system->block = std::max(_system->block, activeA.varMax() >> activeA.bits);
				if (activeA.underlyingEventUpdateds.size())
					_eventId = std::max(_eventId,*(activeA.underlyingEventUpdateds.rbegin()));					
				else if (activeA.historyOverflow)
					_eventId = std::max(_eventId,activeA.historySize);	
				else					
					_eventId = std::max(_eventId,activeA.historyEvent);	
			}
			else
			{
				activeA.var = activeA.system->next(activeA.bits);
				activeA.varSlice = activeA.system->next(activeA.bits);
				activeA.historySize = activeSizeLevel1;
				activeA.induceThreshold = induceThresholdLevel1;
				activeA.decomp = std::make_unique<DecompFudSlicedRepa>();				
				{
					SizeList vv0;
					{
						auto& mm = _ur->mapVarSize();
						auto vscan = std::make_shared<Variable>("scan");
						int start = 360 - (360/_level1Count/2) + (360/_level1Count)*m;
						int end = start + (360/_level1Count);
						for (int i = start; i < end; i++)
							vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
					}
					auto hr1 = std::make_shared<HistoryRepa>();
					{
						auto sh = hr->shape;
						auto& mvv = hr->mapVarInt();
						auto n1 = vv0.size();
						hr1->dimension = n1;
						hr1->vectorVar = new std::size_t[n1];
						auto vv1 = hr1->vectorVar;
						hr1->shape = new std::size_t[n1];
						auto sh1 = hr1->shape;
						for (std::size_t i = 0; i < n1; i++)
						{
							auto v = vv0[i];
							vv1[i] = v;
							sh1[i] = sh[mvv[v]];
						}
						hr1->evient = true;
						hr1->size = activeA.historySize;
						auto z1 = hr1->size;
						hr1->arr = new unsigned char[z1*n1];
						auto rr1 = hr1->arr;
						// memset(rr1, 0, z1*n1);			
					}
					activeA.underlyingHistoryRepa.push_back(hr1);
				}
				{
					auto hr = std::make_unique<HistorySparseArray>();
					{
						auto z = activeA.historySize;
						hr->size = z;
						hr->capacity = 1;
						hr->arr = new std::size_t[z];		
					}		
					activeA.historySparse = std::move(hr);			
				}
			}
			activeA.name = (_model!="" ? _model : "model") + "_1_" + (m<10 ? "0" : "") + std::to_string(m);			
			activeA.logging = level1Logging;
			activeA.summary = level1Summary;
			activeA.underlyingEventsRepa.push_back(_events);
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}
			if (!induceNot)
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), std::ref(_induceParametersLevel1), induceThresholdInitialLevel1, induceIntervalLevel1));
		}	
		{
			_level2.push_back(std::make_shared<Active>());
			auto& activeA = *_level2.front();
			activeA.log = actor_log;
			activeA.layerer_log = layerer_actor_log;
			activeA.system = _system;
			activeA.continousIs = true;
			if (modelInitial.size())
			{
				ActiveIOParameters ppio;
				ppio.filename = modelInitial + "_2" +".ac";
				activeA.logging = true;
				if (!activeA.load(ppio))
				{
					LOG "actor\terror: failed to load model " << ppio.filename UNLOG				
					_system.reset();
					return;
				}								
				_system->block = std::max(_system->block, activeA.varMax() >> activeA.bits);
				if (activeA.underlyingEventUpdateds.size())
					_eventId = std::max(_eventId,*(activeA.underlyingEventUpdateds.rbegin()));					
				else if (activeA.historyOverflow)
					_eventId = std::max(_eventId,activeA.historySize);	
				else					
					_eventId = std::max(_eventId,activeA.historyEvent);	
			}
			else
			{			
				activeA.var = activeA.system->next(activeA.bits);
				activeA.varSlice = activeA.system->next(activeA.bits);
				activeA.historySize = activeSize;
				activeA.induceThreshold = induceThreshold;
				activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
				{
					SizeList vv0;
					{
						auto& mm = _ur->mapVarSize();
						vv0.push_back(mm[Variable("motor")]);
						vv0.push_back(mm[Variable("location")]);
						for (auto v : vv0)
							activeA.induceVarExclusions.insert(v);
						if (_struct=="struct003")
							vv0.push_back(mm[Variable("direction")]);
					}
					auto hr1 = std::make_shared<HistoryRepa>();
					{
						auto n = hr->dimension;
						auto vv = hr->vectorVar;
						auto sh = hr->shape;
						auto& mvv = hr->mapVarInt();
						auto n1 = vv0.size();
						hr1->dimension = n1;
						hr1->vectorVar = new std::size_t[n1];
						auto vv1 = hr1->vectorVar;
						hr1->shape = new std::size_t[n1];
						auto sh1 = hr1->shape;
						for (std::size_t i = 0; i < n1; i++)
						{
							auto v = vv0[i];
							vv1[i] = v;
							sh1[i] = sh[mvv[v]];
						}
						hr1->evient = true;
						hr1->size = activeA.historySize;
						auto z1 = hr1->size;
						hr1->arr = new unsigned char[z1*n1];
						auto rr1 = hr1->arr;
						// memset(rr1, 0, z1*n1);			
					}
					activeA.underlyingHistoryRepa.push_back(hr1);
				}
				for (std::size_t m = 0; m < _level1Count; m++)
				{
					activeA.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeA.historySize,1));
				}
				{
					auto hr = std::make_unique<HistorySparseArray>();
					{
						auto z = activeA.historySize;
						hr->size = z;
						hr->capacity = 1;
						hr->arr = new std::size_t[z];		
					}		
					activeA.historySparse = std::move(hr);			
				}
			}
			activeA.name = (_model!="" ? _model : "model") + "_2";
			activeA.logging = level2Logging;
			activeA.summary = level2Summary;
			activeA.underlyingEventsRepa.push_back(_events);
			for (std::size_t m = 0; m < _level1Count; m++)
			{
				auto& activeB = *_level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
			}
			if (_struct=="struct002")
				activeA.eventsSparse = std::make_shared<ActiveEventsArray>(0);	
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
			if (sizeA)
			{
				LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
			}			
			if (!induceNot)
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), std::ref(_induceParameters), induceThresholdInitial, induceInterval));			
		}
		if (_struct=="struct002")
		{
			std::vector<SizeList> under 
			{
				SizeList{0,1,2,3,4,6,8,10},
				SizeList{0,1,2,3,4}
			};
			std::vector<SizeList> self 
			{
				SizeList{},
				SizeList{5,10}
			};			
			for (std::size_t m = 0; m < under.size(); m++)
			{
				_level3.push_back(std::make_shared<Active>());
				auto& activeA = *_level3.back();
				activeA.log = actor_log;
				activeA.layerer_log = layerer_actor_log;
				activeA.system = _system;
				activeA.continousIs = true;
				if (modelInitial.size() && structInitial == "struct002")
				{
					ActiveIOParameters ppio;
					ppio.filename = modelInitial + "_3_" + (m<10 ? "0" : "") + std::to_string(m) +".ac";
					activeA.logging = true;
					if (!activeA.load(ppio))
					{
						LOG "actor\terror: failed to load model" << ppio.filename UNLOG
						_system.reset();
						return;
					}								
					_system->block = std::max(_system->block, activeA.varMax() >> activeA.bits);
					if (activeA.underlyingEventUpdateds.size())
						_eventId = std::max(_eventId,*(activeA.underlyingEventUpdateds.rbegin()));					
					else if (activeA.historyOverflow)
						_eventId = std::max(_eventId,activeA.historySize);	
					else					
						_eventId = std::max(_eventId,activeA.historyEvent);	
				}
				else
				{			
					activeA.var = activeA.system->next(activeA.bits);
					activeA.varSlice = activeA.system->next(activeA.bits);
					activeA.historySize = activeSize;
					activeA.induceThreshold = induceThreshold;
					activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
					{
						SizeList vv0;
						{
							auto& mm = _ur->mapVarSize();
							vv0.push_back(mm[Variable("motor")]);
							vv0.push_back(mm[Variable("location")]);
							for (auto v : vv0)
								activeA.induceVarExclusions.insert(v);
						}
						auto hr1 = std::make_shared<HistoryRepa>();
						{
							auto n = hr->dimension;
							auto vv = hr->vectorVar;
							auto sh = hr->shape;
							auto& mvv = hr->mapVarInt();
							auto n1 = vv0.size();
							hr1->dimension = n1;
							hr1->vectorVar = new std::size_t[n1];
							auto vv1 = hr1->vectorVar;
							hr1->shape = new std::size_t[n1];
							auto sh1 = hr1->shape;
							for (std::size_t i = 0; i < n1; i++)
							{
								auto v = vv0[i];
								vv1[i] = v;
								sh1[i] = sh[mvv[v]];
							}
							hr1->evient = true;
							hr1->size = activeA.historySize;
							auto z1 = hr1->size;
							hr1->arr = new unsigned char[z1*n1];
							auto rr1 = hr1->arr;
							// memset(rr1, 0, z1*n1);			
						}
						activeA.underlyingHistoryRepa.push_back(hr1);
					}
					{
						activeA.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeA.historySize,1));
					}
					{
						auto hr = std::make_unique<HistorySparseArray>();
						{
							auto z = activeA.historySize;
							hr->size = z;
							hr->capacity = 1;
							hr->arr = new std::size_t[z];		
						}		
						activeA.historySparse = std::move(hr);			
					}
				}
				activeA.name = (_model!="" ? _model : "model") + "_3_" + (m<10 ? "0" : "") + std::to_string(m);
				activeA.logging = level3Logging;
				activeA.summary = level3Summary;
				activeA.underlyingEventsRepa.push_back(_events);
				_events->references++;
				{
					auto& activeB = *_level2.back();
					activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
					activeB.eventsSparse->references++;
				}
				// activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);	
				for (auto i : under[m])
					activeA.frameUnderlyings.insert(i);
				for (auto i : self[m])
					activeA.frameHistorys.insert(i);
				std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;
				if (sizeA)
				{
					LOG activeA.name << "\tfuds cardinality: " << activeA.decomp->fuds.size() << "\tmodel cardinality: " << activeA.decomp->fudRepasSize << "\tactive size: " << sizeA << "\tfuds per threshold: " << (double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA UNLOG				
				}			
				if (!induceNot)
					_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), std::ref(_induceParameters), induceThresholdInitialLevel3, induceInterval));			
			}			
		}
		_threads.push_back(std::thread(run_act, std::ref(*this)));	
	}
	
	if ((_struct=="struct001" || _struct=="struct003") && (_mode=="mode008" || _mode=="mode009"))
	{	
		std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
		auto nloc = locations.size();
		auto bloc = nloc - 6;
		auto& activeA = *_level2.front();	
		std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
		auto& hs = *activeA.historySparse;
		auto over = activeA.historyOverflow;
		auto& mm = _ur->mapVarSize();
		auto& mvv = hr->mapVarInt();
		auto location = mvv[mm[Variable("location")]];
		auto n = hr->dimension;
		auto z = hr->size;
		auto y = activeA.historyEvent;
		auto rr = hr->arr;	
		auto rs = hs.arr;
		auto sliceCount = activeA.historySlicesSetEvent.size();
		_slicesSliceSetNext.reserve(sliceCount*nloc);
		SizeUSet setSliceLocA;
		setSliceLocA.reserve(sliceCount*nloc);
		{
			auto j = over ? y : z;	
			auto sliceLocB = rs[j%z]*nloc + rr[(j%z)*n+location];
			j++;
			while (j < y+z)
			{
				setSliceLocA.insert(sliceLocB);
				auto sliceLocC = rs[j%z]*nloc + rr[(j%z)*n+location];
				if (sliceLocC != sliceLocB)
				{
					if (!activeA.continousIs || !activeA.continousHistoryEventsEvent.count(j%z))
						_slicesSliceSetNext[sliceLocB].insert(sliceLocC);
					sliceLocB = sliceLocC;
				}
				j++;
			}					
		}
		if (_configDeviationMax > 0.0)
		{
			SizeUSet setSliceLocB;
			setSliceLocB.reserve(setSliceLocA.size());
			for (auto sliceLocB : setSliceLocA)
			{
				auto locB = sliceLocB % nloc;
				RecordList recordStandards;
				for (auto ev : activeA.historySlicesSetEvent[sliceLocB/nloc])
					if (rr[ev*n+location] == locB)
						recordStandards.push_back(eventsRecord(ev).standard());
				if (recordsDeviation(recordStandards) <= _configDeviationMax)
					setSliceLocB.insert(sliceLocB);
			}
			if (setSliceLocB.size() < setSliceLocA.size())
			{
				std::unordered_map<std::size_t, Alignment::SizeSet> slicesSliceSetNextB;
				slicesSliceSetNextB.reserve(_slicesSliceSetNext.size());
				for (auto& p : _slicesSliceSetNext)
					if (setSliceLocB.count(p.first))
						for (auto sliceLocC : p.second)						
							if (setSliceLocB.count(sliceLocC))
								slicesSliceSetNextB[p.first].insert(sliceLocC);
				EVAL(_slicesSliceSetNext.size());
				EVAL(slicesSliceSetNextB.size());
				_slicesSliceSetNext = slicesSliceSetNextB;
			}
		}
		{
			std::unordered_map<std::size_t, Alignment::SizeSet> slicesSliceSetPrev;
			slicesSliceSetPrev.reserve(sliceCount);
			for (auto& p : _slicesSliceSetNext)
				for (auto& q : p.second)
					slicesSliceSetPrev[q].insert(p.first);
			for (auto locA = bloc; locA < nloc; locA++)
			{
				auto& slicesStepCount = _locationsSlicesStepCount[locA];
				slicesStepCount.reserve(sliceCount);
				SizeUSet sliceCurrents;
				sliceCurrents.reserve(sliceCount);		
				std::size_t steps = 0;				
				for (auto& p : slicesSliceSetPrev)
					if (p.first % nloc == locA)
					{
						slicesStepCount.insert_or_assign(p.first,steps);
						sliceCurrents.insert(p.first);
					}
				while (sliceCurrents.size())
				{
					steps++;
					SizeList sliceCurrentBs;
					sliceCurrentBs.reserve(sliceCurrents.size() * 20);
					for (auto sliceLocC : sliceCurrents)		
					{
						for (auto sliceLocD : slicesSliceSetPrev[sliceLocC])
						{					
							if (!slicesStepCount.count(sliceLocD))
							{
								sliceCurrentBs.push_back(sliceLocD);			
								slicesStepCount.insert_or_assign(sliceLocD,steps);
							}
						}
					}	
					sliceCurrents.clear();
					sliceCurrents.insert(sliceCurrentBs.begin(), sliceCurrentBs.end());
				}				
			}
		}
	}
	
	{
	_publisherCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)));
	
	_subscriptionScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Actor::callbackScan, this, std::placeholders::_1));
	_subscriptionOdom = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Actor::callbackOdom, this, std::placeholders::_1));
	_subscriptionGoal = this->create_subscription<std_msgs::msg::String>(
		"goal", 10, std::bind(&Actor::callbackGoal, this, std::placeholders::_1));

	_timerUpdate = this->create_wall_timer(updateInterval, std::bind(&Actor::callbackUpdate, this));
	}

	LOG "actor\tSTART" UNLOG
}

Actor::~Actor()
{
	_terminate = true;
	if (_model.size())
	{
		try
		{
			std::ofstream out(_model + ".rec", std::ios::binary);
			recordListsPersistent(*_records, out); 
			out.close();
			LOG "actor\tdump\tfile name:" << _model + ".rec" UNLOG
		}
		catch (const exception&)
		{
			LOG "actor\terror: failed to write records file" <<_model + ".rec" UNLOG
		}			
	}
	if (_system && (_struct=="struct001" || _struct=="struct002" || _struct=="struct003"))
	{
		for (auto activeA : _level1)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto activeA : _level2)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto activeA : _level3)
		{
			activeA->terminate = true;
			if ( _model!="")
			{
				ActiveIOParameters ppio;
				ppio.filename = activeA->name+".ac";
				activeA->logging = true;
				activeA->dump(ppio);		
			}			
		}	
		for (auto& t : _threads)
			t.join();	
	}
	LOG "actor\tTERMINATE" UNLOG
	}

void Actor::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	if (_status != CRASH)
	{
		if (msg->pose.pose.position.z >= 0.02)
		{
			_status = CRASH;
			_terminate = true;
			LOG "actor\tCRASH\ttime " << ((Sec)(Clock::now() - _startTimestamp)).count() << "s" UNLOG			
			_statusTimestamp = Clock::now();
		}
	}
	if (_status != CRASH)
	{	
		if (_poseTimestamp != TimePoint())
		{
			_poseTimestampPrevious = _poseTimestamp;
			_posePrevious = _pose;
		}
		_pose[0] = msg->pose.pose.position.x;
		_pose[1] = msg->pose.pose.position.y;
		_pose[2] = msg->pose.pose.position.z;
		_pose[3] = msg->pose.pose.orientation.x;
		_pose[4] = msg->pose.pose.orientation.y;
		_pose[5] = msg->pose.pose.orientation.z;
		_pose[6] = msg->pose.pose.orientation.w;
		_poseTimestamp = Clock::now();
	}
}

void Actor::callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	if (_status != CRASH)
	{
		if (_scanTimestamp != TimePoint())
		{
			_scanTimestampPrevious = _scanTimestamp;
		}
		for (std::size_t i = 0; i < _scan.size(); i++)
		{
			if (std::isinf(msg->ranges.at(i)))
				_scan[i] = msg->range_max;
			else
				_scan[i] = msg->ranges.at(i);
		}
		_scanTimestamp = Clock::now();
	}
}

void Actor::callbackUpdate()
{
	if (_status == CRASH || _status == STOP)
		return;
	if (_status == START
		&& _poseTimestamp != TimePoint()
		&& _scanTimestamp != TimePoint())
	{
		_status = WAIT_ODOM;
		if (_updateLogging)
		{
			LOG "actor\t" << "WAIT_ODOM" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
		}	
		_statusTimestamp = Clock::now();		
	}
	if (_status == WAIT_ODOM 
		&& _poseTimestampPrevious != TimePoint()
		&& _poseTimestamp != TimePoint())
	{
		double distance = 0.0;
		double x2 = 0.0;
		double y2 = 0.0;
		{
			double x1 = _posePrevious[0];
			double y1 = _posePrevious[1];
			x2 = _pose[0];
			y2 = _pose[1];
			distance = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));			
		}
		double angle = 0.0;
		double yaw2 = 0.0;
		{			
			double yaw1 = 0.0;		
			{
				tf2::Quaternion q(
					_posePrevious[3],
					_posePrevious[4],
					_posePrevious[5],
					_posePrevious[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw1);	
				yaw1 *= RAD2DEG;
			}
			yaw2 = 0.0;		
			{
				tf2::Quaternion q(
					_pose[3],
					_pose[4],
					_pose[5],
					_pose[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw2);
				yaw2 *= RAD2DEG;			
			}
			angle = yaw2 - yaw1;
			if (angle >= 180.0)
				angle -= 360.0;
			else if (angle <= -180.0)
				angle += 360.0;
		}
		if (distance <= _linearStopMaximum && std::fabs(angle) <= _angularStopMaximum)
		{
			_poseStop = _pose;
			_poseStopTimestamp = _poseTimestampPrevious;
			_status = WAIT_SCAN;
			if (_updateLogging)
			{
				LOG "actor\t" << "WAIT_SCAN" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" << "\tx: " << x2 << "\ty: " << y2 << "\tyaw: " << yaw2 UNLOG	
			}			
			_statusTimestamp = Clock::now();
		}
		else
		{
			_publisherCmdVel->publish(geometry_msgs::msg::Twist());			
		}
	}
	if (_status == WAIT_SCAN 
		&& _poseStopTimestamp != TimePoint() 
		&& _scanTimestampPrevious != TimePoint() 
		&& _scanTimestampPrevious > _poseStopTimestamp)
	{
		_status = STOP;
		if (_updateLogging)
		{
			LOG "actor\t" << "STOP" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
		}	
		_statusTimestamp = Clock::now();
	}
	if (_status == AHEAD)
	{
		double distance = 0.0;
		{
			double x1 = _poseStop[0];
			double y1 = _poseStop[1];
			double x2 = _pose[0];
			double y2 = _pose[1];
			distance = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));			
		}
		if (distance >= _linearMaximum)
		{
			_publisherCmdVel->publish(geometry_msgs::msg::Twist());
			_status = WAIT_ODOM;
			if (_updateLogging)
			{
				LOG "actor\t" << "WAIT_ODOM" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" << "\tdistance: " << distance UNLOG	
			}			
			_statusTimestamp = Clock::now();
		}
		else
		{
			geometry_msgs::msg::Twist twist;
			twist.linear.x  = _linearVelocity;
			_publisherCmdVel->publish(twist);		
		}
	}	
	else if (_status == LEFT)
	{
		double angle = 0.0;
		{			
			double yaw1 = 0.0;		
			{
				tf2::Quaternion q(
					_poseStop[3],
					_poseStop[4],
					_poseStop[5],
					_poseStop[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw1);	
				yaw1 *= RAD2DEG;
			}
			double yaw2 = 0.0;		
			{
				tf2::Quaternion q(
					_pose[3],
					_pose[4],
					_pose[5],
					_pose[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw2);
				yaw2 *= RAD2DEG;			
			}
			angle = yaw2 - yaw1;
			if (angle >= 180.0)
				angle -= 360.0;
			else if (angle <= -180.0)
				angle += 360.0;
		}
		if (angle >= (_angularMaximum - _angularMaximumLag))
		{
			_publisherCmdVel->publish(geometry_msgs::msg::Twist());
			_status = WAIT_ODOM;
			if (_updateLogging)
			{
				LOG "actor\t" << "WAIT_ODOM" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" << "\tangle: " << angle UNLOG	
			}			
			_statusTimestamp = Clock::now();
		}
		else
		{
			geometry_msgs::msg::Twist twist;
			twist.angular.z  = _angularVelocity * DEG2RAD;
			_publisherCmdVel->publish(twist);		
		}
	}	
	else if (_status == RIGHT)
	{
		double angle = 0.0;
		{			
			double yaw1 = 0.0;		
			{
				tf2::Quaternion q(
					_poseStop[3],
					_poseStop[4],
					_poseStop[5],
					_poseStop[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw1);	
				yaw1 *= RAD2DEG;
			}
			double yaw2 = 0.0;		
			{
				tf2::Quaternion q(
					_pose[3],
					_pose[4],
					_pose[5],
					_pose[6]);
				tf2::Matrix3x3 m(q);
				double roll, pitch;
				m.getRPY(roll, pitch, yaw2);
				yaw2 *= RAD2DEG;			
			}
			angle = yaw2 - yaw1;
			if (angle >= 180.0)
				angle -= 360.0;
			else if (angle <= -180.0)
				angle += 360.0;
		}
		if (angle <= -1.0 * (_angularMaximum - _angularMaximumLag))
		{
			_publisherCmdVel->publish(geometry_msgs::msg::Twist());
			_status = WAIT_ODOM;
			if (_updateLogging)
			{
				LOG "actor\t" << "WAIT_ODOM" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - _statusTimestamp)).count() << std::defaultfloat << "s" << "\tangle: " << angle UNLOG	
			}		
			_statusTimestamp = Clock::now();
		}
		else
		{
			geometry_msgs::msg::Twist twist;
			twist.angular.z  = -1.0 * _angularVelocity * DEG2RAD;
			_publisherCmdVel->publish(twist);		
		}
	}	
}

void Actor::callbackGoal(const std_msgs::msg::String::SharedPtr msg)
{
	if (msg->data.substr(0,4)=="room")
		_goal = msg->data;
	else
		_actionManual = msg->data;
	LOG "actor\tgoal:" << msg->data UNLOG	
}

TBOT03::Record Actor::eventsRecord(std::size_t ev)
{
	TBOT03::Record record;
	if (_records)
	{
		record = (*_records)[ev];
		if (_level2.size() && _level2.front()->continousHistoryEventsEvent.size())
		{
			auto& activeA = *_level2.front();			
			auto& discont = activeA.continousHistoryEventsEvent;
			if (activeA.historyOverflow)
			{
				auto z = activeA.historySize;
				auto y = activeA.historyEvent;
				auto j = ev + (ev >= y ? 0 : z);	
				SizeSizeMap discont;
				for (auto& pp : activeA.continousHistoryEventsEvent)
					discont.insert_or_assign(pp.first + (pp.first >= y ? 0 : z), pp.second);
				for (auto it = discont.rbegin(); it != discont.rend(); it++)
					if (it->first <= j)
					{
						record = (*_records)[j - it->first + it->second];
						break;
					}
			}
			else
			{
				for (auto it = discont.rbegin(); it != discont.rend(); it++)
					if (it->first <= ev)
					{
						record = (*_records)[ev - it->first + it->second];
						break;
					}
			}
		}		
	}

	return record;
}

int main(int argc, char** argv)
{
	std::string args_filename = string(argc > 1 ? argv[1] : "");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(args_filename));
	rclcpp::shutdown();

	return 0;
}

