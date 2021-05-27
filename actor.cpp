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
	
void run_induce(Actor& actor, Active& active, std::chrono::milliseconds induceInterval, std::size_t induceThresholdInitial)
{
	while (!actor._terminate && !active.terminate)
	{
		if (actor._poseTimestamp != TimePoint() && actor._scanTimestamp != TimePoint() && actor._eventId >= induceThresholdInitial)
			active.induce(actor._induceParametersLevel1);
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
		if (actor._status == actor.STOP && actor._system)
		{
			{
				std::unique_ptr<HistoryRepa> hr;
				{
					SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, actor._pose, actor._scan);
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
				if (actor._actLogging)
				{
					LOG "actor\tevent id: " << actor._eventId << "\ttime " << ((Sec)(Clock::now() - mark)).count() << "s" UNLOG								
				}		
			}	
			if (actor._mode=="mode001")
			{
				actor._status = Actor::AHEAD;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "AHEAD" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode002")
			{
				actor._status = Actor::LEFT;
				if (actor._updateLogging)
				{
					LOG "actor\t" << "LEFT" << "\ttime " << std::fixed << std::setprecision(3) << ((Sec)(Clock::now() - actor._statusTimestamp)).count() << std::defaultfloat << "s" UNLOG	
				}			
				actor._statusTimestamp = Clock::now();			
			}
			else if (actor._mode=="mode003")
			{
				actor._status = Actor::RIGHT;
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
	_actWarning = ARGS_BOOL(warning_action);
	std::chrono::milliseconds updateInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(update_interval,10));
	_linearStopMaximum = ARGS_DOUBLE_DEF(linear_stop,0.02);
	_linearMaximum = ARGS_DOUBLE_DEF(linear_maximum,0.5);
	_linearVelocity = ARGS_DOUBLE_DEF(linear_velocity,0.3);
	_angularStopMaximum = ARGS_DOUBLE_DEF(angular_stop, 1.0);
	_angularMaximum = ARGS_DOUBLE_DEF(angular_maximum, 30.0 - 7.0);
	_angularVelocity = ARGS_DOUBLE_DEF(angular_velocity, 1.5 * RAD2DEG);
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
	std::chrono::milliseconds induceInterval = (std::chrono::milliseconds)(ARGS_INT_DEF(induce_interval,10));	
	bool level3Logging = ARGS_BOOL(logging_level3);
	bool level3Summary = ARGS_BOOL(summary_level3);
	bool induceNot = ARGS_BOOL(no_induce);
	_mode = ARGS_STRING_DEF(mode, "mode001");	
	_modeLogging = ARGS_BOOL(mode_logging);	
	srand(ARGS_INT_DEF(mode_seed,7));
	_distribution[LEFT] = ARGS_DOUBLE(distribution_LEFT);
	_distribution[AHEAD] = ARGS_DOUBLE(distribution_AHEAD);
	_distribution[RIGHT] = ARGS_DOUBLE(distribution_RIGHT);
	{
		double norm = 0.0;
		for (auto& p : _distribution)
			norm += p.second;	
		for (auto& p : _distribution)
			_distribution[p.first] /= norm;	
	}
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
	
	if (_model.size() && modelInitial.size())
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

	if (_struct=="struct001" || _struct=="struct002")
	{
		std::unique_ptr<HistoryRepa> hr;
		{
			SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, _pose, _scan);	
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
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceIntervalLevel1, induceThresholdInitialLevel1));
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
				_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceInterval, induceThresholdInitial));			
		}
		if (_struct=="struct002")
		{
			std::vector<SizeList> under 
			{
				SizeList{0,1,3,6,10,15,21,28,36}, 
				SizeList{0,1,3}, 
				SizeList{0,1,3,6,10}, 
				SizeList{0,1,3,6,10,15,21,28,36}, 
				SizeList{0,2,4,8,16,32}, 
				SizeList{0,2,4,8,16,32}
			};
			std::vector<SizeList> self 
			{
				SizeList{}, 
				SizeList{6,10,15}, 
				SizeList{16,32,64}, 
				SizeList{4,8,16,32,64}, 
				SizeList{}, 
				SizeList{16,32,64}
			};			
			for (std::size_t m = 0; m < under.size(); m++)
			{
				_level3.push_back(std::make_shared<Active>());
				auto& activeA = *_level3.back();
				activeA.log = actor_log;
				activeA.layerer_log = layerer_actor_log;
				activeA.system = _system;
				activeA.continousIs = true;
				if (modelInitial.size() && structInitial != "struct001")
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
					_threads.push_back(std::thread(run_induce, std::ref(*this), std::ref(activeA), induceInterval, induceThresholdInitial));			
			}			
		}
		_threads.push_back(std::thread(run_act, std::ref(*this)));	
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
	if (_system && (_struct=="struct001" || _struct=="struct002"))
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
			LOG "actor\tCRASH\ttime " << ((Sec)(Clock::now() - _statusTimestamp)).count() << "s" UNLOG			
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
		if (angle >= _angularMaximum)
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
		if (angle <= -1.0 * _angularMaximum)
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
	_goal = msg->data;
	LOG "actor\tgoal:" << _goal UNLOG	
}

int main(int argc, char** argv)
{
	std::string args_filename = string(argc > 1 ? argv[1] : "");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(args_filename));
	rclcpp::shutdown();

	return 0;
}

