#include "dev.h"

#include <cmath>
#include <cstring>

using namespace Alignment;
using namespace TBOT03;
using namespace std;

typedef std::chrono::duration<double> sec; 
typedef std::chrono::high_resolution_clock clk;

#define ECHO(x) cout << #x << endl; x
#define EVAL(x) cout << #x << ": " << (x) << endl
#define EVALL(x) cout << #x << ": " << endl << (x) << endl
#define TRUTH(x) cout << #x << ": " << ((x) ? "true" : "false") << endl

Record eventsRecord(const Alignment::Active& activeA, std::shared_ptr<TBOT03::RecordList> records, std::size_t ev)
{
	TBOT03::Record record;
	if (records)
	{
		record = (*records)[ev];
		if (activeA.continousHistoryEventsEvent.size())
		{
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
						record = (*records)[j - it->first + it->second];
						break;
					}
			}
			else
			{
				for (auto it = discont.rbegin(); it != discont.rend(); it++)
					if (it->first <= ev)
					{
						record = (*records)[ev - it->first + it->second];
						break;
					}
			}
		}		
	}

	return record;
}

int main(int argc, char **argv)
{
	if (argc >= 3 && string(argv[1]) == "view_records")
	{
		bool ok = true;
		string model = string(argv[2]);
	
		EVAL(model);
		
		std::ifstream in(model + ".rec", std::ios::binary);
		TRUTH(in.is_open());
		ECHO(std::shared_ptr<TBOT03::RecordList> records = std::move(persistentsRecordList(in)));
		EVAL(*records);
	}
	
	if (argc >= 3 && (string(argv[1]) == "view_active" || string(argv[1]) == "view_active_concise"))
	{
		bool ok = true;
		string model = string(argv[2]);
	
		EVAL(model);
		
		bool concise = string(argv[1]) == "view_active_concise";
		TRUTH(concise);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		if (ok) 
		{
			SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, std::array<double,7>(), std::array<double,360>());	
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		Active activeA;
		activeA.logging = true;		
		if (ok) 
		{
			ActiveIOParameters ppio;
			ppio.filename = model +".ac";
			ok = ok && activeA.load(ppio);
			TRUTH(ok);				
		}		
		if (ok)
		{
			EVAL(activeA.name);				
			EVAL(activeA.underlyingEventUpdateds);		
			std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;				
			EVAL(activeA.historySize);				
			TRUTH(activeA.historyOverflow);				
			EVAL(activeA.historyEvent);				
			EVAL(sizeA);
			EVAL(activeA.historyEvent);		
			TRUTH(activeA.continousIs);				
			EVAL(activeA.continousHistoryEventsEvent);				
			for (auto& hr : activeA.underlyingHistoryRepa)
			{
				EVAL(hr->dimension);				
				EVAL(hr->size);				
				// EVAL(*hr);				
			}
			for (auto& hr : activeA.underlyingHistorySparse)
			{
				EVAL(hr->size);				
				// EVAL(*hr);				
			}			
			if (!concise)
			{
				EVAL(sorted(activeA.underlyingSlicesParent));				
			}
			else 
			{
				EVAL(activeA.underlyingSlicesParent.size());				
			}			
			EVAL(activeA.bits);				
			EVAL(activeA.var);				
			EVAL(activeA.varSlice);				
			EVAL(activeA.induceThreshold);				
			EVAL(activeA.induceVarExclusions);				
			if (activeA.historySparse) {EVAL(activeA.historySparse->size);}
			if (!concise)
			{
				if (activeA.historySparse) {EVAL(*activeA.historySparse);}				
				EVAL(activeA.historySlicesSetEvent);			
			}	
			else 
			{
				EVAL(activeA.underlyingSlicesParent.size());				
			}			
			EVAL(activeA.historySlicesSetEvent.size());				
			EVAL(activeA.induceSlices);				
			EVAL(activeA.induceSliceFailsSize);				
			EVAL(activeA.frameUnderlyings);				
			EVAL(activeA.frameHistorys);				
			EVAL(activeA.framesVarsOffset);				
			if (activeA.decomp) {EVAL(activeA.decomp->fuds.size());}
			if (activeA.decomp) {EVAL(activeA.decomp->fudRepasSize);}
			if (activeA.decomp) {EVAL((double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA);}
			if (!concise)
			{
				if (activeA.decomp) {EVAL(*activeA.decomp);}			
			}	
			else 
			{
				TRUTH(activeA.decomp);				
			}
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "location_entropy")
	{
		bool ok = true;
		string model = string(argv[2]);
	
		EVAL(model);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		if (ok) 
		{
			SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, std::array<double,7>(), std::array<double,360>());	
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		Active activeA;
		activeA.logging = true;		
		if (ok) 
		{
			ActiveIOParameters ppio;
			ppio.filename = model +".ac";
			ok = ok && activeA.load(ppio);			
		}		
		std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;		
		if (ok)
		{

			TRUTH(activeA.historyOverflow);
			EVAL(sizeA);
			EVAL(activeA.decomp->fuds.size());
			EVAL(activeA.decomp->fudRepasSize);
			EVAL((double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA);
		}
		
		ok = ok && (activeA.historyOverflow	|| activeA.historyEvent);
		if (ok)
		{
			std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
			auto over = activeA.historyOverflow;
			auto& mm = ur->mapVarSize();
			auto& mvv = hr->mapVarInt();
			auto location = mvv[mm[Variable("location")]];
			auto n = hr->dimension;
			auto z = hr->size;
			auto rr = hr->arr;	
			auto entropyA = 0.0;
			for (auto& p : activeA.historySlicesSetEvent)
			{
				std::map<std::size_t, double> locsCount;
				for (auto ev : p.second)
					locsCount[rr[ev*n+location]] += 1.0 / p.second.size();	
				for (auto q : locsCount)
					entropyA -= (double)p.second.size() * q.second * std::log(q.second);
			}		
			EVAL(entropyA);
			EVAL(entropyA/sizeA);
			EVAL((double)sizeA * std::log(sizeA));
			EVAL(std::log(sizeA));
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "configuration_deviation")
	{
		bool ok = true;
		string model = string(argv[2]);
	
		EVAL(model);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		if (ok) 
		{
			SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, std::array<double,7>(), std::array<double,360>());	
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			ok = ok && uu && ur && hr;
		}

		Active activeA;
		activeA.logging = true;		
		if (ok) 
		{
			ActiveIOParameters ppio;
			ppio.filename = model +"_2.ac";
			ok = ok && activeA.load(ppio);			
		}		
		std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;		
		if (ok)
		{

			TRUTH(activeA.historyOverflow);
			EVAL(sizeA);
			EVAL(activeA.decomp->fuds.size());
			EVAL(activeA.decomp->fudRepasSize);
			EVAL((double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA);
		}
		
		std::shared_ptr<TBOT03::RecordList> records;
		if (ok) 
		{
			try
			{
				std::ifstream in(model + ".rec", std::ios::binary);
				records = std::move(persistentsRecordList(in));
				ok = ok && records;			
			}
			catch (const exception&)
			{
				ok = false;
			}
		}		
		if (ok)
		{
			EVAL(records->size());
		}
	
		std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6" };
		auto nloc = locations.size();
		std::map<std::size_t, std::vector<Record>> slicesRecords;
		ok = ok && !activeA.historyOverflow && activeA.continousHistoryEventsEvent.size();
		if (ok)
		{
			std::shared_ptr<HistoryRepa> hr = activeA.underlyingHistoryRepa.front();
			auto& mm = ur->mapVarSize();
			auto& mvv = hr->mapVarInt();
			auto location = mvv[mm[Variable("location")]];
			auto n = hr->dimension;
			auto z = hr->size;
			auto rr = hr->arr;	
			auto& discont = activeA.continousHistoryEventsEvent;
			for (auto& p : activeA.historySlicesSetEvent)
			{
				for (auto ev : p.second)
				{
					auto sliceLocA = p.first*nloc + rr[ev*n+location];	
					for (auto it = discont.rbegin(); it != discont.rend(); it++)
						if (it->first <= ev)
						{
							slicesRecords[sliceLocA].push_back(((*records)[ev - it->first + it->second]).standard());
							break;
						}
				}
			}		
			EVAL(slicesRecords.size());
		}
		
		if (ok)
		{
			std::set<std::size_t> sizes;
			for (auto& p : slicesRecords)
				sizes.insert(p.second.size());	
			EVAL(sizes);
		}
		
		std::map<std::size_t, Record> slicesMean;
		if (ok)
		{
			for (auto& p : slicesRecords)
			{
				Record recordA;
				for (auto recordB : p.second)
					recordA += recordB;
				recordA /= p.second.size();
				slicesMean.insert_or_assign(p.first,recordA);
			}		
			EVAL(slicesMean.size());
		}
		
		std::map<std::size_t, double> slicesDeviations;
		if (ok)
		{
			for (auto& p : slicesRecords)
			{
				double variance = 0.0;
				Record mean = slicesMean[p.first];
				for (auto recordB : p.second)
					variance += mean.squared(recordB);
				variance /= p.second.size();
				slicesDeviations.insert_or_assign(p.first,std::sqrt(variance));
			}		
			EVAL(slicesDeviations.size());
		}
		if (ok)
		{
			std::set<double> deviations;
			for (auto& dev : slicesDeviations)
				deviations.insert(dev.second);	
			EVAL(deviations);
		}
		if (ok)
		{
			std::map<std::size_t, std::set<std::size_t>> sizesSlices;
			std::map<std::size_t, std::vector<double>> sizesDeviations;
			for (auto& dev : slicesDeviations)
			{
				sizesSlices[slicesRecords[dev.first].size()].insert(dev.first);	
				sizesDeviations[slicesRecords[dev.first].size()].push_back(dev.second);	
			}
			for (auto& dev : sizesDeviations)
			{
				std::sort(dev.second.begin(),dev.second.end());
				cout << dev.first << ": " << sizesSlices[dev.first].size() << " " << dev.second.front() << "," << dev.second[dev.second.size()/10] << "," << dev.second[dev.second.size()/4] << "," << dev.second[dev.second.size()/2] << "," << dev.second.back() << endl;
			}
		}
		if (ok)
		{
			std::map<double,std::set<std::size_t>> bucketsSlices;
			for (auto& dev : slicesDeviations)
				bucketsSlices[std::round(dev.second*100.0)/100.0].insert(dev.first);	
			std::map<double,std::size_t> bucketsSizes;
			for (auto& p : bucketsSlices)
				bucketsSizes[p.first] = p.second.size();
			EVAL(bucketsSizes);
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "configuration_deviation_all")
	{
		bool ok = true;
		string model = string(argv[2]);
	
		EVAL(model);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		if (ok) 
		{
			SystemHistoryRepaTuple xx = posesScansHistoryRepa(8, std::array<double,7>(), std::array<double,360>());	
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			ok = ok && uu && ur && hr;
		}

		Active activeA;
		activeA.logging = true;		
		if (ok) 
		{
			ActiveIOParameters ppio;
			ppio.filename = model +"_2.ac";
			ok = ok && activeA.load(ppio);			
		}		
		std::size_t sizeA = activeA.historyOverflow ? activeA.historySize : activeA.historyEvent;		
		if (ok)
		{

			TRUTH(activeA.historyOverflow);
			EVAL(sizeA);
			EVAL(activeA.decomp->fuds.size());
			EVAL(activeA.decomp->fudRepasSize);
			EVAL((double)activeA.decomp->fuds.size() * activeA.induceThreshold / sizeA);
		}
		
		std::shared_ptr<TBOT03::RecordList> records;
		if (ok) 
		{
			try
			{
				std::ifstream in(model + ".rec", std::ios::binary);
				records = std::move(persistentsRecordList(in));
				ok = ok && records;			
			}
			catch (const exception&)
			{
				ok = false;
			}
			if (ok)
			{
				EVAL(records->size());
			}		
		}		
	
		if (ok)
		{
			double variance = 0.0;
			std::size_t count = 0;
			for (auto& p : activeA.historySlicesSetEvent)
			{
				RecordList recordStandards;
				RecordList recordFlipStandards;
				for (auto ev : p.second)
				{
					recordStandards.push_back(eventsRecord(activeA,records,ev).standard());										
					recordFlipStandards.push_back(eventsRecord(activeA,records,ev).flip().standard());					
				}
				count += recordStandards.size();
				auto dev = recordsDeviation(recordStandards);
				auto devFlip = recordsDeviation(recordFlipStandards);
				if (dev <= devFlip)
					variance += dev*dev*recordStandards.size();
				else
					variance += devFlip*devFlip*recordFlipStandards.size();
			}		
			variance /= count;
			EVAL(count);
			EVAL(std::sqrt(variance));
		}
	}
	
	return 0;
}
