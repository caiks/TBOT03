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
	return 0;
}
