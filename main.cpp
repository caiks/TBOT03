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
	if (argc >= 3 && string(argv[1]) == "fud_region")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto layerer = parametersSystemsLayererMaxRollByMExcludedSelfHighestIORepa_up;
		
		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 5; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		SizeList vv;		
		{
			auto vvk = *uvars(*uu);
			auto& vvi = ur->mapVarSize();
			auto vvk0 = sorted(vvk);
			for (auto& v : vvk0)
				vv.push_back(vvi[v]);			
		}
		EVAL(vv.size());

		std::unique_ptr<HistoryRepa> hrs;
		{
			size_t seed = 5;
			hrs = hrshuffle(*hr, (unsigned int)seed);	
		}

		int d = 0;
		std::size_t f = 1;
		size_t tint = 4;
		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{
			size_t wmax = 9;
			size_t lmax = 8;
			size_t xmax = 128;
			size_t omax = 10;
			size_t bmax = 10 * 3;
			size_t mmax = 3;
			size_t umax = 128;
			size_t pmax = 1;
			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, f, *ur);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			EVAL(model+".dr");
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); cout << endl;
			applicationRepasPersistent(*dr, out); cout << endl;
			out.close();
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_region")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data009");
		size_t scale = argc >= 6 ? atoi(argv[5]) : 10;
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{	
			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < scale; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
	
	if (argc >= 3 && string(argv[1]) == "fud") 
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto layerer = parametersSystemsLayererMaxRollByMExcludedSelfHighestLogIORepa_up;
		
		auto log = [](Active& active, const std::string& str)
		{
			std::cout << str << std::endl;
			return;
		};
		
		auto layerer_log = [](const std::string& str)
		{
			std::cout << str << std::endl;
			return;
		};

		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		auto dr0 = std::make_unique<ApplicationRepa>();
		{
			auto dr = std::make_unique<ApplicationRepa>();
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr->slices = std::make_shared<SizeTree>();
			dr->slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr->fud = std::make_shared<FudRepa>();
			dr->fud->layers.reserve(dr1->fud->layers.size());
			dr->substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));			
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr->slices->_list.insert(dr->slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr->fud->layers.size())
						dr->fud->layers[l].insert(dr->fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr->fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr->substrate.insert(dr->substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
			dr0 = std::move(dr);
		}

		SizeList vv = *treesElements(*dr0->slices);		
		EVAL(vv.size());
		
		size_t tint = 4;		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * 32;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t mult = 1;
		size_t seed = 5;
	
		std::unique_ptr<HistoryRepa> hrs;
		{
			std::unique_ptr<HistoryRepa> hr1 = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *dr0->fud));
			auto z = hr->size;			
			auto nmax = (std::size_t)std::sqrt(znnmax / (double)(z + mult*z));	
			nmax = std::max(nmax, bmax);	
			EVAL(nmax);				
			if (nmax < vv.size())
			{
				auto ee = prents(*hrpr(vv.size(), vv.data(), *hr1));
				auto m = ee->size();
				SizeList vv0;		
				if (m > nmax)
				{
					std::sort(ee->begin(), ee->end());
					vv0.reserve(nmax);
					for (std::size_t i = m - nmax; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				else
				{
					vv0.reserve(m);
					for (std::size_t i = 0; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				if (vv0.size() < vv.size())
				{
					vv = vv0;
					SizeUSet vv00(hr->dimension);
					for (std::size_t i = 0; i < hr->dimension; i++)
						vv00.insert(hr->vectorVar[i]);
					SizeUSet vv01(vv0.begin(), vv0.end());
					auto er0 = llfr(vv00, *frdep(*dr0->fud, vv01));
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *er0));
					hr = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *er0));					
				}
				else
				{
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
					hr = std::move(hr1);				
				}
			}	
			else
			{
				hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
				hr = std::move(hr1);				
			}
		}

		int d = 0;
		std::size_t f = 1;		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{

			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, f, layerer_log, *ur);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			auto dr1 = drjoin(*dr0, *dr);
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); 
			applicationRepasPersistent(*dr1, out); 
			out.close();
			EVAL(model+".dr");
		}
	}

	if (argc >= 3 && string(argv[1]) == "fud2") 
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrpr = setVarsHistoryRepasRed_u;
		auto prents = histogramRepaRedsListEntropy;
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto llfr = setVariablesListTransformRepasFudRepa_u;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto frdep = fudRepasSetVarsDepends;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto layerer = parametersLayererMaxRollByMExcludedSelfHighestLogIORepa_up;
		
		auto log = [](const std::string& str)
		{
			std::cout << str << std::endl;
			return;
		};

		string model = string(argv[2]);
		size_t size = argc >= 4 ? atoi(argv[3]) : 1000;
		bool logging = argc >= 5;
		EVAL(model);
		EVAL(size);
				
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			if (size < hr->size)
			{
				SizeList ev;
				for (size_t i = 0; i < size; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
		}
		EVAL(hr->size);
		
		auto dr0 = std::make_unique<ApplicationRepa>();
		{
			auto dr = std::make_unique<ApplicationRepa>();
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr->slices = std::make_shared<SizeTree>();
			dr->slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr->fud = std::make_shared<FudRepa>();
			dr->fud->layers.reserve(dr1->fud->layers.size());
			dr->substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));			
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr->slices->_list.insert(dr->slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr->fud->layers.size())
						dr->fud->layers[l].insert(dr->fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr->fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr->substrate.insert(dr->substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
			dr0 = std::move(dr);
		}

		SizeList vv = *treesElements(*dr0->slices);		
		EVAL(vv.size());
		
		size_t tint = 4;		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * 32;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t mult = 1;
		size_t seed = 5;
	
		std::unique_ptr<HistoryRepa> hrs;
		{
			std::unique_ptr<HistoryRepa> hr1 = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *dr0->fud));
			auto z = hr->size;			
			auto nmax = (std::size_t)std::sqrt(znnmax / (double)(z + mult*z));	
			nmax = std::max(nmax, bmax);	
			EVAL(nmax);				
			if (nmax < vv.size())
			{
				auto ee = prents(*hrpr(vv.size(), vv.data(), *hr1));
				auto m = ee->size();
				SizeList vv0;		
				if (m > nmax)
				{
					std::sort(ee->begin(), ee->end());
					vv0.reserve(nmax);
					for (std::size_t i = m - nmax; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				else
				{
					vv0.reserve(m);
					for (std::size_t i = 0; i < m; i++)
						vv0.push_back((*ee)[i].second);
				}
				if (vv0.size() < vv.size())
				{
					vv = vv0;
					SizeUSet vv00(hr->dimension);
					for (std::size_t i = 0; i < hr->dimension; i++)
						vv00.insert(hr->vectorVar[i]);
					SizeUSet vv01(vv0.begin(), vv0.end());
					auto er0 = llfr(vv00, *frdep(*dr0->fud, vv01));
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *er0));
					hr = hrhrred(vv.size(), vv.data(), *frmul(tint, *hr, *er0));					
				}
				else
				{
					hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
					hr = std::move(hr1);				
				}
			}	
			else
			{
				hrs = hrhrred(vv.size(), vv.data(), *frmul(tint, *hrshuffle(*hr, (unsigned int)seed), *dr0->fud));
				hr = std::move(hr1);				
			}
		}

		int d = 0;
		std::size_t f = 1;		
		std::unique_ptr<FudRepa> fr;
		std::unique_ptr<DoubleSizeListPairList> mm;
		try
		{
			auto urv = ur->listVarSizePair.size();

			auto t = layerer(wmax, lmax, xmax, omax, bmax, mmax, umax, pmax, tint, vv, *hr, *hrs, log, logging, urv);
			fr = std::move(std::get<0>(t));
			mm = std::move(std::get<1>(t));
		}
		catch (const std::out_of_range& e)
		{
			std::cout << "out of range exception: " << e.what() << std::endl;
			return 1;
		}
		if (!mm || !mm->size())
		{
			std::cout << "no fud" << std::endl;
			return 1;
		}
		// EVAL(mm->size());			
		// EVAL(mm->back());
		// EVAL(*mm);
		auto& a = mm->back().first;
		auto& kk = mm->back().second;
		auto m = kk.size();
		auto z = hr->size;
		EVAL(m);
		EVAL(a);			
		EVAL(z);	
		EVAL(100.0*(exp(a/z/(m-1))-1.0));
		EVAL(fudRepasSize(*fr));
		EVAL(frvars(*fr)->size());
		EVAL(frder(*fr)->size());
		EVAL(frund(*fr)->size());
		EVAL(sorted(*frund(*fr)));
		
		auto dr = std::make_unique<ApplicationRepa>();
		{
			dr->substrate = vv;
			dr->fud = std::make_shared<FudRepa>();
			dr->slices = std::make_shared<SizeTree>();	
			auto vd = std::make_shared<Variable>(d);
			auto vl = std::make_shared<Variable>("s");
			auto vf = std::make_shared<Variable>((int)f);
			auto vdf = std::make_shared<Variable>(vd, vf);
			auto vfl = std::make_shared<Variable>(vdf, vl);
			SizeUSet kk1(kk.begin(), kk.end());
			SizeUSet vv1(vv.begin(), vv.end());
			auto gr = llfr(vv1, *frdep(*fr, kk1));
			auto ar = hrred(1.0, m, kk.data(), *frmul(tint, *hr, *gr));
			SizeList sl;
			TransformRepaPtrList ll;
			std::size_t sz = 1;
			auto skk = ar->shape;
			auto rr0 = ar->arr;
			for (std::size_t i = 0; i < m; i++)
				sz *= skk[i];
			sl.reserve(sz);
			ll.reserve(sz);
			bool remainder = false;
			std::size_t b = 1;
			auto& llu = ur->listVarSizePair;
			for (std::size_t i = 0; i < sz; i++)
			{
				if (rr0[i] <= 0.0)
				{
					remainder = true;
					continue;
				}
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = 0;
				rr[i] = 1;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			if (remainder)
			{
				auto tr = std::make_shared<TransformRepa>();
				tr->dimension = m;
				tr->vectorVar = new std::size_t[m];
				auto ww = tr->vectorVar;
				tr->shape = new std::size_t[m];
				auto sh = tr->shape;
				for (std::size_t j = 0; j < m; j++)
				{
					ww[j] = kk[j];
					sh[j] = skk[j];
				}
				tr->arr = new unsigned char[sz];
				auto rr = tr->arr;
				for (std::size_t j = 0; j < sz; j++)
					rr[j] = rr0[j] <= 0.0 ? 1 : 0;
				tr->valency = 2;
				auto vb = std::make_shared<Variable>((int)b++);
				auto vflb = std::make_shared<Variable>(vfl, vb);
				llu.push_back(VarSizePair(vflb, 2));
				auto w = llu.size() - 1;
				tr->derived = w;
				sl.push_back(w);
				ll.push_back(tr);
			}
			dr->fud->layers.insert(dr->fud->layers.end(), gr->layers.begin(), gr->layers.end());
			dr->fud->layers.push_back(ll);
			dr->slices->_list.reserve(sz);
			for (auto& s : sl)
				dr->slices->_list.push_back(SizeSizeTreePair(s, std::make_shared<SizeTree>()));			
		}
		{
			auto dr1 = drjoin(*dr0, *dr);
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); 
			applicationRepasPersistent(*dr1, out); 
			out.close();
			EVAL(model+".dr");
		}
	}

	if (argc >= 3 && string(argv[1]) == "entropy")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data009");
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
			
	if (argc >= 3 && string(argv[1]) == "drmul")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto llhs = listSetIntsHistorySparse;
		auto hsll = historySparsesListSetInt;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		auto drmul = historyRepaPtrListsHistorySparseArrayPtrListsDecompFudSlicedRepasEventsPathSlice_u;
		auto drmul2 = listVarValuesDecompFudSlicedRepasPathSlice_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t size = argc >= 5 ? atoi(argv[4]) : 1;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(size);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::shared_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		ECHO(auto z = hr->size > size ? size : hr->size);
		EVAL(z);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto er = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*er->fud));
		EVAL(frder(*er->fud)->size());
		EVAL(frund(*er->fud)->size());
		EVAL(treesSize(*er->slices));
		EVAL(treesLeafElements(*er->slices)->size());
		
		auto dr = erdr(*er);
		EVAL(dr->fuds.size());
		EVAL(dr->fudRepasSize);
		{
			std::set<std::size_t> derived;
			for (auto& fs : dr->fuds)
				for (auto& tr : fs.fud)
					derived.insert(tr->derived);
			EVAL(derived.size());				
		}
		
		for (std::size_t j = 0; z <= 1000 && j < z; j++)
		{
			auto t0 = clk::now();
			std::cout << "drmul: " << j 
				<< " " << *drmul(HistoryRepaPtrList{hr},HistorySparseArrayPtrList{},*dr,j,2)
				<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
		}		
		
		for (std::size_t j = 0; z <= 1000 && j < z; j++)
		{
			auto t0 = clk::now();
			SizeUCharStructList jj;
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto z = hr->size;
				auto rr = hr->arr;	
				jj.reserve(n);
				for (std::size_t i = 0; i < n; i++)
				{
					SizeUCharStruct qq;
					qq.uchar = rr[hr->evient ? j*n + i : i*z + j];			
					if (qq.uchar)
					{
						qq.size = vv[i];
						jj.push_back(qq);
					}
				}
			}
			std::cout << "drmul2: " << j 
				<< " " << *drmul2(jj,*dr,2)
				<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
		}		
		
		if (false)
		{
			std::string filename = "test.bin";
			std::ofstream out(filename, std::ios::binary);
			ECHO(decompFudSlicedRepasPersistent(*dr, out));
			out.close();

			std::ifstream in(filename, std::ios::binary);
			ECHO(auto dr2 = persistentsDecompFudSlicedRepa(in));
			in.close();
			for (std::size_t j = 0; z <= 1000 && j < z; j++)
			{
				auto t0 = clk::now();
				std::cout << "drmul: " << j 
					<< " " << *drmul(HistoryRepaPtrList{hr},HistorySparseArrayPtrList{},*dr2,j,2)
					<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			}		
		}
		
		{
			SizeList ev;
			for (std::size_t j = 0; z <= 1000 && j < z; j++)
				ev.push_back(j);
			auto vv1 = treesLeafElements(*er->slices);
			auto hs1 = hrhs(*hrhrred(vv1->size(), vv1->data(), *frmul(*hrsel(ev.size(), ev.data(), *hr), *er->fud)));
			EVAL(*hs1);
		}
		std::shared_ptr<HistorySparseArray> ha = std::move(hsha(*hrhs(*hr)));

		for (std::size_t j = 0; z <= 1000 && j < z; j++)
		{
			auto t0 = clk::now();
			std::cout << "drmul: " << j 
				<< " " << *drmul(HistoryRepaPtrList{},HistorySparseArrayPtrList{ha},*dr,j,2)
				<< " " << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
		}	
		
		for (std::size_t m = 1; m <= 8; m++)
		{
			auto t1 = clk::now();
			for (std::size_t j = 0; j < z; j++)
			{
				SizeUCharStructList jj;
				{
					auto n = hr->dimension;
					auto vv = hr->vectorVar;
					auto z = hr->size;
					auto rr = hr->arr;	
					jj.reserve(n);
					for (std::size_t i = 0; i < n; i++)
					{
						SizeUCharStruct qq;
						qq.uchar = rr[hr->evient ? j*n + i : i*z + j];			
						if (qq.uchar)
						{
							qq.size = vv[i];
							jj.push_back(qq);
						}
					}
				}
				drmul2(jj,*dr,m);		
			}		
			std::cout << "Average " << m << " " << ((sec)(clk::now() - t1)).count()/z << "s" << std::endl;				
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "update_region")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t scale = argc >= 5 ? atoi(argv[4]) : 1;
		bool logging = argc >= 6;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(scale);

		std::unique_ptr<HistoryRepa> hr;
		{	
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;

			HistoryRepaPtrList ll;
			int s = 17;
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < scale; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		ActiveUpdateParameters pp;		
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
						
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			active.decomp = std::make_unique<DecompFudSlicedRepa>();
			EVAL(*active.decomp);
			
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;

		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			active.induceThreshold = 2;
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}			
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
				
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.logging = logging;
			active.historySize = 10;
			active.induceThreshold = 2;
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(2);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
			
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),1);
			EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			EVAL(*ev0);
			EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "update")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		
		EVAL(model);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		
		ActiveUpdateParameters pp;
		
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
						
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}

			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			active.decomp = std::make_unique<DecompFudSlicedRepa>();
			EVAL(*active.decomp);
			
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			EVAL(*active.eventsSparse);
			std::cout << endl;

		}
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			active.induceThreshold = 2;
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(1);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
				
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
		
		std::cout << endl;
		{		
			EVAL(hr->size);
			Active active;
			active.historySize = 10;
			active.induceThreshold = 2;
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = active.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				active.historySparse = std::move(hr);			
			}
			TRUTH(active.update(pp));
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				active.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
			}
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			std::cout << endl;
			
			active.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			EVAL(*active.eventsSparse);
			
			auto ev0 = std::make_shared<ActiveEventsRepa>(2);
			ev0->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,0)),ev0->references);
			// EVAL(*ev0);
			
			active.underlyingEventsRepa.push_back(ev0);
			
			auto hr0 = std::make_shared<HistoryRepa>();
			{
				auto n = hr->dimension;
				auto vv = hr->vectorVar;
				auto sh = hr->shape;
				hr0->dimension = n;
				hr0->vectorVar = new std::size_t[n];
				auto vv1 = hr0->vectorVar;
				hr0->shape = new std::size_t[n];
				auto sh1 = hr0->shape;
				for (std::size_t i = 0; i < n; i++)
				{
					vv1[i] = vv[i];
					sh1[i] = sh[i];
				}
				hr0->evient = true;
				hr0->size = active.historySize;
				auto z1 = hr0->size;
				hr0->arr = new unsigned char[z1*n];
				auto rr0 = hr0->arr;
				memset(rr0, 0, z1*n);
			}
			// EVAL(*hr0);
			active.underlyingHistoryRepa.push_back(hr0);

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 1; i < 10; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
			
			for (std::size_t i = 10; i < 11; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;
	
			for (std::size_t i = 11; i < 15; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),ev0->references);
			// EVAL(*ev0);	

			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;		
			
			for (std::size_t i = 15; i < 18; i++) 
				ev0->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),1);
			// EVAL(*ev0);	

			auto updatePost = [](Active& active, const SizeSet& eventsA, std::size_t eventA, std::size_t historyEventA, std::size_t sliceA)
			{
				std::cout << "eventsA: " << eventsA << std::endl;
				std::cout << "eventA: " << eventA << std::endl;
				std::cout << "historyEventA: " << historyEventA << std::endl;
				std::cout << "sliceA: " << sliceA << std::endl;
				return true;
			};
			
			active.updateCallback = updatePost;
			{			
				auto t0 = clk::now();
				TRUTH(active.update(pp));
				std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;
			}
			EVAL(active.underlyingEventUpdateds);
			TRUTH(active.historyOverflow);
			EVAL(active.historyEvent);
			EVAL(active.historySize);
			{if (active.historySparse) {EVAL(*active.historySparse);}}
			EVAL(active.historySlicesSetEvent);
			EVAL(active.induceSlices);
			// EVAL(*ev0);
			// EVAL(*hr0);
			EVAL(*active.eventsSparse);
			std::cout << endl;	
		}
	}

	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(8, 10, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 20; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
		}
		
		EVAL(*hr);				
		auto pr1 = hrpr(*hr);
		EVAL(*hrpr(*hr));
		{
			auto t0 = clk::now();
			ECHO(auto hrs1 = hrshuffle(*hr,17));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs1);				
			EVAL(*hrpr(*hrs1));			
		}

		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);				
			EVAL(*hrpr(*hrs2));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(8, 10, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 20; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->transpose();		}
		
		EVAL(*hr);				
		auto pr1 = hrpr(*hr);
		EVAL(*hrpr(*hr));
		{
			auto t0 = clk::now();
			ECHO(auto hrs1 = hrshuffle(*hr,17));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs1);				
			EVAL(*hrpr(*hrs1));			
		}

		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);				
			EVAL(*hrpr(*hrs2));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrhs = historyRepasHistorySparse;
		auto hshr = historySparsesHistoryRepa;
		auto hahs = historySparseArraysHistorySparse;
		auto hsha = historySparsesHistorySparseArray;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 1, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hsha(*hrhs(*hr)));
		// EVAL(*ha);
		// EVAL(*hahs(*ha));
		// EVAL(*hshr(*hahs(*ha)));
		EVAL(*hrpr(*hshr(*hahs(*ha))));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hshr(*hahs(*has2)));
			EVAL(*hrpr(*hshr(*hahs(*has2))));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrha = historyRepasHistorySparseArray;
		auto hahr = historySparseArraysHistoryRepa;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 2, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
			hr->vectorVar[1] = 200;
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		// EVAL(*hrhs(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hrha(*hr));
		EVAL(*hrpr(*hahr(*ha)));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hahr(*has2));
			EVAL(*hrpr(*hahr(*has2)));
		}
	}
	
	if (false)
	{
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrpr = historyRepasRed;
		auto hrha = historyRepasHistorySparseArray;
		auto hahr = historySparseArraysHistoryRepa;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrshuffle2 = historyRepasShuffle_us;
		auto hashuffle2 = historySparseArrayShuffle_us;
		
		std::unique_ptr<HistoryRepa> hr;
		{
			std::unique_ptr<System> uu;
			std::unique_ptr<SystemRepa> ur;
			std::ifstream in("data009.bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepaRegion(2, 2, 17, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
			
			SizeList ev;
			for (size_t i = 0; i < 200; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
			hr->vectorVar[0] = 100;
			hr->vectorVar[1] = 200;
			hr->transpose();
		}
		
		EVAL(*hr);
		EVAL(*hrpr(*hr));
		// EVAL(*hrhs(*hr));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto hrs2 = hrshuffle2(*hr,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hrs2);
			EVAL(*hrpr(*hrs2));
		}
		
		ECHO(auto ha = hrha(*hr));
		EVAL(*hrpr(*hahr(*ha)));
		
		{
			auto t0 = clk::now();
			std::ranlux48_base gen(17);
			ECHO(auto has2 = hashuffle2(*ha,gen));
			std::cout << ((sec)(clk::now() - t0)).count() << "s" << std::endl;		
			EVAL(*hahr(*has2));
			EVAL(*hrpr(*hahr(*has2)));
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "induce01")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string dataset = string(argc >= 3 ? argv[2] : "data009");
		
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		{
			auto eventsA = std::make_shared<ActiveEventsRepa>(1);
			
			Active activeA;
			activeA.system = std::make_shared<ActiveSystem>();
			activeA.var = activeA.system->next(activeA.bits);
			EVAL(activeA.var);
			activeA.varSlice = activeA.system->next(activeA.bits);
			EVAL(activeA.varSlice);
			activeA.historySize = 10;
			activeA.induceThreshold = 5;
			activeA.logging = true;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					for (int i = 330; i < 390; i++)
						vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
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
					memset(rr1, 0, z1*n1);			
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

			bool ok = true;
			
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			
			for (std::size_t i = 0; i < 2; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);	
			
			for (std::size_t i = 2; i < 6; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			EVAL(activeA.historyEvent);
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);
			EVAL(activeA.historyEvent);

			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);
			ECHO(ok = activeA.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeA.historySlicesSetEvent);
			EVAL(activeA.induceSlices);	
			
			if (false)
			{
				{if (activeA.historySparse) {{if (activeA.historySparse) {EVAL(*activeA.historySparse);}}}}
				std::string filename = "test.bin";
				std::ofstream out(filename, std::ios::binary);
				ECHO(historySparseArraysPersistentInitial(*activeA.historySparse, 3, out));
				out.close();

				std::ifstream in(filename, std::ios::binary);
				ECHO(auto hr2 = persistentInitialsHistorySparseArray(in));
				EVAL(*hr2);
				in.close();
			}
			
			if (false)
			{
				{if (activeA.historySparse) {{if (activeA.historySparse) {EVAL(*activeA.historySparse);}}}}
				std::string filename = "test.bin";
				std::ofstream out(filename, std::ios::binary);
				ECHO(historySparseArraysPersistent(*activeA.historySparse,out));
				out.close();

				std::ifstream in(filename, std::ios::binary);
				ECHO(auto hr2 = persistentsHistorySparseArray(in));
				EVAL(*hr2);
				in.close();
			}
		}
		
	}
	
	if (argc >= 3 && string(argv[1]) == "induce02")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		
		EVAL(model);
		EVAL(dataset);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		{
			auto eventsA = std::make_shared<ActiveEventsRepa>(2);
			
			auto systemA = std::make_shared<ActiveSystem>();
			
			Active activeA("activeA");
			activeA.system = systemA;
			activeA.historySize = 10;
			activeA.logging = true;
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				activeA.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
				systemA->block = activeA.decomp->varMax() >> systemA->bits;
			}
			activeA.var = systemA->next(activeA.bits);
			EVAL(activeA.var);
			activeA.varSlice = systemA->next(activeA.bits);
			EVAL(activeA.varSlice);
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					for (int i = 0; i < 360; i++)
						vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
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
					memset(rr1, 0, z1*n1);			
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

			Active activeB("activeB");
			activeB.system = systemA;
			activeB.var = systemA->next(activeB.bits);
			EVAL(activeB.var);
			activeB.varSlice = systemA->next(activeB.bits);
			EVAL(activeB.varSlice);
			activeB.historySize = 10;
			activeB.induceThreshold = 5;
			activeB.logging = true;
			activeB.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeB.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					vv0.push_back(mm[Variable("motor")]);
					vv0.push_back(mm[Variable("location")]);
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
					memset(rr1, 0, z1*n1);			
				}
				activeB.underlyingHistoryRepa.push_back(hr1);
			}
			activeB.underlyingEventsSparse.push_back(activeA.eventsSparse);
			activeB.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeB.historySize,1));
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = activeB.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				activeB.historySparse = std::move(hr);			
			}
			activeB.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			
			bool ok = true;
						
			for (std::size_t i = 0; i < 2; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);

			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			for (std::size_t i = 2; i < 6; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);

			auto inducePost = [](Active& active, std::size_t sliceA, std::size_t sliceSizeA)
			{
				std::cout << "sliceA: " << sliceA << std::endl;
				std::cout << "sliceSizeA: " << sliceSizeA << std::endl;
				return true;
			};
			
			activeB.induceCallback = inducePost;
			// EVAL(sorted(activeB.underlyingSlicesParent));
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			EVAL(activeB.frameUnderlyings);
			EVAL(activeB.frameHistorys);
			EVAL(activeB.framesVarsOffset);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			if (false)
			{
				EVAL(activeA.historySlicesSetEvent);
				EVAL(activeB.induceSlices);
				ActiveInduceParameters ppi;
				ppi.znnmax = 0;
				ppi.bmax = 2;
				ECHO(ok = activeB.induce(ppi));
				TRUTH(ok);
				EVAL(activeA.historySlicesSetEvent);
				EVAL(activeB.induceSlices);	
			}
			
			if (true)
			{
				activeB.frameUnderlyings = SizeSet{1,2,3};
				activeB.frameHistorys = SizeSet{4,5};
				activeB.framesVarsOffset[1][2] = 3;
				activeB.framesVarsOffset[1][4] = 5;
				activeB.framesVarsOffset[2][6] = 7;
				ActiveIOParameters pp;
				pp.filename = "test.bin";
				ECHO(ok = activeB.dump(pp));
				TRUTH(ok);				
				Active activeC("activeC");
				activeC.logging = true;
				activeC.historyOverflow = true;
				ECHO(ok = activeC.load(pp));
				TRUTH(ok);				
				EVAL(activeB.name);				
				EVAL(activeC.name);				
				EVAL(activeB.underlyingEventUpdateds);				
				EVAL(activeC.underlyingEventUpdateds);				
				EVAL(activeB.historySize);				
				EVAL(activeC.historySize);				
				TRUTH(activeB.historyOverflow);				
				TRUTH(activeC.historyOverflow);				
				EVAL(activeB.historyEvent);				
				EVAL(activeC.historyEvent);		
				TRUTH(activeB.continousIs);				
				TRUTH(activeC.continousIs);		
				EVAL(activeB.continousHistoryEventsEvent);				
				EVAL(activeC.continousHistoryEventsEvent);					
				{
					for (auto& hr : activeB.underlyingHistoryRepa)
					{
						EVAL(*hr);				
					}
					for (auto& hr : activeC.underlyingHistoryRepa)
					{
						EVAL(*hr);				
					}
				}	
				{
					for (auto& hr : activeB.underlyingHistorySparse)
					{
						EVAL(*hr);				
					}
					for (auto& hr : activeC.underlyingHistorySparse)
					{
						EVAL(*hr);				
					}
				}				
				EVAL(sorted(activeB.underlyingSlicesParent));				
				EVAL(sorted(activeC.underlyingSlicesParent));	
				EVAL(*activeB.decomp);				
				EVAL(*activeC.decomp);				
				EVAL(activeB.bits);				
				EVAL(activeC.bits);				
				EVAL(activeB.var);				
				EVAL(activeC.var);				
				EVAL(activeB.varSlice);				
				EVAL(activeC.varSlice);				
				EVAL(activeB.induceThreshold);				
				EVAL(activeC.induceThreshold);				
				EVAL(activeB.induceVarExclusions);				
				EVAL(activeC.induceVarExclusions);				
				if (activeB.historySparse) {EVAL(*activeB.historySparse);}				
				if (activeC.historySparse) {EVAL(*activeC.historySparse);}			
				EVAL(activeB.historySlicesSetEvent);				
				EVAL(activeC.historySlicesSetEvent);				
				EVAL(activeB.induceSlices);				
				EVAL(activeC.induceSlices);				
				EVAL(activeB.induceSliceFailsSize);				
				EVAL(activeC.induceSliceFailsSize);				
				EVAL(activeB.frameUnderlyings);				
				EVAL(activeC.frameUnderlyings);				
				EVAL(activeB.frameHistorys);				
				EVAL(activeC.frameHistorys);				
				EVAL(activeB.framesVarsOffset);				
				EVAL(activeC.framesVarsOffset);				
			}

		}
	}
		
	if (false)
	{			
		ActiveSystem systemA;
		systemA.bits = 4;
		EVAL(systemA.bits);
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(6));
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(4));
		EVAL(systemA.block);
		EVAL(systemA.next(3));
		EVAL(systemA.block);
		
		EVAL(std::size_t(-1));
		ECHO(systemA.block = std::size_t(-1) >> 4);
		try 
		{
			EVAL(systemA.next(4));
		} 
		catch (const std::out_of_range& e) 
		{
			EVAL(e.what());
		}		
		ECHO(systemA.block = std::size_t(-1) >> 4);
		try 
		{
			EVAL(systemA.next(6));
		} 
		catch (const std::out_of_range& e) 
		{
			EVAL(e.what());
		}		
	}

	if (argc >= 3 && string(argv[1]) == "induce03")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		auto eventsA = std::make_shared<ActiveEventsRepa>(1);
		
		Active activeA(model);
		activeA.system = std::make_shared<ActiveSystem>();
		activeA.var = activeA.system->next(activeA.bits);
		activeA.varSlice = activeA.system->next(activeA.bits);
		activeA.historySize = hr->size;
		activeA.induceThreshold = induceThreshold;
		activeA.logging = true;
		activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
		activeA.underlyingEventsRepa.push_back(eventsA);
		{
			SizeList vv0;
			{
				auto& mm = ur->mapVarSize();
				auto vscan = std::make_shared<Variable>("scan");
				for (int i = 330; i < 390; i++)
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
		activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

		ActiveUpdateParameters ppu;
		ActiveInduceParameters ppi;
		ppi.tint = 4;
		ppi.wmax = 9;
		ppi.znnmax = activeA.historySize * 2.0 * 300.0 * 300.0;
		ppi.znnmax *= ppi.tint;
		for (std::size_t i = 0; i < activeA.historySize; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);	
			activeA.update(ppu);
			activeA.induce(ppi);			
		}

		ActiveIOParameters ppio;
		ppio.filename = model+".ac";
		activeA.dump(ppio);	
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;		
		applicationRepasPersistent(*drer(*activeA.decomp), out);
		out.close();	
		EVAL(model+".dr");
	}
	
	if (argc >= 3 && string(argv[1]) == "induce04")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		auto eventsA = std::make_shared<ActiveEventsRepa>(1);
		
		Active activeA(model);
		activeA.system = std::make_shared<ActiveSystem>();
		activeA.var = activeA.system->next(activeA.bits);
		activeA.varSlice = activeA.system->next(activeA.bits);
		activeA.historySize = hr->size;
		activeA.induceThreshold = induceThreshold;
		activeA.logging = true;
		activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
		activeA.underlyingEventsRepa.push_back(eventsA);
		{
			SizeList vv0;
			{
				auto& mm = ur->mapVarSize();
				auto vscan = std::make_shared<Variable>("scan");
				for (int i = 330; i < 390; i++)
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
		activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

		activeA.logging = false;
		ActiveUpdateParameters ppu;
		for (std::size_t i = 0; i < activeA.historySize; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);	
			activeA.update(ppu);
		}
		activeA.logging = true;
		ActiveInduceParameters ppi;
		ppi.tint = 4;
		ppi.wmax = 9;
		ppi.znnmax = activeA.historySize * 2.0 * 300.0 * 300.0;
		ppi.znnmax *= ppi.tint;
		activeA.induce(ppi);			

		// ActiveIOParameters ppio;
		// ppio.filename = model+".ac";
		// activeA.dump(ppio);	
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;		
		applicationRepasPersistent(*drer(*activeA.decomp), out);
		out.close();	
		EVAL(model+".dr");
	}
	
	if (argc >= 3 && string(argv[1]) == "induce05")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		auto erconcat = vectorApplicationRepasConcat_u;
		auto erjoin = applicationRepaPairsJoin_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		size_t induceThresholdInitial = argc >= 6 ? atoi(argv[5]) : 10000;
		size_t level1Size = argc >= 7 ? atoi(argv[6]) : 12;
		size_t loops = argc >= 8 ? atoi(argv[7]) : 1;
		size_t induceThresholdLevel1 = argc >= 9 ? atoi(argv[8]) : induceThreshold;
		size_t induceThresholdInitialLevel1 = argc >= 10 ? atoi(argv[9]) : 1;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		EVAL(induceThresholdInitial);
		EVAL(level1Size);
		EVAL(loops);
		EVAL(induceThresholdLevel1);
		EVAL(induceThresholdInitialLevel1);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		auto systemA = std::make_shared<ActiveSystem>();
		auto eventsA = std::make_shared<ActiveEventsRepa>(level1Size+1);
		std::size_t activeSize = hr->size;
		std::vector<std::shared_ptr<Active>> level1;
		for (std::size_t m = 0; m < level1Size; m++)
			level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < level1Size; m++)
		{			
			auto& activeA = *level1[m];
			activeA.name = model + "_1_" + (m<10 ? "0" : "") + std::to_string(m);
			activeA.system = systemA;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSize;
			activeA.induceThreshold = induceThresholdLevel1;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					int start = 360 - (360/level1Size/2) + (360/level1Size)*m;
					int end = start + (360/level1Size);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		}
	
		std::vector<std::shared_ptr<Active>> level2;
		level2.push_back(std::make_shared<Active>());
		{
			auto& activeA = *level2.front();
			activeA.name = model + "_2";
			activeA.system = systemA;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSize;
			activeA.induceThreshold = induceThreshold;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
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
			for (std::size_t m = 0; m < level1Size; m++)
			{
				auto& activeB = *level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);			
		}

		bool ok = true;
		ActiveUpdateParameters ppu;
		ActiveInduceParameters ppi1;
		ppi1.tint = 4;
		ppi1.wmax = 9;
		ppi1.znnmax = activeSize * 2.0 * 300.0 * 300.0;
		ppi1.znnmax *= ppi1.tint;
		ActiveInduceParameters ppi2;
		ppi2.tint = 4;
		ppi2.znnmax = activeSize * 2.0 * 300.0 * 300.0;
		ppi2.znnmax *= ppi2.tint;
		ppi2.logging = false;
		for (std::size_t i = 0; ok && i < activeSize*loops; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,(i % hr->size))),eventsA->references);
			for (std::size_t m = 0; ok && m < level1Size; m++)
			{
				auto& activeA = *level1[m];
				ok = ok && activeA.update(ppu);
				if (i+1 >= induceThresholdInitialLevel1)		
					ok = ok && activeA.induce(ppi1);			
			}
			if (ok)
			{
				auto& activeA = *level2.front();	
				if (i % 100 == 0)
					activeA.logging = true;
				ok = ok && activeA.update(ppu);
				activeA.logging = true;
				if (i+1 >= induceThresholdInitial)		
					ok = ok && activeA.induce(ppi2);	
				activeA.logging = false;
			}
		}

		for (std::size_t m = 0; ok && m < level1Size; m++)
		{
			auto& activeA = *level1[m];
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			activeA.logging = true;
			activeA.dump(ppio);			
		}
		if (ok)
		{
			auto& activeA = *level2.front();	
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			activeA.dump(ppio);							
		}
		if (ok)
		{
			std::vector<std::shared_ptr<ApplicationRepa>> ll;
			for (std::size_t m = 0; m < level1Size; m++)
			{
				auto& activeA = *level1[m];
				ll.push_back(std::move(drer(*activeA.decomp)));
			}
			auto er1 = erconcat(ll);
			std::unique_ptr<ApplicationRepa> er2;
			{
				auto& activeA = *level2.front();	
				er2 = drer(*activeA.decomp);						
			}			
			auto er3 = erjoin(*er1, *er2);
			std::ofstream out(model+".dr", std::ios::binary);
			systemRepasPersistent(*ur, out); cout << endl;		
			applicationRepasPersistent(*er3, out);
			out.close();	
			EVAL(model+".dr");			
		}
		TRUTH(ok);
	}
	
	if (argc >= 3 && string(argv[1]) == "load05")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto hrsel2 = eventsHistoryRepasHistoryRepaSelection_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto hrshuffle = historyRepasShuffle_u;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		auto erconcat = vectorApplicationRepasConcat_u;
		auto erjoin = applicationRepaPairsJoin_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t level1Size = argc >= 5 ? atoi(argv[4]) : 12;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(level1Size);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		bool ok = true;
		auto systemA = std::make_shared<ActiveSystem>();
		auto eventsA = std::make_shared<ActiveEventsRepa>(level1Size+1);
		std::size_t activeSize = hr->size;
		std::vector<std::shared_ptr<Active>> level1;
		for (std::size_t m = 0; m < level1Size; m++)
			level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; ok && m < level1Size; m++)
		{			
			auto& activeA = *level1[m];
			activeA.name = model + "_1_" + (m<10 ? "0" : "") + std::to_string(m);
			activeA.logging = true;
			activeA.system = systemA;
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			ok = ok && activeA.load(ppio);
			// EVAL(activeA.underlyingEventUpdateds);
			// EVAL(activeA.historySize);
			// TRUTH(activeA.historyOverflow);
			// EVAL(activeA.historyEvent);
			EVAL(activeA.decomp->fudRepasSize);
			EVAL(activeA.decomp->fuds.size());
			activeA.logging = false;
			activeA.underlyingEventUpdateds.clear();
			activeA.underlyingEventsRepa.push_back(eventsA);
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		}
	
		std::vector<std::shared_ptr<Active>> level2;
		level2.push_back(std::make_shared<Active>());
		if (ok)
		{
			auto& activeA = *level2.front();
			activeA.name = model + "_2";
			activeA.logging = true;
			activeA.system = systemA;
			ActiveIOParameters ppio;
			ppio.filename = activeA.name+".ac";
			ok = ok && activeA.load(ppio);
			EVAL(activeA.underlyingEventUpdateds);
			EVAL(activeA.historySize);
			TRUTH(activeA.historyOverflow);
			EVAL(activeA.historyEvent);
			EVAL(activeA.decomp->fudRepasSize);
			EVAL(activeA.decomp->fuds.size());
			// EVAL(activeA.decomp->fuds.front());
			// EVAL(activeA.decomp->fuds.back());
			activeA.logging = false;
			activeA.underlyingEventUpdateds.clear();
			activeA.underlyingEventsRepa.push_back(eventsA);
			for (std::size_t m = 0; m < level1Size; m++)
			{
				auto& activeB = *level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
			}
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);			
		}

		// SizeSizeMap dd;
		// SizeSet dev;
		if (ok)
		{
			std::vector<std::shared_ptr<ApplicationRepa>> ll;
			for (std::size_t m = 0; m < level1Size; m++)
			{
				auto& activeA = *level1[m];
				ll.push_back(std::move(drer(*activeA.decomp)));
			}
			auto er1 = erconcat(ll);
			// EVAL(*er1);
			EVAL(fudRepasSize(*er1->fud));
			EVAL(frder(*er1->fud)->size());
			EVAL(frund(*er1->fud)->size());
			EVAL(treesSize(*er1->slices));
			EVAL(treesLeafElements(*er1->slices)->size());
			// EVAL(sorted(er1->substrate));
			std::unique_ptr<ApplicationRepa> er2;
			{
				auto& activeA = *level2.front();	
				er2 = drer(*activeA.decomp);						
			}	
			// EVAL(*er2);	
			EVAL(fudRepasSize(*er2->fud));
			EVAL(frder(*er2->fud)->size());
			EVAL(frund(*er2->fud)->size());				
			EVAL(treesSize(*er2->slices));
			EVAL(treesLeafElements(*er2->slices)->size());	
			// EVAL(sorted(*treesLeafElements(*er2->slices)));				
			// EVAL(sorted(er2->substrate));
			auto er3 = erjoin(*er1, *er2);
			// EVAL(*er3);	
			EVAL(fudRepasSize(*er3->fud));
			EVAL(frder(*er3->fud)->size());
			EVAL(frund(*er3->fud)->size());
			EVAL(treesSize(*er3->slices));
			EVAL(treesLeafElements(*er3->slices)->size());				
			// EVAL(sorted(*treesLeafElements(*er3->slices)));
			// EVAL(sorted(*treesElements(*er3->slices)));
			// EVAL(sorted(er3->substrate));
			// {
				// auto er4 = std::make_unique<ApplicationRepa>();
				// auto frvars = fudRepasSetVar;
				// auto frdep = fudRepasSetVarsDepends;
	
				// er4->substrate = er1->substrate;
				// er4->slices = std::move(pathsTree(*treesPaths(*er2->slices)));
				// SizeUSet vv(er1->substrate.begin(), er1->substrate.end());
				// auto sl = treesElements(*er2->slices);
				// SizeUSet ww(sl->begin(), sl->end());
				// FudRepa fr;
				// fr.layers.reserve(er1->fud->layers.size() + er2->fud->layers.size());
				// fr.layers.insert(fr.layers.end(), er1->fud->layers.begin(), er1->fud->layers.end());
				// fr.layers.insert(fr.layers.end(), er2->fud->layers.begin(), er2->fud->layers.end());
				// auto vv1 = frvars(*er1->fud);
				// for (auto v : er2->substrate)
					// if (vv1->find(v) == vv1->end())
					// {
						// vv.insert(v);
						// er4->substrate.push_back(v);
					// }
				// er4->fud = std::move(listTransformRepasFudRepa_u(*frdep(fr, ww)));
				// std::ofstream out("test.dr", std::ios::binary);
				// systemRepasPersistent(*ur, out); cout << endl;		
				// applicationRepasPersistent(*er4, out);
				// out.close();	
				// EVAL("test.dr");
				// // ./main entropy test 1 data009
			// }
			// {
				// auto frdep = fudRepasSetVarsDepends;
				// auto frmul = historyRepasFudRepasMultiply_u;
				// auto llfr = listTransformRepasFudRepa_u;
				// auto frvars = fudRepasSetVar;
				// SizeSizeMap aa;
				// auto hr2 = frmul(*hr, *er3->fud);
				// auto sl = sorted(*treesLeafElements(*er3->slices));
				// auto m = sl.size();
				// auto z = hr2->size;
				// auto& mvv2 = hr2->mapVarInt();
				// SizeList pvv2;
				// for (auto v : sl)
					// pvv2.push_back(mvv2[v]);
				// auto rr2 = hr2->arr;
				// std::size_t y = 0;
				// for (std::size_t j = 0; j < z; j++)
					// for (std::size_t i = 0; i < m; i++)
					// {
						// std::size_t u = rr2[pvv2[i]*z + j];
						// if (u)
						// {
							// aa[sl[i]]++;
							// y++;
						// }
					// }
				// EVAL(y);
				// EVAL(aa.size());					
				// // EVAL(aa);		
				// dd = aa;
				// auto xx = llfr(*frdep(*er3->fud, SizeUSet{1703943}));
				// EVAL(1703943 >> 16 << 16);
				// EVAL(*frund(*xx));	
				// // EVAL(*xx);	
				// // auto yy = frvars(*xx);
				// // cout << "[";
				// // for (auto v : SizeSet(yy->begin(),yy->end()))
				// // {
					// // cout << "(" << v << ",";
					// // for (std::size_t i = 0; i < er3->fud->layers.size(); i++)
					// // {
						// // auto& ll = er3->fud->layers[i];
						// // for (auto& tr : ll)
							// // if (tr->derived == v)
								// // cout << " " << i;								
					// // }
					// // cout << "),";
				// // }
				// // cout << "]" << endl;
				// // SizeSizeSetMap bb;
				// // for (std::size_t j = 0; j < z; j++)
					// // for (std::size_t i = 0; i < m; i++)
					// // {
						// // std::size_t u = rr2[pvv2[i]*z + j];
						// // if (u && sl[i]==1703943)
						// // {
							// // dev.insert(j);
							// // for (std::size_t k = 0; k < m; k++)
							// // {
								// // std::size_t u = rr2[pvv2[k]*z + j];
								// // if (u && sl[k]!=1703943)
								// // {
									// // bb[j].insert(sl[k]);
								// // }
							// // }
						// // }
					// // }
				// // EVAL(bb);
				// // EVAL(*hrsel(*hr,82817));
				// // // EVAL(*hrsel(*hr2,82817));
			// }
		}
		
		if (ok)
		{
			ActiveUpdateParameters ppu;
			// {
				// eventsA->mapIdEvent[0] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,82817)),eventsA->references);
				// // for (std::size_t m = 0; m < level1Size; m++)
				// // {
					// // auto& activeA = *level1[m];
					// // activeA.logging = true;
					// // TRUTH(activeA.update(ppu));
				// // }
				// // {
					// // auto& activeA = *level2.front();	
					// // activeA.logging = true;
					// // TRUTH(activeA.update(ppu));
				// // }
				// {
					// auto& activeA = *level1[8];
					// activeA.logging = true;
					// TRUTH(activeA.update(ppu));
				// }
			// }
			for (std::size_t i = 0; i < hr->size; i++) 
			{
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,(i % hr->size))),eventsA->references);
				for (std::size_t m = 0; m < level1Size; m++)
				{
					auto& activeA = *level1[m];
					activeA.update(ppu);
				}
				{
					auto& activeA = *level2.front();	
					activeA.update(ppu);
				}
			}			
			SizeSizeMap aa;
			for (auto& p : level2.front()->historySlicesSetEvent)
				aa[p.first] = p.second.size();
			// EVAL(aa.size());					
			// // EVAL(aa);	
			// std::size_t y = 0;
			// for (auto& p : aa)	
				// y+= p.second;	
			// EVAL(y);				
			// // for (auto& p : aa)
				// // if (p.second > 0 && dd[p.first] != aa[p.first])
				// // {
					// // EVAL(p.first);
					// // EVAL(dd[p.first]);
					// // EVAL(aa[p.first]);
				// // }
			// // for (auto& p : dd)
				// // if (p.second > 0 && dd[p.first] != aa[p.first])
				// // {
					// // EVAL(p.first);
					// // EVAL(dd[p.first]);
					// // EVAL(aa[p.first]);
				// // }		
			// EVAL(dev);
			// EVAL(level2.front()->historySlicesSetEvent[1703943]);
			// for (std::size_t m = 0; m < level1Size; m++)
			// {
				// auto& activeA = *level1[m];
				// // auto& llr = activeA.underlyingHistoryRepa;
				// // auto& lla = activeA.underlyingHistorySparse;
				// // for (auto& hr : llr)
				// // {
					// // EVAL(*hrsel(*hr,82817));
				// // }
				// // for (auto& hr : lla)
				// // {
					// // auto rr = hr->arr;
					// // EVAL(rr[82817]);
				// // }
				// auto& ha = activeA.historySparse;
				// auto z = ha->size;
				// EVAL(m);
				// for (std::size_t j = 0; j < z; j++)
				// {
					// if (!ha->arr[j])
						// cout << j << ",";
				// }
				// cout << endl;
			// }
			// {
				// auto& activeA = *level2.front();	
				// // auto& llr = activeA.underlyingHistoryRepa;
				// // auto& lla = activeA.underlyingHistorySparse;
				// // for (auto& hr : llr)
				// // {
					// // EVAL(*hrsel(*hr,82817));
				// // }
				// // for (auto& hr : lla)
				// // {
					// // auto rr = hr->arr;
					// // EVAL(rr[82817]);
				// // }
				// auto& ha = activeA.historySparse;
				// auto z = ha->size;
				// for (std::size_t j = 0; j < z; j++)
				// {
					// if (!ha->arr[j])
						// cout << j << ",";
				// }
				// cout << endl;			
			// }
			auto hrs = hrshuffle(*hr, (unsigned int)(12345 + hr->size));
			for (std::size_t i = hr->size; i < hr->size + hrs->size; i++) 
			{
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hrs,(i % hrs->size))),eventsA->references);
				for (std::size_t m = 0; m < level1Size; m++)
				{
					auto& activeA = *level1[m];
					activeA.update(ppu);
				}
				{
					auto& activeA = *level2.front();	
					activeA.update(ppu);
				}
			}

			SizeSizeMap bb;
			for (auto& p : level2.front()->historySlicesSetEvent)
				bb[p.first] = p.second.size();
			// EVAL(bb.size());					
			SizeSizeMap cc(aa);
			for (auto& p : bb)
				cc[p.first] += p.second;
			// EVAL(cc.size());					
			double z = (double)hr->size;
			double a = z * std::log(z);
			for (auto& p : aa)
				if (p.second > 0)
					a -= (double)p.second *  std::log((double)p.second);
			EVAL(a);
			double b = z * std::log(z);
			for (auto& p : bb)
				if (p.second > 0)
					b -= (double)p.second *  std::log((double)p.second);
			EVAL(b);
			double c = 2.0 * z * std::log(2.0 * z);
			for (auto& p : cc)
				if (p.second > 0)
					c -= (double)p.second *  std::log((double)p.second);
			EVAL(c);
			cout << "likelihood c-a-b: " << (c-a-b) << endl;
		}

	}

	if (argc >= 3 && string(argv[1]) == "induce06")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		auto eventsA = std::make_shared<ActiveEventsRepa>(1);
		
		Active activeA(model);
		activeA.system = std::make_shared<ActiveSystem>();
		activeA.var = activeA.system->next(activeA.bits);
		activeA.varSlice = activeA.system->next(activeA.bits);
		activeA.historySize = hr->size;
		activeA.induceThreshold = induceThreshold;
		activeA.logging = true;
		activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
		activeA.underlyingEventsRepa.push_back(eventsA);
		{
			SizeList vv0;
			{
				auto& mm = ur->mapVarSize();
				auto vscan = std::make_shared<Variable>("scan");
				for (int i = 0; i < 360; i++)
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
		activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

		activeA.logging = false;
		ActiveUpdateParameters ppu;
		for (std::size_t i = 0; i < activeA.historySize; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);	
			activeA.update(ppu);
		}
		activeA.logging = true;
		ActiveInduceParameters ppi;
		ppi.tint = 4;
		ppi.wmax = 9;
		ppi.znnmax = activeA.historySize * 2.0 * 300.0 * 300.0;
		ppi.znnmax *= ppi.tint;
		activeA.induce(ppi);			

		// ActiveIOParameters ppio;
		// ppio.filename = model+".ac";
		// activeA.dump(ppio);	
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;		
		applicationRepasPersistent(*drer(*activeA.decomp), out);
		out.close();	
		EVAL(model+".dr");
	}
	
	if (argc >= 3 && string(argv[1]) == "induce07")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto hrshuffle = historyRepasShuffle_u;
		auto frund = fudRepasUnderlying;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		size_t frame01 = argc >= 6 ? atoi(argv[5]) : 0;
		size_t frame02 = argc >= 7 ? atoi(argv[6]) : 0;
		size_t frame03 = argc >= 8 ? atoi(argv[7]) : 0;
		size_t self01 = argc >= 9 ? atoi(argv[8]) : 0;
		size_t self02 = argc >= 10 ? atoi(argv[9]) : 0;
		size_t self03 = argc >= 11 ? atoi(argv[10]) : 0;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		EVAL(frame01);
		EVAL(frame02);
		EVAL(frame03);
		EVAL(self01);
		EVAL(self02);
		EVAL(self03);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		auto eventsA = std::make_shared<ActiveEventsRepa>(1);
		
		Active activeA(model);
		activeA.system = std::make_shared<ActiveSystem>();
		activeA.var = activeA.system->next(activeA.bits);
		activeA.varSlice = activeA.system->next(activeA.bits);
		activeA.historySize = hr->size;
		activeA.induceThreshold = induceThreshold;
		activeA.logging = false;
		activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
		activeA.underlyingEventsRepa.push_back(eventsA);
		{
			SizeList vv0;
			{
				auto& mm = ur->mapVarSize();
				auto vscan = std::make_shared<Variable>("scan");
				for (int i = 330; i < 390; i++)
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
		activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		
		activeA.frameUnderlyings.insert(frame01);
		if (frame02) activeA.frameUnderlyings.insert(frame02);
		if (frame03) activeA.frameUnderlyings.insert(frame03);
		EVAL(activeA.frameUnderlyings);
		if (self01) activeA.frameHistorys.insert(self01);
		if (self02) activeA.frameHistorys.insert(self02);
		if (self03) activeA.frameHistorys.insert(self03);
		EVAL(activeA.frameHistorys);

		bool ok = true;
		ActiveUpdateParameters ppu;
		ActiveInduceParameters ppi;
		ppi.tint = 4;
		ppi.wmax = 9;
		ppi.znnmax = activeA.historySize * 2.0 * 300.0 * 300.0;
		ppi.znnmax *= ppi.tint;
		for (std::size_t i = 0; ok && i < activeA.historySize; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);	
			activeA.logging = false;
			ok = ok && activeA.update(ppu);
			activeA.logging = false;
			ok = ok && activeA.induce(ppi);			
		}
		EVAL(activeA.varMax());

		if (ok)
		{
			SizeSizeMap aa;
			for (auto& p : activeA.historySlicesSetEvent)
				aa[p.first] = p.second.size();
			auto hrs = hrshuffle(*hr, (unsigned int)(12345 + hr->size));
			for (std::size_t i = hr->size; i < hr->size + hrs->size; i++) 
			{
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hrs,(i % hrs->size))),eventsA->references);
				activeA.logging = false;
				activeA.update(ppu);
			}
			SizeSizeMap bb;
			for (auto& p : activeA.historySlicesSetEvent)
				bb[p.first] = p.second.size();
			SizeSizeMap cc(aa);
			for (auto& p : bb)
				cc[p.first] += p.second;
			double z = (double)hr->size;
			double a = z * std::log(z);
			for (auto& p : aa)
				if (p.second > 0)
					a -= (double)p.second *  std::log((double)p.second);
			EVAL(a);
			double b = z * std::log(z);
			for (auto& p : bb)
				if (p.second > 0)
					b -= (double)p.second *  std::log((double)p.second);
			EVAL(b);
			double c = 2.0 * z * std::log(2.0 * z);
			for (auto& p : cc)
				if (p.second > 0)
					c -= (double)p.second *  std::log((double)p.second);
			EVAL(c);
			cout << "likelihood c-a-b: " << (c-a-b) << endl;
		}
		
		{
			EVAL(activeA.framesVarsOffset);
			auto er = drer(*activeA.decomp);
			EVAL(fudRepasSize(*er->fud));
			EVAL(frund(*er->fud)->size());
			EVAL(sorted(*frund(*er->fud)));
		}

	}
	
	if (argc >= 3 && string(argv[1]) == "induce08")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto hrshuffle = historyRepasShuffle_u;
		auto frund = fudRepasUnderlying;
		auto drer = decompFudSlicedRepasApplicationRepa_u;
		
		auto run_level1 = [](Active& active, bool induce, ActiveUpdateParameters ppu, ActiveInduceParameters ppi)
		{
			active.update(ppu);
			if (induce)		
				active.induce(ppi);
			return;
		};
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		size_t induceThreshold = argc >= 5 ? atoi(argv[4]) : 1000;
		size_t induceThresholdInitial = argc >= 6 ? atoi(argv[5]) : 10000;
		size_t level1Size = argc >= 7 ? atoi(argv[6]) : 12;
		size_t loops = argc >= 8 ? atoi(argv[7]) : 1;
		size_t induceThresholdLevel1 = argc >= 9 ? atoi(argv[8]) : induceThreshold;
		size_t induceThresholdInitialLevel1 = argc >= 10 ? atoi(argv[9]) : 1;
		size_t frame01 = argc >= 11 ? atoi(argv[10]) : 0;
		size_t frame02 = argc >= 12 ? atoi(argv[11]) : 0;
		size_t frame03 = argc >= 13 ? atoi(argv[12]) : 0;
		size_t self01 = argc >= 14 ? atoi(argv[13]) : 0;
		size_t self02 = argc >= 15 ? atoi(argv[14]) : 0;
		size_t self03 = argc >= 16 ? atoi(argv[15]) : 0;
		
		EVAL(model);
		EVAL(dataset);
		EVAL(induceThreshold);
		EVAL(induceThresholdInitial);
		EVAL(level1Size);
		EVAL(loops);
		EVAL(induceThresholdLevel1);
		EVAL(induceThresholdInitialLevel1);
		EVAL(frame01);
		EVAL(frame02);
		EVAL(frame03);
		EVAL(self01);
		EVAL(self02);
		EVAL(self03);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
				
		auto systemA = std::make_shared<ActiveSystem>();
		auto eventsA = std::make_shared<ActiveEventsRepa>(level1Size+1);
		std::size_t activeSize = hr->size;
		std::vector<std::shared_ptr<Active>> level1;
		for (std::size_t m = 0; m < level1Size; m++)
			level1.push_back(std::make_shared<Active>());
		for (std::size_t m = 0; m < level1Size; m++)
		{			
			auto& activeA = *level1[m];
			activeA.name = model + "_1_" + (m<10 ? "0" : "") + std::to_string(m);
			activeA.system = systemA;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSize;
			activeA.induceThreshold = induceThresholdLevel1;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					int start = 360 - (360/level1Size/2) + (360/level1Size)*m;
					int end = start + (360/level1Size);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);
		}
	
		std::vector<std::shared_ptr<Active>> level2;
		level2.push_back(std::make_shared<Active>());
		{
			auto& activeA = *level2.front();
			activeA.name = model + "_2";
			activeA.system = systemA;
			activeA.var = activeA.system->next(activeA.bits);
			activeA.varSlice = activeA.system->next(activeA.bits);
			activeA.historySize = activeSize;
			activeA.induceThreshold = induceThreshold;
			activeA.logging = false;
			activeA.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
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
			for (std::size_t m = 0; m < level1Size; m++)
			{
				auto& activeB = *level1[m];
				activeA.underlyingEventsSparse.push_back(activeB.eventsSparse);
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);			
		}
		
		{
			auto& activeA = *level2.front();		
			activeA.frameUnderlyings.insert(frame01);
			if (frame02) activeA.frameUnderlyings.insert(frame02);
			if (frame03) activeA.frameUnderlyings.insert(frame03);
			EVAL(activeA.frameUnderlyings);
			if (self01) activeA.frameHistorys.insert(self01);
			if (self02) activeA.frameHistorys.insert(self02);
			if (self03) activeA.frameHistorys.insert(self03);
			EVAL(activeA.frameHistorys);
		}
		
		bool ok = true;
		ActiveUpdateParameters ppu;
		ActiveInduceParameters ppi1;
		ppi1.tint = 4;
		ppi1.wmax = 9;
		ppi1.znnmax = activeSize * 2.0 * 300.0 * 300.0;
		ppi1.znnmax *= ppi1.tint;
		ActiveInduceParameters ppi2;
		ppi2.tint = 4;
		ppi2.znnmax = activeSize * 2.0 * 300.0 * 300.0;
		ppi2.znnmax *= ppi2.tint;
		ppi2.logging = false;
		for (std::size_t i = 0; ok && i < activeSize*loops; i++) 
		{
			eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,(i % hr->size))),eventsA->references);
			// for (std::size_t m = 0; ok && m < level1Size; m++)
			// {
				// auto& activeA = *level1[m];
				// ok = ok && activeA.update(ppu);
				// if (i+1 >= induceThresholdInitialLevel1)		
					// ok = ok && activeA.induce(ppi1);		
			// }
			std::vector<std::thread> threads;
			threads.reserve(level1Size);
			for (std::size_t m = 0; ok && m < level1Size; m++)
			{
				auto& activeA = *level1[m];
				threads.push_back(std::thread(run_level1, std::ref(activeA), i+1 >= induceThresholdInitialLevel1,ppu,ppi1));
			}
			for (auto& t : threads)
				t.join();			
			if (ok)
			{
				auto& activeA = *level2.front();	
				if (i % 100 == 0)
					activeA.logging = true;
				ok = ok && activeA.update(ppu);
				activeA.logging = true;
				if (i+1 >= induceThresholdInitial)		
					ok = ok && activeA.induce(ppi2);	
				activeA.logging = false;
			}
		}

		{
			auto& activeA = *level2.front();		
			EVAL(activeA.varMax());
		}

		if (ok)
		{
			for (std::size_t i = activeSize*loops; i < activeSize*loops + hr->size; i++) 
			{
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,(i % hr->size))),eventsA->references);
				// for (std::size_t m = 0; ok && m < level1Size; m++)
				// {
					// auto& activeA = *level1[m];
					// ok = ok && activeA.update(ppu);		
				// }
				std::vector<std::thread> threads;
				threads.reserve(level1Size);
				for (std::size_t m = 0; ok && m < level1Size; m++)
				{
					auto& activeA = *level1[m];
					threads.push_back(std::thread(run_level1, std::ref(activeA), false,ppu,ppi1));
				}
				for (auto& t : threads)
					t.join();			
				if (ok)
				{
					auto& activeA = *level2.front();	
					activeA.logging = false;
					ok = ok && activeA.update(ppu);
				}
			}
			SizeSizeMap aa;
			if (ok)
			{
				auto& activeA = *level2.front();
				for (auto& p : activeA.historySlicesSetEvent)
					aa[p.first] = p.second.size();
			}
			auto hrs = hrshuffle(*hr, (unsigned int)(12345 + hr->size));
			for (std::size_t i = activeSize*loops + hr->size; i < activeSize*loops + hr->size + hrs->size; i++) 
			{
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hrs,(i % hrs->size))),eventsA->references);
				// for (std::size_t m = 0; ok && m < level1Size; m++)
				// {
					// auto& activeA = *level1[m];
					// ok = ok && activeA.update(ppu);		
				// }
				std::vector<std::thread> threads;
				threads.reserve(level1Size);
				for (std::size_t m = 0; ok && m < level1Size; m++)
				{
					auto& activeA = *level1[m];
					threads.push_back(std::thread(run_level1, std::ref(activeA), false,ppu,ppi1));
				}
				for (auto& t : threads)
					t.join();	
				if (ok)
				{
					auto& activeA = *level2.front();	
					activeA.logging = false;
					ok = ok && activeA.update(ppu);
				}
			}
			if (ok)
			{
				auto& activeA = *level2.front();
				SizeSizeMap bb;
				for (auto& p : activeA.historySlicesSetEvent)
					bb[p.first] = p.second.size();
				SizeSizeMap cc(aa);
				for (auto& p : bb)
					cc[p.first] += p.second;
				double z = (double)hr->size;
				double a = z * std::log(z);
				for (auto& p : aa)
					if (p.second > 0)
						a -= (double)p.second *  std::log((double)p.second);
				EVAL(a);
				double b = z * std::log(z);
				for (auto& p : bb)
					if (p.second > 0)
						b -= (double)p.second *  std::log((double)p.second);
				EVAL(b);
				double c = 2.0 * z * std::log(2.0 * z);
				for (auto& p : cc)
					if (p.second > 0)
						c -= (double)p.second *  std::log((double)p.second);
				EVAL(c);
				cout << "likelihood c-a-b: " << (c-a-b) << endl;
			}
		}
		if (ok)		
		{
			auto& activeA = *level2.front();
			EVAL(activeA.framesVarsOffset);
			auto er = drer(*activeA.decomp);
			EVAL(fudRepasSize(*er->fud));
			EVAL(frund(*er->fud)->size());
			EVAL(sorted(*frund(*er->fud)));
		}

	}
	
	if (argc >= 3 && string(argv[1]) == "induce09")
	{
		auto hrsel = [](const HistoryRepa& hr, std::size_t ev)
		{
			SizeList ll {ev};
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto erdr = applicationRepasDecompFudSlicedRepa_u;
		
		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data009");
		
		EVAL(model);
		EVAL(dataset);
		
		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
		{
			std::ifstream in(dataset+".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_4(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVAL(hr->size);
		EVAL(ur->listVarSizePair.size());
		
		{
			auto eventsA = std::make_shared<ActiveEventsRepa>(2);
			
			auto systemA = std::make_shared<ActiveSystem>();
			
			Active activeA("activeA");
			activeA.system = systemA;
			activeA.historySize = 10;
			activeA.continousIs = true;			
			activeA.logging = true;
			{
				StrVarPtrMap m;
				std::ifstream in(model + ".dr", std::ios::binary);
				auto ur = persistentsSystemRepa(in, m);
				activeA.decomp = erdr(*persistentsApplicationRepa(in));
				in.close();		
				systemA->block = activeA.decomp->varMax() >> systemA->bits;
			}
			activeA.var = systemA->next(activeA.bits);
			EVAL(activeA.var);
			activeA.varSlice = systemA->next(activeA.bits);
			EVAL(activeA.varSlice);
			activeA.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					auto vscan = std::make_shared<Variable>("scan");
					for (int i = 0; i < 360; i++)
						vv0.push_back(mm[Variable(vscan, std::make_shared<Variable>((i % 360) + 1))]);
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
					memset(rr1, 0, z1*n1);			
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
			activeA.eventsSparse = std::make_shared<ActiveEventsArray>(1);

			Active activeB("activeB");
			activeB.system = systemA;
			activeB.var = systemA->next(activeB.bits);
			EVAL(activeB.var);
			activeB.varSlice = systemA->next(activeB.bits);
			EVAL(activeB.varSlice);
			activeB.historySize = 10;
			activeB.continousIs = true;			
			activeB.induceThreshold = 5;
			activeB.logging = true;
			activeB.decomp = std::make_unique<DecompFudSlicedRepa>();
			activeB.underlyingEventsRepa.push_back(eventsA);
			{
				SizeList vv0;
				{
					auto& mm = ur->mapVarSize();
					vv0.push_back(mm[Variable("motor")]);
					vv0.push_back(mm[Variable("location")]);
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
					memset(rr1, 0, z1*n1);			
				}
				activeB.underlyingHistoryRepa.push_back(hr1);
			}
			activeB.underlyingEventsSparse.push_back(activeA.eventsSparse);
			activeB.underlyingHistorySparse.push_back(std::make_shared<HistorySparseArray>(activeB.historySize,1));
			{
				auto hr = std::make_unique<HistorySparseArray>();
				{
					auto z = activeB.historySize;
					hr->size = z;
					hr->capacity = 1;
					hr->arr = new std::size_t[z];		
				}		
				activeB.historySparse = std::move(hr);			
			}
			activeB.eventsSparse = std::make_shared<ActiveEventsArray>(1);
			
			bool ok = true;
						
			for (std::size_t i = 0; i < 2; i++) 
				eventsA->mapIdEvent[i] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);

			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			for (std::size_t i = 2; i < 6; i++) 
				eventsA->mapIdEvent[i*2] = HistoryRepaPtrSizePair(std::move(hrsel(*hr,i)),eventsA->references);
	
			ECHO(ok = activeA.update(ActiveUpdateParameters()));
			TRUTH(ok);

			ECHO(ok = activeB.update(ActiveUpdateParameters()));
			TRUTH(ok);

			auto inducePost = [](Active& active, std::size_t sliceA, std::size_t sliceSizeA)
			{
				std::cout << "sliceA: " << sliceA << std::endl;
				std::cout << "sliceSizeA: " << sliceSizeA << std::endl;
				return true;
			};
			
			activeB.induceCallback = inducePost;
			// EVAL(sorted(activeB.underlyingSlicesParent));
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);
			EVAL(activeB.frameUnderlyings);
			EVAL(activeB.frameHistorys);
			EVAL(activeB.framesVarsOffset);
			ECHO(ok = activeB.induce(ActiveInduceParameters()));
			TRUTH(ok);
			EVAL(activeB.historySlicesSetEvent);
			EVAL(activeB.induceSlices);	
			
			if (false)
			{
				EVAL(activeA.historySlicesSetEvent);
				EVAL(activeB.induceSlices);
				ActiveInduceParameters ppi;
				ppi.znnmax = 0;
				ppi.bmax = 2;
				ECHO(ok = activeB.induce(ppi));
				TRUTH(ok);
				EVAL(activeA.historySlicesSetEvent);
				EVAL(activeB.induceSlices);	
			}
			
			if (true)
			{
				activeB.frameUnderlyings = SizeSet{1,2,3};
				activeB.frameHistorys = SizeSet{4,5};
				activeB.framesVarsOffset[1][2] = 3;
				activeB.framesVarsOffset[1][4] = 5;
				activeB.framesVarsOffset[2][6] = 7;
				ActiveIOParameters pp;
				pp.filename = "test.bin";
				ECHO(ok = activeB.dump(pp));
				TRUTH(ok);				
				Active activeC("activeC");
				activeC.logging = true;
				activeC.historyOverflow = true;
				ECHO(ok = activeC.load(pp));
				TRUTH(ok);				
				EVAL(activeB.name);				
				EVAL(activeC.name);				
				EVAL(activeB.underlyingEventUpdateds);				
				EVAL(activeC.underlyingEventUpdateds);				
				EVAL(activeB.historySize);				
				EVAL(activeC.historySize);				
				TRUTH(activeB.historyOverflow);				
				TRUTH(activeC.historyOverflow);				
				EVAL(activeB.historyEvent);				
				EVAL(activeC.historyEvent);		
				TRUTH(activeB.continousIs);				
				TRUTH(activeC.continousIs);		
				EVAL(activeB.continousHistoryEventsEvent);				
				EVAL(activeC.continousHistoryEventsEvent);					
				{
					for (auto& hr : activeB.underlyingHistoryRepa)
					{
						EVAL(*hr);				
					}
					for (auto& hr : activeC.underlyingHistoryRepa)
					{
						EVAL(*hr);				
					}
				}	
				{
					for (auto& hr : activeB.underlyingHistorySparse)
					{
						EVAL(*hr);				
					}
					for (auto& hr : activeC.underlyingHistorySparse)
					{
						EVAL(*hr);				
					}
				}				
				EVAL(sorted(activeB.underlyingSlicesParent));				
				EVAL(sorted(activeC.underlyingSlicesParent));	
				EVAL(*activeB.decomp);				
				EVAL(*activeC.decomp);				
				EVAL(activeB.bits);				
				EVAL(activeC.bits);				
				EVAL(activeB.var);				
				EVAL(activeC.var);				
				EVAL(activeB.varSlice);				
				EVAL(activeC.varSlice);				
				EVAL(activeB.induceThreshold);				
				EVAL(activeC.induceThreshold);				
				EVAL(activeB.induceVarExclusions);				
				EVAL(activeC.induceVarExclusions);				
				if (activeB.historySparse) {EVAL(*activeB.historySparse);}				
				if (activeC.historySparse) {EVAL(*activeC.historySparse);}			
				EVAL(activeB.historySlicesSetEvent);				
				EVAL(activeC.historySlicesSetEvent);				
				EVAL(activeB.induceSlices);				
				EVAL(activeC.induceSlices);				
				EVAL(activeB.induceSliceFailsSize);				
				EVAL(activeC.induceSliceFailsSize);				
				EVAL(activeB.frameUnderlyings);				
				EVAL(activeC.frameUnderlyings);				
				EVAL(activeB.frameHistorys);				
				EVAL(activeC.frameHistorys);				
				EVAL(activeB.framesVarsOffset);				
				EVAL(activeC.framesVarsOffset);				
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
			SystemHistoryRepaTuple xx = recordListsHistoryRepa_4(8, RecordList{ Record() });	
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


	return 0;
}
