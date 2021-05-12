#include "dev.h"

#include <stdlib.h>
#include <random>

using namespace Alignment;
using namespace TBOT03;
using namespace std;

void TBOT03::recordsPersistent(Record& r, std::ostream& out)
{
	out.write(reinterpret_cast<char*>(&r.id), sizeof(std::size_t));
	out.write(reinterpret_cast<char*>(&r.ts), sizeof(double));
	for (std::size_t i = 0; i < 7; i++)
		out.write(reinterpret_cast<char*>(&r.sensor_pose[i]), sizeof(double));
	for (std::size_t i = 0; i < 360; i++)
		out.write(reinterpret_cast<char*>(&r.sensor_scan[i]), sizeof(double));
}

void TBOT03::recordListsPersistent(RecordList& rr, std::ostream& out)
{
	for (auto& r : rr)
		recordsPersistent(r, out);
}

std::unique_ptr<RecordList> TBOT03::persistentsRecordList(std::istream& in)
{
	auto rr = std::make_unique<RecordList>();
	while (true)
	{
		Record r;
		in.read(reinterpret_cast<char*>(&r.id), sizeof(std::size_t));
		if (in.eof())
			break;
		in.read(reinterpret_cast<char*>(&r.ts), sizeof(double));
		for (std::size_t i = 0; i < 7; i++)
			in.read(reinterpret_cast<char*>(&r.sensor_pose[i]), sizeof(double));
		for (std::size_t i = 0; i < 360; i++)
			in.read(reinterpret_cast<char*>(&r.sensor_scan[i]), sizeof(double));
		rr->push_back(r);
	}
	return rr;
}

std::ostream& operator<<(std::ostream& out, const Record& r)
{
	out << "(" << r.id << "," << r.ts << ",(";
	for (std::size_t i = 0; i < 7; i++)
		out << (i ? "," : "") << r.sensor_pose[i];
	out << "),(";
	for (std::size_t i = 0; i < 360; i++)
		out << (i ? "," : "") << r.sensor_scan[i];
	out << "))";
	return out;
}

std::ostream& operator<<(std::ostream& out, const RecordList& rr)
{
	for (auto& r : rr)
		out << r << std::endl;
	return out;
}


std::ostream& operator<<(std::ostream& out, std::istream& in)
{
	while (true)
	{
		Record r;
		in.read(reinterpret_cast<char*>(&r.id), sizeof(std::size_t));
		if (in.eof())
			break;
		in.read(reinterpret_cast<char*>(&r.ts), sizeof(double));
		for (std::size_t i = 0; i < 7; i++)
			in.read(reinterpret_cast<char*>(&r.sensor_pose[i]), sizeof(double));
		for (std::size_t i = 0; i < 360; i++)
			in.read(reinterpret_cast<char*>(&r.sensor_scan[i]), sizeof(double));
		out << r << std::endl;
	}
	return out;
}

typedef std::pair<double, double> Coord;
typedef std::pair<Coord, Coord> CoordP;

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;

	std::size_t n = 360 + 3;
	std::size_t z = qq.size();
	ValSet buckets;
	for (int i = 0; i < d; i++)
		buckets.insert(Value(i));
	ValSet actions;
	for (int i = 0; i < 3; i++)
		actions.insert(Value(i));
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room3"), Value("room4"), Value("room5"), Value("room6") };
	ValSet states{ Value("fail"), Value("success") };
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
	ll.push_back(VarValSetPair(Variable("status"), states));
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
		sh[i] = d;
	sh[n - 3] = actions.size();
	sh[n - 2] = states.size();
	sh[n - 1] = locations.size();
	double f = (double)d / 4.0;
	for (size_t j = 0; j < z; j++)
	{
		size_t jn = j*n;
		auto& r = qq[j];
		for (size_t i = 0; i < n - 3; i++)
			rr[jn + i] = (unsigned char)(r.sensor_scan[i] * f);
		rr[jn + n - 3] = 0;
		rr[jn + n - 2] = 0;
		double x = r.sensor_pose[0];
		double y = r.sensor_pose[1];
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
		rr[jn + n - 1] = (unsigned char)k;
	}
	hr->transpose();
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

