﻿#include "dev.h"

#include <stdlib.h>
#include <random>

using namespace Alignment;
using namespace TBOT03;
using namespace std;

TBOT03::Record TBOT03::Record::standard() const
{					
	return Record((x+5.15)/12.65, (y+5.275)/10.545, (yaw+180.0)/360.0);
}

TBOT03::Record TBOT03::Record::config() const
{					
	return Record(x*12.65 - 5.15, y*10.545-5.275, yaw*360.0-180.0);
}

TBOT03::Record TBOT03::Record::flip() const
{					
	return Record(x, y, yaw >= 0.0 ? yaw - 180.0 : yaw + 180.0);
}

double TBOT03::Record::squared(const Record& b)
{		
	return (x-b.x)*(x-b.x)+(y-b.y)*(y-b.y)+(yaw-b.yaw)*(yaw-b.yaw);
}

void TBOT03::Record::operator+=(const Record& recordB)
{		
	x += recordB.x;
	y += recordB.y;
	yaw += recordB.yaw;
}

void TBOT03::Record::operator/=(double floatA)
{		
	x /= floatA;
	y /= floatA;
	yaw /= floatA;
}

Record TBOT03::recordsMean(const RecordList& ll)
{
	Record recordA;
	for (auto recordB : ll)
		recordA += recordB;
	recordA /= ll.size();
	return recordA;
}

double TBOT03::recordsDeviation(const RecordList& ll)
{
	double variance = 0.0;
	Record mean = recordsMean(ll);
	for (auto recordB : ll)
		variance += mean.squared(recordB);
	variance /= ll.size();
	return std::sqrt(variance);
}

void TBOT03::recordsPersistent(Record& r, std::ostream& out)
{
	out.write(reinterpret_cast<char*>(&r.x), sizeof(double));
	out.write(reinterpret_cast<char*>(&r.y), sizeof(double));
	out.write(reinterpret_cast<char*>(&r.yaw), sizeof(double));
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
		in.read(reinterpret_cast<char*>(&r.x), sizeof(double));
		if (in.eof())
			break;
		in.read(reinterpret_cast<char*>(&r.y), sizeof(double));
		in.read(reinterpret_cast<char*>(&r.yaw), sizeof(double));
		rr->push_back(r);
	}
	return rr;
}

std::ostream& operator<<(std::ostream& out, const Record& r)
{
	out << "(" << r.x << "," << r.y << "," << r.yaw << ")";
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
		in.read(reinterpret_cast<char*>(&r.x), sizeof(double));
		if (in.eof())
			break;
		in.read(reinterpret_cast<char*>(&r.y), sizeof(double));
		in.read(reinterpret_cast<char*>(&r.yaw), sizeof(double));
		out << r << std::endl;
	}
	return out;
}

typedef std::pair<double, double> Coord;
typedef std::pair<Coord, Coord> CoordP;

SystemHistoryRepaTuple TBOT03::posesScansHistoryRepa(int d, const std::array<double,7>& pose, const std::array<double,360>& scan)
{
	auto lluu = listsSystem_u;

	std::size_t n = 360 + 3;
	std::size_t z = 1;
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
	for (size_t i = 0; i < n - 3; i++)
		rr[i] = (unsigned char)(scan[i] * f);
	rr[n - 3] = 0;
	rr[n - 2] = 0;
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

