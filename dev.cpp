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
	out.write(reinterpret_cast<char*>(&r.action_linear), sizeof(double));
	out.write(reinterpret_cast<char*>(&r.action_angular), sizeof(double));
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
		in.read(reinterpret_cast<char*>(&r.action_linear), sizeof(double));
		in.read(reinterpret_cast<char*>(&r.action_angular), sizeof(double));
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
	out << ")," << r.action_linear << "," << r.action_angular << ")";
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
		in.read(reinterpret_cast<char*>(&r.action_linear), sizeof(double));
		in.read(reinterpret_cast<char*>(&r.action_angular), sizeof(double));
		out << r << std::endl;
	}
	return out;
}

Bitmap TBOT03::bminsert(const Bitmap& bm2, int ox, int oy, const Bitmap& bm1)
{
	Bitmap bm3(bm2);
	for (int i = 0; i<bm1.height; i++)
		for (int j = 0; j<bm1.width; j++)
			for (int l = 0; l<3; l++)
				bm3.image[(i + ox)*bm3.width * 3 + (j + oy) * 3 + l] = bm1.image[i*bm1.width * 3 + j * 3 + l];
	return bm3;
}

Bitmap TBOT03::bmborder(int b, const Bitmap& bm)
{
	return bminsert(Bitmap(b * 2 + bm.height, b * 2 + bm.width, 255), b, b, bm);
}

Bitmap TBOT03::bmhstack(const std::vector<Bitmap>& ll)
{
	if (!ll.size())
	{
		cout << "bmhstack : empty list" << endl;
		return Bitmap();
	}
	int h = ll[0].height;
	int w = 0;
	for (int k = 0; k<ll.size(); k++)
		w += ll[k].width;
	Bitmap bm1(h, w);
	int q = 0;
	for (int k = 0; k < ll.size(); k++)
	{
		auto& bm = ll[k];
		for (int i = 0; i<bm.height; i++)
			for (int j = 0; j<bm.width; j++)
				for (int l = 0; l<3; l++)
					bm1.image[(i)*bm1.width * 3 + (j + q) * 3 + l] = bm.image[i*bm.width * 3 + j * 3 + l];
		q += ll[k].width;
	}
	return bm1;
}

Bitmap TBOT03::bmvstack(const std::vector<Bitmap>& ll)
{
	if (!ll.size())
	{
		cout << "bmvstack : empty list" << endl;
		return Bitmap();
	}
	int w = 0;
	int h = 0;
	for (int k = 0; k < ll.size(); k++)
	{
		h += ll[k].height;
		if (ll[k].width > w)
			w = ll[k].width;
	}
	Bitmap bm1(h, w);
	int q = 0;
	for (int k = 0; k < ll.size(); k++)
	{
		auto& bm = ll[k];
		for (int i = 0; i<bm.height; i++)
			for (int j = 0; j<bm.width; j++)
				for (int l = 0; l<3; l++)
					bm1.image[(i + q)*bm1.width * 3 + j * 3 + l] = bm.image[i*bm.width * 3 + j * 3 + l];
		q += ll[k].height;
	}
	return bm1;
}

// https://stackoverflow.com/questions/2654480/writing-bmp-image-in-pure-c-c-without-other-libraries

const int bytesPerPixel = 3; /// red, green, blue
const int fileHeaderSize = 14;
const int infoHeaderSize = 40;

unsigned char* createBitmapFileHeader(int height, int width, int paddingSize) {
	int fileSize = fileHeaderSize + infoHeaderSize + (bytesPerPixel*width + paddingSize) * height;

	static unsigned char fileHeader[] = {
		0,0, /// signature
		0,0,0,0, /// image file size in bytes
		0,0,0,0, /// reserved
		0,0,0,0, /// start of pixel array
	};

	fileHeader[0] = (unsigned char)('B');
	fileHeader[1] = (unsigned char)('M');
	fileHeader[2] = (unsigned char)(fileSize);
	fileHeader[3] = (unsigned char)(fileSize >> 8);
	fileHeader[4] = (unsigned char)(fileSize >> 16);
	fileHeader[5] = (unsigned char)(fileSize >> 24);
	fileHeader[10] = (unsigned char)(fileHeaderSize + infoHeaderSize);

	return fileHeader;
}

unsigned char* createBitmapInfoHeader(int height, int width) {
	static unsigned char infoHeader[] = {
		0,0,0,0, /// header size
		0,0,0,0, /// image width
		0,0,0,0, /// image height
		0,0, /// number of color planes
		0,0, /// bits per pixel
		0,0,0,0, /// compression
		0,0,0,0, /// image size
		0,0,0,0, /// horizontal resolution
		0,0,0,0, /// vertical resolution
		0,0,0,0, /// colors in color table
		0,0,0,0, /// important color count
	};

	infoHeader[0] = (unsigned char)(infoHeaderSize);
	infoHeader[4] = (unsigned char)(width);
	infoHeader[5] = (unsigned char)(width >> 8);
	infoHeader[6] = (unsigned char)(width >> 16);
	infoHeader[7] = (unsigned char)(width >> 24);
	infoHeader[8] = (unsigned char)(height);
	infoHeader[9] = (unsigned char)(height >> 8);
	infoHeader[10] = (unsigned char)(height >> 16);
	infoHeader[11] = (unsigned char)(height >> 24);
	infoHeader[12] = (unsigned char)(1);
	infoHeader[14] = (unsigned char)(bytesPerPixel * 8);

	return infoHeader;
}

void generateBitmapImage(const unsigned char *image, int height, int width, const char* imageFileName) {

	unsigned char padding[3] = { 0, 0, 0 };
	int paddingSize = (4 - (width*bytesPerPixel) % 4) % 4;

	unsigned char* fileHeader = createBitmapFileHeader(height, width, paddingSize);
	unsigned char* infoHeader = createBitmapInfoHeader(height, width);

	FILE* imageFile = fopen(imageFileName, "wb");

	fwrite(fileHeader, 1, fileHeaderSize, imageFile);
	fwrite(infoHeader, 1, infoHeaderSize, imageFile);

	int i;
	for (i = 0; i<height; i++) {
		fwrite(image + (i*width*bytesPerPixel), bytesPerPixel, width, imageFile);
		fwrite(padding, 1, paddingSize, imageFile);
	}

	fclose(imageFile);
}

void TBOT03::bmwrite(string imageFileName, const Bitmap& bm)
{
	try
	{
		generateBitmapImage(bm.image.data(), bm.height, bm.width, imageFileName.data());
	}
	catch (const exception& e)
	{
		cout << "bmwrite : " << e.what() << endl;
		return;
	}
}

Bitmap TBOT03::historyRepasBitmap(int c, int d, const HistoryRepa& hr)
{
	int n = (int)hr.dimension;
	int z = (int)hr.size;
	Bitmap bm(z*c, n);
	auto rr = hr.arr;
	std::size_t p = 0;
	for (int j = 0; j<n; j++) {
		for (int i = 0; i < z; i++) {
			for (int m = 0; m < c; m++)
			{
				int k = (i*c+m)*n * 3 + (n-1 - ((j + n/2) % n)) * 3;
				for (int l = 0; l<3; l++)
					bm.image[k + l] = 255 - rr[p]*255/d;
			}
			p++;
		}
	}
	return bm;
}

Bitmap TBOT03::historyRepasBitmapUncentred(int c, int d, const HistoryRepa& hr)
{
	int n = (int)hr.dimension;
	int z = (int)hr.size;
	Bitmap bm(z*c, n);
	auto rr = hr.arr;
	std::size_t p = 0;
	for (int j = 0; j<n; j++) {
		for (int i = 0; i < z; i++) {
			for (int m = 0; m < c; m++)
			{
				int k = (i*c+m)*n * 3 + (n - 1 - j) * 3;
				for (int l = 0; l<3; l++)
					bm.image[k + l] = 255 - rr[p]*255/d;
			}
			p++;
		}
	}
	return bm;
}

Bitmap TBOT03::historyRepasBitmapAverage(int c, int d, const HistoryRepa& hr)
{
	int n = (int)hr.dimension;
	int z = (int)hr.size;
	Bitmap bm(c, n);
	auto rr = hr.arr;
	vector<size_t> av(n);
	for (std::size_t j = 0; j < z; j++)
		for (std::size_t i = 0; i < n; i++)
			av[i] += rr[hr.evient ? j*n + i : i*z + j];
	for (std::size_t i = 0; i < n; i++)
		av[i] = 255 - av[i] * 255 / (d - 1) / z;
	for (int i = 0; i < n; i++) {
		for (int m = 0; m < c; m++)
		{
			unsigned char x = (unsigned char)av[i];
			int k = m*n * 3 + (n-1 - ((i + n / 2) % n)) * 3;
			for (int l = 0; l<3; l++)
				bm.image[k + l] = x;
		}
	}
	return bm;
}


SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;
	auto uuur = systemsSystemRepa;

	std::size_t n = 360;
	std::size_t z = qq.size();
	ValSet buckets;
	for (int i = 0; i < d; i++)
		buckets.insert(Value(i));
	vector<VarValSetPair> ll;
	auto vscan = std::make_shared<Variable>("scan");
	for (std::size_t i = 0; i < n; i++)
		ll.push_back(VarValSetPair(Variable(vscan, std::make_shared<Variable>((int)i+1)), buckets));
	auto uu = lluu(ll);
	auto ur = uuur(*uu);
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
	for (size_t i = 0; i < n; i++)
		sh[i] = d;
	double f = (double)d/4.0;
	for (size_t j = 0; j < z; j++)
	{
		size_t jn = j*n;
		auto& r = qq[j];
		for (size_t i = 0; i < n; i++)
			rr[jn + i] = (unsigned char)(r.sensor_scan[i] * f);
	}
	hr->transpose();
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepaRegion(int d, int n, int s, const RecordList& qq0, int event_start, int event_end)
{
	auto lluu = listsSystem_u;
	auto uuur = systemsSystemRepa;

	srand(s);
	RecordList qq;
	if (event_end > 0)
		for (size_t i = event_start; i <= event_end; i++)
			qq.push_back(qq0[i]);
	else
		qq = qq0;
	std::size_t z = qq.size();
	ValSet buckets;
	for (int i = 0; i < d; i++)
		buckets.insert(Value(i));
	vector<VarValSetPair> ll;
	auto vscan = std::make_shared<Variable>("scan");
	for (std::size_t i = 0; i < n; i++)
		ll.push_back(VarValSetPair(Variable(vscan, std::make_shared<Variable>((int)i + 1)), buckets));
	auto uu = lluu(ll);
	auto ur = uuur(*uu);
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
	for (size_t i = 0; i < n; i++)
		sh[i] = d;
	double f = (double)d / 4.0;
	for (size_t j = 0; j < z; j++)
	{
		auto oi = rand() % 360;
		size_t jn = j*n;
		auto& r = qq[j];
		for (size_t i = 0; i < n; i++)
			rr[jn + i] = (unsigned char)(r.sensor_scan[(oi + i) % 360] * f);
	}
	hr->transpose();
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

typedef std::pair<double, double> Coord;
typedef std::pair<Coord, Coord> CoordP;

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa_2(int d, const RecordList& qq)
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
	ValSet positions{ Value("centre"), Value("corner"), Value("side") };
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
	ll.push_back(VarValSetPair(Variable("location"), locations));
	ll.push_back(VarValSetPair(Variable("position"), positions));
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
	sh[n - 2] = locations.size();
	sh[n - 1] = positions.size();
	double f = (double)d / 4.0;
	for (size_t j = 0; j < z; j++)
	{
		size_t jn = j*n;
		auto& r = qq[j];
		for (size_t i = 0; i < n - 3; i++)
			rr[jn + i] = (unsigned char)(r.sensor_scan[i] * f);
		rr[jn + n - 3] = (unsigned char)(r.action_angular == 1.5 ? 0 : (r.action_angular == -1.5 ? 2 : 1));
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
		rr[jn + n - 2] = (unsigned char)k;
		k = 0;
		while (k < rooms.size())
		{
			auto room = rooms[k];
			if (room.first.first <= x && x <= room.second.first && room.first.second <= y && y <= room.second.second)
				break;
			k++;
		}
		if (k < rooms.size() && ((x - rooms[k].first.first <= 1.0 && (y - rooms[k].first.second <= 1.0 || rooms[k].second.second - y <= 1.0)) || (rooms[k].second.first - x <= 1.0 && (y - rooms[k].first.second <= 1.0 || rooms[k].second.second - y <= 1.0))))
			rr[jn + n - 1] = (unsigned char)1;
		else if (k < rooms.size() && (x - rooms[k].first.first <= 1.0 || rooms[k].second.first - x <= 1.0 || y - rooms[k].first.second <= 1.0 || rooms[k].second.second - y <= 1.0))
			rr[jn + n - 1] = (unsigned char)2;
		else
			rr[jn + n - 1] = (unsigned char)0;
	}
	hr->transpose();
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa_3(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;
	auto hrjoin = vectorHistoryRepasJoin_u;
	
	auto xx = recordListsHistoryRepa_2(d, qq);
	auto uu = std::move(std::get<0>(xx));
	auto ur = std::move(std::get<1>(xx));
	auto hr = std::move(std::get<2>(xx));

	auto& vvu = ur->mapVarSize();	
	auto& llu = ur->listVarSizePair;	
	auto z = hr->size;
	auto rr = hr->arr;
	auto& mvv = hr->mapVarInt();
	
	vector<Variable> ll {Variable("location"),Variable("position")};
	SizeList pp;
	for (auto v : ll)
		pp.push_back(mvv[vvu[v]]);
	
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room3"), Value("room4"), Value("room5"), Value("room6"), Value("unknown") };
	ValSet positions{ Value("centre"), Value("corner"), Value("side"), Value("unknown") };
	
	vector<VarValSetPair> ll1;
	ll1.push_back(VarValSetPair(Variable("location_next"), locations));
	ll1.push_back(VarValSetPair(Variable("position_next"), positions));
	uu->update(*lluu(ll1));	
	
	std::size_t n1 = ll1.size();
	auto hr1 = make_unique<HistoryRepa>();
	hr1->dimension = n1;
	hr1->vectorVar = new size_t[n1];
	auto vv1 = hr1->vectorVar;
	hr1->shape = new size_t[n1];
	auto sh1 = hr1->shape;
	hr1->size = z;
	hr1->evient = false;
	hr1->arr = new unsigned char[z*n1];
	auto rr1 = hr1->arr;
	for (size_t i = 0; i < n1; i++)
	{
		auto& vww = ll1[i];
		llu.push_back(VarSizePair(std::make_shared<Variable>(vww.first), vww.second.size()));
		vv1[i] = llu.size() - 1;	
		sh1[i] = vww.second.size();
	}
	for (size_t i = 0; i < n1; i++)
	{
		size_t iz = i*z;
		for (size_t j = 0; j < z; j++)
		{
			rr1[iz + j] = (unsigned char)(sh1[i]-1);
			auto u = rr[pp[i]*z + j];
			for (size_t k = j+1; k < z; k++)
			{
				auto w = rr[pp[i]*z + k];
				if (w != u)
				{
					rr1[iz + j] = w;		
					break;					
				}
			}
		}
	}
	hr = hrjoin(HistoryRepaPtrList {move(hr), move(hr1)});	
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa_4(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;
	auto hrjoin = vectorHistoryRepasJoin_u;
	
	auto xx = recordListsHistoryRepa_3(d, qq);
	auto uu = std::move(std::get<0>(xx));
	auto ur = std::move(std::get<1>(xx));
	auto hr = std::move(std::get<2>(xx));

	auto& vvu = ur->mapVarSize();	
	auto& llu = ur->listVarSizePair;	
	auto z = hr->size;
	auto rr = hr->arr;
	auto& mvv = hr->mapVarInt();
	
	vector<Variable> ll {Variable("location")};
	SizeList pp;
	for (auto v : ll)
		pp.push_back(mvv[vvu[v]]);
	
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room3"), Value("room4"), Value("room5"), Value("room6"), Value("unknown") };
	
	vector<VarValSetPair> ll1;
	ll1.push_back(VarValSetPair(Variable("room_next"), locations));
	uu->update(*lluu(ll1));	
	
	std::size_t n1 = ll1.size();
	auto hr1 = make_unique<HistoryRepa>();
	hr1->dimension = n1;
	hr1->vectorVar = new size_t[n1];
	auto vv1 = hr1->vectorVar;
	hr1->shape = new size_t[n1];
	auto sh1 = hr1->shape;
	hr1->size = z;
	hr1->evient = false;
	hr1->arr = new unsigned char[z*n1];
	auto rr1 = hr1->arr;
	for (size_t i = 0; i < n1; i++)
	{
		auto& vww = ll1[i];
		llu.push_back(VarSizePair(std::make_shared<Variable>(vww.first), vww.second.size()));
		vv1[i] = llu.size() - 1;	
		sh1[i] = vww.second.size();
	}
	for (size_t i = 0; i < n1; i++)
	{
		size_t iz = i*z;
		for (size_t j = 0; j < z; j++)
		{
			rr1[iz + j] = (unsigned char)(sh1[i]-1);
			auto u = rr[pp[i]*z + j];
			for (size_t k = j+1; k < z; k++)
			{
				auto w = rr[pp[i]*z + k];
				if (w != u && (int)w >= 5)
				{
					rr1[iz + j] = w;		
					break;					
				}
			}
		}
	}
	hr = hrjoin(HistoryRepaPtrList {move(hr), move(hr1)});	
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa_5(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;

	std::size_t n = 360 + 2;
	std::size_t z = qq.size();
	ValSet buckets;
	for (int i = 0; i < d; i++)
		buckets.insert(Value(i));
	ValSet actions;
	for (int i = 0; i < 3; i++)
		actions.insert(Value(i));
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room2z"), Value("room3"), Value("room3z"), Value("room4"), Value("room5"), Value("room6"), 
		Value("room6z") };
	vector<Coord> doors{ Coord(6.2,-0.175), Coord(2.3,4.5), Coord(2.3,0.375), Coord(-5.15,3.1), Coord(-6.325,0.925) };
	vector<CoordP> rooms{
		CoordP(Coord(2.3,-0.175),Coord(7.5,5.27)),
		CoordP(Coord(4.9,-5.275+1.5),Coord(7.5,-0.175)),
		CoordP(Coord(4.9,-5.275),Coord(7.5,-5.275+1.5)),
		CoordP(Coord(-0.05,0.925+1.5),Coord(2.3,5.27)),
		CoordP(Coord(-0.05,0.925),Coord(2.3,0.925+1.5)),
		CoordP(Coord(-5.15,-0.175),Coord(2.3,5.27)),
		CoordP(Coord(-7.5,0.925),Coord(-5.15,5.27)),
		CoordP(Coord(-7.5,-3.925+1.5),Coord(-5.15,0.925)),
		CoordP(Coord(-7.5,-3.925),Coord(-5.15,-3.925+1.5))
		};
	vector<VarValSetPair> ll;
	auto vscan = std::make_shared<Variable>("scan");
	for (std::size_t i = 0; i < n - 2; i++)
		ll.push_back(VarValSetPair(Variable(vscan, std::make_shared<Variable>((int)i + 1)), buckets));
	ll.push_back(VarValSetPair(Variable("motor"), actions));
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
	for (size_t i = 0; i < n - 2; i++)
		sh[i] = d;
	sh[n - 2] = actions.size();
	sh[n - 1] = locations.size();
	double f = (double)d / 4.0;
	for (size_t j = 0; j < z; j++)
	{
		size_t jn = j*n;
		auto& r = qq[j];
		for (size_t i = 0; i < n - 2; i++)
			rr[jn + i] = (unsigned char)(r.sensor_scan[i] * f);
		rr[jn + n - 2] = (unsigned char)(r.action_angular == 1.5 ? 0 : (r.action_angular == -1.5 ? 2 : 1));
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

SystemHistoryRepaTuple TBOT03::recordListsHistoryRepa_6(int d, const RecordList& qq)
{
	auto lluu = listsSystem_u;
	auto hrjoin = vectorHistoryRepasJoin_u;
	
	auto xx = recordListsHistoryRepa_5(d, qq);
	auto uu = std::move(std::get<0>(xx));
	auto ur = std::move(std::get<1>(xx));
	auto hr = std::move(std::get<2>(xx));

	auto& vvu = ur->mapVarSize();	
	auto& llu = ur->listVarSizePair;	
	auto z = hr->size;
	auto rr = hr->arr;
	auto& mvv = hr->mapVarInt();
	
	vector<Variable> ll {Variable("location")};
	SizeList pp;
	for (auto v : ll)
		pp.push_back(mvv[vvu[v]]);
	
	ValSet locations{ Value("door12"), Value("door13"), Value("door14"), Value("door45"), Value("door56"),
		Value("room1"), Value("room2"), Value("room2z"), Value("room3"), Value("room3z"), Value("room4"), 
		Value("room5"), Value("room6"), Value("room6z"), Value("unknown") };
	
	vector<VarValSetPair> ll1;
	ll1.push_back(VarValSetPair(Variable("room_next"), locations));
	uu->update(*lluu(ll1));	
	
	std::size_t n1 = ll1.size();
	auto hr1 = make_unique<HistoryRepa>();
	hr1->dimension = n1;
	hr1->vectorVar = new size_t[n1];
	auto vv1 = hr1->vectorVar;
	hr1->shape = new size_t[n1];
	auto sh1 = hr1->shape;
	hr1->size = z;
	hr1->evient = false;
	hr1->arr = new unsigned char[z*n1];
	auto rr1 = hr1->arr;
	for (size_t i = 0; i < n1; i++)
	{
		auto& vww = ll1[i];
		llu.push_back(VarSizePair(std::make_shared<Variable>(vww.first), vww.second.size()));
		vv1[i] = llu.size() - 1;	
		sh1[i] = vww.second.size();
	}
	for (size_t i = 0; i < n1; i++)
	{
		size_t iz = i*z;
		for (size_t j = 0; j < z; j++)
		{
			rr1[iz + j] = (unsigned char)(sh1[i]-1);
			auto u = rr[pp[i]*z + j];
			for (size_t k = j+1; k < z; k++)
			{
				auto w = rr[pp[i]*z + k];
				if (w != u && (int)w >= 5)
				{
					rr1[iz + j] = w;		
					break;					
				}
			}
		}
	}
	hr = hrjoin(HistoryRepaPtrList {move(hr), move(hr1)});	
	return SystemHistoryRepaTuple(move(uu), move(ur), move(hr));
}

