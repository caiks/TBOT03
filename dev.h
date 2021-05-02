#ifndef DEV_H
#define DEV_H

#include "AlignmentUtil.h"
#include "Alignment.h"
#include "AlignmentApprox.h"
#include "AlignmentAeson.h"
#include "AlignmentRepa.h"
#include "AlignmentAesonRepa.h"
#include "AlignmentRandomRepa.h"
#include "AlignmentPracticableRepa.h"
#include "AlignmentPracticableIORepa.h"
#include "AlignmentActive.h"

#include <iomanip>
#include <set>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <thread>
#include <chrono>
#include <ctime>
#include <string>

namespace TBOT03
{
	struct Record
	{
		Record() {
			id = 0;
			ts = 0.0;
			for (std::size_t i = 0; i < 7; i++)
				sensor_pose[i] = 0.0;
			for (std::size_t i = 0; i < 360; i++)
				sensor_scan[i] = 0.0;
			action_linear = 0.0;
			action_angular = 0.0;
		}
		std::size_t id;
		double ts;
		double sensor_pose[7];
		double sensor_scan[360];
		double action_linear;
		double action_angular;
	};
	typedef std::vector<Record> RecordList;

	void recordsPersistent(Record&, std::ostream&);

	void recordListsPersistent(RecordList&, std::ostream&);

	std::unique_ptr<RecordList> persistentsRecordList(std::istream&);

	struct Bitmap
	{
		Bitmap(int h = 1, int w = 1, unsigned char x = 0) {
			height = h;
			width = w;
			image.resize(h*w * 3, x);
		}
		int height;
		int width;
		std::vector<unsigned char> image;
	};

	Bitmap bminsert(const Bitmap&, int, int, const Bitmap&);
	Bitmap bmborder(int, const Bitmap&);
	Bitmap bmhstack(const std::vector<Bitmap>&);
	Bitmap bmvstack(const std::vector<Bitmap>&);

	void bmwrite(std::string, const Bitmap&);

	Bitmap historyRepasBitmap(int, int, const Alignment::HistoryRepa&);
	Bitmap historyRepasBitmapUncentred(int, int, const Alignment::HistoryRepa&);
	Bitmap historyRepasBitmapAverage(int, int, const Alignment::HistoryRepa&);

	typedef std::tuple<std::unique_ptr<Alignment::System>, std::unique_ptr<Alignment::SystemRepa>, std::unique_ptr<Alignment::HistoryRepa>> SystemHistoryRepaTuple;

	SystemHistoryRepaTuple recordListsHistoryRepa(int, const RecordList&);
	SystemHistoryRepaTuple recordListsHistoryRepaRegion(int, int, int, const RecordList&, int event_start = 0, int event_end = 0);
	SystemHistoryRepaTuple recordListsHistoryRepa_2(int, const RecordList&);
	SystemHistoryRepaTuple recordListsHistoryRepa_3(int, const RecordList&);
	SystemHistoryRepaTuple recordListsHistoryRepa_4(int, const RecordList&);
	SystemHistoryRepaTuple recordListsHistoryRepa_5(int, const RecordList&);
	SystemHistoryRepaTuple recordListsHistoryRepa_6(int, const RecordList&);
}

std::ostream& operator<<(std::ostream& out, const TBOT03::Record&);
std::ostream& operator<<(std::ostream& out, const TBOT03::RecordList&);
std::ostream& operator<<(std::ostream& out, std::istream&);

#endif