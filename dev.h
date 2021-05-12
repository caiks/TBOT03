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
		}
		std::size_t id;
		double ts;
		double sensor_pose[7];
		double sensor_scan[360];
	};
	typedef std::vector<Record> RecordList;

	void recordsPersistent(Record&, std::ostream&);

	void recordListsPersistent(RecordList&, std::ostream&);

	std::unique_ptr<RecordList> persistentsRecordList(std::istream&);

	typedef std::tuple<std::unique_ptr<Alignment::System>, std::unique_ptr<Alignment::SystemRepa>, std::unique_ptr<Alignment::HistoryRepa>> SystemHistoryRepaTuple;

	SystemHistoryRepaTuple recordListsHistoryRepa(int, const RecordList&);
}

std::ostream& operator<<(std::ostream& out, const TBOT03::Record&);
std::ostream& operator<<(std::ostream& out, const TBOT03::RecordList&);
std::ostream& operator<<(std::ostream& out, std::istream&);

#endif