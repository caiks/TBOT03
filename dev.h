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
#include <array>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <thread>
#include <chrono>
#include <ctime>
#include <string>
#include <cmath>

using Sec = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

namespace TBOT03
{
	struct Record
	{
		Record() {
			x = 0.0;
			y = 0.0;
			yaw = 0.0;
		}
		Record(double x1, double y1, double yaw1) {
			x = x1;
			y = y1;
			yaw = yaw1;
		}
		Record standard() const;
		Record config() const;
		Record flip() const;
		double squared(const Record&);
		void operator+=(const Record&);
		void operator/=(double);
		double x;
		double y;
		double yaw;
	};
	typedef std::vector<Record> RecordList;
	
	Record recordsMean(const RecordList&);
	double recordsDeviation(const RecordList&);

	void recordsPersistent(Record&, std::ostream&);

	void recordListsPersistent(RecordList&, std::ostream&);

	std::unique_ptr<RecordList> persistentsRecordList(std::istream&);

	typedef std::tuple<std::unique_ptr<Alignment::System>, std::unique_ptr<Alignment::SystemRepa>, std::unique_ptr<Alignment::HistoryRepa>> SystemHistoryRepaTuple;

	SystemHistoryRepaTuple posesScansHistoryRepa(int, const std::array<double,7>&, const std::array<double,360>&);

	SystemHistoryRepaTuple posesScansHistoryRepa_2(int, int, const std::array<double,7>&, const std::array<double,360>&);
}

std::ostream& operator<<(std::ostream& out, const TBOT03::Record&);
std::ostream& operator<<(std::ostream& out, const TBOT03::RecordList&);
std::ostream& operator<<(std::ostream& out, std::istream&);

#endif