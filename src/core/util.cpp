#include "util.h"



std::string FormatTime(std::chrono::system_clock::time_point time)
{
	auto ctime = std::chrono::system_clock::to_time_t(time);
	tm b;
	localtime_s(&b, &ctime);
	std::stringstream ss;
	ss  << std::setfill('0') << 1900+b.tm_year << "-" << std::setw(2) << b.tm_mon+1 << "-" << std::setw(2) << b.tm_mday << " " << b.tm_hour << ":" << std::setw(2) << b.tm_min << ":" << std::setw(2) << b.tm_sec;
	return ss.str();
}