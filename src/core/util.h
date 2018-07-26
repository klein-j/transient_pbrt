#pragma once


// ---------- some time formatting stuff ---------
#include <chrono>
#include <time.h>
#include <sstream>
#include <iomanip>

std::string FormatTime(std::chrono::system_clock::time_point time);