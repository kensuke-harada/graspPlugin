#ifndef _GRASPDATAGEN_UTIL_STOPWATCH_H_
#define _GRASPDATAGEN_UTIL_STOPWATCH_H_

#ifdef WIN32
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#else
#include <time.h>
#include <sys/time.h>
#endif

#include <iostream>
#include <string>

namespace grasp {
	class StopWatch {
	public:
		StopWatch() :
			display_str_(""),
			start_time_(0.0),
			end_time_(0.0),
			total_time_(0.0),
			in_running_(false) {
			os_ = &std::cout;
		}

		explicit StopWatch(const char* display) :
			display_str_(std::string(display)),
			start_time_(0.0),
			end_time_(0.0),
			total_time_(0.0),
			in_running_(false) {
			os_ = &std::cout;
		}

		virtual ~StopWatch() {;}

		void start() {
			in_running_ = true;
			start_time_ = getTimeStamp();
		}

		void stop() {
			if (in_running_) {
				end_time_ = getTimeStamp();
				total_time_ += getTime();
			}
			in_running_ = false;
		}

		double getTotalTime() {
			return total_time_;
		}

		void reset() {
			total_time_ = 0;
		}

		void resetAndStart() {
			reset();
			start();
		}

		double getTime() {
#ifdef WIN32
			return (end_time_-start_time_)/1000.0;
#else
			return end_time_-start_time_;
#endif
		};

		void printTime() {
			print(display_str_, getTime());
		}

		void printTime(const std::string& str) {
			print(str, getTime());
		}

		void printTotalTime() {
			print(display_str_, getTotalTime());
		}

		void stopAndPrintTime() {
			stop();
			printTime();
		}

		void stopAndPrintTime(const std::string& str) {
			stop();
			printTime(str);
		}

	private:

		double getTimeStamp() {
#ifdef WIN32
			return (double)timeGetTime();
#else
			struct timeval tv;
			gettimeofday(&tv, NULL);
			return tv.tv_sec + tv.tv_usec * 1e-6;
#endif
		}

		void print(const std::string& str, double time) {
#ifndef NOOUTPUT_TIMELOG
			*os_ << "[time_log] " << str << ": " << time << std::endl;
#endif
		}

		std::ostream* os_;

		double start_time_;
		double end_time_;
		double total_time_;
		std::string display_str_;
		bool in_running_;
	};

	class ScopeStopWatch :
		public StopWatch {
	public:
		explicit ScopeStopWatch(const char* display) :
			StopWatch(display) {
			start();
		}

		~ScopeStopWatch() {
			stop();
			printTotalTime();
		}
	};
}
#endif
