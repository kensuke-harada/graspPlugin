#ifndef _GRASPDATAGEN_UTIL_LOGGER_H_
#define _GRASPDATAGEN_UTIL_LOGGER_H_

#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

//#define DEBUG_OUTPUT

namespace grasp {
	class Logger {
	public:
		enum LEVEL {
			NO_OUTPUT, ERROR, WARN, INFO, DEBUG
		};
		
		Logger() {
			output_level_ = INFO;
			os_ = &std::cout;
		}
	
		explicit Logger(const std::string& ini_file_name) {
			;
		}
	
		virtual ~Logger() {
			//delete os_;
		}

		void error(const std::string& msg) {
			if (isOutputLevel(ERROR)) {
				print("[error]   ", msg);
			}
		}

		void error(const char* format, ...) {
			if (isOutputLevel(ERROR)) {
				va_list args;
				va_start(args, format);
				print("[error]   ", format, args);
				va_end(args);
			}
		}
		
		void warn(const std::string& msg) {
			if (isOutputLevel(WARN)) {
				print("[warning] ", msg);
			}
		}

		void warn(const char* format, ...) {
			if (isOutputLevel(WARN)) {
				va_list args;
				va_start(args, format);
				print("[warning] ", format, args);
				va_end(args);
			}
		}
	
		void info(const std::string& msg) {
			if (isOutputLevel(INFO)) {
				print("[info]    ", msg);
			}
		}

		void info(const char* format, ...) {
			if (isOutputLevel(INFO)) {
				va_list args;
				va_start(args, format);
				print("[info]    ", format, args);
				va_end(args);
			}
		}
	
		void debug(const std::string& msg) {
#ifdef DEBUG_OUTPUT
			if (isOutputLevel(DEBUG)) {
				print("[debug]   ", msg);
			}
#endif
		}

		void debug(const char* format, ...) {
#ifdef DEBUG_OUTPUT
			if (isOutputLevel(DEBUG)) {
				va_list args;
				va_start(args, format);
				print("[debug]   ", format, args);
				va_end(args);
			}
#endif
		}
		
		void setOutputLevel(LEVEL level) {
			output_level_ = level;
		}
	
	private:
		std::ostream* os_;
		LEVEL output_level_;

		bool isOutputLevel(LEVEL target_level) const {
			return (output_level_ >= target_level);
		}

		void print(const std::string& prefix, const std::string& msg) {
			*os_ << prefix << msg << std::endl;
		}

		void print(const std::string& prefix, const char* format, va_list args) {
			char msg[1024];
			vsprintf(msg, format, args);
			*os_ << prefix << msg << std::endl;
		}
	};
}

#endif /* _GRASPDATAGEN_UTIL_LOGGER_H_ */










