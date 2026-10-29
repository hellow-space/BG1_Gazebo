#pragma once
#include <cstdio>
#include <sstream>
#define ETHZ_LOG(level, ...) do { std::fprintf(stderr, "[" level "] "); std::fprintf(stderr, __VA_ARGS__); std::fputc('\n',stderr); std::fflush(stderr); } while(0)
#define ROS_INFO(...) ETHZ_LOG("INFO",__VA_ARGS__)
#define ROS_WARN(...) ETHZ_LOG("WARN",__VA_ARGS__)
#define ROS_ERROR(...) ETHZ_LOG("ERROR",__VA_ARGS__)
#define ROS_DEBUG(...) do {} while(0)
#define ETHZ_STREAM(level, x) do { std::ostringstream ethz_stream; ethz_stream << x; ETHZ_LOG(level,"%s",ethz_stream.str().c_str()); } while(0)
#define ROS_INFO_STREAM(x) ETHZ_STREAM("INFO",x)
#define ROS_WARN_STREAM(x) ETHZ_STREAM("WARN",x)
#define ROS_ERROR_STREAM(x) ETHZ_STREAM("ERROR",x)
#define ROS_DEBUG_STREAM(x) do {} while(0)
#define ROS_ERROR_COND(c,...) do { if(c) ROS_ERROR(__VA_ARGS__); } while(0)
#define ROS_WARN_COND(c,...) do { if(c) ROS_WARN(__VA_ARGS__); } while(0)
