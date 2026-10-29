#pragma once
#include <cstdlib>
#include <cstring>
#include "ros/console.h"
#define ROS_ASSERT(c) do { if(!(c)) { ROS_ERROR("assertion failed: %s (%s:%d)",#c,__FILE__,__LINE__); std::abort(); } } while(0)
#define ROS_ASSERT_MSG(c,...) do { if(!(c)) { ROS_ERROR(__VA_ARGS__); std::abort(); } } while(0)
