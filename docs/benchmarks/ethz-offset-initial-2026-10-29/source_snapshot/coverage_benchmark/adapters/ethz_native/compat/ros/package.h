#pragma once
#include <cstdlib>
#include <stdexcept>
#include <string>
namespace ros { namespace package {
inline std::string getPath(const std::string& name) {
    const char* prefix=std::getenv("ETHZ_BUILD_ROOT");
    if(!prefix) throw std::runtime_error("ETHZ_BUILD_ROOT missing");
    return std::string(prefix)+"/src/"+name;
}
}}
