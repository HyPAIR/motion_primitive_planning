//
// Created by weijian on 17/11/23.
//
#pragma once
#include <chrono>

namespace liom_local_planner {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
  return ((double) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count() / 1000);
}


}