/*
 * StopWatch.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: hannes
 */

#include <iostream>
#include <benchmark/StopWatch.hpp>

namespace stop_watch {
std::ostream & operator << (std::ostream & out, const typename StopWatch::duration::durationType & duration){
  out << std::chrono::duration_cast<std::chrono::duration<double> >(duration).count();
  return out;
}

std::ostream & operator << (std::ostream & out, const typename StopWatch::duration & duration){
  out << duration.timeDuration << "s" << ", CPU:" << ((double) duration.cpuDuration / 1000000) << "s";
  return out;
}

}
