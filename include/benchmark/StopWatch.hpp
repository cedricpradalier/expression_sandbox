/*
 * StopWatch.hpp
 *
 *  Created on: Nov 5, 2013
 *      Author: hannes
 */

#ifndef STOPWATCH_HPP_
#define STOPWATCH_HPP_


#include <chrono>
#include <bits/time.h>
#include <ctime>
#include <string>

namespace stop_watch {

namespace rdtsc {
  std::uint64_t now ();
  double toDouble(const decltype(now()- now()) & dur);
  std::string getUnitString();
  inline std::uint64_t getMaxDuration() { return std::numeric_limits<std::uint64_t>::max(); }
  inline std::uint64_t getMinDuration() { return 0; }
}

namespace hpet {
  timespec now ();
  timespec operator - (const timespec & a, const timespec & b);
  timespec & operator += (timespec & a, const timespec & b);
  timespec & operator -= (timespec & a, const timespec & b);

  double toDouble(const decltype(now()- now()) & dur);
  std::string getUnitString();
}


namespace cclock {
  std::clock_t now ();
  double toDouble(const decltype(now()- now()) & dur);
  std::string getUnitString();
}

namespace highres_clock {
  decltype(std::chrono::high_resolution_clock::now()) now ();
  double toDouble(const decltype(now()- now()) & dur);
  std::string getUnitString();
}

namespace clock = rdtsc;

using namespace clock;

class StopWatch {
 public:
  typedef decltype(now()) TimeType;
  struct Duration {
    typedef decltype(now() - now()) DurationType;
    DurationType duration;

    Duration(bool initWithMaxDuration = false) : duration(initWithMaxDuration ? getMaxDuration() : getMinDuration()){}
    inline Duration(const Duration &) = default;
    inline Duration(DurationType d) : duration(d){}

    void operator += (const Duration & d) {
      duration += d.duration;
    };
    void operator -= (const Duration & d) {
      duration -= d.duration;
    };

    bool operator > (const Duration & d) const {
      return duration > d.duration;
    };
    bool operator < (const Duration & d) const {
      return duration < d.duration;
    };

    operator double () const {
      return toDouble(duration);
    }
  };

  inline Duration read() {
    return now() - startTime;
  }

  Duration readAndReset() { auto r = read(); reset(); return r;}

  StopWatch(){
    reset();
  }
  void reset(){
    startTime = now();
  }
 private:
  TimeType startTime;
};
std::ostream & operator << (std::ostream & out, const typename StopWatch::Duration & duration);


class CollectingStopWatch : public StopWatch {
 public:
   CollectingStopWatch() : running(false) {
   }

   void start (){
     reset();
     running = true;
   }

   void stop (){
     if(running){
       collected += read();
       running = false;
     }
   }

   inline Duration getAndForgetCollected (){
     Duration d = collected;
     collected = Duration();
     return d;
   }

 private:
  Duration collected;
  bool running;
};


} // namespace stop_watch

#endif /* STOPWATCH_HPP_ */
