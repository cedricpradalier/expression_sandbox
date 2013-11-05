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


namespace stop_watch {
class StopWatch {
 public:
  struct duration {
    typedef decltype(std::chrono::steady_clock::now() - std::chrono::steady_clock::now()) durationType;
    decltype(std::chrono::steady_clock::now() - std::chrono::steady_clock::now()) timeDuration;
    std::clock_t cpuDuration;

    void operator += (const duration & d) {
      cpuDuration += d.cpuDuration;
      timeDuration += d.timeDuration;
    };
    void operator -= (const duration & d) {
      cpuDuration -= d.cpuDuration;
      timeDuration -= d.timeDuration;
    };
  };

  duration read() { return {std::chrono::steady_clock::now() - timeStart, std::clock() - cpuStart};}
  duration readAndReset() { auto r = read(); reset(); return r;}

  StopWatch(){
    reset();
  }
  void reset(){
    timeStart = std::chrono::steady_clock::now();
    cpuStart = std::clock();
  }
 private:
  std::chrono::steady_clock::time_point timeStart;
  std::clock_t cpuStart;
};

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

   inline duration getAndForgetCollected (){
     duration d = collected;
     collected = duration();
     return d;
   }

 private:
  duration collected;
  bool running;
};


std::ostream & operator << (std::ostream & out, const typename StopWatch::duration::durationType & duration);
std::ostream & operator << (std::ostream & out, const typename StopWatch::duration & duration);

} // namespace stop_watch

#endif /* STOPWATCH_HPP_ */
