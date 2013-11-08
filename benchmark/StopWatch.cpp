/*
 * StopWatch.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: hannes
 */

#include <cinttypes>
#include <iostream>
#include <benchmark/StopWatch.hpp>

namespace stop_watch {



namespace rdtsc {
std::uint64_t now (){
    unsigned int hi, lo;
    __asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi));
    return ((std::uint64_t)hi << 32) | lo;
}

double toDouble(const decltype(now()- now()) & dur){
  return (double)(dur) / 1E9;
}

  std::string getUnitString() {
    return "Gt";
  }
}

namespace hpet {

struct timespec  now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts;
}

double toDouble(const decltype(now()- now()) & dur){
  return (double)(dur.tv_sec) + double(dur.tv_nsec)/ 1E9;
}

timespec operator - (const timespec & a, const timespec & b){
  return { a.tv_sec - a.tv_sec, a.tv_nsec - b.tv_nsec};
}

timespec & operator += (timespec & a, const timespec & b){
  a.tv_sec += a.tv_sec;
  a.tv_nsec += b.tv_nsec;
  return a;
}

timespec & operator -= (timespec & a, const timespec & b){
  a.tv_sec -= a.tv_sec;
  a.tv_nsec -= b.tv_nsec;
  return a;
}



std::string getUnitString() {
  return "s";
}
}

namespace cclock {
std::clock_t now() {
  return std::clock();
}

double toDouble(const decltype(now()- now()) & dur){
  return (double)(dur) / 1000000;
}

std::string getUnitString() {
  return "s";
}
}

namespace highres_clock {
decltype(std::chrono::high_resolution_clock::now()) now() {
  return std::chrono::high_resolution_clock::now();
}

double toDouble(const decltype(now()- now()) & dur){
  return std::chrono::duration_cast<std::chrono::duration<double> >(dur).count();
}
std::string getUnitString() {
  return "s";
}
}

std::ostream & operator << (std::ostream & out, const typename StopWatch::Duration & duration){
  out << (double)duration << " " << clock::getUnitString();
  return out;
}

}
