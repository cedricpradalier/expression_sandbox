/*
 * Benchmarker.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <fstream>
#include <benchmark/Benchmarker.hpp>

#ifndef NDEBUG
#define DEFAULT_NRUNS 1
#else
#define DEFAULT_NRUNS 1E6
#endif

namespace benchmark {
std::vector<const Benchmark*> Benchmark::benchmarkerRegister;


std::ostream & operator << (std::ostream & out, const typename StopWatch::duration::durationType & duration){
  out << std::chrono::duration_cast<std::chrono::duration<double> >(duration).count();
  return out;
}

std::ostream & operator << (std::ostream & out, const typename StopWatch::duration & duration){
  out << duration.duration << "s" << ", CPU:" << ((double) duration.cpuDuration / 1000000) << "s";
  return out;
}

}


int main(int argc, const char **argv) {
  using namespace benchmark;

  const int nRuns = argc > 1 ? atoi(argv[1]) : DEFAULT_NRUNS;
  const bool verbose = false;

//  if(verbose){
//    std::cout << "found these solver:";
//    int i = 0;
//    for(auto s : ProblemSolver<Problem>::getSolvers()){
//      if(i++) std::cout << ", ";
//      std::cout << s->getName();
//    }
//    std::cout << ".\n";
//  }

  std::string sep("\t");

  struct Output {
    std::ostream & out;
    int nameWidth;
    const char * sep;
    bool showErrors;
  };

  std::filebuf csvBuffer;
  std::ostream csv(&csvBuffer);
  csvBuffer.open("stat.csv", std::ios::out | std::ios::trunc);
  std::vector<Output> outputs = { {std::cout, 50, "\t", true}, {csv, 1, ",", false} };


  for(const Benchmark * bp: Benchmark::getBenchmarks()){
    auto b = bp->createInstance(verbose);
    for(Output & o : outputs) o.out << "NEW PROBLEM (" << b->getProblemName() << "):" << std::endl;
    for(Output & o : outputs) b->printHeader(o.out, o.nameWidth, o.sep, o.showErrors);


    for(int numberOfRepetitions = 1 ; numberOfRepetitions <= 1000; numberOfRepetitions *= 1000){
      int numberOfProblemInstancesToSolve = nRuns / numberOfRepetitions;

      for(Output & o : outputs) o.out
        << "NEW BENCHMARK("
        << "numberOfProblemInstancesToSolve=" << numberOfProblemInstancesToSolve << ","
        << "numberOfRepetitionsPerProblemInstance=" << numberOfRepetitions << ")" << std::endl;

      b->run(argc, argv, numberOfProblemInstancesToSolve, numberOfRepetitions);

      for(Output & o : outputs) {
        b->printStat(o.out, o.nameWidth, o.sep, o.showErrors);
        o.out << std::endl;
      }
      b = bp->createInstance(verbose);
    }
  }
}
