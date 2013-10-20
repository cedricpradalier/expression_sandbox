/*
 * Benchmarker.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <benchmark/Benchmarker.hpp>
#include "ProjectionProblem.hpp"

#include <fstream>

int main(int argc, const char **argv) {
  using namespace benchmark;
  using namespace projection_problem;
  typedef ProjectionProblem Problem;
  typedef Benchmarker<Problem> Bench;

  std::cout << "found these solver:";
  int i = 0;
  for(auto s : ProblemSolver<Problem>::getSolvers()){
    if(i++) std::cout << ", ";
    std::cout << s->getName();
  }
  std::cout << ".\n";



  std::string sep("\t");

  struct Output {
    std::ostream & out;
    int nameWidth;
    const char * sep;
  };

  std::filebuf csvBuffer;
  std::ostream csv(&csvBuffer);
  csvBuffer.open("stat.csv", std::ios::out | std::ios::trunc);
  std::vector<Output> outputs = { {std::cout, 50, "\t"}, {csv, 1, ","} };

  for(Output & o : outputs) Bench::printHeader(o.out, o.nameWidth, o.sep);

  const int nRuns = 1000000;

  for(int numberOfRepetitions = 1 ; numberOfRepetitions <= 1000; numberOfRepetitions *= 1000){
    int numberOfProblemInstancesToSolve = nRuns / numberOfRepetitions;

    for(Output & o : outputs) o.out
      << "NEW BENCHMARK("
      << "numberOfProblemInstancesToSolve=" << numberOfProblemInstancesToSolve << ","
      << "numberOfRepetitionsPerProblemInstance=" << numberOfRepetitions << ")" << std::endl;

    Bench b;

    b.run(argc, argv, ProblemSolver<Problem>::getSolvers(),
        numberOfProblemInstancesToSolve,
        numberOfRepetitions);

    for(Output & o : outputs) b.printStat(o.out, o.nameWidth, o.sep);
  }
}
