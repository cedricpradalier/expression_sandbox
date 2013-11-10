/*
 * Benchmarker.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <fstream>
#include <chrono>
#include <benchmark/Benchmarker.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

#ifndef NDEBUG
#define DEFAULT_NRUNS 1
#define DEFAULT_SHOW_ERROR true
#else
#define DEFAULT_NRUNS 1E5
#define DEFAULT_SHOW_ERROR false
#endif

namespace benchmark {
std::vector<const Benchmark*> Benchmark::benchmarkerRegister;
}

int main(int argc, const char **argv) {
  bool verbose, error;
  long unsigned nRuns, nIterations, maxVariant;
  std::string refSolver;

  po::options_description desc("benchmark options");
  desc.add_options()
          ("help,h", "show help")
          ("verbose,v", po::value(&verbose)->default_value(false), "verbose")
          ("error,e", po::value(&error)->default_value(DEFAULT_SHOW_ERROR), "calculate and show error statistics")
          ("nRuns,n", po::value(&nRuns)->default_value(DEFAULT_NRUNS), "number of total runs per benchmark")
          ("iterations,i", po::value(&nIterations)->default_value(1), "number of iterations per problem instance and solver")
          ("maxVariant,m", po::value(&maxVariant)->default_value(100), "maximal variant index")
          ("ref,r", po::value(&refSolver)->default_value(""), "reference solver name")
          ;

  po::positional_options_description p;
  p.add("inputFile", 1);
  p.add("outputFile", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc)
            //.positional(p)
            .run(), vm);
  po::notify(vm);

  using namespace benchmark;

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
  std::vector<Output> outputs = { {std::cout, 50, "\t", error}, {csv, 1, ",", false} };

  for(const Benchmark * bp: Benchmark::getBenchmarks()){
    if(!bp->getNSolvers()) continue;
    const int refIndex = refSolver.empty() ? -1 : bp->getSolverIndex(refSolver);

    for(Output & o : outputs) o.out << "NEW PROBLEM(" << bp->getProblemName() << "):" << std::endl;

    for(unsigned long long numberOfRepetitions = nIterations ; numberOfRepetitions <= nRuns; numberOfRepetitions *= 1000){
      if(!nRuns || !numberOfRepetitions) continue;
      int numberOfProblemInstancesToSolve = nRuns / numberOfRepetitions;

      auto b = bp->createInstance(maxVariant, verbose);
      for(Output & o : outputs) b->printHeader(o.out, o.nameWidth, o.sep, o.showErrors, !refSolver.empty());
      for(Output & o : outputs) o.out
        << "NEW BENCHMARK("
        << "numberOfProblemInstancesToSolve=" << numberOfProblemInstancesToSolve << ","
        << "numberOfRepetitionsPerProblemInstance=" << numberOfRepetitions << ")" << std::endl;

      b->run(argc, argv, numberOfProblemInstancesToSolve, numberOfRepetitions);
      if(error) b->calcErrorStat();

      for(Output & o : outputs) {
        b->printStat(o.out, o.nameWidth, o.sep, o.showErrors, refIndex);
        o.out << std::endl;
      }
      b = bp->createInstance(verbose);
    }
  }
}
