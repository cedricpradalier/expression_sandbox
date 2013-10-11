/*
 * Benchmarker.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <bits/time.h>
#include <ctime>
#include <vector>

#include "malloc_count.h"
#include "ProblemSolver.hpp"


namespace benchmark {



class StopWatch {
 public:
  //TODO measure CPU time
  struct duration {
    typedef decltype(std::chrono::steady_clock::now() - std::chrono::steady_clock::now()) durationType;
    decltype(std::chrono::steady_clock::now() - std::chrono::steady_clock::now()) duration;
    std::clock_t cpuDuration;
  };

  duration read() { return {std::chrono::steady_clock::now() - start, std::clock() - cpuStart};}
  duration readAndReset() { auto r = read(); reset(); return r;}

  StopWatch(){
    reset();
  }
  void reset(){
    start = std::chrono::steady_clock::now();
    cpuStart = std::clock();
  }
 private:
  std::chrono::steady_clock::time_point start;
  std::clock_t cpuStart;
};
std::ostream & operator << (std::ostream & out, const typename StopWatch::duration::durationType & duration){
  out << std::chrono::duration_cast<std::chrono::duration<double> >(duration).count();
  return out;
}

std::ostream & operator << (std::ostream & out, const typename StopWatch::duration & duration){
  out << duration.duration << "s" << ", CPU:" << ((double) duration.cpuDuration / 1000000) << "s";
  return out;
}



template <typename Problem_>
class Benchmarker {
 public:
  typedef Problem_ Problem;
  typedef ProblemSolver<Problem> Solver;
  void run(int argc, const char ** argv, const std::vector<const Solver*> & solverPtrs,
           int numberOfProblemInstancesToSolve = 1000,
           int numberOfRepetitions = 10000
           );
  void static printHeader(std::ostream& out, const int nameWidth, const std::string& sep);
  void printStat(std::ostream& out, const int nameWidth, const std::string& sep) const;

 private:
  typedef typename Problem::Output Output;
  typedef typename Problem::Input Input;
  typedef typename Problem::InstancePtr ProblemInstancePtr;

  struct Statistics {
    Statistics(int nInstances) : errors(nInstances), usedMemory(0), mean(0), var(0) {}
    friend std::ostream & operator << (std::ostream & out, const Statistics & stat){
      out << "dSolving   = " << stat.durationSolving << std::endl;
      out << "dPreparing = " << stat.durationPreparing << std::endl;
      out << "used memory = " << ((double)stat.usedMemory) << " byte/instance, ";
      out << "error mean = " << stat.mean << ", ";
      out << "error var = " << stat.var << std::endl;
      return out;
    }
    void outList(std::ostream & out, const std::string sep) const {
      out
          << durationSolving.duration << sep
          << durationPreparing.duration << sep
          << usedMemory << sep
          << mean << sep
          << var << sep;
    }
    static void outHeaderList(std::ostream & out, const std::string sep){
      out
          << "solving(s)" << sep
          << "preparing(s)" << sep
          << "mem(byte/inst)" << sep
          << "error mean" << sep
          << "error var" << sep;
    }

    void calc();
   private:
    std::vector<double> errors;
    typename StopWatch::duration durationSolving, durationPreparing;
    size_t usedMemory;
    double mean, var;
    friend Benchmarker;
  };

  struct SolverData {
    SolverData(const Solver & solver, int nInstances) : solver(solver), instances(nInstances), outputs(nInstances), stat(nInstances) {
    }
    const Solver & solver;
    std::vector<typename Solver::InstancePtr> instances;
    std::vector<Output> outputs;
    Statistics stat;
  };

  void createSolverInstances(const std::vector<ProblemInstancePtr> & inputs) {
    for (SolverData& sd : solverData) {
      StopWatch stopWatch;
      size_t beforeMem = malloc_count_current();
      for (int i = 0, end = inputs.size(); i < end; i++) {
        sd.instances[i] = sd.solver.createNewInstance(inputs[i]->getConstInput());
      }
      sd.stat.durationPreparing = stopWatch.read();
      sd.stat.usedMemory = (malloc_count_current() - beforeMem) / inputs.size();
    }
  }

  void createProblemInstances(std::vector<ProblemInstancePtr> & problemInstances) {
    for(ProblemInstancePtr & i : problemInstances) {
      i = p.createInstance();
    }
  }

  void createSolverData(const std::vector<const Solver*>& solverPtrs, int numberOfProblemInstancesToSolve) {
    solverData.reserve(solverPtrs.size());
    for (const Solver* sp : solverPtrs) {
      solverData.emplace_back(*sp, numberOfProblemInstancesToSolve);
    }
  }

  Problem p;
  std::vector<SolverData> solverData;
};


template <typename Problem_>
void Benchmarker<Problem_> ::Statistics::calc() {
  double tmp = 0;
  for(double e : errors){
    tmp += e;
  }
  mean = tmp / errors.size();
  tmp = 0;
  for(double e : errors){
    double d = e - mean;
    tmp += d * d;
  }
  var = tmp / errors.size();
}

template<typename Problem_>
void Benchmarker<Problem_>::printHeader(std::ostream& out, const int nameWidth, const std::string& sep) {
  out << std::setw(nameWidth) << "Name" << sep;
  Statistics::outHeaderList(out, sep);
  out << std::endl;
}

template<typename Problem_>
void Benchmarker<Problem_>::printStat(std::ostream& out, const int nameWidth, const std::string& sep) const {
  for (const SolverData& sd : solverData) {
    out << std::setw(nameWidth) << sd.solver.getName() << sep;
    sd.stat.outList(out, sep);
    out << std::endl;
  }
}

template <typename Problem_>
void Benchmarker<Problem_>::run(int argc, const char ** argv, const std::vector<const Solver*> & solverPtrs,
                                int numberOfProblemInstancesToSolve, int numberOfRepetitions
                                ){

  StopWatch stopWatch;
  createSolverData(solverPtrs, numberOfProblemInstancesToSolve);
  std::cout << "createSolverData:" << stopWatch.readAndReset() << std::endl;

  std::vector<ProblemInstancePtr> problemInstances(numberOfProblemInstancesToSolve);
  createProblemInstances(problemInstances);
  std::cout << "createProblemInstances:" << stopWatch.readAndReset() << std::endl;
  createSolverInstances(problemInstances);
  std::cout << "createSolverInstances:" << stopWatch.readAndReset() << std::endl;

  for(SolverData & sd : solverData) {
    StopWatch stopWatch;
    for(size_t i = 0, end = problemInstances.size(); i < end; i++) {
      for(int rep = 0; rep < numberOfRepetitions; rep ++){
        sd.instances[i]->solveInto(problemInstances[i]->getInput(), sd.outputs[i]);
      }
    }
    sd.stat.durationSolving = stopWatch.read();
  }
  std::cout << "runningSolverInstances:" << stopWatch.readAndReset() << std::endl;

  for(SolverData & sd : solverData) {
    StopWatch stopWatch;
    int i = 0;
    for(auto & pi : problemInstances) {
      sd.stat.errors[i] = pi->calcError(sd.outputs[i]);
      i++;
    }
    sd.stat.calc();
  }
  std::cout << "checkingResults:" << stopWatch.readAndReset() << std::endl;
}

}
