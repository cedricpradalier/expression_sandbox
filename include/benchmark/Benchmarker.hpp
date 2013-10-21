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
#include <cmath>
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

  Benchmarker(bool verbose = false) : verbose(verbose) {}

  void run(int argc, const char ** argv, const std::vector<const Solver*> & solverPtrs,
           int numberOfProblemInstancesToSolve = 1000,
           int numberOfRepetitions = 10000
           );
  void printHeader(std::ostream& out, const int nameWidth, const std::string& sep) const;
  void printStat(std::ostream& out, const int nameWidth, const std::string& sep) const;

 private:
  typedef typename Problem::Output Output;
  typedef typename Problem::Input Input;
  typedef typename Problem::Variant Variant;
  typedef typename Problem::InstancePtr ProblemInstancePtr;

  struct Statistics {
    Statistics(int nInstances, int nVariants) : solvingStats(nVariants), usedMemory(0) {
      for(auto & v : solvingStats) v.errors.resize(nInstances);
    }
    friend std::ostream & operator << (std::ostream & out, const Statistics & stat){
      out << "dPreparing = " << stat.durationPreparing << std::endl;
      for(auto & stat : stat.solvingStats){
        out << stat.variant << ":" ;
        out << "dSolving   = " << stat.durationSolving  << ", ";
        out << "error mean = " << stat.mean << ", ";
        out << "error var = " << stat.svar << std::endl;
      }
      out << "used memory = " << ((double)stat.usedMemory) << " byte/instance, ";
      return out;
    }
    void outList(std::ostream & out, const std::string sep) const {
      out
          << durationPreparing.duration << sep
          << usedMemory << sep;
      for(auto & s : solvingStats){
        out
          << s.durationSolving.duration << sep
          << s.mean << sep
          << s.svar << sep;
      }
    }
    static void outHeaderList(const Problem &p, std::ostream & out, const std::string sep) {
      out
        << "prepare(s)" << sep
        << "mem(byte/inst)" << sep;
      for(auto v : p.getVariants()){
        out
          << v << ":"
          << "solve(s)" << sep
          << "error mean" << sep
          << "deviation" << sep;
      }
    }

    void calc();

    struct SolvingVariantStat {
      SolvingVariantStat(): mean(0), svar(0) {}
      void calc();
      typename StopWatch::duration durationSolving;
      std::vector<double> errors;
      double mean, svar;
    };

    SolvingVariantStat & operator [] (Variant v){
        return solvingStats[(int)v];
    }
   private:
    typename StopWatch::duration durationPreparing;
    std::vector<SolvingVariantStat> solvingStats;
    size_t usedMemory;

    friend Benchmarker;
  };

  struct SolverData {
    SolverData(const Solver & solver, int nInstances, int nVariants) : solver(solver), instances(nInstances), outputs(nInstances), stat(nInstances, nVariants) {}
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

  void createSolverData(const std::vector<const Solver*>& solverPtrs, int nProblemInstancesToSolve) {
    solverData.reserve(solverPtrs.size());
    const int nVariants = p.getVariants().size();
    for (const Solver* sp : solverPtrs) {
      solverData.emplace_back(*sp, nProblemInstancesToSolve, nVariants);
    }
  }

  Problem p;
  std::vector<SolverData> solverData;
  bool verbose;
};


template <typename Problem_>
void Benchmarker<Problem_> ::Statistics::calc() {
  for(auto & v: solvingStats) v.calc();
}

template <typename Problem_>
void Benchmarker<Problem_> ::Statistics::SolvingVariantStat::calc() {
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
  svar = std::sqrt(tmp / errors.size());
}


template<typename Problem_>
void Benchmarker<Problem_>::printHeader(std::ostream& out, const int nameWidth, const std::string& sep) const {
  out << std::setw(nameWidth) << "Name" << sep;
  Statistics::outHeaderList(p, out, sep);
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
  if(verbose) std::cout << "createSolverData:" << stopWatch.readAndReset() << std::endl;

  std::vector<ProblemInstancePtr> problemInstances(numberOfProblemInstancesToSolve);
  createProblemInstances(problemInstances);
  if(verbose) std::cout << "createProblemInstances:" << stopWatch.readAndReset() << std::endl;
  createSolverInstances(problemInstances);
  if(verbose) std::cout << "createSolverInstances:" << stopWatch.readAndReset() << std::endl;

  for(auto variant : p.getVariants()){
    for(SolverData & sd : solverData) {
      StopWatch stopWatch;
      for(size_t i = 0, end = problemInstances.size(); i < end; i++) {
        for(int rep = 0; rep < numberOfRepetitions; rep ++){
          sd.instances[i]->solveInto(problemInstances[i]->getInput(), sd.outputs[i], variant);
        }
      }
      sd.stat[variant].durationSolving = stopWatch.read();
    }
    if(verbose) std::cout << "runningSolverInstances:" << stopWatch.readAndReset() << std::endl;

    for(SolverData & sd : solverData) {
      StopWatch stopWatch;
      int i = 0;
      for(auto & pi : problemInstances) {
        sd.stat[variant].errors[i] = pi->calcError(sd.outputs[i], variant);
        i++;
      }
      sd.stat.calc();
    }
    if(verbose) std::cout << "checkingResults:" << stopWatch.readAndReset() << std::endl;
  }
}

}
