/*
 * Benchmarker.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */
#include <unordered_map>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include "StopWatch.hpp"
#include "malloc_count.h"
#include "ProblemSolver.hpp"


namespace benchmark {

using namespace stop_watch;

class Benchmark {
 public:
  Benchmark(){ registerBenchmark(*this);}
  virtual ~Benchmark(){}

  class Instance {
   public:
    virtual void run(int argc, const char ** argv, int numberOfProblemInstancesToSolve, int numberOfRepetitions) = 0;
    virtual void calcErrorStat() = 0;
    virtual void printHeader(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors = true, bool relative = false) const = 0;
    virtual void printStat(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors = true, int refSolverIndex = -1) const = 0;
    virtual ~Instance(){}
  };

  virtual std::unique_ptr<Instance> createInstance(int maxVariant, bool verbose = false) const = 0;
  virtual int getSolverIndex(const std::string name) const = 0;
  virtual std::string getProblemName() const = 0;

  static std::vector<const Benchmark*> getBenchmarks() {
    return benchmarkerRegister;
  }
 private:
  static void registerBenchmark(const Benchmark& benchmark){
    benchmarkerRegister.push_back(&benchmark);
  }
  static std::vector<const Benchmark*> benchmarkerRegister;
};

template <typename Problem_>
class BenchmarkInstance : public Benchmark::Instance {
 public:
  typedef Problem_ Problem;
  typedef ProblemSolver<Problem> Solver;

  BenchmarkInstance(const Problem & p, int maxVariant, bool verbose = false) : p(p), verbose(verbose), maxVariant(maxVariant) {}

  virtual void run(int argc, const char ** argv, int numberOfProblemInstancesToSolve, int numberOfRepetitions);
  virtual void calcErrorStat();
  virtual void printHeader(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors, bool relative) const;
  virtual void printStat(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors, int refSolverIndex) const;

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
    void outList(std::ostream & out, const std::string & sep, bool showErrors, const Statistics * ref = nullptr) const {
      outDuration(out, durationPreparing, ref ? &ref->durationPreparing : nullptr); out << sep;
      out << usedMemory << sep;
      int i = 0;
      for(auto & s : solvingStats){
        outDuration(out, s.durationSolving, ref ? &ref->solvingStats[i].durationSolving : nullptr); out << sep;
        if(showErrors)
          out
            << s.mean << sep
            << s.svar << sep;
        i++;
      }
    }
    static void outHeaderList(const Problem &p, int maxVariant, std::ostream & out, const std::string sep, bool showErrors, bool relative) {
      std::string durationUnit = (relative? "(f)":"(s)");
      out
        << "prepare" << durationUnit << sep
        << "mem(byte/inst)" << sep;
      int i = 0;
      for(auto v : p.getVariants()){
        if(i++ > maxVariant) break;
        out
          << v << ":"
          << "solve"<< durationUnit << sep;
        if(showErrors)
          out
            << "error mean" << sep
            << "deviation" << sep;
      }
    }

    void calc();

    struct SolvingVariantStat {
      SolvingVariantStat(): mean(0), svar(0) {}
      void calc();
      typename StopWatch::Duration durationSolving;
      std::vector<double> errors;
      double mean, svar;
    };

    SolvingVariantStat & operator [] (Variant v){
        return solvingStats[(int)v];
    }
   private:
    static void outDuration(std::ostream & out, const StopWatch::Duration & duration, const StopWatch::Duration * refDuration = nullptr) {
      if(refDuration){
        out << (double)duration / (double)*refDuration;
      }else{
        out << (double)duration;
      }
    }
    typename StopWatch::Duration durationPreparing;
    std::vector<SolvingVariantStat> solvingStats;
    size_t usedMemory;
    friend BenchmarkInstance;
  };

  struct SolverData {
    SolverData(const Solver & solver, int nInstances, int nVariants) : solver(solver), instances(nInstances), outputs(nInstances), stat(nInstances, nVariants) {}
    const Solver & solver;
    std::vector<typename Solver::InstancePtr> instances;
    std::vector<Output> outputs;
    Statistics stat;
  };

  void createSolverInstances() {
    if(problemInstances.empty()) return;

    for (SolverData& sd : solverData) {
      StopWatch stopWatch;
      size_t beforeMem = malloc_count_current();
      for (int i = 0, end = problemInstances.size(); i < end; i++) {
        sd.instances[i] = sd.solver.createNewInstance(problemInstances[i]->getConstInput());
      }
      sd.stat.durationPreparing = stopWatch.read();
      sd.stat.usedMemory = (malloc_count_current() - beforeMem) / problemInstances.size();
    }
  }

  void createProblemInstances(const int numberOfProblemInstancesToSolve) {
    problemInstances.resize(numberOfProblemInstancesToSolve);
    for(ProblemInstancePtr & i : problemInstances) {
      i = p.createInstance();
    }
  }

  void createSolverData(const std::vector<const Solver*>& solverPtrs, int nProblemInstancesToSolve) {
    solverData.reserve(solverPtrs.size());
    const int nVariants = std::min(maxVariant + 1, (int)p.getVariants().size());
    for (const Solver* sp : solverPtrs) {
      solverData.emplace_back(*sp, nProblemInstancesToSolve, nVariants);
    }
  }

  const Problem & p;
  std::vector<SolverData> solverData;
  bool verbose;
  int maxVariant;
  std::vector<ProblemInstancePtr> problemInstances;
};


template <typename Problem_>
void BenchmarkInstance<Problem_> ::Statistics::calc() {
  for(auto & v: solvingStats) v.calc();
}

template <typename Problem_>
void BenchmarkInstance<Problem_> ::Statistics::SolvingVariantStat::calc() {
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
void BenchmarkInstance<Problem_>::printHeader(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors, bool relative = false) const {
  out << std::setw(nameWidth) << "Name" << sep;
  Statistics::outHeaderList(p, maxVariant, out, sep, showErrors, relative);
  out << std::endl;
}

template<typename Problem_>
void BenchmarkInstance<Problem_>::printStat(std::ostream& out, const int nameWidth, const std::string& sep, bool showErrors, int refSolverIndex) const {
  for (const SolverData& sd : solverData) {
    out << std::setw(nameWidth) << sd.solver.getName() << sep;
    sd.stat.outList(out, sep, showErrors, refSolverIndex == -1 ? nullptr : &solverData[std::max(std::min((size_t) refSolverIndex, solverData.size()), (size_t)0)].stat);
    out << std::endl;
  }
}

template <typename Problem_>
void BenchmarkInstance<Problem_>::run(int argc, const char ** argv, int numberOfProblemInstancesToSolve, int numberOfRepetitions){
  const std::vector<const Solver*> & solverPtrs = ProblemSolver<Problem>::getSolvers();

  if(numberOfProblemInstancesToSolve <= 0 || numberOfRepetitions <= 0){
    std::cerr << "no instances to run" << std::endl;
    return;
  }

  if(solverPtrs.size() == 0){
    std::cerr << "no solvers found" << std::endl;
    return;
  }

  StopWatch stopWatch;
  createSolverData(solverPtrs, numberOfProblemInstancesToSolve);
  if(verbose) std::cout << "createSolverData:" << stopWatch.readAndReset() << std::endl;

  createProblemInstances(numberOfProblemInstancesToSolve);
  if(verbose) std::cout << "createProblemInstances:" << stopWatch.readAndReset() << std::endl;
  createSolverInstances();
  if(verbose) std::cout << "createSolverInstances:" << stopWatch.readAndReset() << std::endl;

  for(auto variant : p.getVariants()){
    if((int)variant > maxVariant) break;
    for(SolverData & sd : solverData) {
      StopWatch stopWatch;
      for(size_t i = 0, end = problemInstances.size(); i < end; i++) {
        for(int rep = 0; rep < numberOfRepetitions; rep ++){
          sd.instances[i]->solveInto(problemInstances[i]->getInput(), sd.outputs[i], variant);
        }
      }
      sd.stat[variant].durationSolving = stopWatch.read();
    }
    if(verbose) std::cout << "runningSolverInstances took:" << stopWatch.readAndReset() << std::endl;
  }
}


template <typename Problem_>
void BenchmarkInstance<Problem_>::calcErrorStat() {
  StopWatch stopWatch;
  for(auto variant : p.getVariants()){
    if((int)variant > maxVariant) break;
    for(SolverData & sd : solverData) {
      StopWatch stopWatch;
      int i = 0;
      for(auto & pi : problemInstances) {
        sd.stat[variant].errors[i] = pi->calcError(sd.outputs[i], variant);
        i++;
      }
      sd.stat.calc();
    }
    if(verbose) std::cout << "checkingResults("<< variant << ") took :" << stopWatch.readAndReset() << std::endl;
  }
}

template <typename Problem_>
class ProblemBenchmark : public Benchmark {
  virtual std::unique_ptr<Instance> createInstance(int maxVariant, bool verbose) const override {
    return std::unique_ptr<Instance>(new BenchmarkInstance<Problem_>(p, maxVariant, verbose));
  }

  virtual int getSolverIndex(const std::string name) const {
    int i = 0;
    for(auto s: ProblemSolver<Problem_>::getSolvers()){
      if(s->getName().find(name) != std::string::npos){
        return i;
      }
      i++;
    }
    return -1;
  }

  virtual std::string getProblemName() const { return p.getName(); };
 private:
  Problem_ p;
};

}
