/*
 * ProblemSolver.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef PROBLEMSOLVER_HPP_
#define PROBLEMSOLVER_HPP_

#include <memory>
#include <vector>

namespace benchmark {

template <typename Problem>
class ProblemSolver {
public:
  class Instance {
   public:
    virtual void solveInto(const typename Problem::Input & input, typename Problem::Output & output) = 0;
    virtual ~Instance(){}
  };
  typedef std::unique_ptr<Instance> InstancePtr;

  virtual std::string getName() const = 0;
  virtual InstancePtr createNewInstance(const typename Problem::ConstInput & constInput) const = 0;

  virtual ~ProblemSolver(){}

  static const std::vector<const ProblemSolver*> & getSolvers() { return solverRegister;}
protected:
  ProblemSolver(){
    registerSolver(*this);
  }
private:
  static void registerSolver(const ProblemSolver& solver){
    solverRegister.push_back(&solver);
  }
  static std::vector<const ProblemSolver*> solverRegister;
};

template<typename Problem>
std::vector<const ProblemSolver<Problem>*> ProblemSolver<Problem>::solverRegister;


}

#endif /* PROBLEMSOLVER_HPP_ */
