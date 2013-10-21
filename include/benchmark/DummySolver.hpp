/*
 * DummySolver.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef DUMMYSOLVER_HPP_
#define DUMMYSOLVER_HPP_
#include <string>
#include "ProblemSolver.hpp"

namespace benchmark {

template<typename Problem>
class DummySolver : public ProblemSolver<Problem> {
 public:
  typedef ProblemSolver<Problem> Base;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return std::string("DummySolverDoingNothing(toMeasureTheOverhead)");
  }

  virtual InstancePtr createNewInstance(const typename Problem::ConstInput & constInput) const;

  virtual ~DummySolver() = default;
};


#ifdef IMPLEMENT_DUMMY_SOLVER
template <typename Problem>
auto DummySolver<Problem>::createNewInstance(const typename Problem::ConstInput & constInput) const -> InstancePtr
{
  struct Instance : public Base::Instance {
    virtual void solveInto(const typename Problem::Input & input, typename Problem::Output & output, const typename Problem::Variant v) override {
    }
    virtual ~Instance(){}
  };


  return InstancePtr(new Instance());
}
#endif

}
#endif /* DUMMYSOLVER_HPP_ */
