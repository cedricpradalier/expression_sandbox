/*
 * ProjectionProblemDummySolver.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */


#include <typed_expressions/EigenLinalg.hpp>
#define IMPLEMENT_DUMMY_SOLVER
#include <benchmark/DummySolver.hpp>
#include "ProjectionProblem.hpp"

#include <typed_expressions/UnitQuaternions.hpp>

namespace projection_problem {

auto ProjectionProblem::createInstance() const -> InstancePtr {
  struct Instance : public Base::Instance {
    Instance() {
      input.setRandom();
      calcSolutionInto(input, solution);
    }
    double calcError(Output & output) const override {
      return (solution.xy - output.xy).norm();
    }

    virtual ~Instance() {}
  };
  return InstancePtr(new Instance());
}

void Input::setRandom() {
  qC12.setRandom();
  qC12.normalize();
  qC23.setRandom();
  qC23.normalize();
  p3.setRandom();
}

void ProjectionProblem::calcSolutionInto(const Input & input, Output & output) {
  using namespace tex;
  UnitQuaternion qC12(asMatrixConvertible(input.qC12));
  UnitQuaternion qC23(asMatrixConvertible(input.qC23));
  EuclideanPoint<3> p3(asMatrixConvertible(input.p3));

  UnitQuaternion qC13 = (inverse(qC12) * qC23).eval();
  EuclideanPoint<3> v = qC13.rotate(p3).eval();
  for (int i : {0, 1}) output.xy[i] = v.getValue()[i] * 0.5;
}

}
benchmark::DummySolver<projection_problem::ProjectionProblem> dummySolverInstance;

typedef benchmark::ProblemSolver<projection_problem::ProjectionProblem> Solver;
