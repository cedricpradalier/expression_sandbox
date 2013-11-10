/*
 * ProjectionProblemDummySolver.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */


#include <typed_expressions/EigenLinalg.hpp>
#define IMPLEMENT_DUMMY_SOLVER
#include <benchmark/DummySolver.hpp>
#include <benchmark/Benchmarker.hpp>
#include "ProjectionProblem.hpp"

#include <typed_expressions/UnitQuaternions.hpp>

namespace one_rotation_projection_problem {

auto ProjectionProblem::createInstance() const -> InstancePtr {
  struct Instance : public Base::Instance {
    Instance() {
      input.setRandom();
      calcSolutionInto(input, solution);
    }
    double calcError(Output & output, const Variant variant) const override {
      double xyNorm = 0;
      double jPhi12ErrorNorm = 0;
      double jP2ErrorNorm = 0;

      switch(variant){
        case Variant::Eval:
          xyNorm = (solution.xy - output.xy).norm();
        break;
        case Variant::EvalJacobian:
          jPhi12ErrorNorm = (solution.jPhi12 - output.jPhi12).norm();
          jP2ErrorNorm = (solution.jP2 - output.jP2).norm();
        break;
      }
#ifndef NDEBUG
      std::cout << "**************" << std::endl;
      std::cout << "xyNorm=" << std::endl << xyNorm << std::endl;
      std::cout << "jPhi12ErrorNorm=" << std::endl << jPhi12ErrorNorm << std::endl;
      std::cout << "jP2ErrorNorm=" << std::endl << jP2ErrorNorm << std::endl;
      std::cout << "solution.xy=" << std::endl << solution.xy << std::endl;
      std::cout << "output.xy=" << std::endl << output.xy << std::endl;
      std::cout << "solution.jPhi12=" << std::endl << solution.jPhi12 << std::endl;
      std::cout << "output.jPhi12=" << std::endl << output.jPhi12 << std::endl;
      std::cout << "solution.jP2=" << std::endl << solution.jP2 << std::endl;
      std::cout << "output.jP2=" << std::endl << output.jP2 << std::endl;
#endif
      return
          xyNorm +
          jPhi12ErrorNorm +
          jP2ErrorNorm;
    }

    virtual ~Instance() {}
  };
  return InstancePtr(new Instance());
}

void Input::setRandom() {
  qC12.setRandom();
//  qC12 << 0, 0, 1, 0;
  qC12.normalize();
  p2.setRandom();
//  p2 << 0, 0, 1;
}

using namespace linalg;

Vector<3> calcRotation (const Input & input){
  using namespace tex;
  UnitQuaternion qC12(asMatrixConvertible(input.qC12));
  EuclideanPoint<3> p2(asMatrixConvertible(input.p2));
  return qC12.rotate(p2).eval().getValue();
}

void ProjectionProblem::calcSolutionInto(const Input & input, Output & output) {
  using namespace tex;
  auto pRot = calcRotation(input);
  output.xy = pRot.block<2, 1>(0, 0) * 0.5;

  typedef typename UnitQuaternion::Calc Calc;

  UnitQuaternion qC12(asMatrixConvertible(input.qC12));

  for (int i : {0, 1, 2}) {
    Vector<3>v(0, 0, 0); v[i] = 1;
    Input tmp = input;
    tmp.p2 = v;
    output.jP2.block<2, 1>(0, i) = calcRotation(tmp).block<2, 1>(0, 0) * 0.5;
    output.jPhi12.block<2, 1>(0, i) = v.cross(pRot).block<2, 1>(0, 0);
  }
}

::benchmark::DummySolver<ProjectionProblem> dummySolverInstance;
::benchmark::ProblemBenchmark<ProjectionProblem> benchmark;

} // namespace one_rotation_projection_problem
