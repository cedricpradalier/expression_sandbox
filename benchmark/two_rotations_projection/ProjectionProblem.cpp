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

namespace projection_problem {


std::ostream & operator << (std::ostream & out, const EvalVariants v){
  switch (v) {
    case EvalVariants::Eval:
      out << "Eval";
      break;
    case EvalVariants::EvalJacobian:
      out << "eJac";
      break;
  }
  return out;
}


auto ProjectionProblem::createInstance() const -> InstancePtr {
  struct Instance : public Base::Instance {
    Instance() {
      input.setRandom();
      calcSolutionInto(input, solution);
    }
    double calcError(Output & output, const Variant variant) const override {
      double xyNorm = 0;
      double jPhi12ErrorNorm = 0;
      double jPhi23ErrorNorm = 0;
      double jP3ErrorNorm = 0;

      switch(variant){
        case Variant::Eval:
          xyNorm = (solution.xy - output.xy).norm();
        break;
        case Variant::EvalJacobian:
          jPhi12ErrorNorm = (solution.jPhi12 - output.jPhi12).norm();
          jPhi23ErrorNorm = (solution.jPhi23 - output.jPhi23).norm();
          jP3ErrorNorm = (solution.jP3 - output.jP3).norm();
        break;
      }
#ifndef NDEBUG

      std::cout << "**************" << std::endl;
      std::cout << "xyNorm=" << std::endl << xyNorm << std::endl;
      std::cout << "jPhi12ErrorNorm=" << std::endl << jPhi12ErrorNorm << std::endl;
      std::cout << "jPhi23ErrorNorm=" << std::endl << jPhi23ErrorNorm << std::endl;
      std::cout << "jP3ErrorNorm=" << std::endl << jP3ErrorNorm << std::endl;
      std::cout << "solution.jPhi12=" << std::endl << solution.jPhi12 << std::endl;
      std::cout << "output.jPhi12=" << std::endl << output.jPhi12 << std::endl;
      std::cout << "solution.jPhi23=" << std::endl << solution.jPhi23 << std::endl;
      std::cout << "output.jPhi23=" << std::endl << output.jPhi23 << std::endl;
      std::cout << "solution.jP3=" << std::endl << solution.jP3 << std::endl;
      std::cout << "output.jP3=" << std::endl << output.jP3 << std::endl;
#endif
      return
          xyNorm +
          jPhi12ErrorNorm +
          jPhi23ErrorNorm +
          jP3ErrorNorm;
    }

    virtual ~Instance() {}
  };
  return InstancePtr(new Instance());
}

void Input::setRandom() {
  qC12.setRandom();
//  qC12 << 0, 0, 1, 0;
  qC12.normalize();
  qC23.setRandom();
//  qC23 << 0, 1, 0, 0;
  qC23.normalize();
  p3.setRandom();
//  p3 << 0, 0, 1;
}

using namespace linalg;

Vector<3> calcRotation (const Input & input){
  using namespace tex;
  UnitQuaternion qC12(asMatrixConvertible(input.qC12));
  UnitQuaternion qC23(asMatrixConvertible(input.qC23));
  EuclideanPoint<3> p3(asMatrixConvertible(input.p3));

  UnitQuaternion qC13 = (inverse(qC12) * qC23).eval();
  return qC13.rotate(p3).eval().getValue();
}

void ProjectionProblem::calcSolutionInto(const Input & input, Output & output) {
  using namespace tex;
  auto pRot = calcRotation(input);
  output.xy = pRot.block<2, 1>(0, 0) * 0.5;

  typedef typename UnitQuaternion::Calc Calc;

  UnitQuaternion qC21(inverse(UnitQuaternion(asMatrixConvertible(input.qC12))).eval());

  for (int i : {0, 1, 2}) {
    Vector<3>v(0, 0, 0); v[i] = 1;
    Input tmp = input;
    tmp.p3 = v;
    output.jP3.template block<2, 1>(0, i) = calcRotation(tmp).template block<2, 1>(0, 0) * 0.5;
    output.jPhi12.template block<2, 1>(0, i) = pRot.cross(qC21.rotate(v).eval().getValue()).template block<2, 1>(0, 0);
  }
  output.jPhi23 = -output.jPhi12;
}

::benchmark::DummySolver<ProjectionProblem> dummySolverInstance;
::benchmark::ProblemBenchmark<ProjectionProblem> benchmark;

}

