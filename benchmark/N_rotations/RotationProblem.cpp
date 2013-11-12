/*
 * RotationProblemDummySolver.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */


#include <typed_expressions/EigenLinalg.hpp>
#define IMPLEMENT_DUMMY_SOLVER
#include <benchmark/DummySolver.hpp>
#include <benchmark/Benchmarker.hpp>
#include "RotationProblem.hpp"

#include <typed_expressions/UnitQuaternions.hpp>

namespace rotation_problem {

template <unsigned N>
auto RotationProblem<N>::createInstance() const -> InstancePtr {
  struct Instance : public Base::Instance {
    Instance() {
      this->input.setRandom();
      calcSolutionInto(this->input, this->solution);
    }
    double calcError(typename Base::Output & output, const typename Base::Variant variant) const override {
      double rNorm = 0;
      double jPhiErrorNorm[N];
      for(unsigned i = 0 ; i < N; i++)  jPhiErrorNorm[i] = 0;
      double jRErrorNorm = 0;

      switch(variant){
        case Base::Variant::Eval:
          rNorm = (this->solution.r - output.r).norm();
        break;
        case Base::Variant::EvalJacobian:
          for(unsigned i = 0 ; i < N; i++)
            jPhiErrorNorm[i] = (this->solution.jPhi[i] - output.jPhi[i]).norm();
          jRErrorNorm = (this->solution.jP - output.jP).norm();
        break;
      }
#ifndef NDEBUG
      std::cout << "**************" << std::endl;
      std::cout << "rNorm=" << std::endl << rNorm << std::endl;
      for(unsigned i = 0; i < N; i++)
        std::cout << "jPhiErrorNorm["<< i << "]=" << std::endl << jPhiErrorNorm[i] << std::endl;
      std::cout << "jRErrorNorm=" << std::endl << jRErrorNorm << std::endl;
      std::cout << "solution.r=" << std::endl << this->solution.r << std::endl;
      std::cout << "output.r=" << std::endl << output.r << std::endl;
      for(unsigned i = 0 ; i < N; i++){
        std::cout << "solution.jPhi["<< i << "]=" << std::endl << this->solution.jPhi[i] << std::endl;
        std::cout << "output.jPhi["<< i << "]=" << std::endl << output.jPhi[i] << std::endl;
      }
      std::cout << "solution.jP=" << std::endl << this->solution.jP << std::endl;
      std::cout << "output.jP=" << std::endl << output.jP << std::endl;
#endif
      double r = 0;
      for(unsigned i = 0 ; i < N; i++)  r+=jPhiErrorNorm[i];
      return
          rNorm +
          r +
          jRErrorNorm;
    }

    virtual ~Instance() {}
  };
  return InstancePtr(new Instance());
}

template <unsigned N>
void Input<N>::setRandom() {
  for(unsigned i = 0; i < N; i++){
      qC[i].setRandom();
  //  qC << 0, 0, 1, 0;
      qC[i].normalize();
  }
  p.setRandom();
//  p << 0, 0, 1;
}

using namespace linalg;

template <unsigned N>
const Vector<3> & calcRotation (const Input<N> & input, Vector<3> * output, unsigned size = N){
  using namespace tex;

  const Vector<3> *r = & input.p;
  for(unsigned i = 0; i < N; i++){
    output[i % size] = Ref<UnitQuaternion>(asMatrixConvertible(input.qC[i])).rotate(*r).eval().getValue();
    r = & output[i % size];
  }
  return *r;
}

template <unsigned N>
void RotationProblem<N>::calcSolutionInto(const Input<N> & input, Output<N> & output) {
  using namespace tex;
  Vector<3> rotatedVectors[N], tmpV;
  output.r = calcRotation(input, rotatedVectors);


  typedef typename UnitQuaternion::Calc Calc;

  Input<N> tmp = input;
  for (int i : {0, 1, 2}) {
    tmp.p.setZero();
    tmp.p[i] = 1;
    output.jP.template block<3, 1>(0, i) = calcRotation(tmp, &tmpV, 1);
    tmpV = tmp.p;
    for(unsigned k = 0; k < N; k++){
      tmpV = tmp.p.cross(rotatedVectors[k]);
      tmpV *= 2;
      for(unsigned j = k + 1; j < N; j ++){
        tmpV = Ref<UnitQuaternion>(asMatrixConvertible(input.qC[j])).rotate(tmpV).eval().getValue();
      }
      output.jPhi[k].template block<3, 1>(0, i) = tmpV;
    }
  }
}

CREATE_INSTANCES(::benchmark::ProblemBenchmark<RotationProblem<, >>, benchmark)


} // namespace rotation_problem
