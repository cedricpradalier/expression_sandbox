/*
 * RotationProblem.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef ROTATION_PROBLEM_HPP_
#define ROTATION_PROBLEM_HPP_

#include <sstream>
#include <benchmark/Problem.hpp>
#include <benchmark/EvalVariants.hpp>
#include <Eigen/Core>

namespace rotation_problem {

template <unsigned N>
struct Output {
  Eigen::Matrix<double, 3, 1> r;
  Eigen::Matrix<double, 3, 3> jPhi[N];
  Eigen::Matrix<double, 3, 3> jP;
  Output() {
    r.setZero();
    for(unsigned i = 0 ; i < N; i++)
      jPhi[i].setZero();
    jP.setZero();
  }
};

struct ConstInput {
};

template <unsigned N>
struct Input {
  Eigen::Vector4d qC[N];
  Eigen::Vector3d p;

  void setRandom();
};

template <unsigned N>
class RotationProblem : public benchmark::Problem<ConstInput, Input<N>, Output<N>, RotationProblem<N>, benchmark::EvalVariants>{
public:
  typedef benchmark::Problem<ConstInput, Input<N>, Output<N>, RotationProblem<N>, benchmark::EvalVariants> Base;
  RotationProblem() : Base(static_cast<std::stringstream &>(std::stringstream() <<"rotation_problem(N=" << N << ")").str()) {}
  typedef typename Base::InstancePtr InstancePtr;
  virtual InstancePtr createInstance() const;
  virtual ~RotationProblem() {}
  virtual std::vector<benchmark::EvalVariants> getVariants() const override { return {benchmark::EvalVariants::Eval, benchmark::EvalVariants::EvalJacobian}; };
 private:
  static void calcSolutionInto(const Input<N> & input, Output<N> & output);
};

#define CREATE_INSTANCES(PREFIX, POSTFIX, NAME) \
PREFIX 1 POSTFIX NAME##1;\
PREFIX 19 POSTFIX NAME##19;\
//*/

}

#endif /* ROTATION_PROBLEM_HPP_ */
