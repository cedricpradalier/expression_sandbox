/*
 * ProjectionProblem.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef PROJECTIONPROBLEM_HPP_
#define PROJECTIONPROBLEM_HPP_

#include <benchmark/Problem.hpp>
#include <benchmark/EvalVariants.hpp>
#include <Eigen/Core>

namespace projection_problem {

struct Output {
  Eigen::Matrix<double, 2, 1> xy;
  Eigen::Matrix<double, 2, 3> jPhi12, jPhi23, jP3;
  Output() {
    xy.setZero();
    jPhi12.setZero();
    jPhi23.setZero();
    jP3.setZero();
  }
};

struct ConstInput {
};

struct Input {
  Eigen::Vector4d qC12;
  Eigen::Vector4d qC23;
  Eigen::Vector3d p3;

  void setRandom();
};

using benchmark::EvalVariants;

class ProjectionProblem : public benchmark::Problem<ConstInput, Input, Output, ProjectionProblem, EvalVariants>{
public:
  typedef benchmark::Problem<ConstInput, Input, Output, ProjectionProblem, EvalVariants> Base;
  ProjectionProblem() : Base("two rotations projection problem") {}
  typedef typename Base::InstancePtr InstancePtr;
  virtual InstancePtr createInstance() const;
  virtual ~ProjectionProblem() {}
  virtual std::vector<Variant> getVariants() const override { return {EvalVariants::Eval, EvalVariants::EvalJacobian}; };
 private:
  static void calcSolutionInto(const Input & input, Output & output);
};

}

#endif /* PROJECTIONPROBLEM_HPP_ */
