/*
 * ProjectionProblem.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef PROJECTIONPROBLEM_HPP_
#define PROJECTIONPROBLEM_HPP_

#include <benchmark/Problem.hpp>
#include <Eigen/Core>

namespace one_rotation_projection_problem {

struct Output {
  Eigen::Matrix<double, 2, 1> xy;
  Eigen::Matrix<double, 2, 3> jPhi12, jP2;
  Output() {
    xy.setZero();
    jPhi12.setZero();
    jP2.setZero();
  }
};

struct ConstInput {
};

struct Input {
  Eigen::Vector4d qC12;
  Eigen::Vector3d p2;

  void setRandom();
};

enum class EvalVariants {
  Eval,
  EvalJacobian
};

std::ostream & operator << (std::ostream & out, const EvalVariants v);

class ProjectionProblem : public benchmark::Problem<ConstInput, Input, Output, ProjectionProblem, EvalVariants>{
public:
  typedef benchmark::Problem<ConstInput, Input, Output, ProjectionProblem, EvalVariants> Base;
  ProjectionProblem() : Base("one rotation projection problem") {}
  typedef typename Base::InstancePtr InstancePtr;
  virtual InstancePtr createInstance() const;
  virtual ~ProjectionProblem() {}
  virtual std::vector<Variant> getVariants() const override { return {EvalVariants::Eval, EvalVariants::EvalJacobian}; };
 private:
  static void calcSolutionInto(const Input & input, Output & output);
};

}

#endif /* PROJECTIONPROBLEM_HPP_ */
