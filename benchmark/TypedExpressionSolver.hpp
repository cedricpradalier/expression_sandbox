/*
 * TypedExpressionSolver.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef TYPEDEXPRESSIONSOLVER_HPP_
#define TYPEDEXPRESSIONSOLVER_HPP_

#include <benchmark/ProblemSolver.hpp>
#include "ProjectionProblem.hpp"


namespace projection_problem {

namespace SOLVER {

class TypedExpressionSolver : public benchmark::ProblemSolver<ProjectionProblem>{
 public:
  TypedExpressionSolver();
  typedef ProblemSolver<ProjectionProblem> Base;
  typedef ProjectionProblem Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const;

  virtual InstancePtr createNewInstance(const ProjectionProblem::ConstInput & constInput) const;

  virtual ~TypedExpressionSolver() {};
};

}
} /* namespace projection_problem */
#endif /* TYPEDEXPRESSIONSOLVER_HPP_ */
