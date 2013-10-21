/*
 * GeneratedExpressionSolver.cpp
 *
 *  Created on: Oct 21, 2013
 *      Author: hannes
 */



#include "ProjectionProblem.hpp"
#include <benchmark/ProblemSolver.hpp>
#include "generated/PaulsGeneratedError.h"
#include "generated/PaulsGeneratedErrorInTangentSpace.h"
#include <iostream>

namespace projection_problem {
namespace generated {

class GeneratedExpressionSolver : public benchmark::ProblemSolver<ProjectionProblem>{
 public:
  typedef ProblemSolver<ProjectionProblem> Base;
  typedef ProjectionProblem Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return "GeneratedExpressionSolver";
  }

  virtual InstancePtr createNewInstance(const ProjectionProblem::ConstInput & constInput) const;

  virtual ~GeneratedExpressionSolver() {};
};


Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(3, 1);

GeneratedExpressionSolver::InstancePtr GeneratedExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput & constInput) const{
  struct Instance : public GeneratedExpressionSolver::Instance {

    error_term::PaulsGeneratedError exp;
    error_term::PaulsGeneratedErrorInTangentSpace expJ;

    Instance(const Problem::ConstInput& constInput) :
      exp(), expJ()
    {
    }

    virtual void solveInto(const Problem::Input & input, Problem::Output & output, const Problem::Variant v) override {
      switch(v){
        case EvalVariants::Eval :
          exp.evaluate(input.qC12, input.qC23, input.p3, &output.xy, nullptr, nullptr, nullptr);
        break;
        case EvalVariants::EvalJacobian :
          expJ.evaluate(input.qC12, input.qC23, input.p3, nullptr, &output.jPhi12, &output.jPhi23, &output.jP3);
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}


GeneratedExpressionSolver generatedSolver;


}
}
