/*
 * TypedExpressionSolverImpl.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include "TypedExpressionSolver.hpp"
#include "typed_expressions/UnitQuaternions.hpp"

namespace projection_problem {

namespace SOLVER {

#define SOLVER_NAME__(X) #X
#define SOLVER_NAME_(X) SOLVER_NAME__(X)
#define SOLVER_NAME SOLVER_NAME_(SOLVER)

std::string TypedExpressionSolver::getName() const {
  return std::string("TypedExpressionSolver(") + SOLVER_NAME +  ")";
}

auto TypedExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput& constInput) const -> InstancePtr {
  using namespace tex;
  struct Instance : public TypedExpressionSolver::Instance {

    Variable<UnitQuaternion> qC12, qC23;
    Variable<EuclideanPoint<3>> p3;
//    VExp<EuclideanPoint<3> > exp;
    decltype((inverse(Variable<UnitQuaternion>()) * Variable<UnitQuaternion>()).rotate(Variable<EuclideanPoint<3>>()) * Scalar<double>(0.5)) exp;

    Instance(const ProjectionProblem::ConstInput& constInput) :
      exp((qC12.inverse() * qC23).rotate(p3) * Scalar<double>(0.5))
    {
    }

    virtual void solveInto(const TypedExpressionSolver::Problem::Input & input, TypedExpressionSolver::Problem::Output & output) {
      qC12 = UnitQuaternion(asMatrixConvertible(input.qC12));
      qC23 = UnitQuaternion(asMatrixConvertible(input.qC23));
      p3 = EuclideanPoint<3>(asMatrixConvertible(input.p3));

      auto v = exp.eval();
      for(int i : {0, 1}) output.xy[i] = v[i];
    }
  };

  return InstancePtr(new Instance(constInput));
}

TypedExpressionSolver::TypedExpressionSolver() {
}

TypedExpressionSolver instance;

}
} /* namespace projection_problem */
