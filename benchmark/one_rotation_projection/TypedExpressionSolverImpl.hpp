/*
 * TypedExpressionSolverImpl.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <typed_expressions/UnitQuaternions.hpp>
#include <typed_expressions/SimpleLinalgWithEigenAdapter.hpp>

namespace one_rotation_projection_problem {

#ifndef SOLVER
#define SOLVER solver
#endif

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


#define SOLVER_NAME__(X) #X
#define SOLVER_NAME_(X) SOLVER_NAME__(X)
#define SOLVER_NAME SOLVER_NAME_(SOLVER)
std::string TypedExpressionSolver::getName() const {
  return std::string("TypedExpressionSolver(") + SOLVER_NAME +  ")";
}

auto TypedExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput& constInput) const -> InstancePtr {
  using namespace tex;
  struct Instance : public TypedExpressionSolver::Instance {

    ExtVariable<UnitQuaternion> qC12;
    ExtVariable<EuclideanPoint<3>> p2;
//    VExp<EuclideanPoint<3> > exp;
    decltype(ExtVariable<UnitQuaternion>().rotate(ExtVariable<EuclideanPoint<3>>()) * Scalar<double>(0.5)) exp;

    Instance(const ProjectionProblem::ConstInput& constInput) :
      exp(qC12.rotate(p2) * Scalar<double>(0.5))
    {
    }

    virtual void solveInto(const TypedExpressionSolver::Problem::Input & input, TypedExpressionSolver::Problem::Output & output, const EvalVariants v) override {
      qC12.setStorage(reinterpret_cast<const UnitQuaternion*>(&input.qC12));
      p2.setStorage(reinterpret_cast<const EuclideanPoint<3>*>(&input.p2));

      auto pRotated = exp.eval().getValue();

      switch(v){
        case EvalVariants::Eval :{
          for(int i : {0, 1}) output.xy[i] = pRotated[i];
        }
        break;
        case EvalVariants::EvalJacobian:
        {
          UnitQuaternion qC21(inverse(qC12).eval());

          const static EuclideanPoint<3> E[3] = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};

          for (int i : {0, 1, 2}) {
            auto & v = E[i];
            p2.setStorage(&v);
            output.jP2.template block<2, 1>(0, i) = toEigen(exp.eval().template block<2, 1>(0, 0));
            output.jPhi12.template block<2, 1>(0, i) = toEigen(v.cross(pRotated).template block<2, 1>(0, 0) * 2);
          }
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));
}

TypedExpressionSolver::TypedExpressionSolver() {
}

TypedExpressionSolver instance;

}
} /* namespace one_rotation_projection_problem */
