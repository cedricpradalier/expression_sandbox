/*
 * TypedExpressionSolverImpl.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <typed_expressions/UnitQuaternions.hpp>
#include <typed_expressions/SimpleLinalgWithEigenAdapter.hpp>

namespace projection_problem {

#ifndef SOLVER
#define SOLVER solver
#endif

namespace SOLVER {

using namespace tex;

template<typename A, typename B, typename C>
inline auto getExp(A qC12, B qC23, C p3) -> decltype((rotate(inverse(qC12), rotate(qC23, p3)) * tex::Scalar<double>(0.5)).template segment<2>(0)){
  return (rotate(inverse(qC12), rotate(qC23, p3)) * tex::Scalar<double>(0.5)).template segment<2>(0);
}


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

//    ExtVariable<UnitQuaternion> qC12, qC23;
//    ExtVariable<EuclideanPoint<3>> p3;
//    VExp<EuclideanPoint<3> > exp;
//    decltype((inverse(ExtVariable<UnitQuaternion>()) * ExtVariable<UnitQuaternion>()).rotate(ExtVariable<EuclideanPoint<3>>()) * Scalar<double>(0.5)) exp;
//    decltype(inverse(ExtVariable<UnitQuaternion>()).rotate(ExtVariable<UnitQuaternion>().rotate(ExtVariable<EuclideanPoint<3>>())) * Scalar<double>(0.5)) exp;

    Instance(const ProjectionProblem::ConstInput& constInput)
//        :
//      exp(qC12.inverse().rotate(qC23.rotate(p3)) * Scalar<double>(0.5))
    {
    }

    virtual void solveInto(const TypedExpressionSolver::Problem::Input & input, TypedExpressionSolver::Problem::Output & output, const EvalVariants v) override {
//      qC12.setStorage(reinterpret_cast<const UnitQuaternion*>(&input.qC12));
//      qC23.setStorage(reinterpret_cast<const UnitQuaternion*>(&input.qC23));
//      p3.setStorage(reinterpret_cast<const EuclideanPoint<3>*>(&input.p3));

      auto qC12 = Diffable<Ref<UnitQuaternion>, 0>(reinterpret_cast<const UnitQuaternion&>(input.qC12));
      auto qC23 = Diffable<Ref<UnitQuaternion>, 1>(reinterpret_cast<const UnitQuaternion&>(input.qC23));
      auto p3 = Diffable<Ref<EuclideanPoint<3>>, 2>(reinterpret_cast<const EuclideanPoint<3>&>(input.p3));
      auto exp = getExp(qC12, qC23, p3);


      switch(v){
        case EvalVariants::Eval :{
//          output.xy = evalExp(exp);
          auto cache = createCache(exp);
          cache.update(exp);
          output.xy = toEigen(cache.accessValue(exp));
        }
        break;
        case EvalVariants::EvalJacobian:
        {
//          UnitQuaternion qC21(inverse(qC12).eval());
//
//          const static EuclideanPoint<3> E[3] = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
//
//          for (int i : {0, 1, 2}) {
//            auto & v = E[i];
////            p3.setStorage(&v);
//            output.jP3.block<2, 1>(0, i) = toEigen(getExp(qC12, qC23, v).eval().block<2, 1>(0, 0));
//            output.jPhi12.block<2, 1>(0, i) = toEigen(pRotated.cross(qC21.rotate(v).eval().getValue()).block<2, 1>(0, 0) * 2);
//            output.jPhi23.block<2, 1>(0, i) = toEigen(-pRotated.cross(qC21.rotate(v).eval().getValue()).block<2, 1>(0, 0) * 2);
//          }

          auto cache = createCache(exp);
          cache.update(exp);
          output.jPhi12.setZero();
          evalFullDiffIntoCached(exp, qC12, cache, output.jPhi12);
          output.jPhi23.setZero();
          evalFullDiffIntoCached(exp, qC23, cache, output.jPhi23);
          output.jP3.setZero();
          evalFullDiffIntoCached(exp, p3, cache, output.jP3);

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
} /* namespace projection_problem */
