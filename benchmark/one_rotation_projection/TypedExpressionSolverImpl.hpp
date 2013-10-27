/*
 * TypedExpressionSolverImpl.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <typed_expressions/UnitQuaternions.hpp>
#include <typed_expressions/SimpleLinalgWithEigenAdapter.hpp>
//#include <ceres/rotation.h>

namespace one_rotation_projection_problem {

#ifndef SOLVER
#define SOLVER solver
#endif

namespace SOLVER {

using namespace tex;

template <typename A, typename B>
inline auto getExp(const A & qC12, const B & p2) -> decltype((qC12.rotate(p2) * Scalar<double>(0.5)).template segment<2>(0)){
  return (qC12.rotate(p2) * Scalar<double>(0.5)).template segment<2>(0); //TODO optimize change order of segment and * 0.5
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

//const static tex::EuclideanPoint<3> E[3] = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};

auto TypedExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput& constInput) const -> InstancePtr {
  using namespace tex;
  struct Instance : public TypedExpressionSolver::Instance {

//    ExtVariable<UnitQuaternion> qC12;
//    ExtVariable<EuclideanPoint<3>> p2;
//    VExp<EuclideanPoint<3> > exp;
//    decltype(ExtVariable<UnitQuaternion>().rotate(ExtVariable<EuclideanPoint<3>>()) * Scalar<double>(0.5)) exp;

    Instance(const ProjectionProblem::ConstInput& constInput)
//        :
//      exp(qC12.rotate(p2) * Scalar<double>(0.5))
    {
    }


    virtual void solveInto(const TypedExpressionSolver::Problem::Input & input, TypedExpressionSolver::Problem::Output & output, const EvalVariants v) override {
      auto qC12 = Diffable<Ref<UnitQuaternion>, 0>(reinterpret_cast<const UnitQuaternion&>(input.qC12));
      auto p2 = Diffable<Ref<EuclideanPoint<3>>, 1>(reinterpret_cast<const EuclideanPoint<3>&>(input.p2));
//      auto exp = getExp(qC12, p2);
      auto exp = (rotate(qC12,p2) * Scalar<double>(0.5)).segment<2>(0);

//      qC12.setStorage(reinterpret_cast<const UnitQuaternion*>(&input.qC12));
//      p2.setStorage(reinterpret_cast<const EuclideanPoint<3>*>(&input.p2));

//      auto pRotated = exp.eval();

      switch(v){
        case EvalVariants::Eval :{
//          EuclideanPoint<3> pRotated; //(exp.eval())
//          double p[3];
//          pRotated = exp.getA().getA().eval().evalRotate(exp.getA().getB());
//          pRotated*= exp.getB().eval();
//          pRotated = exp.eval().getValue();
//          ceres::UnitQuaternionRotatePoint<double>(
//              reinterpret_cast<const double*>(&input.qC12),
//              reinterpret_cast<const double*>(&input.p2),
//              reinterpret_cast<double*>(&pRotated));
//              output.xy[0]= p[0] * 0.5;
//              output.xy[1]= p[1] * 0.5;
//            output.xy[0] = pRotated[0];
//            output.xy[1] = pRotated[1];
//          output.xy = evalExp(exp);
          {
            auto cache = createCache(exp); //impossible for now as the last application of Segment renders it useless, static_cast<EuclideanPoint<2>&>(output.xy)); 
            cache.update(exp);
            output.xy = toEigen(cache.accessValue(exp));
          }
        }
        break;
        case EvalVariants::EvalJacobian:
//        {
//          auto pRotated = exp.eval().getValue();
//          for (int i : {0, 1, 2}) {
//            auto & v = E[i];
//            output.jP2.block<2, 1>(0, i) = toEigen(getExp(qC12, v).eval().block<2, 1>(0, 0));
//            output.jPhi12.block<2, 1>(0, i) = toEigen(v.cross(pRotated).block<2, 1>(0, 0) * 2);
//          }
//        {
//          output.jPhi12.setZero();
//          evalFullDiffInto(exp, qC12, output.jPhi12);
//          output.jP2.setZero();
//          evalFullDiffInto(exp, p2, output.jP2);
//        }
        {
          auto cache = createCache(exp);
          cache.update(exp);
          output.jPhi12.setZero();
          evalFullDiffIntoCached(exp, qC12, cache, reinterpret_cast<Matrix<double, 2, 3>&>(output.jPhi12)); // TODO support simple matrices here again
          output.jP2.setZero();
          evalFullDiffIntoCached(exp, p2, cache, reinterpret_cast<Matrix<double, 2, 3>&>(output.jP2));
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
