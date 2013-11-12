/*
 * TypedExpressionSolverImpl.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <sstream>
#include <typed_expressions/UnitQuaternions.hpp>
#include <typed_expressions/SimpleLinalgWithEigenAdapter.hpp>

//#include <ceres/rotation.h>

namespace rotation_problem {

#ifndef SOLVER
#define SOLVER solver
#endif

namespace SOLVER {

using namespace tex;


template <unsigned N, typename Rot, typename Vector>
struct RotateType {
  typedef RotateType<N - 1, Rot, Vector> Previous;
  typedef Rotate<Diffable<Ref<UnitQuaternion>, N>, typename Previous::type> type;

  static type getExp(const Rot* rotations, const Vector & v) {
    return rotate(Diffable<Ref<UnitQuaternion>, N>(reinterpret_cast<const UnitQuaternion&>(rotations[N - 1])), Previous::getExp(rotations, v));
  }

  template <typename Exp, typename Cache, typename Matrix_>
  static TEX_STRONG_INLINE void evalFullDiffCached(const Exp & exp, Cache & cache, const Rot * rotations, Matrix_ * matrices){
    matrices[N - 1].setZero();
    Previous::evalFullDiffCached(exp, cache, rotations, matrices);
    evalFullDiffIntoCached(exp, Diffable<Ref<UnitQuaternion>, N>(reinterpret_cast<const UnitQuaternion&>(rotations[N - 1])), cache, reinterpret_cast<Matrix<double, 3, 3>&>(matrices[N - 1]));
  }

};
template <typename Rot, typename Vector>
struct RotateType<0, Rot, Vector> {
  typedef Vector type;

  static inline type getExp(const Rot* rotations, const Vector & v){
    return v;
  }

  template <typename Exp, typename Cache, typename Matrix_>
  static void evalFullDiffCached(const Exp & exp, Cache & cache, const Rot * rotations, Matrix_ * matrices){
  }
};


template <unsigned N>
class TypedExpressionSolver : public benchmark::ProblemSolver<RotationProblem<N>>{
 public:
  TypedExpressionSolver();
  typedef benchmark::ProblemSolver<RotationProblem<N>> Base;
  typedef RotationProblem<N> Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const;

  virtual InstancePtr createNewInstance(const typename RotationProblem<N>::ConstInput & constInput) const;

  virtual ~TypedExpressionSolver() {};
};


#define SOLVER_NAME__(X) #X
#define SOLVER_NAME_(X) SOLVER_NAME__(X)
#define SOLVER_NAME SOLVER_NAME_(SOLVER)


template <unsigned N>
std::string TypedExpressionSolver<N>::getName() const {
  std::stringstream out;
  out << "TypedExpressionSolver[" << N << "](" << SOLVER_NAME << ")";
  return out.str();
}

template <unsigned N>
auto TypedExpressionSolver<N>::createNewInstance(const typename RotationProblem<N>::ConstInput& constInput) const -> InstancePtr {
  using namespace tex;
  struct Instance : public TypedExpressionSolver::Instance {
    Instance(const typename RotationProblem<N>::ConstInput& constInput){
    }

    virtual void solveInto(const typename TypedExpressionSolver<N>::Problem::Input & input, typename TypedExpressionSolver<N>::Problem::Output & output, const benchmark::EvalVariants v) override {
      auto p = Diffable<Ref<EuclideanPoint<3>>, 0>(reinterpret_cast<const EuclideanPoint<3>&>(input.p));

      typedef RotateType<N, typename std::remove_reference<decltype(input.qC[0])>::type, typename std::remove_reference<decltype(p)>::type> RotateType;

      auto exp = RotateType::getExp(input.qC, p);
      auto cache = createCache(exp);
      cache.update(exp);

      switch(v){
        case benchmark::EvalVariants::Eval :{
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
            output.r = tex::toEigen(cache.accessValue(exp).getValue());
          }
        }
        break;
        case benchmark::EvalVariants::EvalJacobian:
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
          RotateType::evalFullDiffCached(exp, cache, input.qC, output.jPhi);
          output.jP.setZero();
          evalFullDiffIntoCached(exp, p, cache, reinterpret_cast<Matrix<double, 3, 3>&>(output.jP));
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));
}

template <unsigned N>
TypedExpressionSolver<N>::TypedExpressionSolver() {
}

CREATE_INSTANCES(TypedExpressionSolver<,>, instance)

}
} /* namespace rotation_problem */
