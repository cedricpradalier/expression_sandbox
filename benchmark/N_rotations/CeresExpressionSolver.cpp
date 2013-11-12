#include <stdexcept>
#include <cstring>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>
#include <benchmark/ProblemSolver.hpp>
#include "RotationProblem.hpp"

namespace rotation_problem {

namespace ceres_solver {
using namespace ceres;


template <unsigned N>
struct Impl {
  struct TestErrorQuat {
    template <typename T>
    bool operator()(const T* const q,
                    const T* const p,
                    T* residuals) const {

      T tmp[3];

      if(N == 2){
        ceres::UnitQuaternionRotatePoint(q, p, tmp);
        ceres::UnitQuaternionRotatePoint(q + 4, tmp, residuals);
        return true;
      }

      T *ptrs[2];
      ptrs[(N - 1) % 2] = residuals;
      ptrs[N % 2] = tmp;
      ceres::UnitQuaternionRotatePoint(q, p, ptrs[0]);
      for(unsigned i = 1; i < N; i++){
        ceres::UnitQuaternionRotatePoint(q + i * 4, ptrs[(i - 1) % 2], ptrs[i % 2]);
      }
      return true;
    }
  };
  typedef AutoDiffCostFunction<TestErrorQuat,3,4 * N,3> AutoDiff;
};

template <unsigned N>
class CeresExpressionSolver : public benchmark::ProblemSolver<RotationProblem<N>>{
 public:
  typedef benchmark::ProblemSolver<RotationProblem<N>> Base;
  typedef RotationProblem<N> Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return "CeresAutoDiffCostFunction";
  }

  virtual InstancePtr createNewInstance(const typename Problem::ConstInput & constInput) const;

  virtual ~CeresExpressionSolver() {};

};

template <unsigned N>
typename CeresExpressionSolver<N>::InstancePtr CeresExpressionSolver<N>::createNewInstance(const typename Problem::ConstInput & constInput) const{
  struct Instance : public CeresExpressionSolver::Instance {
    typename Impl<N>::AutoDiff exp;
    ceres::QuaternionParameterization parametrization;

    Instance(const typename Problem::ConstInput& constInput) :
      exp(new typename Impl<N>::TestErrorQuat())
    {
    }

    virtual void solveInto(const typename Problem::Input & input, typename Problem::Output & output, const typename Problem::Variant v) override {
      const double *parameters[] {&input.qC[0][0],  &input.p[0]};
      switch(v){
        case benchmark::EvalVariants::Eval :{
          if(!exp.Evaluate(parameters, (double*)&output.r[0], nullptr)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }
        }
        break;
        case benchmark::EvalVariants::EvalJacobian:
        {
          double J_q[3 * 4 * N], J_p[3 * 3];
          double *jacobians[2] = {J_q, J_p};
          double result[3];
          if(!exp.Evaluate(parameters, result, jacobians)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }

          Eigen::Map<Eigen::Matrix<double, 3, 4 * N, Eigen::RowMajor> > J_qMap(J_q);
          double localParamJac[4 * 3];
          Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > localParamJacMap(localParamJac);
          for(unsigned i = 0; i < N; i++){
            parametrization.ComputeJacobian(&input.qC[i][0], localParamJac);
            output.jPhi[i] = J_qMap.template block<3, 4>(0, i * 4) * localParamJacMap;
          }
          output.jP = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(J_p);
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}


CREATE_INSTANCES(CeresExpressionSolver<, >, ceresSolver)

}
} /* namespace rotation_problem */

