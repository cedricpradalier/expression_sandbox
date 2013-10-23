#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>
#include <benchmark/ProblemSolver.hpp>
#include "ProjectionProblem.hpp"


namespace one_rotation_projection_problem {

namespace ceres_solver {
using namespace ceres;

struct TestErrorQuat {
    template <typename T>
        bool operator()(const T* const q_1_2,
                const T* const p,
                T* residuals) const {
            T p1[3];

            ceres::UnitQuaternionRotatePoint(q_1_2, p, p1);

            residuals[0] = p1[0] * 0.5;
            residuals[1] = p1[1] * 0.5;
            return true;
        }
};

class CeresExpressionSolver : public benchmark::ProblemSolver<ProjectionProblem>{
 public:
  typedef ProblemSolver<ProjectionProblem> Base;
  typedef ProjectionProblem Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return "CeresAutoDiffCostFunction";
  }

  virtual InstancePtr createNewInstance(const ProjectionProblem::ConstInput & constInput) const;

  virtual ~CeresExpressionSolver() {};
};

CeresExpressionSolver::InstancePtr CeresExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput & constInput) const{
  struct Instance : public CeresExpressionSolver::Instance {
    AutoDiffCostFunction<TestErrorQuat,2,4,3> exp;

    Instance(const Problem::ConstInput& constInput) :
      exp(new TestErrorQuat())
    {
    }

    virtual void solveInto(const Problem::Input & input, Problem::Output & output, const Problem::Variant v) override {
      const double *q_1_2 = &input.qC12[0];
      const double *parameters[] = {q_1_2, &input.p2[0]};
      switch(v){
        case EvalVariants::Eval :{
          if(!exp.Evaluate(parameters, (double*)&output.xy[0], nullptr)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }
        }
        break;
        case EvalVariants::EvalJacobian:
        {
          double J_q_1_2[8], J_p_3[6];
          double *jacobians[] = {J_q_1_2, J_p_3};
          double result[2];
          if(!exp.Evaluate(parameters, result, jacobians)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }

    #ifdef DEBUG
          std::cout << "qC12=" << std::endl << input.qC12 << std::endl;
    #endif

          double localParamJac[4 * 3];
          ceres::QuaternionParameterization().ComputeJacobian(q_1_2, localParamJac);
          /* this is slower than the line below and uses Eigen anyway. (slower is the extra copy to adapt column major to row major)
          double jac[2 * 3];
          ceres::internal::MatrixMatrixMultiply<2, 4, 4, 3, 0>(
              J_q_1_2,
              2,
              4,
              localParamJac,
              4,
              3,
              jac, 0, 0,  2, 3);
          output.jPhi12 = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(jac);
          */
          output.jPhi12 = Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> >(J_q_1_2) * Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> >(localParamJac);
          output.jP2 = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(J_p_3);
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}


CeresExpressionSolver ceresSolver;

}
} /* namespace one_rotation_projection_problem */

