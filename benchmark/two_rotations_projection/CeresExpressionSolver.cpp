#include <stdexcept>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>
#include <benchmark/ProblemSolver.hpp>
#include "ProjectionProblem.hpp"


namespace projection_problem {

namespace ceres_solver {
using namespace ceres;

struct TestErrorQuat {
//    TestErrorQuat(double u, double v, double weight=1.0)
//        : u(u), v(v), weight(weight)
//    {
//    }

    template <typename T>
        bool operator()(const T* const q_1_2,
                const T* const q_2_3,
                const T* const p,
                T* residuals) const {
            T p1[3], p2[3];

            ceres::UnitQuaternionRotatePoint(q_2_3, p, p1);

            // A nice function to do that would be cleaner, but...
            T q_1_2_inv[4] = {q_1_2[0],-q_1_2[1],-q_1_2[2],-q_1_2[3]};
            ceres::UnitQuaternionRotatePoint(q_1_2_inv, p1, p2);

            // The error is the difference between the predicted and a position.
//            residuals[0] = T(weight)*(T(u) - p[0]/p[2]);
//            residuals[1] = T(weight)*(T(v) - p[1]/p[2]);
            residuals[0] = p2[0] * 0.5;
            residuals[1] = p2[1] * 0.5;

            return true;
        }

//    double u;
//    double v;
//    double weight;

    static void test_functor() {
        double q_1_2[4] = {1,0,0,0};
        double q_2_3[4] = {1,0,0,0};
        double p_3[3] = {1,0,0};
        double res[2] = {5.0, 2.0};

        double *parameters[] = {q_1_2,q_2_3,p_3};
        double J_q_1_2[8];
        double J_q_2_3[8];
        double J_p_3[6];
        double *jacobians[] = {J_q_1_2,J_q_2_3,J_p_3};

        CostFunction * cost_function = new AutoDiffCostFunction<TestErrorQuat,2,4,4,3>(
//            new TestErrorQuat(5.0,2.0,1.0));
                new TestErrorQuat());

        cost_function->Evaluate(parameters, res, jacobians);

        delete cost_function;
    }
};

// Same with local parameterization (not in the ceres way though)
struct TestErrorAngleAxis : public TestErrorQuat{
    TestErrorAngleAxis(double u, double v, double weight=1.0) 
        : TestErrorQuat() { }

    template <typename T>
        bool operator()(const T* const aa_1_2,
                const T* const aa_2_3,
                const T* const P,
                T* residuals) const {
            T q_1_2[4], q_2_3[4];
            ceres::AngleAxisToQuaternion(aa_1_2,q_1_2);
            ceres::AngleAxisToQuaternion(aa_2_3,q_2_3);
            return TestErrorQuat::operator()(q_1_2,q_2_3,P,residuals);
        }

    static void test_functor() {
        double aa_1_2[3] = {0,0,0};
        double aa_2_3[3] = {0,0,0};
        double p_3[3] = {1,0,0};
        double res[2] = {5.0, 2.0};

        double *parameters[] = {aa_1_2,aa_2_3,p_3};
        double J_aa_1_2[6];
        double J_aa_2_3[6];
        double J_p_3[6];
        double *jacobians[] = {J_aa_1_2,J_aa_2_3,J_p_3};

        CostFunction * cost_function = new AutoDiffCostFunction<TestErrorAngleAxis,2,3,3,3>(
                new TestErrorAngleAxis(5.0,2.0,1.0));

        cost_function->Evaluate(parameters, res, jacobians);

        delete cost_function;
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
    AutoDiffCostFunction<TestErrorQuat,2,4,4,3> exp;

    Instance(const Problem::ConstInput& constInput) :
      exp(new TestErrorQuat())
    {
    }

    virtual void solveInto(const Problem::Input & input, Problem::Output & output, const Problem::Variant v) override {
      const double *q_1_2 = &input.qC12[0];
      const double *q_2_3 = &input.qC23[0];
      const double *parameters[] = {q_1_2, q_2_3, &input.p3[0]};
      switch(v){
        case EvalVariants::Eval :{
          if(!exp.Evaluate(parameters, (double*)&output.xy[0], nullptr)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }
        }
        break;
        case EvalVariants::EvalJacobian:
        {
          double J_q_1_2[8], J_q_2_3[8], J_p_3[6];
          double *jacobians[] = {J_q_1_2, J_q_2_3, J_p_3};
          double result[2];
          if(!exp.Evaluate(parameters, result, jacobians)){
            throw(std::runtime_error("ceres AutoDiffCostFunction.Evaluate failed."));
          }

    #ifdef DEBUG
          std::cout << "qC12=" << std::endl << input.qC12 << std::endl;
          std::cout << "qC23=" << std::endl << input.qC23 << std::endl;
    #endif

          double localParamJac[4 * 3];
          ceres::QuaternionParameterization().ComputeJacobian(q_1_2, localParamJac);
          output.jPhi12 = Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> >(J_q_1_2) * Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> >(localParamJac);
          ceres::QuaternionParameterization().ComputeJacobian(q_2_3, localParamJac);
          output.jPhi23 = Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> >(J_q_2_3) * Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> >(localParamJac);
          output.jP3 = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(J_p_3);
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}


CeresExpressionSolver ceresSolver;

}
} /* namespace projection_problem */

