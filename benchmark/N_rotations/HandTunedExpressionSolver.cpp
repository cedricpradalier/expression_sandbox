#include <stdexcept>
#include <Eigen/Geometry>
#include <benchmark/ProblemSolver.hpp>
#include "RotationProblem.hpp"

namespace rotation_problem {

namespace hand_solver {

template <unsigned N>
class HandExpressionSolver : public benchmark::ProblemSolver<RotationProblem<N>>{
 public:
  typedef benchmark::ProblemSolver<RotationProblem<N>> Base;
  typedef RotationProblem<N> Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return "HandOptimizedFunction";
  }

  virtual InstancePtr createNewInstance(const typename Problem::ConstInput & constInput) const;

  virtual ~HandExpressionSolver() {};
};

template <unsigned N>
typename HandExpressionSolver<N>::InstancePtr HandExpressionSolver<N>::createNewInstance(const typename Problem::ConstInput & constInput) const{
  struct Instance : public HandExpressionSolver::Instance {

    Instance(const typename Problem::ConstInput& constInput)
    {
    }

    virtual void solveInto(const typename Problem::Input & input, typename Problem::Output & output, const typename Problem::Variant v) override {
      switch(v){
        case benchmark::EvalVariants::Eval :{
          Eigen::Quaterniond q(input.qC[0][0], input.qC[0][1], input.qC[0][2], input.qC[0][3]);
          for(int i = 1; i < N; i ++){
            const Eigen::Quaterniond qI(input.qC[i][0], input.qC[i][1], input.qC[i][2], input.qC[i][3]);
            q = qI * q;
          }
          output.r = q._transformVector(input.p);
        }
        break;
        case benchmark::EvalVariants::EvalJacobian:
        {
          Eigen::Quaterniond quats[N];
          quats[0] = Eigen::Quaterniond(input.qC[0][0], input.qC[0][1], input.qC[0][2], input.qC[0][3]);
          for(int i = 1; i < N; i ++){
            quats[i] = Eigen::Quaterniond(input.qC[i][0], input.qC[i][1], input.qC[i][2], input.qC[i][3]) * quats[i-1];
          }
          output.jP = quats[N - 1];

          for(int i = 0; i < N; i ++){
            auto rot = 2 * quats[i]._transformVector(input.p);

            output.jPhi[i] <<
                0, rot[2], -rot[1],
                -rot[2], 0, rot[0],
                rot[1], -rot[0], 0;
            if(i != N - 1){
              output.jPhi[i] = (quats[N - 1] * quats[i].conjugate()).matrix() * output.jPhi[i];
            }
          }
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}

CREATE_INSTANCES(HandExpressionSolver<, >, solver)
}
} /* namespace rotation_problem */

