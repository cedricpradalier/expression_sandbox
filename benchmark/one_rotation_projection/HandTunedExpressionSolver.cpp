#include <stdexcept>
#include <Eigen/Geometry>
#include <benchmark/ProblemSolver.hpp>
#include "ProjectionProblem.hpp"

namespace one_rotation_projection_problem {

namespace hand_solver {

class HandExpressionSolver : public benchmark::ProblemSolver<ProjectionProblem>{
 public:
  typedef ProblemSolver<ProjectionProblem> Base;
  typedef ProjectionProblem Problem;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const {
    return "HandOptimizedFunction";
  }

  virtual InstancePtr createNewInstance(const ProjectionProblem::ConstInput & constInput) const;

  virtual ~HandExpressionSolver() {};
};

HandExpressionSolver::InstancePtr HandExpressionSolver::createNewInstance(const ProjectionProblem::ConstInput & constInput) const{
  struct Instance : public HandExpressionSolver::Instance {

    Instance(const Problem::ConstInput& constInput)
    {
    }

    virtual void solveInto(const Problem::Input & input, Problem::Output & output, const Problem::Variant v) override {
      const Eigen::Quaterniond q(input.qC12[0], input.qC12[1], input.qC12[2], input.qC12[3]);
      switch(v){
        case EvalVariants::Eval :{
          output.xy = q._transformVector(input.p2).segment<2>(0) * 0.5;
        }
        break;
        case EvalVariants::EvalJacobian:
        {
          auto mat = q.toRotationMatrix();
          auto rot = mat * input.p2;
          output.jPhi12 <<
              0, rot[2], -rot[1],
              -rot[2], 0, rot[0];
          ;
          output.jP2 = mat.block<2, 3>(0, 0) * 0.5;
        }
        break;
      }
    }
  };

  return InstancePtr(new Instance(constInput));

}
  HandExpressionSolver solver;
}
} /* namespace one_rotation_projection_problem */

