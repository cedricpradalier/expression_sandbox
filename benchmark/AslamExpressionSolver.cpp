/*
 * AslamExpressionSolver.cpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#include <aslam/backend/DesignVariableUnitQuaternion.hpp>
#include <aslam/backend/QuaternionExpression.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include "AslamExpressionSolver.hpp"

namespace projection_problem {
namespace aslam_solver {
using namespace aslam::backend;
using namespace aslam::backend::quaternion;

template<>
std::string AslamExpressionSolver<false>::getName() const {
  return std::string("AslamExpressionSolver(originalImplementation)");
}
template<>
std::string AslamExpressionSolver<true>::getName() const {
  return std::string("AslamExpressionSolver(newGenericQuats)");
}

template<bool UseNewGenericMatrixStuff>
struct Exp : public EuclideanExpression {
  RotationQuaternion qC12, qC23;
  EuclideanPoint p3;
  Exp(const ProjectionProblem::ConstInput& constInput) : EuclideanExpression(nullptr), qC12(sm::kinematics::quatIdentity()), qC23(sm::kinematics::quatIdentity()), p3(Eigen::Vector3d::Ones()) {
    (EuclideanExpression&)(*this) = (RotationExpression(&qC12) * RotationExpression(&qC23).inverse()) * EuclideanExpression(&p3);
  }

  void setVars(const typename AslamExpressionSolver<UseNewGenericMatrixStuff>::Problem::Input & input){
    qC12.setParameters(input.qC12);
    qC23.setParameters(input.qC23);
    p3.setParameters(input.p3 * 0.5); // TODO this is a little bit of cheating, but it is not implemented so far to scale a Euclidean expression.
  }
};

template<>
struct Exp<true> : public GenericMatrixExpression<3, 1> {
  DesignVariableUnitQuaternion<> qC12, qC23;
  DesignVariableGenericVector<3> p3;
  Exp(const ProjectionProblem::ConstInput& constInput)
  {
    (GenericMatrixExpression<3, 1>&)(*this) = ((UnitQuaternionExpression<>(&qC12).inverse() * UnitQuaternionExpression<>(&qC23)).rotate3Vector(GenericMatrixExpression<3,1>(&p3))) * 0.5;
  }
  void setVars(const typename AslamExpressionSolver<true>::Problem::Input & input){
    qC12.setParameters(input.qC12);
    qC23.setParameters(input.qC23);
    p3.setParameters(input.p3);
  }
};

template<bool UseNewGenericMatrixStuff>
auto AslamExpressionSolver<UseNewGenericMatrixStuff>::createNewInstance(const ProjectionProblem::ConstInput & constInput) const -> InstancePtr {
  struct Instance : public AslamExpressionSolver::Instance {
    Exp<UseNewGenericMatrixStuff> exp;

    Instance(const ProjectionProblem::ConstInput& constInput) : exp(constInput){}

    virtual void solveInto(const AslamExpressionSolver::Problem::Input & input, AslamExpressionSolver::Problem::Output & output) {
      exp.setVars(input);
      output.xy = exp.evaluate().template segment<2>(0);
      /*TODO discuss: the above is slower(!) then the following:
      auto v = exp.evaluate();
      for(int i : {0, 1}) output.xy[i] = v[i];
      */
    }
  };

  return InstancePtr(new Instance(constInput));
}

AslamExpressionSolver<true> instanceNew;
AslamExpressionSolver<false> instanceOld;

}
}

