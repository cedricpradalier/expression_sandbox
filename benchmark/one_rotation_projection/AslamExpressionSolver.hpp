/*
 * AslamExpressionSolver.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef ASLAMEXPRESSIONSOLVER_HPP_
#define ASLAMEXPRESSIONSOLVER_HPP_

#include <benchmark/ProblemSolver.hpp>
#include "ProjectionProblem.hpp"

namespace one_rotation_projection_problem {
namespace aslam_solver {

template<bool UseNewGenericMatrixStuff>
class AslamExpressionSolver : public benchmark::ProblemSolver<ProjectionProblem> {
 public:
  typedef ProjectionProblem Problem;
  typedef ProblemSolver<ProjectionProblem> Base;
  typedef typename Base::InstancePtr InstancePtr;

  virtual std::string getName() const;

  virtual InstancePtr createNewInstance(const ProjectionProblem::ConstInput & constInput) const;

  virtual ~AslamExpressionSolver() = default;
};

}
}
#endif /* ASLAMEXPRESSIONSOLVER_HPP_ */
