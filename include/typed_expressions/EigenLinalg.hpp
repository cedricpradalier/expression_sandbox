/*
 * SimpleLinalg.hpp
 *
 *  Created on: Oct 9, 2013
 *      Author: hannes
 */

#ifndef EIGENLINALG_HPP_
#define EIGENLINALG_HPP_
#ifndef EXP_LINALG
#define EXP_LINALG
#define TYPED_EXP_NAMESPACE_POSTFIX _eig

#include <Eigen/Core>

namespace linalg {

template <typename DERIVED>
using MatrixBase = Eigen::MatrixBase<DERIVED>;

template <typename Scalar, int Rows, int Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

template <int Dim_>
using Vector = Matrix<double, Dim_, 1>;

typedef int MatrixSize;

template<typename T>
T asMatrixConvertible(const T & t) {
  return t;
}

}

using namespace linalg;

#endif
#endif /* EIGENLINALG_HPP_ */
