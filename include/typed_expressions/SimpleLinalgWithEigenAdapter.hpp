/*
 * SimpleLinalgEigenAdapter.hpp
 *
 *  Created on: Oct 18, 2013
 *      Author: hannes
 */

#ifndef SIMPLELINALGEIGENADAPTER_HPP_
#define SIMPLELINALGEIGENADAPTER_HPP_

namespace simple_linalg {
  template <typename DERIVED>
  class MatrixBase;
}

namespace eigen_linalg {
template<typename DERIVED>
struct MatrixConvertible;

template<typename DERIVED>
struct MatrixConvertible<simple_linalg::MatrixBase<DERIVED> > {
  typedef ::Eigen::Matrix<typename DERIVED::Scalar, DERIVED::Rows, DERIVED::Cols> type;
  static inline type asMatrixConvertible(DERIVED & t){
    type v;
    for(int i = 0; i < DERIVED::Rows; i++){
      for(int j = 0; j < DERIVED::Cols; j++){
        v(i, j) = t(i, j);
      }
    }
    return v;
  }
};

}

namespace simple_linalg {
template <typename DERIVED>
class MatrixBase;

template <typename DERIVED>
inline auto toEigen(const MatrixBase<DERIVED> & m) -> decltype (eigen_linalg::MatrixConvertible<MatrixBase<DERIVED> >::asMatrixConvertible((DERIVED&)m)){
  return eigen_linalg::MatrixConvertible<MatrixBase<DERIVED> >::asMatrixConvertible((DERIVED&)m);
}
template <typename DERIVED>
inline auto toEigen(const MatrixBase<DERIVED> && m) -> decltype (eigen_linalg::MatrixConvertible<MatrixBase<DERIVED> >::asMatrixConvertible((DERIVED&)m)){
  return eigen_linalg::MatrixConvertible<MatrixBase<DERIVED> >::asMatrixConvertible((DERIVED&)m);
}

}
#endif /* SIMPLELINALGEIGENADAPTER_HPP_ */
