/*
 * UnitQuaternions.hpp
 *
 *  Created on: Oct 9, 2013
 *      Author: hannes
 */

#ifndef UNITQUATERNIONS_HPP_
#define UNITQUATERNIONS_HPP_

#include "TypedExpressions.hpp"
#include "SimpleLinalg.hpp" //TODO discuss how to do this
#include "EuclideanPoint.hpp"

namespace quat_calc {
enum class QuaternionMode {
  FIRST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 0,
  FIRST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 1,
  LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 2,
  LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 3,
}constexpr DefaultQuaternionMode = QuaternionMode::LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER;

enum class UnitQuaternionGeometry {
  LEFT_TRANSLATED,
  RIGHT_TRANSLATED
} const DefaultUnitQuaternionGeometry  = UnitQuaternionGeometry::RIGHT_TRANSLATED;


namespace internal {
enum {
  OPPOSITE_MULT_ORDER_MASK = 1,
  REAL_IS_LAST_MASK = 2,
};

inline constexpr bool isRealFirst(const QuaternionMode mode) {
  return ((int) mode & internal::REAL_IS_LAST_MASK) == 0;
}

inline constexpr bool isTraditionalMultOrder(const QuaternionMode mode) {
  return ((int) mode & internal::OPPOSITE_MULT_ORDER_MASK) == 0;
}
inline constexpr int getRealIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 0 : 3;
}
inline constexpr int getIIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 1 : 0;
}
inline constexpr int getJIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 2 : 1;
}
inline constexpr int getKIndex(const QuaternionMode mode) {
  return isRealFirst(mode) ? 3 : 2;
}

template<typename TScalar, enum QuaternionMode EMode = DefaultQuaternionMode>
struct EigenQuaternionCalculator {
  typedef Vector<4> vector_t;
  typedef Vector<3> pure_imag_vector_t;
  typedef Vector<3> lie_algebra_vector_t;

  enum {
    RIndex = getRealIndex(EMode),
    IIndex = getIIndex(EMode),
    JIndex = getJIndex(EMode),
    KIndex = getKIndex(EMode),
    IPureIndex = 0,
    JPureIndex = 1,
    KPureIndex = 2
  };

  inline static vector_t getIdentity() {
    return isRealFirst(EMode) ? vector_t(1, 0, 0, 0) : vector_t(0, 0, 0, 1);
  }

  inline static void quatMultTraditionalInto(const vector_t & a, const vector_t & b, vector_t & res) {
    // aIIndex*bRIndex + aJIndex*bKIndex - aKIndex*bJIndex + aRIndex*bIIndex
    res[IIndex] = a[IIndex] * b[RIndex] + a[JIndex] * b[KIndex] - a[KIndex] * b[JIndex] + a[RIndex] * b[IIndex];
    // aKIndex*bIIndex - aIIndex*bKIndex + aJIndex*bRIndex + aRIndex*bJIndex
    res[JIndex] = a[KIndex] * b[IIndex] - a[IIndex] * b[KIndex] + a[JIndex] * b[RIndex] + a[RIndex] * b[JIndex];
    // aIIndex*bJIndex - aJIndex*bIIndex + aKIndex*bRIndex + aRIndex*bKIndex
    res[KIndex] = a[IIndex] * b[JIndex] - a[JIndex] * b[IIndex] + a[KIndex] * b[RIndex] + a[RIndex] * b[KIndex];
    // aRIndex*bRIndex - aJIndex*bJIndex - aKIndex*bKIndex - aIIndex*bIIndex
    res[RIndex] = a[RIndex] * b[RIndex] - a[JIndex] * b[JIndex] - a[KIndex] * b[KIndex] - a[IIndex] * b[IIndex];
  }
  inline static void quatMultTraditionalInto(const vector_t & a, const vector_t & b, pure_imag_vector_t & res) {
    // aIIndex*bRIndex + aJIndex*bKIndex - aKIndex*bJIndex + aRIndex*bIIndex
    res[IPureIndex] = a[IIndex] * b[RIndex] + a[JIndex] * b[KIndex] - a[KIndex] * b[JIndex] + a[RIndex] * b[IIndex];
    // aKIndex*bIIndex - aIIndex*bKIndex + aJIndex*bRIndex + aRIndex*bJIndex
    res[JPureIndex] = a[KIndex] * b[IIndex] - a[IIndex] * b[KIndex] + a[JIndex] * b[RIndex] + a[RIndex] * b[JIndex];
    // aIIndex*bJIndex - aJIndex*bIIndex + aKIndex*bRIndex + aRIndex*bKIndex
    res[KPureIndex] = a[IIndex] * b[JIndex] - a[JIndex] * b[IIndex] + a[KIndex] * b[RIndex] + a[RIndex] * b[KIndex];
  }

  inline static void quatMultTraditionalInto(const pure_imag_vector_t & a, const vector_t & b, vector_t & res) {
    res[IIndex] = a[IPureIndex] * b[RIndex] + a[JPureIndex] * b[KIndex] - a[KPureIndex] * b[JIndex];
    res[JIndex] = a[KPureIndex] * b[IIndex] - a[IPureIndex] * b[KIndex] + a[JPureIndex] * b[RIndex];
    res[KIndex] = a[IPureIndex] * b[JIndex] - a[JPureIndex] * b[IIndex] + a[KPureIndex] * b[RIndex];
    res[RIndex] = -a[JPureIndex] * b[JIndex] - a[KPureIndex] * b[KIndex] - a[IPureIndex] * b[IIndex];
  }

  inline static void quatMultTraditionalInto(const vector_t & a, const pure_imag_vector_t & b, vector_t & res) {
    res[IIndex] = +a[JIndex] * b[KPureIndex] - a[KIndex] * b[JPureIndex] + a[RIndex] * b[IPureIndex];
    res[JIndex] = a[KIndex] * b[IPureIndex] - a[IIndex] * b[KPureIndex] + +a[RIndex] * b[JPureIndex];
    res[KIndex] = a[IIndex] * b[JPureIndex] - a[JIndex] * b[IPureIndex] + +a[RIndex] * b[KPureIndex];
    res[RIndex] = -a[JIndex] * b[JPureIndex] - a[KIndex] * b[KPureIndex] - a[IIndex] * b[IPureIndex];
  }

  inline static void quatMultTraditionalInto(const vector_t & a, const pure_imag_vector_t & b, pure_imag_vector_t & res) {
    res[IPureIndex] = +a[JIndex] * b[KPureIndex] - a[KIndex] * b[JPureIndex] + a[RIndex] * b[IPureIndex];
    res[JPureIndex] = a[KIndex] * b[IPureIndex] - a[IIndex] * b[KPureIndex] + +a[RIndex] * b[JPureIndex];
    res[KPureIndex] = a[IIndex] * b[JPureIndex] - a[JIndex] * b[IPureIndex] + +a[RIndex] * b[KPureIndex];
  }

  inline static void quatMultTraditionalInto(const pure_imag_vector_t & a, const vector_t & b, pure_imag_vector_t & res) {
    res[IPureIndex] = a[IPureIndex] * b[RIndex] + a[JPureIndex] * b[KIndex] - a[KPureIndex] * b[JIndex];
    res[JPureIndex] = a[KPureIndex] * b[IIndex] - a[IPureIndex] * b[KIndex] + a[JPureIndex] * b[RIndex];
    res[KPureIndex] = a[IPureIndex] * b[JIndex] - a[JPureIndex] * b[IIndex] + a[KPureIndex] * b[RIndex];
  }

  template<MatrixSize ISizeA, MatrixSize ISizeB, MatrixSize ISizeC>
  inline static void quatMultInto(const Vector<ISizeA> & a, const Vector<ISizeB> & b, Vector<ISizeC> & result) {
    return isTraditionalMultOrder(EMode) ? quatMultTraditionalInto(a, b, result) : quatMultTraditionalInto(b, a, result);
  }

  template<MatrixSize ISizeA, MatrixSize ISizeB>
  inline static vector_t quatMult(const Vector<ISizeA> & a, const Vector<ISizeB> & b) {
    vector_t result;
    quatMultInto(a, b, result);
    return result;
  }


  inline static vector_t conjugate(const vector_t & v) {
    vector_t r(v);
    r.template block<3, 1>(IIndex, 0) *= -1;
    return r;
  }
  inline static vector_t invert(const vector_t & v) {
    vector_t r(conjugate(v));
    r /= r.dot(r);
    return r;
  }

  inline static auto getImagPart(const vector_t & v) -> decltype(v.template block<3, 1>(IIndex, 0)) {
    return v.template block<3, 1>(IIndex, 0);
  }

  template<enum QuaternionMode EOtherMode, int ICols = 1, typename DERIVED_MATRIX>
  inline static linalg::Matrix<double, 4, ICols> convertFromOtherMode(const linalg::MatrixBase<DERIVED_MATRIX > & v){
    return EigenQuaternionCalculator<TScalar, EOtherMode>::template convertToOtherMode<EMode, ICols>(v);
  }

};
}
}


namespace TEX_NAMESPACE {
class UnitQuaternion : public EuclideanPoint<4> {
 private:
  typedef quat_calc::internal::EigenQuaternionCalculator<double> Calc;
 public:
  typedef EuclideanPoint<4> Base;

  const UnitQuaternion & eval() const { // TODO make this function unnecessary
    return *this;
  }

  UnitQuaternion() = default;
  UnitQuaternion(const UnitQuaternion & q) = default;
  UnitQuaternion(const Vector<4> & q) : Base(q){}

  UnitQuaternion evalNegate() const {
     return UnitQuaternion(-getValue());
  }

  UnitQuaternion evalInverse() const {
     return UnitQuaternion(Calc::conjugate(getValue()));
  }
  UnitQuaternion evalInverseDiff(const Vector<3> & thisTangent) const {
     Vector<4> con = Calc::conjugate(getValue());
     return UnitQuaternion(Calc::quatMult(Calc::quatMult(con, Vector<3>(-thisTangent)), con));
  }

  template <typename Ret = Inverse<UnitQuaternion> > const
  inline typename Ret::App inverse() const {
    return Ret(*this);
  }

  UnitQuaternion evalTimes(const UnitQuaternion & other) const{
    return UnitQuaternion(Calc::quatMult(getValue(), other.getValue()));
  }

  Vector<3> evalTimesDiff(const Vector<3> & thisTangent, const UnitQuaternion & other, const Vector<3> & otherTangent) const{
    Vector<3> ret;
    //TODO optimize this calculation with the right cross products.
    Calc::quatMultInto(getValue(), Calc::quatMult(otherTangent, getValue()), ret);
    return thisTangent + ret;
  }

  inline EuclideanPoint<3> evalRotate(const EuclideanPoint<3> & other) const{
//TODO reactivate optimized code here, when comparing to ceres
//    EuclideanPoint<3> ret;
//    Calc::quatMultInto(Calc::quatMult(getValue(), other.getValue()), Calc::conjugate(getValue()), ret.getValue());
//    EuclideanPoint<3> ret;
//    return ret;
    return Calc::getImagPart(Calc::quatMult(Calc::quatMult(getValue(), other.getValue()), Calc::conjugate(getValue())));
  }

  inline EuclideanPoint<3> evalRotateDiff(const Vector<3> & thisTangent, const EuclideanPoint<3> & other, const Vector<3> & otherTangent) const{
//TODO reactivate optimized code here, when comparing to ceres
//    EuclideanPoint<3> ret;
//    Calc::quatMultInto(Calc::quatMult(getValue(), other.getValue()), Calc::conjugate(getValue()), ret.getValue());
//    EuclideanPoint<3> ret;
//    return ret;
    return Calc::getImagPart(Calc::quatMult(Calc::quatMult(getValue(), other.getValue()), Calc::conjugate(getValue())));
  }


  template <typename Ret = Rotate<UnitQuaternion, EuclideanPoint<3> > >
  inline typename Ret::App rotate(const EuclideanPoint<3> & other) const{
    return Ret(*this, other);
  }

  template <typename Ret=EuclideanPoint<3> >
  inline Ret imag() const {
     return Ret(Calc::getImagPart(getValue()));
  }

  UnitQuaternion evalTimesJac(const UnitQuaternion & other) const;

  bool operator == (const UnitQuaternion & other) const {
    return getValue() == other.getValue();
  }

  friend
  std::ostream & operator << (std::ostream & out, const UnitQuaternion & op) {
    return out << "UnitQuat" << "(" << static_cast<const Base&>(op) << ")";
  }
};


template <typename DERIVED>
struct OpMemberBase<UnitQuaternion, DERIVED> {
  template <typename Ret = Inverse<DERIVED> >
  inline typename Ret::App inverse () const {
    return Ret(static_cast<const DERIVED&>(*this));
  }
  template <typename Other, typename Ret = Rotate<DERIVED, Other> >
  inline typename Ret::App rotate(const Other & other) const {
    return Ret(static_cast<const DERIVED&>(*this), other);
  }
};

}

#endif /* UNITQUATERNIONS_HPP_ */
