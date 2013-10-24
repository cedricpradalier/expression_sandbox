/*
 * EuclideanPoint.hpp
 *
 *  Created on: Oct 10, 2013
 *      Author: hannes
 */

#ifndef EUCLIDEANPOINT_HPP_
#define EUCLIDEANPOINT_HPP_


#include <initializer_list>
#include <iostream>

#include "TypedExpressions.hpp"
#include "SimpleLinalg.hpp" //TODO discuss: how to do this

#define _USE_MATH_DEFINES
#include <cmath>

namespace TEX_NAMESPACE {
using namespace linalg;

template <typename PrimScalar_>
class Scalar {
 public:
  Scalar(PrimScalar_ v = 0) : value_(v){}

  Scalar evalSum(const Scalar & other) const {
    return Scalar(value_ + other.value_);
  }

  friend
  std::ostream & operator << (std::ostream & out, const Scalar & ob) {
    return out << ob.value_;
  }

  inline PrimScalar_ getValue() const {
    return value_;
  }
  inline void setVal(PrimScalar_ v) {
    value_ = v;
  }

  inline operator PrimScalar_ () const {
    return getValue();
  }

 private:
  PrimScalar_ value_;
};


template <typename A, typename B, typename Space_ = RESULT_SPACE(evalDot, A, B)>
class Dot : public BinOpBase<A, B, Space_, Dot<A, B, Space_> > {
 public:
  typedef BinOpBase<A, B, Space_, Dot<A, B, Space_> > Base;

  Dot(const A & a, const B & b) : Base(a, b){}

  const Space_ evalImpl() const {
    return evalExp(this->getA()).evalDot(evalExp(this->getB()));
  }

  friend
  std::ostream & operator << (std::ostream & out, const Dot & op) {
    return out << "<" << op.getA() << ", " << op.getB() << ">";
  }
};

namespace internal {
  template <typename A, typename B, typename Space>
  struct get_space<Dot<A, B, Space >>{
   public:
    typedef Space type;
  };
}

template <typename A, typename B, typename Result = Dot<A, B> >
inline typename Result::App dot(const A & a, const B & b){
  return Result(a, b);
}

template <MatrixSize Dim_>
class EuclideanPoint : public Vector<Dim_> {
 public:
  constexpr static MatrixSize Dimension = Dim_;
  typedef Scalar<typename Vector<Dimension>::Scalar> ScalarSpace;

  EuclideanPoint() = default;

  EuclideanPoint(std::initializer_list<double> entries) : Vector<Dim_>(MatrixConvertible<std::initializer_list<double>>::asMatrixConvertible(entries, Dim_)) {
  }
  template <typename Value>
  EuclideanPoint(const Value & v) : Vector<Dim_>(v) {
  }

  inline EuclideanPoint evalNegate() const { //TODO switch to operators for Spaces too and make them eventually convertible to their Constant<T>?
    return -getValue();
  }

  template <typename Ret = Neg<EuclideanPoint, EuclideanPoint > >
  inline typename Ret::App operator -() const {
    return Ret(*this);
  }

  inline EuclideanPoint evalTimes(const ScalarSpace & other) const {
    return EuclideanPoint(getValue() * other.getValue());
  }

  inline EuclideanPoint evalSum(const EuclideanPoint & other) const {
    return getValue() + other.getValue();
  }

  inline ScalarSpace evalDot(const EuclideanPoint & other) const {
    return getValue().dot(other.getValue());
  }

  template <typename Other, typename Ret = Dot<EuclideanPoint, Other, ScalarSpace > >
  inline typename Ret::App dot (const Other & other) const {
    return Ret(*this, other);
  }

  inline double & operator[](MatrixSize i) {
    return getValue()[i];
  }

  inline const ScalarSpace operator[](MatrixSize i) const {
    return getValue()[i];
  }

  friend
  std::ostream & operator << (std::ostream & out, const EuclideanPoint & ob) {
    static_assert(Dimension > 0, "");
    return out << ob.getValue();
  }

  bool operator == (const EuclideanPoint & other) const {
    return getValue() == other.getValue();
  }

  inline Vector<Dimension> & getValue() {
    return *this;
  }

  inline const Vector<Dimension> & getValue() const {
    return *this;
  }
};

template <MatrixSize Dim_, typename DERIVED>
struct OpMemberBase<EuclideanPoint<Dim_>, DERIVED> {
  template <typename Other, typename Ret = Dot<DERIVED, Other> >
  inline typename Ret::App dot (const Other & other){
    return Ret(static_cast<DERIVED&>(*this), other);
  }
};

}

#define SUPPORTS_EUCLIDEAN_DOT

#define SUPPORTS_EUCLIDEAN_ROTATION


#endif /* EUCLIDEANPOINT_HPP_ */
