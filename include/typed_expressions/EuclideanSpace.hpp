/*
 * EuclideanSpace.hpp
 *
 *  Created on: Sep 25, 2013
 *      Author: hannes
 */

#ifndef EUCLIDEANSPACE_HPP_
#define EUCLIDEANSPACE_HPP_

#include "TypedExpressions.hpp"
#include <initializer_list>
#include <iostream>


#define _USE_MATH_DEFINES
#include <cmath>

namespace tex{

template <typename PrimScalar_>
class Scalar {
 public:
  const Scalar & eval() const { // TODO make this function unnecessary
    return *this;
  }

  Scalar(PrimScalar_ v = 0) : value_(v){}

  Scalar evalSum(const Scalar & other) const {
    return Scalar(value_ + other.value_);
  }

  friend
  std::ostream & operator << (std::ostream & out, const Scalar & ob) {
    return out << ob.value_;
  }

  inline PrimScalar_ getVal() const {
    return value_;
  }
  inline void setVal(PrimScalar_ v) {
    value_ = v;
  }

  inline operator PrimScalar_ () const {
    return getVal();
  }

 private:
  PrimScalar_ value_;
};


template <typename A, typename B, typename ValueType_ = RESULT_TYPE(evalDot, A, B)>
class Dot : public BinOpBase<A, B, ValueType_, Dot<A, B, ValueType_> > {
 public:
  typedef BinOpBase<A, B, ValueType_, Dot<A, B, ValueType_> > Base;
  typedef ValueType_ ValueType;

  Dot(const A & a, const B & b) : Base(a, b){}

  ValueType eval() const {
    return this->getA().eval().evalDot(this->getB().eval());
  }

  friend
  std::ostream & operator << (std::ostream & out, const Dot & op) {
    return out << "<" << op.getA() << ", " << op.getB() << ">";
  }
};

template <typename A, typename B, typename Result = Dot<A, B> >
inline typename Result::App dot(const A & a, const B & b){
  return Result(a, b);
}

template <size_t Dim_>
class EuclideanPoint {
 public:
  constexpr static size_t Dimension = Dim_;
  const EuclideanPoint & eval() const { // TODO make this function unnecessary
    return *this;
  }

  EuclideanPoint(){
    for(int i = 0; i < Dimension; i ++) { value[i] = 0; }
  }

  EuclideanPoint evalSum (const EuclideanPoint & other) const {
    EuclideanPoint r;
    for(int i = 0; i < Dimension; i ++) { r.value[i] = value[i] + other.value[i]; }
    return r;
  }

  EuclideanPoint(std::initializer_list<double> entries){
    if(entries.size() > Dimension) throw std::runtime_error("too many initializing entries");
    int i = 0;
    for(double d : entries) { value[i++] = d; }
    for(; i < Dimension; i ++) { value[i] = 0; }
  }

  Scalar<double> evalDot(const EuclideanPoint & other) const {
    double r = 0;
    for(int i = 0; i < Dimension; i ++) { r+= value[i] * other.value[i]; }
    return r;
  }

  template <typename Other, typename Ret = Dot<EuclideanPoint, Other, Scalar<double> > >
  inline typename Ret::App dot (const Other & other){
    return Ret(*this, other);
  }

  Scalar<double> operator[](size_t i) const {
    return value[i];
  }

  friend
  std::ostream & operator << (std::ostream & out, const EuclideanPoint & ob) {
    static_assert(Dimension > 0, "");
    out << "[" << ob.value[0];
    for(int i = 1; i < Dimension; i ++) { out << ", " << ob.value[i]; }
    return out << "]";
  }

  bool operator == (const EuclideanPoint & other) const {
    return memcmp(value, other.value, sizeof(value)) == 0;
  }
 private:
  double value[Dimension];
};

namespace internal {
  template <typename DERIVED, typename Other, typename ValueType>
  struct UnwrapValueType<Dot<DERIVED, Other, ValueType >>{
   public:
    typedef ValueType type;
  };
}

template <size_t Dim_, typename DERIVED>
struct OpMemberBase<EuclideanPoint<Dim_>, DERIVED> {
  template <typename Other, typename Ret = Dot<DERIVED, Other> >
  inline typename Ret::App dot (const Other & other){
    return Ret(static_cast<DERIVED&>(*this), other);
  }
};

#define SUPPORTS_EUCLIDEAN_DOT

template <typename DERIVED>
class Angle : public Scalar<double>{
 public:
  explicit Angle(double v) : Scalar<double>(v) {}
  template <typename T>
  Angle(const Angle<T> & other) : Angle(other.getVal() * DERIVED::getHalf() / T::getHalf()) {}

  constexpr double getFull(){
    return DERIVED::getHalf() * 2;
  }
  bool operator == (const DERIVED & other) const {
    return fmod(fabs(getVal() - other.getVal()), getFull()) == 0;
  }
};

class Radian : public Angle<Radian> {
 public:
  explicit Radian(double v) : Angle(v) {}
  constexpr static double getHalf() {return M_PI;}
  template <typename T>
  Radian(const Angle<T> & other) : Angle(other) {}
};

class Degree : public Angle<Degree> {
 public:
  constexpr static double getHalf() {return 180.0;}
  explicit Degree(double v) : Angle(v) {}
  template <typename T>
  Degree(const Angle<T> & other) : Angle(other) {}
};

class RotationFraction : public Angle<RotationFraction> {
 public:
  constexpr static double getHalf() {return 0.5;}
  explicit RotationFraction(double v) : Angle(v) {}
  template <typename T>
  RotationFraction(const Angle<T> & other) : Angle(other) {}
};

#define SUPPORTS_ANGLE_TYPES

template <size_t Dim_>
class EuclideanRotation {
 public:
  constexpr static size_t Dimension = Dim_;
  ~EuclideanRotation(){
    static_assert(Dim_ > 0, "Dimension must be positive!");
    static_assert(Dim_ == 0, "This dimension is not implemented yet!");
  }
};

template <typename A, typename B, typename ValueType_ = RESULT_TYPE(evalRotate, A, B)>
class Rotate : public BinOpBase<A, B, ValueType_, Rotate<A, B, ValueType_> > {
 public:
  typedef BinOpBase<A, B, ValueType_, Rotate<A, B, ValueType_> > Base;
  typedef ValueType_ ValueType;

  Rotate(const A & a, const B & b) : Base(a, b){}

  ValueType eval() const {
    return this->getA().eval().evalRotate(this->getB().eval());
  }

  friend
  std::ostream & operator << (std::ostream & out, const Rotate & op) {
    return out << op.getA() << ".rotate(" << op.getB() << ")";
  }
};

namespace internal {
  template <typename DERIVED, typename Other, typename ValueType>
  struct UnwrapValueType<Rotate<DERIVED, Other, ValueType >>{
   public:
    typedef ValueType type;
  };
}

template <typename A, typename B, typename Result = Rotate<A, B> >
inline typename Result::App rotate(const A & a, const B & b){
  return Result(a, b);
}

template <>
class EuclideanRotation<2> {
 public:
  constexpr static size_t Dimension = 2;
  const EuclideanRotation & eval() const { // TODO make this function unnecessary
    return *this;
  }

  EuclideanRotation(const RotationFraction & angle) : angle_(angle){}

  EuclideanPoint<Dimension> evalRotate(const EuclideanPoint<Dimension> & other) const {
    if(angle_.getHalf() * 2 == angle_){
      return other;
    }
    else if(angle_.getHalf() == angle_){
      return EuclideanPoint<Dimension>({-other[0], -other[1]});;
    }
    else if(angle_.getHalf() / 2 == angle_){
      return EuclideanPoint<Dimension>({-other[1], other[0]});
    }
    else if(angle_.getHalf() / 2 == -angle_){
      return EuclideanPoint<Dimension>({other[1], -other[0]});
    }
    Radian rad(angle_);
    return EuclideanPoint<Dimension>({cos(rad) * other[0] + - sin(rad) * other[1], sin(rad) * other[0] + cos(rad) * other[1]});
  }

  EuclideanRotation evalTimes(const EuclideanRotation & other) const {
    return EuclideanRotation(RotationFraction(angle_.getVal() + other.angle_.getVal()));
  }

  bool operator == (const EuclideanRotation & other) const {
    return angle_ == other.angle_;
  }

  template <typename Ret = Rotate<EuclideanRotation, EuclideanPoint<Dimension> > >
  inline Ret rotate(const EuclideanPoint<Dimension> & other){
    return Ret(*this, other);
  }

  friend
  std::ostream & operator << (std::ostream & out, const EuclideanRotation & op) {
    return out << "Rot" << Dimension << "d(" << Degree(op.angle_).getVal() << "°)";
  }

 private:
  RotationFraction angle_;
};


template <size_t Dim_, typename DERIVED>
struct OpMemberBase<EuclideanRotation<Dim_>, DERIVED> {
  template <typename Other, typename Ret = Rotate<DERIVED, Other> >
  inline typename Ret::App rotate(const Other & other){
    return Ret(static_cast<DERIVED&>(*this), other);
  }
};

#define SUPPORTS_EUCLIDEAN_ROTATION

template <int Dim_>
struct EuclideanSpace {
  typedef EuclideanPoint<Dim_> Point;
  typedef EuclideanPoint<Dim_> Vector;
  typedef EuclideanRotation<Dim_> Rotation;
};

}

#endif /* EUCLIDEANSPACE_HPP_ */
