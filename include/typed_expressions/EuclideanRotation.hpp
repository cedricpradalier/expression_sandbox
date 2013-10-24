/*
 * EuclideanRotation.hpp
 *
 *  Created on: Oct 10, 2013
 *      Author: hannes
 */

#ifndef EUCLIDEANROTATION_HPP_
#define EUCLIDEANROTATION_HPP_

#include "TypedExpressions.hpp"
#include "SimpleLinalg.hpp" //TODO discuss: how to do this
#include "UnitQuaternions.hpp"

namespace TEX_NAMESPACE {

template <typename DERIVED>
class Angle : public Scalar<double>{
 public:
  explicit Angle(double v) : Scalar<double>(v) {}
  template <typename T>
  Angle(const Angle<T> & other) : Angle(other.getValue() * DERIVED::getHalf() / T::getHalf()) {}

  constexpr double getFull(){
    return DERIVED::getHalf() * 2;
  }
  bool operator == (const DERIVED & other) const {
    return fmod(fabs(getValue() - other.getValue()), getFull()) == 0;
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

template <typename T >
struct Expander {
  inline const T & expand(const T & t) { return t; }
};

template <MatrixSize Dim_>
class EuclideanRotation {
 public:
  constexpr static MatrixSize Dimension = Dim_;
  ~EuclideanRotation(){
    static_assert(Dim_ > 0, "Dimension must be positive!");
    static_assert(Dim_ == 0, "This dimension is not implemented yet!");
  }
};

template <MatrixSize Dim_, typename DERIVED>
struct OpMemberBase<EuclideanRotation<Dim_>, DERIVED> {
  template <typename Other, typename Ret = Rotate<DERIVED, Other> >
  inline typename Ret::App rotate(const Other & other){
    return Ret(static_cast<DERIVED&>(*this), other);
  }
};

template <>
class EuclideanRotation<2> {
 public:
  constexpr static MatrixSize Dimension = 2;

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
    return EuclideanRotation(RotationFraction(angle_.getValue() + other.angle_.getValue()));
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
    return out << "Rot" << Dimension << "d(" << Degree(op.angle_).getValue() << "°)";
  }

 private:
  RotationFraction angle_;
};

template <>
class EuclideanRotation<3> {
 public:
  constexpr static MatrixSize Dimension = 3;

  EuclideanRotation(std::initializer_list<double> q) : q_(asMatrixConvertible(q)){}
  template <typename T>
  EuclideanRotation(const T & q) : q_(q){}

  /*
  template <typename A, typename B>
  inline static auto expandRotate(const AnyExp<EuclideanRotation, A> & rotation, const AnyExp<EuclideanPoint<Dimension>, B> & vector) const -> decltype(rotation.eval().q_.inverse() * vector * rotation.eval().q_){
    auto & q = rotation.eval().q_;
    return q_.inverse() * vector * q;
  }
  */

  EuclideanPoint<Dimension> evalRotate(const EuclideanPoint<Dimension> & vector) const {
    return EuclideanPoint<Dimension>(q_.rotate(vector).eval());
  }

  EuclideanRotation evalTimes(const EuclideanRotation & other) const {
    return EuclideanRotation((q_ * other.q_).eval());
  }

  EuclideanRotation evalTimesJac(const EuclideanRotation & other) const {
    return EuclideanRotation((q_ * other.q_).eval());
  }

  bool operator == (const EuclideanRotation & other) const {
    return q_ == other.q_ || (-q_).eval() == other.q_;
  }

  template <typename Ret = Rotate<EuclideanRotation, EuclideanPoint<Dimension> > > const
  inline Ret rotate(const EuclideanPoint<Dimension> & other){
    return Ret(*this, other);
  }

  friend
  std::ostream & operator << (std::ostream & out, const EuclideanRotation & op) {
    return out << "Rot" << Dimension << "D(" << op.q_ << ")";
  }

 private:
  UnitQuaternion q_;
};

}

#endif /* EUCLIDEANROTATION_HPP_ */
