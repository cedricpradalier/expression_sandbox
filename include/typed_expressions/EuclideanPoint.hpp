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
#include "Derivatives.hpp"

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

template <typename A, MatrixSize SegmentSize, typename Space_>
class Segment : public BinOpBase<A, MatrixSize, Space_, Segment<A, SegmentSize, Space_> > {
 public:
  typedef BinOpBase<A, MatrixSize, Space_, Segment<A, SegmentSize, Space_> > Base;

  Segment(const A & a, MatrixSize b) : Base(a, b){}

  friend
  std::ostream & operator << (std::ostream & out, const Segment & op) {
    return out << op.getA() << "[" << op.getB() << ".." << (op.getB() + SegmentSize -1) << "]";
  }
};

namespace internal {
  template <typename A, MatrixSize SegmentSize, typename Space>
  struct get_space<Segment<A, SegmentSize, Space >>{
   public:
    typedef Space type;
  };
}

template <MatrixSize Dim_, unsigned BasisIndex>
struct EuclideanPointBasisVector;

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

  inline Vector<Dimension> & getValue() {
    return *this;
  }

  inline const Vector<Dimension> & getValue() const {
    return *this;
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

  template <MatrixSize SegmentSize>
  inline auto evalSegment(const MatrixSize & startIndex) const -> decltype(this->getValue().template block<SegmentSize, 1>(startIndex, 0)) {
    return getValue().template block<SegmentSize, 1>(startIndex, 0);
  }

  template <MatrixSize SegmentSize, typename Ret = Segment<EuclideanPoint, SegmentSize, EuclideanPoint<SegmentSize> > >
  inline typename Ret::App segment(const MatrixSize & startIndex) const {
    return Ret(*this, startIndex);
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
  bool operator != (const EuclideanPoint & other) const {
    return getValue() != other.getValue();
  }

  template<unsigned BasisIndex>
  static const EuclideanPointBasisVector<Dimension, BasisIndex> & getBasisVector(){
    return EuclideanPointBasisVector<Dimension, BasisIndex>::getInstance();
  }
};

template<MatrixSize Dim, unsigned BasisIndex>
struct EuclideanPointBasisVector : public EuclideanPoint<Dim> {
 static const EuclideanPointBasisVector& getInstance() {
   const static EuclideanPointBasisVector Instance; //TODO optimize to static global var
   return Instance;
 }
 private:
  EuclideanPointBasisVector() : EuclideanPoint<Dim>(MatrixConvertible<std::function<double(MatrixSize i)>>::asMatrixConvertible([](MatrixSize i)->double { return i == BasisIndex ? 1.0 : 0.0;}, Dim)){}
};


//template<MatrixSize Dimension>
//template<unsigned BasisIndex>
//const EuclideanPointBasisVector<Dimension, BasisIndex> & EuclideanPoint<Dimension>::template getBasisVector(){
//  return EuclideanPointBasisVector<Dimension, BasisIndex>::getInstance();
//}

template <MatrixSize Dim_, typename DERIVED>
struct OpMemberBase<EuclideanPoint<Dim_>, DERIVED> {

  template <typename Other, typename Ret = Dot<DERIVED, Other> >
  inline typename Ret::App dot (const Other & other) const {
    return Ret(static_cast<const DERIVED&>(*this), other);
  }

  template <MatrixSize SegmentSize, typename Ret = Segment<DERIVED, SegmentSize, EuclideanPoint<SegmentSize> > >
  inline typename Ret::App segment(const MatrixSize & startIndex) const {
    return Ret(static_cast<const DERIVED&>(*this), startIndex);
  }
};

template <typename A, MatrixSize SegmentSize>
inline const auto evalExp(const Segment<A, SegmentSize, EuclideanPoint<SegmentSize>> & r) -> decltype(evalExp(r.getA()).template evalSegment<SegmentSize>(r.getB())){
  return evalExp(r.getA()).template evalSegment<SegmentSize>(r.getB());
}

template <typename A, MatrixSize SegmentSize>
struct get_op<Segment<A, SegmentSize, EuclideanPoint<SegmentSize>>> {
  typedef Segment<A, SegmentSize, EuclideanPoint<SegmentSize>> type;
};


template <typename A, MatrixSize SegmentSize>
struct Cache<Segment<A, SegmentSize, EuclideanPoint<SegmentSize>>> {
  typedef Segment<A, SegmentSize, EuclideanPoint<SegmentSize>> Exp;
  typedef EuclideanPoint<SegmentSize> Space;
  Cache<typename get_op<A>::type> a;

  void update(const Exp & exp) {
    a.update(exp.getA());
  }
  Space accessValue(const Exp & exp) const {
    return a.accessValue(exp.getA()).template block<SegmentSize, 1>(0, exp.getB());
  }
};

template <typename A, MatrixSize SegmentSize, typename Cache_>
inline const EuclideanPoint<SegmentSize> evalExpCached(const Segment<A, SegmentSize, EuclideanPoint<SegmentSize>> & r, Cache_ & cache){
  return cache.a.accessValue(r.getA()).template evalSegment<SegmentSize>(r.getB());
}


template <unsigned diffIndex, unsigned basisIndex, MatrixSize Dim_, typename A, typename B, typename Differential>
inline void evalDiff(const AnyBinOp<EuclideanPoint<Dim_>, typename EuclideanPoint<Dim_>::ScalarSpace, Times<A, B, EuclideanPoint<Dim_> > > & anyExp, Differential & d)
{
  auto & exp = anyExp.getExp();
  typedef EuclideanPoint<Dim_> TangentVector;
  typedef typename EuclideanPoint<Dim_>::ScalarSpace Scalar;

  auto && point = evalExp(exp.getA());
  Scalar scalar = evalExp(exp.getB());
  {
    auto diff = createDiff([&d, scalar](const TangentVector & vector){ d.apply((vector * scalar).eval()); });
    evalDiff<diffIndex, basisIndex>(exp.getA(), diff);
  }
  {
    auto diff = createDiff([&d, &point](const Scalar & scalarTangentVector){ d.apply((point * scalarTangentVector).eval()); });
    evalDiff<diffIndex, basisIndex>(exp.getB(), diff);
  }
}

template <unsigned diffIndex, unsigned basisIndex, MatrixSize Dim_, typename A, typename B, typename Differential, typename Cache>
inline void evalDiffCached(const AnyBinOp<EuclideanPoint<Dim_>, typename EuclideanPoint<Dim_>::ScalarSpace, Times<A, B, EuclideanPoint<Dim_> > > & anyExp, Differential & d, Cache & cache)
{
  auto & exp = anyExp.getExp();
  typedef EuclideanPoint<Dim_> TangentVector;
  typedef typename EuclideanPoint<Dim_>::ScalarSpace Scalar;

  auto && point = cache.a.accessValue(exp.getA());
  Scalar scalar = exp.getB();
  {
    auto diff = createDiff([&d, scalar](const TangentVector & vector){ d.apply((vector * scalar).eval()); });
    evalDiffCached<diffIndex, basisIndex>(exp.getA(), diff, cache.a);
  }
  {
    auto diff = createDiff([&d, & point](const Scalar & scalarTangentVector){ d.apply((point * scalarTangentVector).eval()); });
    evalDiffCached<diffIndex, basisIndex>(exp.getB(), diff, cache.b);
  }
}


template <unsigned diffIndex, unsigned basisIndex, MatrixSize Dim_, typename A, MatrixSize SegmentSize, typename Differential>
inline void evalDiff(const AnyBinOp<EuclideanPoint<Dim_>, MatrixSize, Segment<A, SegmentSize, EuclideanPoint<SegmentSize> > > & anyExp, Differential & d)
{
  auto & exp = anyExp.getExp();
  typedef EuclideanPoint<Dim_> TangentVector;
  MatrixSize startIndex = exp.getB();
  auto diff = createDiff([&d, startIndex](const TangentVector & vector){ d.apply(vector.template block<SegmentSize, 1>(startIndex, 0)); });
  evalDiff<diffIndex, basisIndex>(exp.getA(), diff);
}

template <unsigned diffIndex, unsigned basisIndex, MatrixSize Dim_, typename A, MatrixSize SegmentSize, typename Differential, typename Cache>
inline void evalDiffCached(const AnyBinOp<EuclideanPoint<Dim_>, MatrixSize, Segment<A, SegmentSize, EuclideanPoint<SegmentSize> > > & anyExp, Differential & d, Cache & cache)
{
  auto & exp = anyExp.getExp();
  typedef EuclideanPoint<Dim_> TangentVector;
  MatrixSize startIndex = exp.getB();
  auto diff = createDiff([&d, startIndex](const TangentVector & vector){ d.apply(vector.template block<SegmentSize, 1>(startIndex, 0)); });
  evalDiffCached<diffIndex, basisIndex>(exp.getA(), diff, cache.a);
}

}

#define SUPPORTS_EUCLIDEAN_DOT

#define SUPPORTS_EUCLIDEAN_ROTATION


#endif /* EUCLIDEANPOINT_HPP_ */
