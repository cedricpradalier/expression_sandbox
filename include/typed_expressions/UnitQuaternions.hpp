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
#include "Derivatives.hpp"


namespace quat_calc {
using namespace linalg;

enum class QuaternionMode {
  FIRST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 0,
  FIRST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 1,
  LAST_IS_REAL_AND_TRADITIONAL_MULT_ORDER = 2,
  LAST_IS_REAL_AND_OPPOSITE_MULT_ORDER = 3,
}constexpr DefaultQuaternionMode = QuaternionMode::FIRST_IS_REAL_AND_TRADITIONAL_MULT_ORDER;

enum class UnitQuaternionGeometry {
  LEFT_TRANSLATED,
  RIGHT_TRANSLATED
} const DefaultUnitQuaternionGeometry  = UnitQuaternionGeometry::RIGHT_TRANSLATED;


namespace internal {
enum {
  OPPOSITE_MULT_ORDER_MASK = 1,
  REAL_IS_LAST_MASK = 2,
};

TEX_INLINE constexpr bool isRealFirst(const QuaternionMode mode) {
  return ((int) mode & internal::REAL_IS_LAST_MASK) == 0;
}

TEX_INLINE constexpr bool isTraditionalMultOrder(const QuaternionMode mode) {
  return ((int) mode & internal::OPPOSITE_MULT_ORDER_MASK) == 0;
}
TEX_INLINE constexpr int getRealIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 0 : 3;
}
TEX_INLINE constexpr int getIIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 1 : 0;
}
TEX_INLINE constexpr int getJIndex(QuaternionMode mode) {
  return isRealFirst(mode) ? 2 : 1;
}
TEX_INLINE constexpr int getKIndex(const QuaternionMode mode) {
  return isRealFirst(mode) ? 3 : 2;
}

template<typename TScalar, enum QuaternionMode EMode = DefaultQuaternionMode>
struct QuaternionCalculator {
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



  TEX_INLINE static vector_t getIdentity() {
    return isRealFirst(EMode) ? vector_t(1, 0, 0, 0) : vector_t(0, 0, 0, 1);
  }

  TEX_INLINE static void quatMultTraditionalInto(const vector_t & a, const vector_t & b, vector_t & res) {
    // aIIndex*bRIndex + aJIndex*bKIndex - aKIndex*bJIndex + aRIndex*bIIndex
    res[IIndex] = a[IIndex] * b[RIndex] + a[JIndex] * b[KIndex] - a[KIndex] * b[JIndex] + a[RIndex] * b[IIndex];
    // aKIndex*bIIndex - aIIndex*bKIndex + aJIndex*bRIndex + aRIndex*bJIndex
    res[JIndex] = a[KIndex] * b[IIndex] - a[IIndex] * b[KIndex] + a[JIndex] * b[RIndex] + a[RIndex] * b[JIndex];
    // aIIndex*bJIndex - aJIndex*bIIndex + aKIndex*bRIndex + aRIndex*bKIndex
    res[KIndex] = a[IIndex] * b[JIndex] - a[JIndex] * b[IIndex] + a[KIndex] * b[RIndex] + a[RIndex] * b[KIndex];
    // aRIndex*bRIndex - aJIndex*bJIndex - aKIndex*bKIndex - aIIndex*bIIndex
    res[RIndex] = a[RIndex] * b[RIndex] - a[JIndex] * b[JIndex] - a[KIndex] * b[KIndex] - a[IIndex] * b[IIndex];
  }
  TEX_INLINE static void quatMultTraditionalInto(const vector_t & a, const vector_t & b, pure_imag_vector_t & res) {
    // aIIndex*bRIndex + aJIndex*bKIndex - aKIndex*bJIndex + aRIndex*bIIndex
    res[IPureIndex] = a[IIndex] * b[RIndex] + a[JIndex] * b[KIndex] - a[KIndex] * b[JIndex] + a[RIndex] * b[IIndex];
    // aKIndex*bIIndex - aIIndex*bKIndex + aJIndex*bRIndex + aRIndex*bJIndex
    res[JPureIndex] = a[KIndex] * b[IIndex] - a[IIndex] * b[KIndex] + a[JIndex] * b[RIndex] + a[RIndex] * b[JIndex];
    // aIIndex*bJIndex - aJIndex*bIIndex + aKIndex*bRIndex + aRIndex*bKIndex
    res[KPureIndex] = a[IIndex] * b[JIndex] - a[JIndex] * b[IIndex] + a[KIndex] * b[RIndex] + a[RIndex] * b[KIndex];
  }

  TEX_INLINE static void quatMultTraditionalInto(const pure_imag_vector_t & a, const vector_t & b, vector_t & res) {
    res[IIndex] = a[IPureIndex] * b[RIndex] + a[JPureIndex] * b[KIndex] - a[KPureIndex] * b[JIndex];
    res[JIndex] = a[KPureIndex] * b[IIndex] - a[IPureIndex] * b[KIndex] + a[JPureIndex] * b[RIndex];
    res[KIndex] = a[IPureIndex] * b[JIndex] - a[JPureIndex] * b[IIndex] + a[KPureIndex] * b[RIndex];
    res[RIndex] = -a[JPureIndex] * b[JIndex] - a[KPureIndex] * b[KIndex] - a[IPureIndex] * b[IIndex];
  }

  TEX_INLINE static void quatMultTraditionalInto(const pure_imag_vector_t & a, const pure_imag_vector_t & b, vector_t & res) {
    res[IIndex] = + a[JPureIndex] * b[KPureIndex] - a[KPureIndex] * b[JPureIndex];
    res[JIndex] = a[KPureIndex] * b[IPureIndex] - a[IPureIndex] * b[KPureIndex];
    res[KIndex] = a[IPureIndex] * b[JPureIndex] - a[JPureIndex] * b[IPureIndex];
    res[RIndex] = -a[JPureIndex] * b[JPureIndex] - a[KPureIndex] * b[KPureIndex] - a[IPureIndex] * b[IPureIndex];
  }


  TEX_INLINE static void quatMultTraditionalInto(const vector_t & a, const pure_imag_vector_t & b, vector_t & res) {
    res[IIndex] = +a[JIndex] * b[KPureIndex] - a[KIndex] * b[JPureIndex] + a[RIndex] * b[IPureIndex];
    res[JIndex] = a[KIndex] * b[IPureIndex] - a[IIndex] * b[KPureIndex] + +a[RIndex] * b[JPureIndex];
    res[KIndex] = a[IIndex] * b[JPureIndex] - a[JIndex] * b[IPureIndex] + +a[RIndex] * b[KPureIndex];
    res[RIndex] = -a[JIndex] * b[JPureIndex] - a[KIndex] * b[KPureIndex] - a[IIndex] * b[IPureIndex];
  }

  TEX_INLINE static void quatMultTraditionalInto(const vector_t & a, const pure_imag_vector_t & b, pure_imag_vector_t & res) {
    res[IPureIndex] = +a[JIndex] * b[KPureIndex] - a[KIndex] * b[JPureIndex] + a[RIndex] * b[IPureIndex];
    res[JPureIndex] = a[KIndex] * b[IPureIndex] - a[IIndex] * b[KPureIndex] + +a[RIndex] * b[JPureIndex];
    res[KPureIndex] = a[IIndex] * b[JPureIndex] - a[JIndex] * b[IPureIndex] + +a[RIndex] * b[KPureIndex];
  }

  TEX_INLINE static void quatMultTraditionalInto(const pure_imag_vector_t & a, const vector_t & b, pure_imag_vector_t & res) {
    res[IPureIndex] = a[IPureIndex] * b[RIndex] + a[JPureIndex] * b[KIndex] - a[KPureIndex] * b[JIndex];
    res[JPureIndex] = a[KPureIndex] * b[IIndex] - a[IPureIndex] * b[KIndex] + a[JPureIndex] * b[RIndex];
    res[KPureIndex] = a[IPureIndex] * b[JIndex] - a[JPureIndex] * b[IIndex] + a[KPureIndex] * b[RIndex];
  }

  template<MatrixSize ISizeA, MatrixSize ISizeB, MatrixSize ISizeC>
  TEX_INLINE static void quatMultInto(const Vector<ISizeA> & a, const Vector<ISizeB> & b, Vector<ISizeC> & result) {
    return isTraditionalMultOrder(EMode) ? quatMultTraditionalInto(a, b, result) : quatMultTraditionalInto(b, a, result);
  }

  template<MatrixSize ISizeA, MatrixSize ISizeB>
  TEX_INLINE static vector_t quatMult(const Vector<ISizeA> & a, const Vector<ISizeB> & b) {
    vector_t result;
    quatMultInto(a, b, result);
    return result;
  }


  TEX_INLINE static vector_t conjugate(const vector_t & v) {
    vector_t r(v);
    r.template block<3, 1>(IIndex, 0) *= -1;
    return r;
  }
  TEX_INLINE static vector_t invert(const vector_t & v) {
    vector_t r(conjugate(v));
    r /= r.dot(r);
    return r;
  }

  TEX_INLINE static auto getImagPart(const vector_t & v) -> decltype(v.template block<3, 1>(IIndex, 0)) {
    return v.template block<3, 1>(IIndex, 0);
  }

  template<enum QuaternionMode EOtherMode, int ICols = 1, typename DERIVED_MATRIX>
  TEX_INLINE static linalg::Matrix<double, 4, ICols> convertFromOtherMode(const linalg::MatrixBase<DERIVED_MATRIX > & v){
    return QuaternionCalculator<TScalar, EOtherMode>::template convertToOtherMode<EMode, ICols>(v);
  }

  template <int index>
  static TEX_INLINE vector_t clacBasisVector(){
    vector_t ret;
    ret.setZero();
    ret[isRealFirst(EMode) ? index : (index + 3) % 4] = typename vector_t::Scalar(1);
    return ret;
  }
};
}
}



namespace TEX_NAMESPACE {

using namespace linalg;
class UnitQuaternion : public EuclideanPoint<4> {
 private:
 public:
  constexpr static size_t Dimension = 3;

  typedef quat_calc::internal::QuaternionCalculator<double> Calc;
  typedef EuclideanPoint<4> Base;

  struct ZeroTangentVector;
  template<unsigned BasisIndex>
  struct BasisTangentVector;

  struct TangentVector : public EuclideanPoint<3> {
    typedef ZeroTangentVector ZeroVector;
    template<unsigned BasisIndex>
    using BasisVector = BasisTangentVector<BasisIndex>;

    TangentVector(const EuclideanPoint<3>& p) : EuclideanPoint<3>(p){}
  };

  template<unsigned BasisIndex>
  struct BasisTangentVector : public TangentVector{
   static const BasisTangentVector<BasisIndex>& getInstance() {
     const static BasisTangentVector<BasisIndex> Instance; //TODO optimize to static global var
     return Instance;
   }
   private:
   BasisTangentVector() : TangentVector(EuclideanPoint<3>(asMatrixConvertible([](MatrixSize i, MatrixSize j){ return i == BasisIndex ? 1.0 : 0.0;}))){}
  };

  template<unsigned BasisIndex>
  static const BasisTangentVector<BasisIndex> & getBasisVector(){
    return BasisTangentVector<BasisIndex>::getInstace();
  }

  struct ZeroTangentVector : public TangentVector{
    static const ZeroTangentVector& getInstance() {
      const static ZeroTangentVector Instance;
      return Instance;
    }
   private:
   ZeroTangentVector() : TangentVector(EuclideanPoint<3>({ 0.0, 0.0, 0.0 })){}
  };

  UnitQuaternion() = default;
  UnitQuaternion(const UnitQuaternion & q) = default;
  UnitQuaternion(const Vector<4> & q) : Base(q){}

  UnitQuaternion evalNegate() const {
     return UnitQuaternion(-getValue());
  }

  void evalInverseInto(UnitQuaternion & result) const {
     result = Calc::conjugate(getValue());
  }

  UnitQuaternion evalInverse() const {
    UnitQuaternion result;
    evalInverseInto(result);
    return result;
  }

  UnitQuaternion evalInverseDiff(const Vector<3> & thisTangent) const {
     Vector<4> con = Calc::conjugate(getValue());
     return UnitQuaternion(Calc::quatMult(Calc::quatMult(con, Vector<3>(-thisTangent)), con));
  }

  template <typename Ret = Inverse<UnitQuaternion> > const
  TEX_INLINE typename Ret::App inverse() const {
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

  TEX_INLINE EuclideanPoint<3> evalRotate(const EuclideanPoint<3> & other) const{
    EuclideanPoint<3> ret;
    evalRotateInto(other, ret);
    return ret;
  }
  
  TEX_INLINE void evalRotateInto(const EuclideanPoint<3> & other, EuclideanPoint<3> & result) const{
    //Calc::quatMultInto(Calc::quatMult(getValue(), other.getValue()), Calc::conjugate(getValue()), retsult.getValue());

    /* Identify v with the pure imaginary quaternion given by other.
     * We simplify then the pure imaginary quaternion. Let ~= denote "equal imaginary part"
     *   q v conj(q)
     * = (q_R + q_I) v (q_R - q_I)
     * = q_R^2 v - q_R v q_I + q_I v q_R - q_I v q_I
     * (importing R^3 cross and dot product to pure imaginary quaternions : a * b = -<a, b> + a x b ~= a x b)
     *~= q_R^2 v - 2 q_R (v x q_I) - q_I (-<v, q_I> + v x q_I)
     *~= q_R^2 v - 2 q_R (v x q_I) + <v, q_I>q_I - q_I x (v x q_I)
     * = q_R^2 v - 2 q_R (v x q_I) + <v, q_I>q_I - (<q_I, q_I>v - <q_I, v>q_I)
     * = (q_R^2 - <q_I, q_I>)v - 2<v, q_I>q_I + 2 q_R (v x q_I)
     */
    auto & q = getValue();
    auto & q_R = q[Calc::RIndex];
    auto q_I = Calc::getImagPart(q);
    auto & v = other.getValue();
    result = (q_R * q_R - q_I.dot(q_I)) * v + (2 * v.dot(q_I)) * q_I + (-2 * q_R) * (v.cross(q_I));

//    typedef double T;
//    auto & q = getValue();
//    const T t2 =  q[Calc::RIndex] * q[Calc::IIndex];
//    const T t3 =  q[Calc::RIndex] * q[Calc::JIndex];
//    const T t4 =  q[Calc::RIndex] * q[Calc::KIndex];
//    const T t5 = -q[Calc::IIndex] * q[Calc::IIndex];
//    const T t6 =  q[Calc::IIndex] * q[Calc::JIndex];
//    const T t7 =  q[Calc::IIndex] * q[Calc::KIndex];
//    const T t8 = -q[Calc::JIndex] * q[Calc::JIndex];
//    const T t9 =  q[Calc::JIndex] * q[Calc::KIndex];
//    const T t1 = -q[Calc::KIndex] * q[Calc::KIndex];
//    result[0] = T(2) * ((t8 + t1) * other[0] + (t6 - t4) * other[1] + (t3 + t7) * other[2]) + other[0];  // NOLINT
//    result[1] = T(2) * ((t4 + t6) * other[0] + (t5 + t1) * other[1] + (t9 - t2) * other[2]) + other[1];  // NOLINT
//    result[2] = T(2) * ((t7 - t3) * other[0] + (t2 + t9) * other[1] + (t5 + t8) * other[2]) + other[2];  // NOLINT
  }

  TEX_INLINE EuclideanPoint<3> evalInverseRotate(const EuclideanPoint<3> & other) const{
    EuclideanPoint<3> ret;
    evalInverseRotateInto(other, ret);
    return ret;
  }
  TEX_INLINE void evalInverseRotateInto(const EuclideanPoint<3> & other, EuclideanPoint<3> & result) const{
    /* see evalRotateInto */
    auto & q = getValue();
    auto & q_R = q[Calc::RIndex];
    auto q_I = Calc::getImagPart(q);
    auto & v = other.getValue();
    result = (q_R * q_R - q_I.dot(q_I)) * v + (2 * v.dot(q_I)) * q_I + (2 * q_R) * (v.cross(q_I));
  }


  template <typename Ret = Rotate<UnitQuaternion, EuclideanPoint<3> > >
  TEX_INLINE typename Ret::App rotate(const EuclideanPoint<3> & other) const{
    return Ret(*this, other);
  }

  template <typename Ret=EuclideanPoint<3> >
  TEX_INLINE Ret imag() const {
     return Ret(Calc::getImagPart(getValue()));
  }

  UnitQuaternion evalTimesJac(const UnitQuaternion & other) const;

  bool operator == (const UnitQuaternion & other) const {
    return getValue() == other.getValue();
  }
  bool operator != (const UnitQuaternion & other) const {
    return getValue() != other.getValue();
  }

  friend
  std::ostream & operator << (std::ostream & out, const UnitQuaternion & op) {
    return out << "UnitQuat" << "(" << static_cast<const Base&>(op) << ")";
  }

  static const UnitQuaternion & getIdentity(){
    const static UnitQuaternion Identity(UnitQuaternion::Calc::clacBasisVector<0>());
    return Identity;
  }
  static const UnitQuaternion & getI(){
    const static UnitQuaternion I(UnitQuaternion::Calc::clacBasisVector<1>());
    return I;
  }
  static const UnitQuaternion & getJ(){
    const static UnitQuaternion J(UnitQuaternion::Calc::clacBasisVector<2>());
    return J;
  }
  static const UnitQuaternion & getK(){
    const static UnitQuaternion K(UnitQuaternion::Calc::clacBasisVector<3>());
    return K;
  }
};



template <typename A, typename B>
TEX_STRONG_INLINE const EuclideanPoint<3> evalExp(const Rotate<Inverse<A>, B, EuclideanPoint<3> > & r){
  return evalExp(r.getA().getA()).evalInverseRotate(evalExp(r.getB()));
}

template <typename DERIVED>
struct OpMemberBase<UnitQuaternion, DERIVED> {
  template <typename Ret = Inverse<DERIVED> >
  TEX_STRONG_INLINE typename Ret::App inverse () const {
    return Ret(static_cast<const DERIVED&>(*this));
  }
  template <typename Other, typename Ret = Rotate<DERIVED, Other> >
  TEX_STRONG_INLINE typename Ret::App rotate(const Other & other) const {
    return Ret(static_cast<const DERIVED&>(*this), other);
  }
};

template <>
struct get_tangent_space<UnitQuaternion> { //TODO should default to nested type
  typedef UnitQuaternion::TangentVector type;
};

template <unsigned diffIndex, unsigned basisIndex, typename A, typename Differential, typename Cache>
TEX_STRONG_INLINE void evalDiffCached(const AnyUnOp<UnitQuaternion, Inverse<A>> & anyExp, Differential & d, Cache & cache)
{
  auto & exp = anyExp.getExp();
  auto & a = cache.accessValue(exp);
  auto diff = createDiff([&d, &a](const UnitQuaternion::TangentVector & vector){ d.apply((a.rotate(-vector.getValue())).eval()); });
  evalDiffCached<diffIndex, basisIndex>(exp.getA(), diff, cache.a);
}

template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential>
TEX_STRONG_INLINE void evalDiff(const AnyBinOp<UnitQuaternion, UnitQuaternion, Times<A, B>> & anyTimes, Differential & d)
{
  auto & times = anyTimes.getExp();
  auto b = evalExp(times.getB());
  evalDiff<diffIndex, basisIndex>(times.getA(), d);
  auto diff = createDiff([&d, &b](const UnitQuaternion::TangentVector & vector){ d.apply(b.rotate(vector).eval()); });
  evalDiff<diffIndex, basisIndex>(times.getB(), diff);
}

template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential, typename Cache>
TEX_STRONG_INLINE void evalDiffCached(const AnyBinOp<UnitQuaternion, UnitQuaternion, Times<A, B>> & anyTimes, Differential & d, Cache & cache)
{
  auto & times = anyTimes.getExp();
  auto & b = cache.b.accessValue(times.getB());
  evalDiff<diffIndex, basisIndex>(times.getA(), d);
  auto diff = createDiff([&d, &b](const UnitQuaternion::TangentVector & vector){ d.apply(b.rotate(vector).eval()); });
  evalDiffCached<diffIndex, basisIndex>(times.getB(), diff, cache.b);
}


/*
 *
//          auto pRotated = (Ref<UnitQuaternion>(qC12).rotate(Ref<EuclideanPoint<3>>(p2)) * Scalar<double>(0.5)).eval().getValue();
//          for (int i : {0, 1, 2}) {
//            auto & v = E[i];
//            output.jP2.template block<2, 1>(0, i) = toEigen(getExp(qC12, v).eval());
//            output.jPhi12.template block<2, 1>(0, i) = toEigen(v.cross(pRotated).template block<2, 1>(0, 0) * 2);
//          }
//        }
 */


//TODO optimize: create rotate diff struct optimized for basis vectors
//template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential>
//struct RotateDiff {
//  template<unsigned BasisIndex>
//  void apply(BasisVector.) {
//
//  }
//};

template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential>
TEX_STRONG_INLINE void evalDiff(const AnyBinOp<UnitQuaternion, EuclideanPoint<3>, Rotate<A, B, EuclideanPoint<3>>> & anyExp, Differential & d)
{
  auto & exp = anyExp.getExp();
  const UnitQuaternion a = evalExp(exp.getA());
  const EuclideanPoint<3> rotated = evalExp(exp);
  {
    auto diff = createDiff([&d, &rotated](const UnitQuaternion::TangentVector & vector){ d.apply(vector.cross(rotated) * 2); });
    evalDiff<diffIndex, basisIndex>(exp.getA(), diff);
  }
  {
    auto diff = createDiff([&d, &a](const EuclideanPoint<3> & vector){ d.apply(a.rotate(vector).eval()); });
    evalDiff<diffIndex, basisIndex>(exp.getB(), diff);
  }
}

template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential, typename Cache>
TEX_STRONG_INLINE void evalDiffCached(const AnyBinOp<UnitQuaternion, EuclideanPoint<3>, Rotate<A, B, EuclideanPoint<3>>> & anyExp, Differential & d, Cache &cache)
{
  auto & exp = anyExp.getExp();
  if(doesDependOn<diffIndex>(exp.getA())){
    const EuclideanPoint<3> & rotated = cache.accessValue(exp);
    auto diff = createDiff([&d, &rotated](const UnitQuaternion::TangentVector & vector){ d.apply(vector.cross(rotated) * 2); });
    evalDiffCached<diffIndex, basisIndex>(exp.getA(), diff, cache.a);
  }
  if(doesDependOn<diffIndex>(exp.getB())){
    const auto & a = cache.a.accessValue(exp.getA());
//    const auto & aM = cache.a.getMatrix(a);

    auto diff = createDiff([&d, &a](const EuclideanPoint<3> & vector){ d.apply((a.rotate(vector)).eval()); });
    evalDiffCached<diffIndex, basisIndex>(exp.getB(), diff, cache.b);
  }
}

template <unsigned diffIndex, unsigned basisIndex, typename A, typename B, typename Differential, typename Cache>
TEX_STRONG_INLINE void evalDiffCached(const AnyBinOp<UnitQuaternion, EuclideanPoint<3>, Rotate<Inverse<A>, B, EuclideanPoint<3>>> & anyExp, Differential & d, Cache &cache)
{
  auto & exp = anyExp.getExp();
  const UnitQuaternion & a = cache.a.accessValue(exp.getA());
  {
    auto & b = cache.b.accessValue(exp.getB());
    auto diff = createDiff([&d, &a, &b](const UnitQuaternion::TangentVector & vector){ d.apply((a.rotate(b.cross(vector)) * 2.0).eval()); });
    evalDiffCached<diffIndex, basisIndex>(exp.getA().getA(), diff, cache.a.a);
  }
  {
    auto diff = createDiff([&d, &a](const EuclideanPoint<3> & vector){ d.apply(a.rotate(vector).eval()); });
    evalDiffCached<diffIndex, basisIndex>(exp.getB(), diff, cache.b);
  }
}

//template <>
//struct Transformations<UnitQuaternion>{
//  const Matrix<double, 3, 3> & getMatrix(const UnitQuaternion & q) {
//    if(!isTransformed){
//      const double &a = q[0], &b = q[1], &c = q[2], &d = q[3];
//      double a2 = a * a;
//      double b2 = b * b;
//      double c2 = c * c;
//      double d2 = d * d;
//
//      double a2mb2 = a2 - b2;
//      double c2md2 = c2 - d2;
//
//      double ab = 2 * a * b;
//      double ac = 2 * a * c;
//      double ad = 2 * a * d;
//      double bc = 2 * b * c;
//      double bd = 2 * b * d;
//      double cd = 2 * c * d;
//
//      R(0, 0) = a2 + b2 - c2 - d2; R(0, 1) = bc - ad; R(0, 2) = bd + ac;
//      R(1, 0) = bc + ad; R(1, 1) = a2mb2 + c2md2; R(1, 2) = cd - ab;
//      R(2, 0) = bd - ac; R(2, 1) = cd + ab; R(2, 2) = a2mb2 - c2md2;
//      isTransformed = true;
//    }
//    return R;
//  }
// private:
//  Matrix<double, 3, 3> R;
//  bool isTransformed;
//};


//
//template <typename DiffableExp, unsigned DiffIndex, typename Matrix, typename Cache, typename A, typename B, unsigned ADiffIndex>
//TEX_STRONG_INLINE void evalFullDiffIntoCached(const AnyBinOp<UnitQuaternion, EuclideanPoint<3>, Rotate<Diffable<A, ADiffIndex>, B, EuclideanPoint<3>>> & anyExp, const Diffable<DiffableExp, DiffIndex> & diffable, Cache & cache, Matrix & result) {
//  typedef Rotate<Diffable<A, ADiffIndex>, B, EuclideanPoint<3>> Exp;
//  const auto & exp = anyExp.getExp();
//
//  if(DiffIndex == ADiffIndex){
//    Vector<3> rot = cache.accessValue(exp).getValue() * 2;
//    result(0, 1) += rot[2]; result(0, 2) -= rot[1];
//    result(1, 0) -= rot[2]; result(1, 2) += rot[0];
//    result(2, 0) += rot[1]; result(2, 1) -= rot[0];
//  }
//  else{ //TODO this needs checking whether ADiffIndex isn't part of exp.getB();
//    constexpr size_t dimDiffable = get_dim<DiffableExp>::value;
//    auto & q = cache.a.accessValue(exp.getA());
//    const linalg::Matrix<double, 3, 3> & R = cache.a.getMatrix(q);
//
//    linalg::Matrix<double, 3, dimDiffable> M;
//    M.setZero();
//    evalFullDiffIntoCached(exp.getB(), diffable, cache.b, M);
////    std::cout << "M=" << std::endl << M << std::endl; // XXX: debug output of M
//    result = R * M;
//  }
//}


} // namespace TEX_NAMESPACE

#endif /* UNITQUATERNIONS_HPP_ */
