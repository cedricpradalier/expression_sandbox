/*
 * requirements.cpp
 *
 *  Created on: Sep 25, 2013
 *      Author: hannes
 */

#include <type_traits>
#include <gtest/gtest.h>
#include <typed_expressions/EuclideanSpace.hpp>

constexpr static double eps = std::numeric_limits<double>::epsilon();

#define PRINT_EXP(EXP) std::cout <<  #EXP << " = " << EXP << " = " << EXP.eval() << std::endl;

using namespace tex;

template <int Dim_>
bool isNear(typename EuclideanSpace<Dim_>::Point a, typename EuclideanSpace<Dim_>::Point b, double threshold){
  for(int i = 0; i < Dim_; i++){
    if(!(fabs(a[i] - b[i]) < threshold)) return false;
  }
  return true;
}

void SomeCompileTimeTests(){
  ::testing::StaticAssertTypeEq<EuclideanSpace<2>::Point, typename get_space<tex::Plus<EuclideanSpace<2>::Point, EuclideanSpace<2>::Point, EuclideanSpace<2>::Point > >::type >();
  ::testing::StaticAssertTypeEq<EuclideanSpace<2>::Point, typename get_space<tex::Rotate<EuclideanSpace<2>::Rotation, EuclideanSpace<2>::Point, EuclideanSpace<2>::Point > >::type >();
}

TEST(Requirements, Scalar){
  Scalar<double> x{1.0}, y{2.0};
  auto xPlusY = x+y;
  std::cout << "x+y=" << xPlusY << "=" << xPlusY.eval() << std::endl;

  ASSERT_EQ(Scalar<double>(3.0), xPlusY.eval());
}

#ifdef SUPPORTS_VARIABLES
TEST(Requirements, ScalarVariables){
  Variable<Scalar<double> > x{1.0}, y{2.0};
  auto xPlusY = x+y;
  PRINT_EXP(xPlusY);

  ASSERT_EQ(Scalar<double>(3.0), xPlusY.eval());

  x.setVal(x.getVal() + 1);
  y.setVal(y.getVal() + 1);
  PRINT_EXP(xPlusY);
  ASSERT_EQ(Scalar<double>(5.0), xPlusY.eval());

#ifdef SUPPORTS_DERIVATIVES
  ASSERT_EQ(Scalar(1), xPlusY.evalDerivative()); //TODO define a interface for deriving derivatives
#endif
}
#endif

TEST(Requirements, EuclideanPointSum){
  EuclideanSpace<2>::Point x{1, 0}, y{0, 1};

  ASSERT_EQ(EuclideanSpace<2>::Point({1, 1}), (x+y).eval());
#ifdef SUPPORTS_DERIVATIVES
  ASSERT_EQ(EuclideanSpace<2>::Point({1, 1}), (x+y).evalDerivative()); //TODO define a interface for deriving derivatives
#endif
}

#ifdef SUPPORTS_EUCLIDEAN_DOT
TEST(Requirements, EuclideanDotProduct){
  EuclideanSpace<2>::Point x{1, 0}, y{0, 1};

  ASSERT_EQ(0.0, x.dot(y).eval());
  ASSERT_EQ(1.0, x.dot(x).eval());
  ASSERT_EQ(1.0, y.dot(y).eval());
  ASSERT_EQ(1.0, x.dot(y + x).eval());
  ASSERT_EQ(1.0, (x + y).dot(y).eval());
}
#endif

#ifdef SUPPORTS_ANGLE_TYPES
TEST(Requirements, Angles){

  RotationFraction t = Degree(90);
  ASSERT_EQ(0.25, t);
  ASSERT_EQ(Degree(90), RotationFraction(0.25));
  ASSERT_NEAR(Radian(M_PI / 2), Radian(RotationFraction(0.25)), eps);
}
#endif

#ifdef SUPPORTS_EUCLIDEAN_ROTATION
TEST(Requirements, EuclideanRotation){
  EuclideanSpace<2>::Point x{1, 0}, y{0, 1}, w{sqrt(0.5), sqrt(0.5)};

  EuclideanSpace<2>::Rotation rQuater{RotationFraction(1.0/4)};
  EuclideanSpace<2>::Rotation rEighth{RotationFraction(1.0/8)};

  auto rEighthSquared = rEighth * rEighth;
  auto rQuaterToThePowerOfFour = rQuater * rQuater * rQuater * rQuater;

  PRINT_EXP(rEighthSquared);
  PRINT_EXP(rQuaterToThePowerOfFour);

  ASSERT_PRED3(isNear<2>, y.eval(), rQuater.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<2>, w.eval(), rEighth.rotate(x).eval(), eps);
  ASSERT_EQ(rQuater, rEighthSquared.eval());
  ASSERT_EQ(rQuaterToThePowerOfFour.eval(), EuclideanSpace<2>::Rotation(RotationFraction(1.0)));
  ASSERT_PRED3(isNear<2>, y.eval(), rEighthSquared.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<2>, x.eval(), rQuaterToThePowerOfFour.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<2>, y.eval(), rQuaterToThePowerOfFour.rotate(y).eval(), eps);
}
#endif

#ifdef SUPPORTS_USER_FUNCTION

// simple compound function using virtual expression calculations
VExp<EuclideanSpace<2>::Point> addY(const VExp<EuclideanSpace<2>::Point> x){
  EuclideanSpace<2>::Point y{0, 1};
  return x + y;
}

// simple compound function fully templated on the input expressions
template <typename Input>
auto addYT(const Input x) -> decltype(x + EuclideanSpace<2>::Point()){ // with c++14 this duplication will most likely no longer be necessary
  EuclideanSpace<2>::Point y{0, 1};
  return x + y;
}

// simple compound function using virtual expression return type but templated on the input space - enforcing space type compatibility rule
template <typename SpaceType, typename A, typename B>
VExp<SpaceType> add(const AnyExp<SpaceType, A> & x, const AnyExp<SpaceType, B> & y) {
  return x.getExp() + y.getExp(); // these getExp could be made unnecessary, but it would hurt a bit ...
}

//// simple elemental function overloaded for different input spaces
EuclideanSpace<1>::Point addElementalImpl(EuclideanSpace<1>::Point x, EuclideanSpace<1>::Point y) {
  EuclideanSpace<1>::Point ret;
  ret[0] = x[0] + y[0];
  return ret;
}

EuclideanSpace<2>::Point addElementalImpl(EuclideanSpace<2>::Point x, EuclideanSpace<2>::Point y) {
  EuclideanSpace<2>::Point ret;
  ret[0] = x[0] + y[0];
  ret[1] = x[1] + y[1];
  return ret;
}

// introducing the new name - sufficient for all implementations
template <typename A, typename B, typename Space_ = RESULT_SPACE_GLOBAL(addElementalImpl, A, B)>
class AddElemental : public BinOpBase<A, B, Space_, AddElemental<A, B, Space_> > {
 public:
  typedef BinOpBase<A, B, Space_, AddElemental<A, B, Space_> > Base;

  AddElemental(const A & a, const B & b) : Base(a, b){}

  Space_ eval() const {
    return addElementalImpl(this->getA().eval(), this->getB().eval());
  }

  friend
  std::ostream & operator << (std::ostream & out, const AddElemental & op) {
    return out << "AddElemental(" << op.getA() << ", " << op.getB() << ")";
  }
};

template <typename A, typename B, typename Result = AddElemental<A, B> >
Result addElemental(const A & a, const B & b) {
  return Result(a, b);
}

namespace tex{
namespace internal {
  template <typename A, typename B, typename Space>
  struct get_space<AddElemental<A, B, Space >>{
   public:
    typedef Space type;
  };
}
}

TEST(Requirements, UserFunction){
  EuclideanSpace<2>::Point x{1, 0}, y{0, 1};

  auto addYappliedToX = addY(x);
  PRINT_EXP(addYappliedToX);
  ASSERT_EQ((x+y).eval(), addYappliedToX.eval());

  auto addYTAppliedToX = addYT(x);
  PRINT_EXP(addYTAppliedToX);
  ASSERT_EQ((x+y).eval(), addYTAppliedToX.eval());

  auto addAppliedToXAndXPlusY = add(toExp(x), x + y);
  PRINT_EXP(addAppliedToXAndXPlusY);
  ASSERT_EQ((x+(x + y)).eval(), addAppliedToXAndXPlusY.eval());

  // The following does correctly not compile, yielding instead : ".... note: deduced conflicting types for parameter ‘SpaceType’ (‘tex::EuclideanPoint<1ul>’ and ‘tex::EuclideanPoint<2ul>’)"
  //auto addTappliedToXAndXPlusY = add(toExp(EuclideanSpace<2>::Point ), x + y);

  auto addElementalAppliedToXAndXPlusY = addElemental(x, x + y);
  PRINT_EXP(addElementalAppliedToXAndXPlusY);
  ASSERT_EQ((x+(x + y)).eval(), addElementalAppliedToXAndXPlusY.eval());

  EuclideanSpace<1>::Point x1{1}, y1{1};

  auto addElementalAppliedToX1AndX1PlusY1 = addElemental(x1, x1 + y1);
  PRINT_EXP(addElementalAppliedToX1AndX1PlusY1);
  ASSERT_EQ((x1+(x1 + y1)).eval(), addElementalAppliedToX1AndX1PlusY1.eval());

}

#endif

