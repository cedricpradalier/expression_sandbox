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

// simple function using virtual expression calculations
Exp<EuclideanSpace<2>::Point> addY(const Exp<EuclideanSpace<2>::Point> x){
  EuclideanSpace<2>::Point y{0, 1};
  return x + y;
}

TEST(Requirements, UserFunction){
  EuclideanSpace<2>::Point x{1, 0}, y{0, 1}, w{sqrt(0.5), sqrt(0.5)};

  auto addYappliedToX = addY(x);

  PRINT_EXP(addYappliedToX);

  ASSERT_EQ((x+y).eval(), addYappliedToX.eval());
}

#endif

