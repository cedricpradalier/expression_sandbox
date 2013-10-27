/*
 * TestQuaternion3D.cpp
 *
 *  Created on: Oct 10, 2013
 *      Author: hannes
 */


#include <type_traits>
#include <gtest/gtest.h>
//#include <typed_expressions/EigenLinalg.hpp>
#include "TestTools.hpp"
#include <typed_expressions/Derivatives.hpp>

using namespace tex;

EuclideanSpace<3>::Point x{1, 0}, y{0, 1}, z{0, 0, 1};
auto & i = UnitQuaternion::getI();
auto & j = UnitQuaternion::getJ();
auto & k = UnitQuaternion::getK();
auto & id = UnitQuaternion::getIdentity();


auto ij = i * j;
auto ii = i * i;
auto iiii = ii * ii;

auto minusId = (-UnitQuaternion::getIdentity()).eval();
UnitQuaternion halfI = ((UnitQuaternion::getIdentity().getValue() + UnitQuaternion::getI().getValue()) * sqrt(0.5)).eval();


TEST(Quaternion, Rotating3dVectors){
  ASSERT_PRED3(isNear<3>, (-x).eval(), j.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<3>, (-y).eval(), k.rotate(y).eval(), eps);
  ASSERT_PRED3(isNear<3>, (-z).eval(), i.rotate(z).eval(), eps);
  ASSERT_EQ(k, ij.eval());
  ASSERT_NE(id, minusId); // because we are comparing Quaternions!
  ASSERT_EQ(minusId, ii.eval());
  ASSERT_EQ(id, iiii.eval());
  ASSERT_PRED3(isNear<3>, z, halfI.rotate(y).eval(), eps * 2);
  ASSERT_PRED3(isNear<3>, x, ii.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<3>, y, ii.rotate(y).eval(), eps);
}

TEST(Quaternion, Derivatives){
  Diffable<UnitQuaternion, 0> U(UnitQuaternion::getI());
  PRINT_EXP(U);

  Vector<3> dest;
  ToVectorDifferential<UnitQuaternion, Vector<3> > diff(dest);


  Matrix<double, 2,3> m;
  m.setZero();
  m.block<2,1>(0,1)(1, 0) = 1;
  ASSERT_EQ(decltype(m)({0.0, 0, 0, 0, 1, 0}), m);




  {
    diff.reset();
    auto exp = U*U;
    auto cache = createCache(exp);
    cache.update(exp);

    std::cout << "cache.accessValue()=" << std::endl << cache.accessValue(exp) << std::endl; // XXX: debug output of U.accessValue()

  }

  {

    diff.reset();
    auto cache = createCache(U);
    cache.update(U);

    //  evalDiff<0, 0>(U, diff);
    evalDiffCached<0, 0>(U, diff, cache);
    ASSERT_EQ(dest, Vector<3>({1, 0, 0}));

    diff.reset();
    evalDiffCached<1, 0>(U, diff, cache);
    ASSERT_EQ(dest, Vector<3>({0, 0, 0}));

    diff.reset();
    evalDiffCached<0, 1>(U, diff, cache);
    ASSERT_EQ(dest, Vector<3>({0, 1, 0}));

    diff.reset();
    evalDiffCached<0, 2>(U, diff, cache);
    ASSERT_EQ(dest, Vector<3>({0, 0, 1}));



    auto J = evalFullDiffCached(U, U, cache);
    ASSERT_EQ(decltype(J)({
      1, 0, 0,
      0, 1, 0,
      0, 0, 1}), J);
  }

  {
    auto exp = U*U;
    auto cache = createCache(exp);
    cache.update(exp);

    auto J = evalFullDiffCached(exp, U, cache);
    ASSERT_EQ(decltype(J)({
      2, 0, 0,
      0, 0, 0,
      0, 0, 0}), J); //TODO verify
  }
}

