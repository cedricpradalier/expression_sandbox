/*
 * TestEuclideanPointsAndScalars.cpp
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

extern EuclideanSpace<3>::Point x, y, z;

typedef Scalar<double> ScalarD;

Scalar<double> one = 1;
Scalar<double> two = 2;


TEST(EuclideanPointAndScalar, BasicArithmetic){
  ASSERT_EQ(two, (one + one).eval());

//  ASSERT_PRED3(isNear<3>, (-x).eval(), j.rotate(x).eval(), eps);
//  ASSERT_PRED3(isNear<3>, (-y).eval(), k.rotate(y).eval(), eps);
//  ASSERT_PRED3(isNear<3>, (-z).eval(), i.rotate(z).eval(), eps);
//  ASSERT_EQ(k, ij.eval());
//  ASSERT_NE(id, minusId);
//  ASSERT_EQ(minusId, ii.eval());
//  ASSERT_EQ(id, iiii.eval());
//  ASSERT_PRED3(isNear<3>, z, halfI.rotate(y).eval(), eps * 2);
//  ASSERT_PRED3(isNear<3>, x, ii.rotate(x).eval(), eps);
//  ASSERT_PRED3(isNear<3>, y, ii.rotate(y).eval(), eps);
}

TEST(EuclideanPointAndScalar, Derivatives){
  Diffable<Ref<ScalarD>, 0> One(one);

  Diffable<Ref<EuclideanSpace<3>::Point>, 1> X(x);


  {
    auto J = evalFullDiffCached(One, One);
    ASSERT_EQ(decltype(J)({1}), J);
  }
  {
    auto J = evalFullDiffCached(One * One, One);
    ASSERT_EQ(decltype(J)({2}), J);
  }
  {
    auto J = evalFullDiffCached(X * One, One);
    ASSERT_EQ(decltype(J)({1, 0, 0}), J);
  }

  {
    auto J = evalFullDiffCached(X - X, X);
    ASSERT_EQ(decltype(J)({
      0, 0, 0,
      0, 0, 0,
      0, 0, 0
    }), J);
  }
  {
    auto J = evalFullDiffCached(X - X * two, X);
    ASSERT_EQ(decltype(J)({
      -1, 0, 0,
      0, -1, 0,
      0, 0, -1
    }), J);
  }
  {
    auto tmp = X * two;
    auto J = evalFullDiffCached(X + tmp, X);
    ASSERT_EQ(decltype(J)({
      3, 0, 0,
      0, 3, 0,
      0, 0, 3
    }), J);
  }
  {
//    auto J = evalFullDiffCached(X && X, X);
//    ASSERT_EQ(decltype(J)({
//      2, 0, 0,
//      0, 2, 0,
//      0, 0, 2
//    }), J);
  }




  {
    Vector<3> dest;
    ToVectorDifferential<EuclideanSpace<3>::Point, Vector<3> > diff(dest);
    auto exp = X * One;
    PRINT_EXP(exp);

    diff.reset();
    auto cache = createCache(exp);
    cache.update(exp);

    //  evalDiff<0, 0>(U, diff);
    evalDiffCached<0, 0>(exp, diff, cache);
    ASSERT_EQ(dest, Vector<3>({1, 0, 0}));


    auto J = evalFullDiffCached(exp, X, cache);
    ASSERT_EQ(decltype(J)({
      1, 0, 0,
      0, 1, 0,
      0, 0, 1}), J);

    J = evalFullDiffCached(X * two, X, cache);
    ASSERT_EQ(decltype(J)({
      2, 0, 0,
      0, 2, 0,
      0, 0, 2}), J);
  }

  {
//    auto exp = U*U;
//    auto cache = createCache(exp);
//    cache.update(exp);
//
//    auto J = evalFullDiffCached(exp, U, cache);
//    ASSERT_EQ(decltype(J)({
//      2, 0, 0,
//      0, 0, 0,
//      0, 0, 0}), J); //TODO verify
  }
}

