/*
 * TestRotation3D.cpp
 *
 *  Created on: Oct 10, 2013
 *      Author: hannes
 */


#include <type_traits>
#include <gtest/gtest.h>
#include "TestTools.hpp"

#include <typed_expressions/EuclideanSpace.hpp>

using namespace tex;

TEST(Rotation, Basics3d){
  EuclideanSpace<3>::Point x{1, 0}, y{0, 1}, z{0, 0, 1};
  PRINT_EXP(x);
  PRINT_EXP(y);
  PRINT_EXP(z);

  EuclideanSpace<3>::Rotation i({1.0});
  EuclideanSpace<3>::Rotation j{0.0, 1.0};
  EuclideanSpace<3>::Rotation k{0.0, 0.0, 1.0};
  EuclideanSpace<3>::Rotation minusId({0., 0., 0., -1});
  EuclideanSpace<3>::Rotation id({0., 0., 0., 1});
  EuclideanSpace<3>::Rotation halfI({sqrt(0.5), 0., 0., sqrt(0.5)});

  auto ij = i * j;
  auto ii = i * i;
  auto iiii = ii * ii;

  PRINT_EXP(ii);
  PRINT_EXP(iiii);
  PRINT_EXP(ij);

  ASSERT_PRED3(isNear<3>, (-x).eval(), j.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<3>, (-y).eval(), k.rotate(y).eval(), eps);
  ASSERT_PRED3(isNear<3>, (-z).eval(), i.rotate(z).eval(), eps);
  ASSERT_EQ(k, ij.eval());
  ASSERT_EQ(id, minusId); // because we are comparing rotations!
  ASSERT_EQ(minusId, ii.eval());
  ASSERT_EQ(id, iiii.eval());
  ASSERT_PRED3(isNear<3>, z.eval(), halfI.rotate(y).eval(), eps * 2);
  ASSERT_PRED3(isNear<3>, x.eval(), ii.rotate(x).eval(), eps);
  ASSERT_PRED3(isNear<3>, y.eval(), ii.rotate(y).eval(), eps);
}
