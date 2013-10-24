/*
 * TestTools.hpp
 *
 *  Created on: Oct 10, 2013
 *      Author: hannes
 */

#ifndef TESTTOOLS_HPP_
#define TESTTOOLS_HPP_

#include <type_traits>
#include <gtest/gtest.h>
#include <typed_expressions/EuclideanSpace.hpp>

constexpr static double eps = std::numeric_limits<double>::epsilon();

#define PRINT_EXP(EXP) std::cout <<  #EXP << " = " << EXP << " = " << evalExp(EXP) << std::endl;

namespace TEX_NAMESPACE {
  template <int Dim_>
  bool isNear(typename EuclideanSpace<Dim_>::Point a, typename EuclideanSpace<Dim_>::Point b, double threshold){
    for(int i = 0; i < Dim_; i++){
      if(!(fabs(a[i] - b[i]) < threshold)) return false;
    }
    return true;
  }
}
#endif /* TESTTOOLS_HPP_ */
