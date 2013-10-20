/*
 * EuclideanSpace.hpp
 *
 *  Created on: Sep 25, 2013
 *      Author: hannes
 */

#ifndef EUCLIDEANSPACE_HPP_
#define EUCLIDEANSPACE_HPP_

#include "EuclideanPoint.hpp"
#include "EuclideanRotation.hpp"

namespace TEX_NAMESPACE {

template <int Dim_>
struct EuclideanSpace {
  typedef EuclideanPoint<Dim_> Point;
  typedef EuclideanPoint<Dim_> Vector;
  typedef EuclideanRotation<Dim_> Rotation;
};

}

#endif /* EUCLIDEANSPACE_HPP_ */
