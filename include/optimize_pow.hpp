/*
 * optimize_pow.hpp
 *
 *  Created on: Oct 22, 2013
 *      Author: hannes
 */

#ifndef OPTIMIZE_POW_HPP_
#define OPTIMIZE_POW_HPP_


#include <cmath>

inline double pow(double base, double exp){
  if(exp == 2){
    return base * base;
  }
  if(exp == -2){
    return 1.0 / base / base;
  }
  return std::pow(base, exp);
}


#endif /* OPTIMIZE_POW_HPP_ */
