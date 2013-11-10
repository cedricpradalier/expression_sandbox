/*
 * EvalVariants.hpp
 *
 *  Created on: Nov 8, 2013
 *      Author: hannes
 */

#ifndef EVALVARIANTS_HPP_
#define EVALVARIANTS_HPP_

#include <iostream>

namespace benchmark {


enum class EvalVariants {
  Eval,
  EvalJacobian
};

std::ostream & operator << (std::ostream & out, const EvalVariants v);
}



#endif /* EVALVARIANTS_HPP_ */
