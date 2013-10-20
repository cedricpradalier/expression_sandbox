/*
 * Problem.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef PROBLEM_HPP_
#define PROBLEM_HPP_


namespace benchmark {

template <typename ConstInput_, typename Input_, typename Output_, typename DERIVED>
class Problem {
public:
  typedef ConstInput_ ConstInput;
  typedef Input_ Input;
  typedef Output_ Output;

  class Instance {
   public:
    virtual double calcError(Output & output) const = 0;
    const ConstInput & getConstInput() const { return constInput; }
    const Input & getInput() const { return input; }
   protected:
    ConstInput constInput;
    Input input;
    Output solution;
  };

  typedef std::unique_ptr<const Instance> InstancePtr;

  virtual InstancePtr createInstance() const = 0;
  virtual ~Problem() {};
};


}
#endif /* PROBLEM_HPP_ */
