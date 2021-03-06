/*
 * Problem.hpp
 *
 *  Created on: Oct 11, 2013
 *      Author: hannes
 */

#ifndef PROBLEM_HPP_
#define PROBLEM_HPP_

#include <memory>
#include <vector>
#include <string>

namespace benchmark {

template <typename ConstInput_, typename Input_, typename Output_, typename DERIVED, typename Variant_ = int>
class Problem {
public:
  typedef ConstInput_ ConstInput;
  typedef Input_ Input;
  typedef Output_ Output;
  typedef Variant_ Variant;

  Problem(std::string name) : name(name) {}

  class Instance {
   public:
    virtual double calcError(Output & output, const Variant variant) const = 0;
    const ConstInput & getConstInput() const { return constInput; }
    const Input & getInput() const { return input; }
   protected:
    ConstInput constInput;
    Input input;
    Output solution;
  };

  typedef std::unique_ptr<const Instance> InstancePtr;

  virtual std::vector<Variant> getVariants() const { return std::vector<Variant>({0}); };
  virtual InstancePtr createInstance() const = 0;
  virtual ~Problem() {};
  std::string getName() const { return name; }
private:
  std::string name;
};


}
#endif /* PROBLEM_HPP_ */
