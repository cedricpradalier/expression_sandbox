#include <benchmark/EvalVariants.hpp>

namespace benchmark {
std::ostream & operator << (std::ostream & out, const EvalVariants v){
  switch (v) {
    case EvalVariants::Eval:
      out << "Eval";
      break;
    case EvalVariants::EvalJacobian:
      out << "eJac";
      break;
  }
  return out;
}
}
