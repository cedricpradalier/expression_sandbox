/*
 * TestTypedExpressions.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: hannes
 */

#include <iostream>

#include "typed_expressions/TypedExpressions.hpp"


#define NAMED(X) ::typed_expressions::NamedExp<decltype(X)>(#X, X)



template <int Level_>
struct L {
  constexpr static int Level = Level_;
};

void someUnitTests(){
  using namespace tex;
  static_assert(std::is_same<SimpleSpace, typename UnwrapValueType<SimpleSpace>::type >::value, "");
  static_assert(std::is_same<SimpleSpace, typename UnwrapValueType<Plus<SimpleSpace, SimpleSpace>>::type >::value, "");
  static_assert(std::is_same<SimpleSpace, typename UnwrapValueType<ErasingPtr<SimpleSpace> >::type >::value, "");
  static_assert(std::is_same<SimpleSpace, typename UnwrapValueType<ErasingPtr<SimpleSpace> >::type >::value, "");

  static_assert(1 == L<1>::Level, "");
  static_assert(0 == internal::getNextLevel<L<1>, L<2> >(), "");
  static_assert(0 == internal::getNextLevel<L<-1>, L<2> >(), "");
}

template <typename EXP>
int evalIt_JustToLookAtTheCompilersListings (const EXP & ex) {
  return ex.eval().getVal();
}

int main(int argc, char **argv) {
  someUnitTests();

  using namespace tex;

  SimpleSpace a(10);
//  SimpleSpace b(2);
  Variable<SimpleSpace, true> b(1);

  auto c = a + b;
  auto d = ("d", c + a);
  auto e = ("e", d + a);
  auto f = ("f", e + a);
  auto g = ("g", f + a);
  auto h = ("h", g + a);
  auto i = ("i", h + a);
  auto x = ("x", i + a);
  auto y = ("y", x + a);
  auto z = ("z", y + a);

  std::cout << "z=" << std::endl << z << "=" << z.eval().getVal() << std::endl; // XXX: debug output of x.eval()

  b.setVal(3);
  std::cout << "z=" << std::endl << z << "=" << z.eval().getVal() << std::endl; // XXX: debug output of x.eval()

  TemplatedSpace<2> A{1, 0}, B{0, 1};
  auto C = ("C", A + B);
  std::cout << "C=" << C << "="<< C.eval() << std::endl; // XXX debug output of C
//  auto U = A + a; // should fail compilation!

  return z.eval().getVal();
}
