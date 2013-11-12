/*
 * TypedExpressions.hpp
 *
 *  Created on: Sep 23, 2013
 *      Author: hannes
 */

#ifndef TYPEDEXPRESSIONS_HPP_
#define TYPEDEXPRESSIONS_HPP_

#include <iostream>
#include <stdexcept>
#include <memory>

#define TEX_INLINE inline

#ifdef NDEBUG
//#define TEX_STRONG_INLINE __attribute__((always_inline))
#define TEX_STRONG_INLINE inline
#else
#define TEX_STRONG_INLINE inline
#endif

#ifndef TYPED_EXP_NAMESPACE_POSTFIX
#define TYPED_EXP_NAMESPACE_POSTFIX
#endif

#ifndef TYPED_EXP_MAX_DEPTH
#define TYPED_EXP_MAX_DEPTH 30
#endif

#define CONCAT(A,B, C) A ## B ## C
#define TEX_NAMESPACE_(D,S) CONCAT(tex, D, S)

#define TEX_NAMESPACE TEX_NAMESPACE_(TYPED_EXP_MAX_DEPTH, TYPED_EXP_NAMESPACE_POSTFIX)

namespace TEX_NAMESPACE {

constexpr static int DefaultLevel = TYPED_EXP_MAX_DEPTH;

// **************** operational base ***************

template <typename Space_, typename ExpType_>
struct AnyExp {
  inline const ExpType_ & getExp() const { return static_cast<ExpType_ const &>(*this); }
  inline ExpType_ & getExp() { return static_cast<ExpType_&>(*this); }
  inline operator const ExpType_ & () const { return getExp(); }
  inline operator ExpType_ & () { return getExp(); }
};

template <typename Space_>
struct OpBase {
  typedef Space_ Space;
};

template <typename Space_, typename DERIVED>
struct OpMemberBase {
};

// **************** tools ****************

namespace internal {
  template <typename R>
  struct get_space{
   private:
    template <typename Space_>
    static Space_& unwrap(OpBase<Space_>*);
    static R& unwrap(...);
   public:
    typedef typename std::remove_reference<decltype(unwrap((R*)nullptr))>::type type;
  };

#define GET_SPACE(T) typename get_space<T>::type

  constexpr int max(int a, int b) {
    return a > b ? a : b;
  }
  constexpr int min(int a, int b) {
    return a < b ? a : b;
  }

  template <typename T, int Level = T::Level>
  constexpr int getLevel(int defaultLevel) {
    return Level;
  }

  template <typename T>
  constexpr int getLevel(long defaultLevel) {
    return defaultLevel;
  }

  template <typename T>
  constexpr int getLevel() {
    return getLevel<T>(DefaultLevel);
  }

  template <typename A, typename B>
  constexpr int getNextLevel() {
    return max(0, min(getLevel<A>(), getLevel<B>()) - 1);
  };

  template <typename A>
  constexpr int getNextLevel() {
    return max(0, getLevel<A>() - 1);
  };

  template <typename A>
  struct get_dim {
    constexpr static size_t value = get_space<A>::type::Dimension;
  };
}
using internal::get_space;
using internal::get_dim;

// ************* default evalExp functions **************8

template <typename ExpType, typename = typename std::enable_if<! std::is_same<ExpType, typename get_space<ExpType>::type>::value>::type>
inline const typename get_space<ExpType>::type evalExp(const ExpType & t) {
  return t.evalImpl();
}

template <typename T, typename = typename std::enable_if<std::is_same<T, typename get_space<T>::type>::value>::type>
inline const T & evalExp(const T & t){
  return t;
}

// **************** virtual operational base ***************

template <typename Space_>
class VOpBase : public OpBase<Space_>, public AnyExp<Space_, VOpBase<Space_> > {
 public:
  typedef Space_ Space;

  inline const Space eval() const { return evalVImpl(); }

  friend std::ostream & operator <<(std::ostream & out, const VOpBase & op){
    op.printImpl(out);
    return out;
  }

  virtual ~VOpBase(){};
 private:
  virtual const Space evalVImpl() const = 0;
  virtual void printImpl(std::ostream & out) const = 0;
};

template <typename ExpType_, typename PtrType_ = std::shared_ptr<const ExpType_>, bool cloneIt = true>
struct ExpPtr : public OpBase<typename get_space<ExpType_>::type>, public OpMemberBase<typename get_space<ExpType_>::type, ExpPtr<ExpType_, PtrType_, cloneIt> >, public AnyExp<typename get_space<ExpType_>::type, ExpType_>{
  typedef typename get_space<ExpType_>::type Space;
  typedef ExpType_ ExpType;
  typedef PtrType_ PtrType;

  constexpr static int Level = internal::getLevel<ExpType_>();

  ExpPtr() = default;
  ExpPtr(const PtrType & p) : ptr_(p){}
  ExpPtr(const ExpType_ & p) : ptr_(cloneIt ? new ExpType_(p) : &p){}

  operator ExpType const & () const {
    return *ptr_;
  }

  inline Space evalImpl() const { return ptr_->eval(); }
  inline Space eval() const { return evalImpl(); }
  Space evalVImpl() const { return evalImpl(); }
  friend std::ostream & operator <<(std::ostream & out, const ExpPtr & eptr){
    return out << "@" << *eptr.ptr_;
  }
 protected:
  PtrType ptr_;
};

template <typename Space_>
struct ErasingPtr : public ExpPtr<VOpBase<Space_> >,  public AnyExp<Space_, ErasingPtr<Space_> > {
  typedef ExpPtr<VOpBase<Space_> > Base;
  typedef Space_ Space;
  typedef typename Base::PtrType PtrType;
  constexpr static int Level = DefaultLevel;

  ErasingPtr() = default;
  ErasingPtr(const ErasingPtr &) = default;
  ErasingPtr(const PtrType & p) : Base(p){}
  template <typename DERIVED>
  ErasingPtr(const DERIVED & p) : Base(PtrType(new DERIVED(p))){}

  friend std::ostream & operator <<(std::ostream & out, const ErasingPtr & eptr){
    return out << "@erased:" << *eptr.ptr_;
  }
};

namespace internal {
  template <typename ExpType_, typename PtrType_, bool cloneIt>
  struct get_space<ExpPtr<ExpType_, PtrType_, cloneIt> >{
   public:
    typedef typename get_space<ExpType_>::type type;
  };
  template <typename Space_>
  struct get_space<ErasingPtr<Space_>>{
   public:
    typedef Space_ type;
  };
}


template <typename T, int Level>
struct OperandStorage{
  typedef T StorageType;
  typedef const T & EvaluableType;
};

template <typename Space>
struct OperandStorage<ErasingPtr<Space>, 0>{
  typedef ErasingPtr<Space> StorageType;
  typedef ErasingPtr<Space> EvaluableType;
};

template <typename Space_, typename DERIVED, int Level_>
class GenericOp : public OpMemberBase<Space_, DERIVED>, public OpBase<Space_>, public AnyExp<Space_, DERIVED> {
 public:
  typedef DERIVED App;
  constexpr static int Level = Level_;
  typedef Space_ Space;

  inline auto eval() const -> decltype(evalExp(static_cast<const DERIVED&>(*this))){
    return evalExp(static_cast<const DERIVED&>(*this));
  }
};


template <typename Space_, typename DERIVED>
class GenericOp<Space_, DERIVED, 0> : public VOpBase<Space_> , public OpMemberBase<Space_, DERIVED>, public AnyExp<Space_, DERIVED> {
 public:
  typedef ErasingPtr<Space_> App;
  constexpr static int Level = DefaultLevel;

  inline const Space_ eval() const {
    return evalImpl();
  }
  inline const Space_ evalImpl() const {
    return evalExp(static_cast<const DERIVED&>(*this));
  }

  virtual ~GenericOp(){};
 private:
  virtual const Space_ evalVImpl() const {
    static_assert(!std::is_same<decltype(&DERIVED::eval), decltype(&VOpBase<Space_>::eval)>::value, "Eval must be shadowed in an Operation");
    return static_cast<const DERIVED&>(*this).eval();
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const DERIVED&>(*this);
  }
};

// **************** operators ****************


template <typename Exp>
struct get_op {
  typedef Exp type;
};


template <typename SpaceA_, typename ExpType_>
struct AnyUnOp {
  typedef AnyUnOp OP;
  inline const ExpType_ & getExp() const { return static_cast<ExpType_ const &>(*this); }
  inline ExpType_ & getExp() { return static_cast<ExpType_&>(*this); }
  inline operator const ExpType_ & () const { return getExp(); }
  inline operator ExpType_ & () { return getExp(); }
};

namespace internal {
template <typename SpaceA_, typename ExpType_>
  struct get_space<AnyUnOp<SpaceA_, ExpType_> >{
   public:
    typedef typename get_space<ExpType_>::type type;
  };
};

template<typename A_, typename Space_, typename DERIVED>
class UnOpBase : public GenericOp<Space_, DERIVED, internal::getNextLevel<A_>()>, public AnyUnOp<GET_SPACE(A_), DERIVED> {
 public:
  typedef GenericOp<Space_, DERIVED, internal::getNextLevel<A_>()> Base;
  typedef A_ A;
  const typename OperandStorage<A, Base::Level>::EvaluableType & getA() const { return a_; }
 protected:
  UnOpBase(const A & a) : a_(a) {}
 private:
  typename OperandStorage<A, Base::Level>::StorageType a_;
};

template <typename SpaceA_, typename SpaceB_, typename ExpType_>
struct AnyBinOp {
  typedef AnyBinOp OP;
  inline const ExpType_ & getExp() const { return static_cast<ExpType_ const &>(*this); }
  inline ExpType_ & getExp() { return static_cast<ExpType_&>(*this); }
  inline operator const ExpType_ & () const { return getExp(); }
  inline operator ExpType_ & () { return getExp(); }
};

namespace internal {
template <typename SpaceA_, typename SpaceB_, typename ExpType_>
  struct get_space<AnyBinOp<SpaceA_, SpaceB_, ExpType_> >{
   public:
    typedef typename get_space<ExpType_>::type type;
  };
};

template<typename A_, typename B_, typename Space_, typename DERIVED>
class BinOpBase : public GenericOp<Space_, DERIVED, internal::getNextLevel<A_, B_>()>, public AnyBinOp<GET_SPACE(A_), GET_SPACE(B_), DERIVED> {
 public:
  typedef GenericOp<Space_, DERIVED, internal::getNextLevel<A_, B_>()> Base;
  typedef A_ A;
  typedef B_ B;

  constexpr static size_t DimA = get_dim<A>::value;
  constexpr static size_t DimB = get_dim<B>::value;
  constexpr static size_t DimResult = get_dim<Space_>::value;
  const typename OperandStorage<A, Base::Level>::EvaluableType & getA() const { return a_; }
  const typename OperandStorage<B, Base::Level>::EvaluableType & getB() const { return b_; }
 protected:
  BinOpBase(const A & a, const B & b) : a_(a), b_(b) {}
 private:
  typename OperandStorage<A, Base::Level>::StorageType a_;
  typename OperandStorage<B, Base::Level>::StorageType b_;
};

#define RESULT_SPACE(METHOD, TYPE_A, TYPE_B) decltype(static_cast<typename get_space<TYPE_A>::type*>(nullptr)->METHOD(*static_cast<typename get_space<TYPE_B>::type*>(nullptr)))
#define RESULT_SPACE1(METHOD, TYPE_A) decltype(static_cast<typename get_space<TYPE_A>::type*>(nullptr)->METHOD())

#define RESULT_SPACE_GLOBAL(METHOD, TYPE_A, TYPE_B) decltype(METHOD(*static_cast<typename get_space<TYPE_A>::type*>(nullptr), *static_cast<typename get_space<TYPE_B>::type*>(nullptr)))

template <typename A, typename B, typename Space_ = RESULT_SPACE(evalPlus, A, B)>
class Plus : public BinOpBase<A, B, Space_, Plus<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, Plus<A, B, Space_> > Base;
  typedef Space_ Space;

  Plus(const A & a, const B & b) : Base(a, b){}

  const Space evalImpl() const {
    return evalExp(this->getA()).evalPlus(evalExp(this->getB()));
  }

  friend
  std::ostream & operator << (std::ostream & out, const Plus & sum) {
    return out << "(" << sum.getA() << " + " << sum.getB() << ")";
  }
};

template <typename A, typename B, typename Result = Plus<A, B> >
inline typename Result::App operator + (const A & a, const B & b){
  return Result(a, b);
}

template <typename A, typename B, typename Space_ = RESULT_SPACE(evalMinus, A, B)>
class Minus : public BinOpBase<A, B, Space_, Minus<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, Minus<A, B, Space_> > Base;
  typedef Space_ Space;

  Minus(const A & a, const B & b) : Base(a, b){}

  const Space evalImpl() const {
    return evalExp(this->getA()).evalMinus(evalExp(this->getB()));
  }

  friend
  std::ostream & operator << (std::ostream & out, const Minus & sum) {
    return out << "(" << sum.getA() << " - " << sum.getB() << ")";
  }
};

template <typename A, typename B, typename Result = Minus<A, B> >
inline typename Result::App operator - (const A & a, const B & b){
  return Result(a, b);
}


/* TODO remove the space template parameter
#define RESULT_SP(Op, A, B) typename ResultBinOp<Op,A,B>::type

template <template <typename, typename> class BinOp, typename A, typename B>
struct ResultBinOp;

template <template <typename, typename> class BinOp, typename A>
struct ResultBinOp<BinOp, A, A>{
  typedef A type;
};


template <typename A, typename B>
class Times : public BinOpBase<A, B, Times<A, B> >{
 public:
  typedef BinOpBase<A, B, RESULT_SP(evalTimes, A, B), Times<A, B> > Base;
  typedef Space_ Space;
*/


template <typename A, typename B, typename Space_ = RESULT_SPACE(evalTimes, A, B)>
class Times : public BinOpBase<A, B, Space_, Times<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, Times<A, B, Space_> > Base;
  typedef Space_ Space;

  Times(const A & a, const B & b) : Base(a, b){}

  inline const Space evalImpl() const {
    return evalExp(this->getA()).evalTimes(evalExp(this->getB()));
  }

  friend
  std::ostream & operator << (std::ostream & out, const Times & op) {
    return out << "(" << op.getA() << " * " << op.getB() << ")";
  }
};

template <typename A, typename B, typename Result = Times<A, B> >
inline typename Result::App operator * (const A & a, const B & b){
  return Result(a, b);
}

template <typename A, typename B, typename Space_ = RESULT_SPACE(evalCoeffwiseTimes, A, B)>
class CoeffwiseTimes : public BinOpBase<A, B, Space_, CoeffwiseTimes<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, CoeffwiseTimes<A, B, Space_> > Base;
  typedef Space_ Space;

  CoeffwiseTimes(const A & a, const B & b) : Base(a, b){}

  inline const Space evalImpl() const {
    return evalExp(this->getA()).evalCoeffwiseTimes(evalExp(this->getB()));
  }

  friend
  std::ostream & operator << (std::ostream & out, const CoeffwiseTimes & op) {
    return out << "(" << op.getA() << " * " << op.getB() << ")";
  }
};

template <typename A, typename B, typename Result = CoeffwiseTimes<A, B> >
inline typename Result::App operator && (const A & a, const B & b){
  return Result(a, b);
}


template <typename A, typename B, typename Space_ = RESULT_SPACE(evalRotate, A, B)>
class Rotate : public BinOpBase<A, B, Space_, Rotate<A, B, Space_> > {
 public:
  typedef BinOpBase<A, B, Space_, Rotate<A, B, Space_> > Base;
  typedef Space_ Space;

  Rotate(const A & a, const B & b) : Base(a, b){}

  inline const Space evalImpl() const {
    return evalExp(*this);
  }

  /* TODO implement expand
  inline auto expand() const -> decltype(A::expandRotate(this->getA(), this->getB())) {
    return A::expandRotate(this->getA(), this->getB());
  }
  */

  friend
  std::ostream & operator << (std::ostream & out, const Rotate & op) {
    return out << op.getA() << ".rotate(" << op.getB() << ")";
  }
};


template <typename A, typename B, typename Space_>
inline const Space_ evalExp(const Rotate<A, B, Space_> & r){
  return evalExp(r.getA()).evalRotate(evalExp(r.getB()));
}

namespace internal {
  template <typename A, typename B, typename Space>
  struct get_space<Rotate<A, B, Space >>{
   public:
    typedef Space type;
  };
}

template <typename A, typename B, typename Result = Rotate<A, B> >
inline typename Result::App rotate(const A & a, const B & b){
  return Result(a, b);
}

template <typename A, typename Space_ = RESULT_SPACE1(evalInverse, A)>
class Inverse : public UnOpBase<A, Space_, Inverse<A, Space_> > {
 public:
  typedef UnOpBase<A, Space_, Inverse<A, Space_> > Base;

  Inverse(const A & a) : Base(a){}

  Space_ evalImpl() const {
    return evalExp(this->getA()).evalInverse();
  }

  friend
  std::ostream & operator << (std::ostream & out, const Inverse & op) {
    return out << "(" << op.getA() << ")^-1";
  }
};

namespace internal {
  template <typename A, typename Space>
  struct get_space<Inverse<A, Space >>{
   public:
    typedef Space type;
  };
}

template <typename A, typename Result = Inverse<A> >
inline typename Result::App inverse(const A & a){
  return Result(a);
}

template <typename A, typename Space_ = RESULT_SPACE1(evalNegate, A)>
class Neg : public UnOpBase<A, Space_, Neg<A, Space_> > {
 public:
  typedef UnOpBase<A, Space_, Neg<A, Space_> > Base;

  Neg(const A & a) : Base(a){}

  const Space_ evalImpl() const {
    return evalExp(this->getA()).evalNegate();
  }

  friend
  std::ostream & operator << (std::ostream & out, const Neg & op) {
    return out << "-(" << op.getA() << ")";
  }
};

namespace internal {
  template <typename A, typename Space>
  struct get_space<Neg<A, Space >>{
   public:
    typedef Space type;
  };
}

template <typename A, typename Result = Neg<A> >
inline typename Result::App negate(const A & a){
  return Result(a);
}


namespace internal {
  template <typename A, typename B, typename Space>
  struct get_space<Plus<A, B, Space >>{
   public:
    typedef Space type;
  };
  template <typename A, typename B, typename Space>
  struct get_space<Minus<A, B, Space >>{
   public:
    typedef Space type;
  };
  template <typename A, typename B, typename Space>
  struct get_space<Times<A, B, Space >>{
   public:
    typedef Space type;
  };
  template <typename A, typename B, typename Space>
  struct get_space<CoeffwiseTimes<A, B, Space >>{
   public:
    typedef Space type;
  };
}

// **************** modifier ****************

template <typename T>
class NamedExp : public T{
 public:
  constexpr static int Level = internal::getLevel<T>();

  inline NamedExp(const char * name, const T & t) : T(t), name_(name){}
  inline NamedExp();

  friend std::ostream & operator << (std::ostream & out, const NamedExp<T> & namedExp) {
    out << namedExp.name_ << "(L" << (Level) <<  "):" << (static_cast<const T&>(namedExp));
    return out;
  }
 private:
  std::string name_;
};

template <typename T>
NamedExp<T> operator , (const char *name, const T & t) {
  return NamedExp<T>(name, t);
}

template <typename Space, bool CloneVariableIntoOperationsTree = false>
class Variable : public OpMemberBase<typename get_space<Space>::type, Variable<Space, CloneVariableIntoOperationsTree> > {
 public:
  constexpr static int Level = internal::getLevel<Space>();

  inline Variable(const Space & t) : s(t) {}
  inline Variable(){}

  template <typename Other>
  Variable & operator = (const Other & t)  {
    s = t;
    return *this;
  }

  inline const Space & evalImpl() const {
    return s;
  }

  inline const Space & eval() {
    return evalImpl();
  }

  friend std::ostream & operator << (std::ostream & out, const Variable & namedExp) {
    out << "$" << (namedExp.s);
    return out;
  }
 private:
  Space s; // of course it would be nicer to have this as a private base, but the compiler then yields ambiguous member access errors, when OpMemberBase has identical member functions... (compiler bug?)
};

namespace internal {
  template <typename Space, bool CloneVariableIntoOperationsTree>
  struct get_space<Variable<Space, CloneVariableIntoOperationsTree>>{
   public:
    typedef Space type;
  };
}

template <typename ExpType, int Level>
struct OperandStorage<Variable<ExpType, false>, Level> {
  typedef ExpPtr<Variable<ExpType, false>, const Variable<ExpType, false> *, false> StorageType;
  typedef Variable<ExpType, false> EvaluableType;
};

template <typename ExpType, int Level>
struct OperandStorage<Variable<ExpType, true>, Level> {
  typedef ExpPtr<Variable<ExpType, true>, std::shared_ptr<const Variable<ExpType, true>>, true> StorageType;
  typedef Variable<ExpType, true> EvaluableType;
};


template <typename Space>
class ExtVariable : public OpMemberBase<Space, ExtVariable<Space> > {
 public:
  constexpr static int Level = internal::getLevel<Space>();

  inline ExtVariable(const Space * ps = nullptr) : s(ps) {}

  inline const Space & evalImpl() const {
    return *s;
  }

  inline void setStorage(const Space * ps){
    s = ps;
  }

  friend std::ostream & operator << (std::ostream & out, const ExtVariable & namedExp) {
    out << "$" << (*namedExp.s);
    return out;
  }
 private:
  const Space *s;
};

namespace internal {
  template <typename Space>
  struct get_space<ExtVariable<Space>>{
   public:
    typedef Space type;
  };
}

template <typename Space, int Level>
struct OperandStorage<ExtVariable<Space>, Level> {
  typedef ExpPtr<ExtVariable<Space>, const ExtVariable<Space> *, false> StorageType;
  typedef ExtVariable<Space> EvaluableType;
};

template <typename Space>
class Ref : public OpMemberBase<Space, Ref<Space> >, public AnyExp<Space, Ref<Space> > {
 public:
  constexpr static int Level = internal::getLevel<Space>();

  Ref(const Space & s) : s(s){}

  inline const Space & eval() const {
    return s;
  }

  inline operator const Space & () const {
    return s;
  }

  friend std::ostream & operator << (std::ostream & out, const Ref & ref) {
    out << "&" << (ref.s);
    return out;
  }
 private:
  const Space & s;
};

namespace internal {
  template <typename Space>
  struct get_space<Ref<Space>>{
   public:
    typedef Space type;
  };
}

template <typename Space, int Level>
struct OperandStorage<Ref<Space>, Level> {
  typedef Ref<Space> StorageType;
  typedef const Space & EvaluableType;
};

template <typename Space>
inline const Space & evalExp(const Ref<Space> & r){
  return r;
}

template <typename Space, const Space& s_>
class StaticRef : public OpMemberBase<Space, StaticRef<Space, s_> > {
 public:
  constexpr static int Level = internal::getLevel<Space>();
  constexpr static Space & s = s_;

  inline const Space & eval() const {
    return s;
  }

  friend std::ostream & operator << (std::ostream & out, const StaticRef & namedExp) {
    out << "$" << (*namedExp.s);
    return out;
  }
};


namespace internal {
  template <typename Space, Space & s>
  struct get_space<StaticRef<Space, s>>{
   public:
    typedef Space type;
  };
}

template <typename Space, Space & s, int Level>
struct OperandStorage<StaticRef<Space, s>, Level> {
  typedef StaticRef<Space, s> StorageType;
  typedef StaticRef<Space, s> EvaluableType;
};

template <typename Space, const Space& s_>
class Functor : public OpMemberBase<Space, Functor<Space, s_> > {
 public:
  constexpr static int Level = internal::getLevel<Space>();
  constexpr static Space & s = s_;

  inline const Space & eval() const {
    return s;
  }

  friend std::ostream & operator << (std::ostream & out, const Functor & namedExp) {
    out << "$" << (*namedExp.s);
    return out;
  }
};

namespace internal {
  template <typename Space, Space & s>
  struct get_space<Functor<Space, s>>{
   public:
    typedef Space type;
  };
}

template <typename Space, Space & s, int Level>
struct OperandStorage<Functor<Space, s>, Level> {
  typedef Functor<Space, s> StorageType;
  typedef Functor<Space, s> EvaluableType;
};

#define SUPPORTS_VARIABLES


template <typename T>
struct VWrapper : public T, public VOpBase<typename get_space<T>::type> {
  typedef typename get_space<T>::type Space;
  VWrapper(const T & t) : T(t) {}
  virtual ~VWrapper(){}
  virtual const Space evalVImpl() const {
    return evalExp(static_cast<const T&>(*this));
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const T&>(*this);
  }
};


template <typename Space_>
class VExp : public ErasingPtr<Space_>, public AnyExp<Space_, VExp<Space_> > {
 public:
  typedef ErasingPtr<Space_> Base;
  VExp() : Base(){}
  VExp(const Base & p) : Base(p){}

  template<typename PtrType_, bool cloneIt>
  VExp(const ExpPtr<Space_, PtrType_, cloneIt> & p) : Base(p){}

  VExp(const VOpBase<Space_> & p) : Base(p){}

  template<typename T, typename = std::enable_if<std::is_same<typename get_space<T>::type, Space_>::value> >
  VExp(const T & t) : Base(typename Base::PtrType(new VWrapper<T>(t))){}

  friend std::ostream & operator << (std::ostream & out, const VExp & exp) {
    out << "@vexp" << *exp.ptr_;
    return out;
  }
};

namespace internal {
  template <typename Space>
  struct get_space<VWrapper<Space >>{
   public:
    typedef Space type;
  };

  template <typename Space>
  struct get_space<VExp<Space >>{
   public:
    typedef Space type;
  };
}


template <typename Space_> struct SWrapper : public Space_, public AnyExp<Space_, SWrapper<Space_> > {
  typedef SWrapper<Space_> WrappedType;
  SWrapper(const Space_ & v) : Space_(v) {}
  static WrappedType wrap(Space_ v) {
    return v;
  }
};

template  <typename Space_, typename DERIVED>
struct SWrapper<AnyExp<Space_, DERIVED>> {
  typedef DERIVED WrappedType;
  static WrappedType wrap(Space_ v) {
    return v;
  }
};

template <typename Any_, typename = typename get_space<Any_>::type >
auto toExp(Any_ v) -> typename SWrapper<Any_>::WrappedType {
  return SWrapper<Any_>::wrap(v);
};


#define SUPPORTS_USER_FUNCTION


// **************** cache *******************


template <typename A, typename Space_>
struct get_op<Inverse<A, Space_>> {
  typedef typename Inverse<A, Space_>::OP type;
};

template <typename A, typename B, typename Space_>
struct get_op<Times<A, B, Space_>> {
  typedef typename Times<A, B, Space_>::OP type;
};
template <typename A, typename B, typename Space_>
struct get_op<CoeffwiseTimes<A, B, Space_>> {
  typedef typename CoeffwiseTimes<A, B, Space_>::OP type;
};

template <typename A, typename B, typename Space_>
struct get_op<Rotate<A, B, Space_>> {
  typedef typename Rotate<A, B, Space_>::OP type;
};
template <typename A, typename B, typename Space_>
struct get_op<Plus<A, B, Space_>> {
  typedef typename Plus<A, B, Space_>::OP type;
};

template <typename A, typename B, typename Space_>
struct get_op<Minus<A, B, Space_>> {
  typedef typename Minus<A, B, Space_>::OP type;
};


template <typename Space>
struct Transformations {
  inline void updateTransformations(const Space & s) {
  }
};

template <typename Cache>
struct CacheBase {
  Cache & getCache() {
    return static_cast<Cache&>(*this);
  }
};

template <typename Exp, typename Storage = typename get_space<Exp>::type>
struct Cache : public CacheBase<Cache<Exp, Storage>>, public Transformations<typename get_space<Exp>::type> {
  inline void update(const Exp &exp) { this->updateTransformations(accessValue(exp)); }
  inline auto accessValue(const Exp & exp) const -> decltype(evalExp(exp)){
    if(!std::is_reference<decltype(evalExp(exp))>::value ){
      std::cout << "NotCached::exp=" << std::endl << exp << std::endl; // XXX: debug output of exp
    }
    return evalExp(exp);
  }
};

template <typename Exp>
struct Cache<Ref<Exp>> : public CacheBase<Cache<Ref<Exp>>>, public Transformations<typename get_space<Exp>::type> {
  typedef typename get_space<Exp>::type Space;
  void update(const Ref<Exp> & exp) { this->updateTransformations(accessValue(exp)); }
  const Space & accessValue(const Ref<Exp> & exp) const { return exp.eval();}
};


template <typename T>
inline const T & evalExpCached(const T & t, ...){
  return evalExp(t);
}


template <typename A, typename Exp, typename Storage>
struct Cache<AnyUnOp<A, Exp>, Storage> : public CacheBase<Cache<AnyUnOp<A, Exp>, Storage>>, public Transformations<typename get_space<Exp>::type> {
  typedef typename get_space<Exp>::type Space;
  Cache() = default;
  Cache(Storage value) : value(value) {}

  void update(const Exp & exp) {
    a.update(exp.getA());
    value = evalExpCached(exp, *this);
    this->updateTransformations(accessValue(exp));
  }
  const Space & accessValue(const Exp & exp) const { return value;}
 public:
  Cache<typename get_op<typename Exp::A>::type> a;
 private:
  Storage value;
};


template <typename A, typename B, typename Exp, typename Storage>
struct Cache<AnyBinOp<A, B, Exp>, Storage> : public CacheBase<Cache<AnyBinOp<A, B, Exp>, Storage>>, public Transformations<typename get_space<Exp>::type> {
  typedef typename get_space<Exp>::type Space;
  Cache() = default;
  Cache(Storage value) : value(value) {}

  void update(const Exp & exp) {
    a.update(exp.getA());
    b.update(exp.getB());
    value = evalExpCached(exp, *this);
    this->updateTransformations(accessValue(exp));
  }
  const Space & accessValue(const Exp & exp) const { return value;}
 public:
  Cache<typename get_op<typename Exp::A>::type> a;
  Cache<typename get_op<typename Exp::B>::type> b;
 private:
  Storage value;
};

template <typename Exp>
inline Cache<typename get_op<Exp>::type> createCache(const Exp & exp){
  return Cache<typename get_op<Exp>::type>();
}

template <typename Exp>
inline auto createCache(const Exp & exp, typename get_space<Exp>::type & result) -> Cache<typename get_op<Exp>::type, decltype(result)>{
  return Cache<typename get_op<Exp>::type, decltype(result)>(result);
}



template <typename A, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const Inverse<A, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalInverse();
}

template <typename A, typename B, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const Rotate<A, B, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalRotate(cache.b.accessValue(r.getB()));
}

template <typename A, typename B, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const Times<A, B, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalTimes(cache.b.accessValue(r.getB()));
}
template <typename A, typename B, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const CoeffwiseTimes<A, B, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalCoeffwiseTimes(cache.b.accessValue(r.getB()));
}

template <typename A, typename B, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const Plus<A, B, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalPlus(cache.b.accessValue(r.getB()));
}
template <typename A, typename B, typename Space_, typename Cache_>
inline const Space_ evalExpCached(const Minus<A, B, Space_> & r, const Cache_ & cache){
  return cache.a.accessValue(r.getA()).evalMinus(cache.b.accessValue(r.getB()));
}


// **************** example spaces ****************
class SimpleSpace {
 public:
  SimpleSpace(int v = 0) : value(v){}

  SimpleSpace evalPlus(const SimpleSpace & other) const {
    return SimpleSpace(value + other.value);
  }

  friend
  std::ostream & operator << (std::ostream & out, const SimpleSpace & ob) {
    return out << ob.value;
  }

  int getValue() const {
    return value;
  }
  SimpleSpace & operator = (int v) {
    value = v;
    return *this;
  }

 private:
  int value;
};

template <size_t Dim_>
class TemplatedSpace {
 public:
  constexpr static size_t Dimension = Dim_;
  TemplatedSpace(){
    for(int i = 0; i < Dimension; i ++) { value[i] = 0; }
  }

  TemplatedSpace(std::initializer_list<double> entries){
    if(entries.size() > Dimension) throw std::runtime_error("too many initializing entries");
    int i = 0;
    for(int d : entries) { value[i++] = d; }
    for(; i < Dimension; i ++) { value[i] = 0; }
  }
  TemplatedSpace evalPlus (const TemplatedSpace & other) const {
    TemplatedSpace r;
    for(int i = 0; i < Dimension; i ++) { r.value[i] = value[i] + other.value[i]; }
    return r;
  }

  friend
  std::ostream & operator << (std::ostream & out, const TemplatedSpace & ob) {
    static_assert(Dimension > 0, "");
    out << "[" << ob.value[0];
    for(int i = 1; i < Dimension; i ++) { out << ", " << ob.value[i]; }
    return out << "]";
  }

  int value[Dimension];
};

} // namespace TEX_NAMESPACE

namespace tex = TEX_NAMESPACE;

#endif /* TYPEDEXPRESSIONS_HPP_ */
