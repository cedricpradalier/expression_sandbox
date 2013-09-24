/*
 * TypedExpressions.hpp
 *
 *  Created on: Sep 23, 2013
 *      Author: hannes
 */

#ifndef TYPEDEXPRESSIONS_HPP_
#define TYPEDEXPRESSIONS_HPP_

#include <memory>

namespace tex {

constexpr static int DefaultLevel = 10;

// **************** operational base ***************

template <typename ValueType_>
struct OpBase {
  typedef ValueType_ ValueType;
};

template <typename ValueType_, typename DERIVED>
struct OpMemberBase {
};

// **************** tools ****************

namespace internal {
  template <typename R>
  struct UnwrapValueType{
   private:
    template <typename ValueType_>
    static ValueType_& unwrap(OpBase<ValueType_>*);
    static R& unwrap(...);
   public:
    typedef typename std::remove_reference<decltype(unwrap((R*)nullptr))>::type type;
  };

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
}
using internal::UnwrapValueType;

// **************** virtual operational base ***************

template <typename ValueType_>
class VOpBase : public OpBase<ValueType_> {
 public:
  typedef ValueType_ ValueType;

  inline ValueType eval() const { return evalImpl(); }

  friend std::ostream & operator <<(std::ostream & out, const VOpBase & op){
    op.printImpl(out);
    return out;
  }

  virtual ~VOpBase(){};
 private:
  virtual ValueType evalImpl() const = 0;
  virtual void printImpl(std::ostream & out) const = 0;
};

template <typename ExpType_, typename PtrType_ = std::shared_ptr<const ExpType_>, bool cloneIt = true>
struct ExpPtr : public OpBase<typename UnwrapValueType<ExpType_>::type>, public OpMemberBase<typename UnwrapValueType<ExpType_>::type, ExpPtr<ExpType_, PtrType_, cloneIt> >{
  typedef typename UnwrapValueType<ExpType_>::type ValueType;
  typedef ExpType_ ExpType;
  typedef PtrType_ PtrType;

  constexpr static int Level = internal::getLevel<ExpType_>();

  ExpPtr() = default;
  ExpPtr(const PtrType & p) : ptr_(p){}
  ExpPtr(const ExpType_ & p) : ptr_(cloneIt ? new ExpType_(p) : &p){}

  operator ExpType const & () const {
    return *ptr_;
  }

  inline ValueType eval() const { return ptr_->eval(); }
  friend std::ostream & operator <<(std::ostream & out, const ExpPtr & eptr){
    return out << "@" << *eptr.ptr_;
  }
 protected:
  PtrType ptr_;
};

template <typename ValueType_>
struct ErasingPtr : public ExpPtr<VOpBase<ValueType_> > {
  typedef ExpPtr<VOpBase<ValueType_> > Base;
  typedef ValueType_ ValueType;
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
  struct UnwrapValueType<ExpPtr<ExpType_, PtrType_, cloneIt> >{
   public:
    typedef typename UnwrapValueType<ExpType_>::type type;
  };
  template <typename ValueType_>
  struct UnwrapValueType<ErasingPtr<ValueType_>>{
   public:
    typedef ValueType_ type;
  };
}


template <typename T, int Level>
struct OperandStorage{
  typedef T StorageType;
  typedef T EvaluableType;
};

template <typename ValueType>
struct OperandStorage<ErasingPtr<ValueType>, 0>{
  typedef ErasingPtr<ValueType> StorageType;
  typedef ErasingPtr<ValueType> EvaluableType;
};

template <typename ValueType_, typename DERIVED, int Level_>
class GenericOp : public OpMemberBase<ValueType_, DERIVED>, public OpBase<ValueType_> {
 public:
  typedef DERIVED App;
  constexpr static int Level = Level_;
  typedef ValueType_ ValueType;
};


template <typename ValueType_, typename DERIVED>
class GenericOp<ValueType_, DERIVED, 0> : public VOpBase<ValueType_> , public OpMemberBase<ValueType_, DERIVED> {
 public:
  typedef ErasingPtr<ValueType_> App;
  constexpr static int Level = DefaultLevel;

  virtual ~GenericOp(){};
 private:
  virtual ValueType_ evalImpl() const {
    static_assert(!std::is_same<decltype(&DERIVED::eval), decltype(&VOpBase<ValueType_>::eval)>::value, "Eval must be shadowed in an Operation");
    return static_cast<const DERIVED&>(*this).eval();
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const DERIVED&>(*this);
  }
};

// **************** operators ****************
template<typename A, typename B, typename ValueType_, typename DERIVED>
class BinOpBase : public GenericOp<ValueType_, DERIVED, internal::getNextLevel<A, B>()> {
  typedef GenericOp<ValueType_, DERIVED, internal::getNextLevel<A, B>()> Base;
 protected:
  BinOpBase(const A & a, const B & b) : a_(a), b_(b) {}
  const typename OperandStorage<A, Base::Level>::EvaluableType & getA() const { return a_; }
  const  typename OperandStorage<B, Base::Level>::EvaluableType & getB() const { return b_; }
 private:
  typename OperandStorage<A, Base::Level>::StorageType a_;
  typename OperandStorage<B, Base::Level>::StorageType b_;
};

#define RESULT_TYPE(METHOD, TYPE_A, TYPE_B) decltype(static_cast<typename UnwrapValueType<TYPE_A>::type*>(nullptr)->METHOD(*static_cast<typename UnwrapValueType<TYPE_B>::type*>(nullptr)))

template <typename A, typename B, typename ValueType_ = RESULT_TYPE(evalSum, A, B)>
class Plus : public BinOpBase<A, B, ValueType_, Plus<A, B, ValueType_> >{
 public:
  typedef BinOpBase<A, B, ValueType_, Plus<A, B, ValueType_> > Base;
  typedef ValueType_ ValueType;

  Plus(const A & a, const B & b) : Base(a, b){}

  ValueType eval() const {
    return this->getA().eval().evalSum(this->getB().eval());
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

template <typename A, typename B, typename ValueType_ = RESULT_TYPE(evalTimes, A, B)>
class Times : public BinOpBase<A, B, ValueType_, Times<A, B, ValueType_> >{
 public:
  typedef BinOpBase<A, B, ValueType_, Times<A, B, ValueType_> > Base;
  typedef ValueType_ ValueType;

  Times(const A & a, const B & b) : Base(a, b){}

  ValueType eval() const {
    return this->getA().eval().evalTimes(this->getB().eval());
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

namespace internal {
  template <typename DERIVED, typename Other, typename ValueType>
  struct UnwrapValueType<Plus<DERIVED, Other, ValueType >>{
   public:
    typedef ValueType type;
  };
  template <typename DERIVED, typename Other, typename ValueType>
  struct UnwrapValueType<Times<DERIVED, Other, ValueType >>{
   public:
    typedef ValueType type;
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

template <typename T, bool CloneVariableIntoOperationsTree = false>
class Variable : public T {
 public:
  constexpr static int Level = internal::getLevel<T>();

  inline Variable(const T & t) : T(t) {}
  inline Variable();

  friend std::ostream & operator << (std::ostream & out, const Variable & namedExp) {
    out << "$" << (static_cast<const T&>(namedExp));
    return out;
  }
};

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

#define SUPPORTS_VARIABLES



template <typename T>
struct VWrapper : public T, public VOpBase<typename UnwrapValueType<T>::type> {
  typedef typename UnwrapValueType<T>::type ValueType;
  VWrapper(const T & t) : T(t) {}
  virtual ~VWrapper(){}
  virtual ValueType evalImpl() const {
    return static_cast<const T&>(*this).eval();
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const T&>(*this);
  }
};


template <typename ValueType_>
class Exp : public ErasingPtr<ValueType_> {
 public:
  typedef ErasingPtr<ValueType_> Base;
  Exp(const ErasingPtr<ValueType_> & p) : Base(p){}

  template<typename PtrType_, bool cloneIt>
  Exp(const ExpPtr<ValueType_, PtrType_, cloneIt> & p) : Base(p){}

  Exp(const VOpBase<ValueType_> & p) : Base(p){}

  template<typename T, typename = std::enable_if<std::is_same<typename UnwrapValueType<T>::type, ValueType_>::value> >
  Exp(const T & t) : Base(typename Base::PtrType(new VWrapper<T>(t))){}

  friend std::ostream & operator << (std::ostream & out, const Exp & exp) {
    out << "@exp" << *exp.ptr_;
    return out;
  }
};

namespace internal {
  template <typename ValueType>
  struct UnwrapValueType<VWrapper<ValueType >>{
   public:
    typedef ValueType type;
  };

  template <typename ValueType>
  struct UnwrapValueType<Exp<ValueType >>{
   public:
    typedef ValueType type;
  };
}


#define SUPPORTS_USER_FUNCTION


// **************** example spaces ****************
class SimpleSpace {
 public:
  const SimpleSpace & eval() const { // TODO make this function unnecessary
    return *this;
  }

  SimpleSpace(int v = 0) : value(v){}

  SimpleSpace evalSum(const SimpleSpace & other) const {
    return SimpleSpace(value + other.value);
  }

  friend
  std::ostream & operator << (std::ostream & out, const SimpleSpace & ob) {
    return out << ob.value;
  }

  int getVal() const {
    return value;
  }
  void setVal(int v) {
    value = v;
  }

 private:
  int value;
};

template <size_t Dim_>
class TemplatedSpace {
 public:
  constexpr static size_t Dimension = Dim_;
  const TemplatedSpace & eval() const { // TODO make this function unnecessary
    return *this;
  }

  TemplatedSpace(){
    for(int i = 0; i < Dimension; i ++) { value[i] = 0; }
  }

  TemplatedSpace(std::initializer_list<double> entries){
    if(entries.size() > Dimension) throw std::runtime_error("too many initializing entries");
    int i = 0;
    for(int d : entries) { value[i++] = d; }
    for(; i < Dimension; i ++) { value[i] = 0; }
  }
  TemplatedSpace evalSum (const TemplatedSpace & other) const {
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

}

#endif /* TYPEDEXPRESSIONS_HPP_ */
