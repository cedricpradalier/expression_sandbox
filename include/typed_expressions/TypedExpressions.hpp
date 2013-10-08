/*
 * TypedExpressions.hpp
 *
 *  Created on: Sep 23, 2013
 *      Author: hannes
 */

#ifndef TYPEDEXPRESSIONS_HPP_
#define TYPEDEXPRESSIONS_HPP_

#include <stdexcept>
#include <memory>

namespace tex {

constexpr static int DefaultLevel = 10;

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
using internal::get_space;

// **************** virtual operational base ***************

template <typename Space_>
class VOpBase : public OpBase<Space_>, public AnyExp<Space_, VOpBase<Space_> > {
 public:
  typedef Space_ Space;

  inline Space eval() const { return evalImpl(); }

  friend std::ostream & operator <<(std::ostream & out, const VOpBase & op){
    op.printImpl(out);
    return out;
  }

  virtual ~VOpBase(){};
 private:
  virtual Space evalImpl() const = 0;
  virtual void printImpl(std::ostream & out) const = 0;
};

template <typename ExpType_, typename PtrType_ = std::shared_ptr<const ExpType_>, bool cloneIt = true>
struct ExpPtr : public OpBase<typename get_space<ExpType_>::type>, public OpMemberBase<typename get_space<ExpType_>::type, ExpPtr<ExpType_, PtrType_, cloneIt> >{
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

  inline Space eval() const { return ptr_->eval(); }
  friend std::ostream & operator <<(std::ostream & out, const ExpPtr & eptr){
    return out << "@" << *eptr.ptr_;
  }
 protected:
  PtrType ptr_;
};

template <typename Space_>
struct ErasingPtr : public ExpPtr<VOpBase<Space_> > {
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
  typedef T EvaluableType;
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
};


template <typename Space_, typename DERIVED>
class GenericOp<Space_, DERIVED, 0> : public VOpBase<Space_> , public OpMemberBase<Space_, DERIVED>, public AnyExp<Space_, DERIVED> {
 public:
  typedef ErasingPtr<Space_> App;
  constexpr static int Level = DefaultLevel;

  virtual ~GenericOp(){};
 private:
  virtual Space_ evalImpl() const {
    static_assert(!std::is_same<decltype(&DERIVED::eval), decltype(&VOpBase<Space_>::eval)>::value, "Eval must be shadowed in an Operation");
    return static_cast<const DERIVED&>(*this).eval();
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const DERIVED&>(*this);
  }
};

// **************** operators ****************
template<typename A, typename B, typename Space_, typename DERIVED>
class BinOpBase : public GenericOp<Space_, DERIVED, internal::getNextLevel<A, B>()> {
  typedef GenericOp<Space_, DERIVED, internal::getNextLevel<A, B>()> Base;
 protected:
  BinOpBase(const A & a, const B & b) : a_(a), b_(b) {}
  const typename OperandStorage<A, Base::Level>::EvaluableType & getA() const { return a_; }
  const  typename OperandStorage<B, Base::Level>::EvaluableType & getB() const { return b_; }
 private:
  typename OperandStorage<A, Base::Level>::StorageType a_;
  typename OperandStorage<B, Base::Level>::StorageType b_;
};

#define RESULT_SPACE(METHOD, TYPE_A, TYPE_B) decltype(static_cast<typename get_space<TYPE_A>::type*>(nullptr)->METHOD(*static_cast<typename get_space<TYPE_B>::type*>(nullptr)))

#define RESULT_SPACE_GLOBAL(METHOD, TYPE_A, TYPE_B) decltype(METHOD(*static_cast<typename get_space<TYPE_A>::type*>(nullptr), *static_cast<typename get_space<TYPE_B>::type*>(nullptr)))


template <typename A, typename B, typename Space_ = RESULT_SPACE(evalSum, A, B)>
class Plus : public BinOpBase<A, B, Space_, Plus<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, Plus<A, B, Space_> > Base;
  typedef Space_ Space;

  Plus(const A & a, const B & b) : Base(a, b){}

  Space eval() const {
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

template <typename A, typename B, typename Space_ = RESULT_SPACE(evalTimes, A, B)>
class Times : public BinOpBase<A, B, Space_, Times<A, B, Space_> >{
 public:
  typedef BinOpBase<A, B, Space_, Times<A, B, Space_> > Base;
  typedef Space_ Space;

  Times(const A & a, const B & b) : Base(a, b){}

  Space eval() const {
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
  template <typename A, typename B, typename Space>
  struct get_space<Plus<A, B, Space >>{
   public:
    typedef Space type;
  };
  template <typename A, typename B, typename Space>
  struct get_space<Times<A, B, Space >>{
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
struct VWrapper : public T, public VOpBase<typename get_space<T>::type> {
  typedef typename get_space<T>::type Space;
  VWrapper(const T & t) : T(t) {}
  virtual ~VWrapper(){}
  virtual Space evalImpl() const {
    return static_cast<const T&>(*this).eval();
  }
  virtual void printImpl(std::ostream & out) const {
    out << static_cast<const T&>(*this);
  }
};


template <typename Space_>
class VExp : public ErasingPtr<Space_>, public AnyExp<Space_, VExp<Space_> > {
 public:
  typedef ErasingPtr<Space_> Base;
  VExp(const ErasingPtr<Space_> & p) : Base(p){}

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
