/*
 * Derivatives.hpp
 *
 *  Created on: Oct 23, 2013
 *      Author: hannes
 */

#ifndef DERIVATIVES_HPP_
#define DERIVATIVES_HPP_

#include "TypedExpressions.hpp"

namespace TEX_NAMESPACE {

template <typename Space>
struct get_tangent_space {
  typedef Space type;
};

template <typename Exp, unsigned DiffIndex>
struct Diffable : public Exp, public AnyExp<typename get_space<Exp>::type, Diffable<Exp, DiffIndex>> {
  typedef typename get_space<Exp>::type Space;

  Diffable(const Exp & e) : Exp(e) {}

  TEX_STRONG_INLINE const Space eval() const {
    return evalImpl();
  }

  const Exp & getExp() const {
    return static_cast<const Exp&>(*this);
  }

  TEX_STRONG_INLINE const Space evalImpl() const {
    return evalExp(getExp());
  }

  friend std::ostream & operator << (std::ostream & out, const Diffable & diffable) {
    out << "Diffable" << DiffIndex <<  ":" << (diffable.getExp());
    return out;
  }
};

namespace internal {
  template <typename Exp, unsigned DiffIndex>
  struct get_space<Diffable<Exp, DiffIndex> >{
   public:
    typedef typename get_space<Exp>::type type;
  };
}

template <typename Exp, unsigned DiffIndex, typename DERIVED>
struct OpMemberBase<Diffable<Exp, DiffIndex>, DERIVED> : public OpMemberBase<Exp, DERIVED> {
};


namespace internal {
template <typename TangentSpace, int baseIndex, bool isZero>
struct Diff {
  typedef typename TangentSpace::template BaseVector<baseIndex> type;
};

template <typename TangentSpace, int baseIndex>
struct Diff<TangentSpace, baseIndex, true> {
  typedef typename TangentSpace::ZeroVector type;
};
}


template <typename T>
class Scalar;


template <typename ResultSpace, typename Vector>
class ToVectorDifferential {
 public:
  ToVectorDifferential(Vector & dest) : dest(dest) {}

  template<typename T>
  void apply(const T & vector){
    dest += vector;
  }

  template<typename T>
  void apply(const Scalar<T> & vector){
    dest += static_cast<const linalg::Matrix<double, 1,1> &>(vector);
  }


  void reset(){
    dest.setZero();
  }
 private:
   Vector & dest;
};


template <unsigned EvalDiffIndex, unsigned BasisIndex, typename Space, typename Differential, typename = typename std::enable_if<std::is_same<Space,typename get_space<Space>::type>::value>::type>
void evalDiff(const Space &, Differential & differential) {
}

template <unsigned EvalDiffIndex, unsigned BasisIndex, typename Space, unsigned DiffIndex, typename Differential>
void evalDiff(const Diffable<Space, DiffIndex> &, Differential & differential) {
  if(DiffIndex == EvalDiffIndex){
    differential.apply(get_tangent_space<typename get_space<Space>::type>::type::template getBasisVector<BasisIndex>());
  }
}

template <typename Exp, typename DiffableExp, unsigned DiffIndex, unsigned MaxBasisIndex = get_dim<DiffableExp>::value - 1>
void evalFullDiffInto(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable, linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > & result);

namespace internal {
  template <typename Exp, typename DiffableExp, unsigned DiffIndex, unsigned MaxBasisIndex, typename Matrix>
  struct NextDiffEvaluator {
    void operator()(const Exp & exp, const Diffable<DiffableExp, DiffIndex>  & diffable, Matrix & result){
      evalFullDiffInto<Exp, DiffableExp, DiffIndex, MaxBasisIndex - 1>(exp, diffable, result);
    }
  };

  template <typename Exp, typename DiffableExp, unsigned DiffIndex, typename Matrix>
  struct NextDiffEvaluator<Exp, DiffableExp, DiffIndex, 0, Matrix> {
    void operator()(const Exp & exp, const Diffable<DiffableExp, DiffIndex>  & diffable, Matrix & result){
    }
  };
}

template <typename Exp, typename DiffableExp, unsigned DiffIndex, unsigned MaxBasisIndex>
void evalFullDiffInto(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable, linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > & result) {
  constexpr size_t dim = get_dim<Exp>::value;
  auto block = result.template block<dim, 1>(0, MaxBasisIndex);
  ToVectorDifferential<typename get_space<Exp>::type, decltype(block)> diff(block);
  evalDiff<DiffIndex, MaxBasisIndex>(exp, diff);
  internal::NextDiffEvaluator<Exp, DiffableExp, DiffIndex, MaxBasisIndex, decltype(result)>()(exp, diffable, result);
}

template <typename Exp, typename DiffableExp, unsigned DiffIndex>
TEX_STRONG_INLINE linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > evalFullDiff(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable) {
  linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > result;
  result.setZero();
  evalFullDiffInto(exp, diffable, result);
  return result;
}

// ********** diff functor *****
template <typename Functor>
struct DiffFunctor : private Functor {
  typedef Functor ApplyFunc;
  DiffFunctor(const Functor & applyFunc) : ApplyFunc(applyFunc){}

  template <typename TangentVector>
  TEX_STRONG_INLINE void apply(const TangentVector & vector){
    ApplyFunc::operator()(vector);
  }
};

template <typename Functor>
inline DiffFunctor<Functor> createDiff(const Functor & applyFunc){
  return DiffFunctor<Functor>(applyFunc);
}

template <typename Exp, unsigned DiffIndex>
struct Cache<Diffable<Exp, DiffIndex>> : public Cache<Exp> {
};

template <unsigned EvalDiffIndex, unsigned BasisIndex, typename Space, typename Differential, typename Cache, typename = typename std::enable_if<std::is_same<Space,typename get_space<Space>::type>::value>::type>
void evalDiffCached(const Space &, Differential & differential, Cache &) {
}

template <unsigned EvalDiffIndex, unsigned BasisIndex, typename Space, unsigned DiffIndex, typename Differential, typename Cache>
TEX_STRONG_INLINE void evalDiffCached(const Diffable<Space, DiffIndex> &, Differential & differential, Cache &) {
  if(DiffIndex == EvalDiffIndex){
    differential.apply(get_tangent_space<typename get_space<Space>::type>::type::template getBasisVector<BasisIndex>());
  }
}


template <typename Exp, typename DiffableExp, unsigned DiffIndex, typename Matrix, typename Cache, unsigned MaxBasisIndex = get_dim<DiffableExp>::value - 1>
TEX_STRONG_INLINE void evalFullDiffIntoCached(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable, Cache & cache, Matrix & result);

namespace internal {
  template <typename Exp, typename DiffableExp, unsigned DiffIndex, unsigned MaxBasisIndex, typename Matrix, typename Cache>
  struct NextDiffEvaluatorCached {
    TEX_STRONG_INLINE void operator()(const Exp & exp, const Diffable<DiffableExp, DiffIndex>  & diffable, Cache & cache, Matrix & result){
      evalFullDiffIntoCached<Exp, DiffableExp, DiffIndex, Matrix, Cache, MaxBasisIndex - 1>(exp, diffable, cache, result);
    }
  };

  template <typename Exp, typename DiffableExp, unsigned DiffIndex, typename Matrix, typename Cache>
  struct NextDiffEvaluatorCached<Exp, DiffableExp, DiffIndex, 0, Matrix, Cache> {
    TEX_STRONG_INLINE void operator()(const Exp & exp, const Diffable<DiffableExp, DiffIndex>  & diffable, Cache & cache, Matrix & result){
    }
  };
}
template <typename Exp, typename DiffableExp, unsigned DiffIndex, typename Matrix, typename Cache, unsigned MaxBasisIndex>
TEX_STRONG_INLINE void evalFullDiffIntoCached(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable, Cache & cache, Matrix & result) {
  constexpr size_t dim = get_dim<Exp>::value;
  auto block = result.template block<dim, 1>(0, MaxBasisIndex);
  ToVectorDifferential<typename get_space<Exp>::type, decltype(block)> diff(block);
  evalDiffCached<DiffIndex, MaxBasisIndex>(exp, diff, cache);
  internal::NextDiffEvaluatorCached<Exp, DiffableExp, DiffIndex, MaxBasisIndex, decltype(result), Cache>()(exp, diffable, cache, result);
}

template <typename Exp, typename DiffableExp, unsigned DiffIndex, typename Cache>
TEX_STRONG_INLINE linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > evalFullDiffCached(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable, Cache & cache) {
  linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > result;
  result.setZero();
  evalFullDiffIntoCached(exp, diffable, cache, result);
  return result;
}

template <typename Exp, typename DiffableExp, unsigned DiffIndex>
TEX_STRONG_INLINE linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > evalFullDiffCached(const Exp & exp, const Diffable<DiffableExp, DiffIndex> & diffable) {
  auto cache = createCache(exp);
  cache.update(exp);
  linalg::Matrix<double, get_dim<Exp>::value, get_dim<DiffableExp>::value > result;
  result.setZero();
  evalFullDiffIntoCached(exp, diffable, cache, result);
  return result;
}



}// namespace TEX_NAMESPACE

#endif /* DERIVATIVES_HPP_ */
