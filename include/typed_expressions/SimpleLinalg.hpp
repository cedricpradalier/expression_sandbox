/*
 * SimpleLinalg.hpp
 *
 *  Created on: Oct 9, 2013
 *      Author: hannes
 */


#ifndef SIMPLELINALG_HPP_
#define SIMPLELINALG_HPP_
#ifndef EXP_LINALG

#define EXP_LINALG
#define EXP_SIMPLE_LINALG

#include <type_traits>

namespace simple_linalg {
template <typename DERIVED>
class MatrixBase {
};

namespace internal {
template <typename Scalar, size_t Size, bool IsPointer>
struct ArrayOrPointer {
 public:
  constexpr static size_t stride = 0;
  Scalar value[Size];
};

template <typename Scalar, size_t Size>
struct ArrayOrPointer<Scalar, Size, true> {
  ArrayOrPointer() = default;
  ArrayOrPointer(Scalar* p, int stride) :value(p), stride(stride){}
  Scalar * value;
  size_t stride;
};


template <typename Scalar_, size_t Rows_, size_t Cols_, bool IsMap = false>
struct Storage : private ArrayOrPointer<Scalar_, Rows_ * Cols_, IsMap> {
  constexpr static size_t Rows = Rows_, Cols = Cols_;
  constexpr static size_t Size = Rows * Cols;

  size_t getMemRowLength() const {
    return Cols + this->stride;
  }

  inline size_t calcIndex(size_t row, size_t col) const {
    return col + getMemRowLength() * row;
  }
  Scalar_ & accessEntry(size_t i, size_t j) {
    return this->value[calcIndex(i, j)];
  }

  const Scalar_ & accessEntry(size_t i, size_t j) const {
    return this->value[calcIndex(i, j)];
  }

  const Scalar_ & accessEntry(size_t i) const {
    return this->value[calcIndex(i / Cols, i % Cols)];
  }

  Scalar_ & accessEntry(size_t i) {
    return this->value[calcIndex(i / Cols, i % Cols)];
  }

  Scalar_ & operator[](size_t i) {
    return this->value[i];
  }

  const Scalar_ & operator[](size_t i) const {
    return this->value[i];
  }

  Storage() = default;
  Storage(Scalar_ * p, size_t stride) : ArrayOrPointer<Scalar_, Size, IsMap>(p, stride) { };
  Storage(const Scalar_ * p, size_t stride) : ArrayOrPointer<Scalar_, Size, IsMap>(p, stride) { };
};
}

template <typename Scalar_, size_t Rows_, size_t Cols_, bool IsMap = false>
class Matrix : public internal::Storage<Scalar_, Rows_, Cols_, IsMap>, public MatrixBase< Matrix<Scalar_, Rows_, Cols_, IsMap> > {
  typedef internal::Storage<Scalar_, Rows_, Cols_, IsMap> Storage;
  typedef Matrix<Scalar_, Rows_, Cols_ , false> NonMapMatrix;
 public:
  typedef Scalar_ Scalar;
  constexpr static size_t Rows = Rows_, Cols = Cols_;
  constexpr static size_t Size = Rows * Cols;


  Matrix(Scalar * p, size_t stride) : Storage(p, stride){}

  Matrix(): Matrix(Scalar(0)){}

  Matrix(Scalar v){
    setAll(v);
  }

  void setAll(Scalar v){
    for(size_t i = 0; i < Size; i ++) { Storage::accessEntry(i) = v; }
  }

  void setZero(){
    setAll(Scalar(0));
  }

  void setOne(){
    setAll(Scalar(1));
  }

  inline Matrix & eval() {
    return *this;
  }
  inline const Matrix & eval() const {
    return *this;
  }

  template<typename MatrixFunctor, typename = decltype (static_cast<MatrixFunctor*>(nullptr)->operator()(size_t(0), size_t(0)))>
  inline Matrix(MatrixFunctor getEntry) {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         Storage::accessEntry(i, j) = getEntry(i, j);
       }
    }
  }

  template<typename VectorFunctor, typename = decltype (static_cast<VectorFunctor*>(nullptr)->operator()(size_t(0)))>
  inline Matrix(VectorFunctor getEntry, void * dummy = nullptr) { // without this dummy this function would be to similar to the one before
    for(size_t i = 0; i < Size; i++) {
     (*this)[i] = getEntry(i);
    }
  }

  template<typename VectorFunctor>
  inline Matrix(VectorFunctor getEntry, size_t size) {
    size_t i = 0;
    for(; i < size; i++) {
     (*this)[i] = getEntry(i);
    }
    for(; i < Size; i ++) { (*this)[i] = 0; }
  }


  Matrix(std::initializer_list<Scalar> entries){
    if(entries.size() > Size) throw std::runtime_error("too many initializing entries");
    size_t i = 0;
    for(double d : entries) { (*this)[i++] = d; }
    for(; i < Size; i ++) { (*this)[i] = 0; }
  }

  Scalar_ & operator()(size_t i, size_t j) {
    return Storage::accessEntry(i, j);
  }

  const Scalar_ & operator()(size_t i, size_t j) const {
    return Storage::accessEntry(i, j);
  }

  template <typename Functor>
  Matrix modify(const Functor & modifier) {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         modifier((*this)(i, j), i, j);
       }
    }
    return *this;
  }

  template <typename Other>
  Matrix & operator = (const MatrixBase<Other> & other) {
    modify([&other](Scalar & s, size_t i, size_t j) { s = other(i, j); });
  }

  template <bool OtherIsMap>
  Matrix operator += (const Matrix<Scalar, Rows, Cols, OtherIsMap> & other) {
    modify([&other](Scalar & s, size_t i, size_t j) { return s += other(i, j); });
    return *this;
  }

  template <bool OtherIsMap>
  Matrix operator -= (const Matrix<Scalar, Rows, Cols, OtherIsMap> & other) {
    modify([&other](Scalar & s, size_t i, size_t j) { return s -= other(i, j); });
    return *this;
  }


  template <bool OtherIsMap>
  NonMapMatrix operator + (const Matrix<Scalar, Rows, Cols, OtherIsMap> & other) const {
    return NonMapMatrix([&other, this](size_t i, size_t j) { return (*this)(i, j) + other(i, j); });
  }

  template <size_t OtherCols, bool OtherIsMap>
  NonMapMatrix operator * (const Matrix<Scalar, Cols, OtherCols, OtherIsMap> & other) const {
    return NonMapMatrix([&other, this](size_t i, size_t j) {
      Scalar r = 0;
      for(size_t k = 0; k < Cols; k++){
        r += (*this)(i, k) * other(k, j);
      }
      return r;
    });
  }

  template <bool OtherIsMap>
  inline NonMapMatrix cross (const Matrix<Scalar, 3, 1, OtherIsMap> & other) const {
    static_assert(Rows == 3 && Cols == 1 && (int)OtherIsMap != -1, "cross is only defined when Rows = 3 and Cols = 1");
    return NonMapMatrix([&other, this](size_t i, size_t j) {
      if(j != 0) throw std::runtime_error("index out of bounds");
      switch(i){
        case 0:
          return (*this)[1] * other[2] - (*this)[2] * other[1];
        case 1:
          return (*this)[2] * other[0] - (*this)[0] * other[2];
        case 2:
          return (*this)[0] * other[1] - (*this)[1] * other[0];
        default:
          throw std::runtime_error("index out of bounds");
      }
    });
  }

  inline NonMapMatrix operator * (Scalar s) const {
    return NonMapMatrix([s, this](size_t i, size_t j) {
      return (*this)(i, j) * s;
    });
  }

  inline friend NonMapMatrix operator * (Scalar s, const Matrix & m){
    return m * s;
  }

  NonMapMatrix operator - () const {
    return NonMapMatrix([this](size_t i, size_t j) { return -(*this)(i, j); });
  }

  Matrix & operator /= (Scalar d) {
    modifyScalars([d](Scalar & s){ s/= d; });
    return *this;
  }

  Matrix & operator *= (Scalar d) {
    modifyScalars([d](Scalar & s){ s*= d; });
    return *this;
  }

  template <bool OtherIsMap>
  Scalar dot(const Matrix<Scalar, Rows, Cols, OtherIsMap> & other) const {
    Scalar r = 0;
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         r += (*this)(i, j) * other(i, j);
       }
    }
    return r;
  }

  friend
  std::ostream & operator << (std::ostream & out, const Matrix & ob) {
    static_assert(Size > 0, "");
    out << "[";
    for(size_t i = 0; i < Rows; i++) {
       out << ob(i, 0);
       for(size_t j = 1; j < Cols; j++) {
         out << ", " << ob(i, j);
       }
       if(i != Rows - 1) out << "; ";
    }
    return out << "]";
  }

  /*
  template <size_t rows, size_t cols>
  inline Matrix<Scalar, rows, cols> copyBlock(size_t startI, size_t startJ) const {
    return Matrix<Scalar, rows, cols>([&v = value, startI, startJ](size_t i, size_t j) { return v[calcIndex(startI + i, startJ + j)]; });
  }
  */

  template <size_t rows, size_t cols>
  inline const Matrix<Scalar, rows, cols, true> block(size_t startI, size_t startJ) const {
    return Matrix<Scalar, rows, cols, true>(&const_cast<Matrix&>(*this)(startI, startJ), this->getMemRowLength() - cols);
  }

  template <size_t rows, size_t cols>
  inline Matrix<Scalar, rows, cols, true> block(size_t startI, size_t startJ) {
    return Matrix<Scalar, rows, cols, true>(&(*this)(startI, startJ), this->getMemRowLength() - cols);
  }

  bool operator == (const Matrix & other) const {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         if(other.accessEntry(i, j) != this->accessEntry(i, j)) return false;
       }
    }
    return true;
  }

  bool operator != (const Matrix & other) const {
    return ! ((*this) == other);
  }
 private:
  template <typename Functor>
  void modifyScalars(Functor modifier) {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         modifier((*this)(i, j));
       }
    }
  }
};

template <size_t Rows, size_t Cols>
using MatrixD = Matrix<double, Rows, Cols>;


template <size_t Dim_>
using Vector = Matrix<double, Dim_, 1>;

typedef size_t MatrixSize;


template<typename T>
struct MatrixConvertible {
  typedef const T & type;
  static inline const T & asMatrixConvertible(const T & t, MatrixSize size = -1){
    return t;
  }
};

template<typename T>
inline typename MatrixConvertible<T>::type asMatrixConvertible(const T & t, MatrixSize size = -1) {
  return MatrixConvertible<T>::asMatrixConvertible(t, size);
}


template<>
struct MatrixConvertible<std::initializer_list<double>> {
  typedef std::initializer_list<double> type;
  static inline type asMatrixConvertible(std::initializer_list<double> t, MatrixSize size = -1){
    return t;
  }
};

}

namespace linalg = simple_linalg;

#endif
#endif /* SIMPLELINALG_HPP_ */
