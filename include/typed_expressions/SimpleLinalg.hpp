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

namespace linalg {

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

  inline size_t calcIndex(size_t row, size_t col) const {
    return col + (Cols + this->stride) * row;
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
 public:
  typedef Scalar_ Scalar;
  constexpr static size_t Rows = Rows_, Cols = Cols_;
  constexpr static size_t Size = Rows * Cols;

  Matrix(Scalar * p, size_t stride) : Storage(p, stride){
  }

  Matrix(): Matrix(Scalar(0)){
  }
  Matrix(Scalar v){
    for(size_t i = 0; i < Size; i ++) { Storage::accessEntry(i) = v; }
  }
  inline Matrix(std::function<Scalar (size_t i, size_t j)> getEntry) {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         Storage::accessEntry(i, j) = getEntry(i, j);
       }
    }
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

  template <typename Other>
  Matrix operator = (const MatrixBase<Other> & other) {
    Matrix r;
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         *this(i, j) = other(i, j);
       }
    }
    return r;
  }


  template <bool OtherIsMap>
  Matrix operator + (const Matrix<Scalar, Rows, Cols, OtherIsMap> & other) const {
    return Matrix([&other, this](size_t i, size_t j) { return (*this)(i, j) + other(i, j); });
  }

  template <size_t OtherCols, bool OtherIsMap>
  Matrix operator * (const Matrix<Scalar, Cols, OtherCols, OtherIsMap> & other) const {
    return Matrix([&other, this](size_t i, size_t j) {
      Scalar r = 0;
      for(size_t k = 0; k < Cols; k++){
        r += (*this)(i, k) * other(k, j);
      }
      return r;
    });
  }

  Matrix operator * (Scalar s) const {
    return Matrix([s, this](size_t i, size_t j) {
      return (*this)(i, j) * s;
    });
  }


  Matrix operator - () const {
    return Matrix([this](size_t i, size_t j) { return -(*this)(i, j); });
  }

  void modify(std::function<void(Scalar & s)> modifier) {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         modifier((*this)(i, j));
       }
    }
  }

  Matrix & operator /= (Scalar d) {
    modify([d](Scalar & s){ s/= d; });
    return *this;
  }

  Matrix & operator *= (Scalar d) {
    modify([d](Scalar & s){ s*= d; });
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
    return Matrix<Scalar, rows, cols, true>(&const_cast<Matrix&>(*this)(startI, startJ), startI);
  }

  template <size_t rows, size_t cols>
  inline Matrix<Scalar, rows, cols, true> block(size_t startI, size_t startJ) {
    return Matrix<Scalar, rows, cols, true>(&(*this)(startI, startJ), startJ);
  }

  bool operator == (const Matrix & other) const {
    for(size_t i = 0; i < Rows; i++) {
       for(size_t j = 0; j < Cols; j++) {
         if(other.accessEntry(i, j) != this->accessEntry(i, j)) return false;
       }
    }
    return true;
  }
 private:
};

template <size_t Rows, size_t Cols>
using MatrixD = Matrix<double, Rows, Cols>;


template <size_t Dim_>
using Vector = Matrix<double, Dim_, 1>;

typedef size_t MatrixSize;

template<typename T>
std::function<double(MatrixSize i, MatrixSize j)> asMatrixConvertible(const T & t) {
  return t;
}

}

using namespace linalg;

#endif
#endif /* SIMPLELINALG_HPP_ */
