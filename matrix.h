/**
 ******************************************************************************
 * @file    matrix.h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <cstring> // memcpy
#include <cmath>   // sqrtf, fabs, fmin

// Matrix class
template <int _rows, int _cols>
class Matrixf {
 public:
  // Constructor without input data
  Matrixf(void) : rows_(_rows), cols_(_cols) {
    for (int i = 0; i < _rows * _cols; i++) data_[i] = 0.0f;
  }
  // Constructor with input data
  Matrixf(float data[_rows * _cols]) : Matrixf() {
    std::memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }
  // Copy constructor
  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    std::memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }
  // Destructor
  ~Matrixf(void) {}

  // Row size
  int rows(void) { return _rows; }
  // Column size
  int cols(void) { return _cols; }

  // Element access
  float* operator[](const int& row) { return &this->data_[row * _cols]; }
  const float* operator[](const int& row) const { return &this->data_[row * _cols]; }

  // Operators
  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    std::memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }
  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    for (int i = 0; i < _rows * _cols; i++) {
      this->data_[i] += mat.data_[i];
    }
    return *this;
  }
  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    for (int i = 0; i < _rows * _cols; i++) {
      this->data_[i] -= mat.data_[i];
    }
    return *this;
  }
  Matrixf<_rows, _cols>& operator*=(const float& val) {
    for (int i = 0; i < _rows * _cols; i++) {
      this->data_[i] *= val;
    }
    return *this;
  }
  Matrixf<_rows, _cols>& operator/=(const float& val) {
    float inv_val = 1.0f / val;
    for (int i = 0; i < _rows * _cols; i++) {
      this->data_[i] *= inv_val;
    }
    return *this;
  }
  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res = *this;
    res += mat;
    return res;
  }
  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res = *this;
    res -= mat;
    return res;
  }



    // 新增：一元负号运算符重载
  Matrixf<_rows, _cols> operator-() const {
    Matrixf<_rows, _cols> res;
    for (int i = 0; i < _rows * _cols; i++) {
      res.data_[i] = -this->data_[i];
    }
    return res;
  }

  
  Matrixf<_rows, _cols> operator*(const float& val) {
    Matrixf<_rows, _cols> res = *this;
    res *= val;
    return res;
  }
  friend Matrixf<_rows, _cols> operator*(const float& val,
                                         const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res = mat;
    res *= val;
    return res;
  }
  Matrixf<_rows, _cols> operator/(const float& val) {
    Matrixf<_rows, _cols> res = *this;
    res /= val;
    return res;
  }
  // Matrix multiplication
  template <int cols>
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    Matrixf<_rows, cols> res;
    for (int i = 0; i < _rows; i++) {
      for (int j = 0; j < cols; j++) {
        float sum = 0.0f;
        for (int k = 0; k < _cols; k++) {
          sum += mat1[i][k] * mat2[k][j];
        }
        res[i][j] = sum;
      }
    }
    return res;
  }

  // Submatrix
  template <int rows, int cols>
  Matrixf<rows, cols> block(const int& start_row, const int& start_col)  {
    Matrixf<rows, cols> res;
    for (int row = start_row; row < start_row + rows; row++) {
      std::memcpy((float*)res[0] + (row - start_row) * cols,
                  (float*)this->data_ + row * _cols + start_col,
                  cols * sizeof(float));
    }
    return res;
  }
  // Specific row
  Matrixf<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }
  // Specific column
  Matrixf<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

  // Transpose
  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    for (int i = 0; i < _rows; i++) {
      for (int j = 0; j < _cols; j++) {
        res[j][i] = (*this)[i][j];
      }
    }
    return res;
  }
  // Trace
  float trace(void) {
    float res = 0;
    for (int i = 0; i < std::fmin(_rows, _cols); i++) {
      res += (*this)[i][i];
    }
    return res;
  }
  // Norm
  float norm(void) { return std::sqrt((this->trans() * *this)[0][0]); }

 protected:
  // size
  int rows_, cols_;
  // data
  float data_[_rows * _cols];
};

// Matrix functions
namespace matrixf {

// Special Matrices
// Zero matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}
// Ones matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; i++) {
    data[i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Identity matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < std::fmin(_rows, _cols); i++) {
    data[i * _cols + i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Diagonal matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  for (int i = 0; i < std::fmin(_rows, _cols); i++) {
    res[i][i] = vec[i][0];
  }
  return res;
}

// Inverse
template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  // Extended matrix [A|I]
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; i++) {
    std::memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
    ext_mat[i][_dim + i] = 1;
  }
  // Gauss-Jordan elimination
  for (int i = 0; i < _dim; i++) {
    // Find maximum absolute value in the first column in lower right block
    float abs_max = std::fabs(ext_mat[i][i]);
    int abs_max_row = i;
    for (int row = i; row < _dim; row++) {
      if (abs_max < std::fabs(ext_mat[row][i])) {
        abs_max = std::fabs(ext_mat[row][i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // Singular
      return matrixf::zeros<_dim, _dim>();
    }
    if (abs_max_row != i) {  // Row exchange
      Matrixf<1, 2 * _dim> row_i = ext_mat.row(i);
      Matrixf<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
      std::memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
      std::memcpy(ext_mat[abs_max_row], row_i[0], 2 * _dim * sizeof(float));
    }
    float k = 1.0f / ext_mat[i][i];
    for (int col = i; col < 2 * _dim; col++) {
      ext_mat[i][col] *= k;
    }
    for (int row = 0; row < _dim; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat[row][i];
      for (int j = i; j < 2 * _dim; j++) {
        ext_mat[row][j] -= k * ext_mat[i][j];
      }
    }
  }
  // Inverse = ext_mat(:, n+1:2n)
  Matrixf<_dim, _dim> res;
  for (int i = 0; i < _dim; i++) {
    std::memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

}  // namespace matrixf

namespace vector3f {

// Hat of vector (skew-symmetric matrix)
Matrixf<3, 3> hat(Matrixf<3, 1> vec);

// Cross product
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_H
