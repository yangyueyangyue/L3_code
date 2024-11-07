/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       complex_matrix.h
 * @brief      复数矩阵运算
 * @details    定义相关复数矩阵运算算法
 *
 * @author     pengc
 * @date       2020.06.01
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_MATH_COMPLEX_MATRIX_H_
#define PHOENIX_COMMON_MATH_COMPLEX_MATRIX_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "math/matrix.h"

namespace phoenix {
namespace common {


// Enable to output debug information
#define ENABLE_MATH_COMPLEX_MATRIX_TRACE (0)

/**
 * @class ComplexNum
 * @brief 定义了复数的相关操作
 * @param Scalar 复数中元素的数据类型
 */
template<typename Scalar>
class ComplexNum {
public:
  typedef Scalar value_type;

public:
  /**
   * @brief 构造函数
   */
  ComplexNum() {
    real_ = Scalar(0);
    imag_ = Scalar(0);
  }

  /**
   * @brief 构造函数
   * @param[in] r 实数部分的值
   * @param[in] i 虚数部分的值
   */
  ComplexNum(Scalar r, Scalar i) {
    real_ = r;
    imag_ = i;
  }

  /**
   * @brief 拷贝构造函数
   * @param[in] other 另一个复数
   */
  ComplexNum(const ComplexNum& other) {
    real_ = other.real_;
    imag_ = other.imag_;
  }

  /**
   * @brief 重载赋值操作符号
   * @param[in] other 另一个复数
   */
  void operator =(const ComplexNum& other) {
    real_ = other.real_;
    imag_ = other.imag_;
  }

  /**
   * @brief 复数加法
   * @param[in] c2 另一个复数
   */
  ComplexNum operator +(const ComplexNum &c2) const {
    return ComplexNum(real_ + c2.real_, imag_ + c2.imag_);
  }

  /**
   * @brief 复数减法
   * @param[in] c2 另一个复数
   */
  ComplexNum operator -(const ComplexNum &c2) const {
    return ComplexNum(real_ - c2.real_, imag_ - c2.imag_);
  }

  /**
   * @brief 复数乘法
   * @param[in] c2 另一个复数
   */
  ComplexNum operator *(const ComplexNum &c2) const {
    return ComplexNum(real_*c2.real_ - imag_*c2.imag_,
                      real_*c2.imag_ + imag_*c2.real_);
  }

  /**
   * @brief 复数除法
   * @param[in] c2 除数
   */
  ComplexNum operator /(const ComplexNum &c2) const {
    Scalar c2_norm = c2.real_*c2.real_ + c2.imag_*c2.imag_;
    return ComplexNum((real_*c2.real_ + imag_*c2.imag_) / c2_norm,
                      (-real_*c2.imag_ + imag_*c2.real_) / c2_norm);
  }

  /**
   * @brief 复数 + 实数
   * @param[in] s2 实数
   */
  ComplexNum operator +(const Scalar &s2) const {
    return ComplexNum(real_ + s2, imag_);
  }

  /**
   * @brief 复数 - 实数
   * @param[in] s2 实数
   */
  ComplexNum operator -(const Scalar &s2) const {
    return ComplexNum(real_ - s2, imag_);
  }

  /**
   * @brief 复数 * 实数
   * @param[in] s2 实数
   */
  ComplexNum operator *(const Scalar &s2) const {
    return ComplexNum(real_*s2, imag_*s2);
  }

  /**
   * @brief 复数 / 实数
   * @param[in] s2 实数(除数)
   */
  ComplexNum operator /(const Scalar &s2) const {
    return ComplexNum(real_ / s2, imag_ / s2);
  }

  /**
   * @brief 复数 += 另一个复数
   * @param[in] c2 另一个复数
   */
  void operator +=(const ComplexNum &c2) {
    real_ += c2.real_;
    imag_ += c2.imag_;
  }

  /**
   * @brief 复数 -= 另一个复数
   * @param[in] c2 另一个复数
   */
  void operator -=(const ComplexNum &c2) {
    real_ -= c2.real_;
    imag_ -= c2.imag_;
  }

  /**
   * @brief 复数 *= 另一个复数
   * @param[in] c2 另一个复数
   */
  void operator *=(const ComplexNum &c2) {
    //return ComplexNum(real_*c2.real_ - imag_*c2.imag_,
    //                  real_*c2.imag_ + imag_*c2.real_);
    Scalar tmp = real_;
    real_ = real_*c2.real_ - imag_*c2.imag_;
    imag_ = tmp*c2.imag_ + imag_*c2.real_;
  }

  /**
   * @brief 复数 /= 另一个复数
   * @param[in] c2 另一个复数
   */
  void operator /=(const ComplexNum &c2) {
    Scalar c2_norm = c2.real_*c2.real_ + c2.imag_*c2.imag_;
    Scalar tmp = real_;
    real_ = (real_*c2.real_ + imag_*c2.imag_) / c2_norm;
    imag_ = (-tmp*c2.imag_ + imag_*c2.real_) / c2_norm;
  }

  /**
   * @brief 复数 += 实数
   * @param[in] s2 实数
   */
  void operator +=(const Scalar &s2) {
    real_ += s2;
  }

  /**
   * @brief 复数 -= 实数
   * @param[in] s2 实数
   */
  void operator -=(const Scalar &s2) {
    real_ -= s2;
  }

  /**
   * @brief 复数 *= 实数
   * @param[in] s2 实数
   */
  void operator *=(const Scalar &s2) {
    real_ *= s2;
    imag_ *= s2;
  }

  /**
   * @brief 复数 /= 实数
   * @param[in] s2 实数
   */
  void operator /=(const Scalar &s2) {
    real_ /= s2;
    imag_ /= s2;
  }

  /**
   * @brief 获取复数的实数部分
   * @return 复数的实数部分
   */
  inline const Scalar& real() const { return real_; }

  /**
   * @brief 获取复数的虚数部分
   * @return 复数的虚数部分
   */
  inline const Scalar& imag() const { return imag_; }

  /**
   * @brief 获取复数的实数部分
   * @return 复数的实数部分
   */
  inline Scalar& real() { return real_; }

  /**
   * @brief 获取复数的虚数部分
   * @return 复数的虚数部分
   */
  inline Scalar& imag() { return imag_; }

private:
  // 复数的实数部分
  Scalar real_;
  // 复数的虚数部分
  Scalar imag_;
};

/**
 * @class ComplexMatrix
 * @brief 定义了Complex矩阵的存储结构(列主序矩阵)
 * @param Scalar 矩阵中元素的数据类型
 * @param Rows 最大行数
 * @param Cols 最大列数
 */
template<typename Scalar, Int32_t MaxRows, Int32_t MaxCols>
class ComplexMatrix {
public:
  enum {
    RowsAtCompileTime = MaxRows,
    ColsAtCompileTime = MaxCols
  };
  typedef Scalar value_type;
  typedef ComplexNum<Scalar> ComplexScalar;

  /**
   * @struct Block
   * @brief 矩阵分块信息
   */
  struct Block {
    /// 分块矩阵的行数
    Int32_t rows;
    /// 矩阵分块的列数
    Int32_t cols;
    /// 分块矩阵在原始矩阵中起始行号(从0开始)
    Int32_t start_row;
    /// 分块矩阵在原始矩阵的起始列号(从0开始)
    Int32_t start_col;

    /**
     * @brief 构造函数
     */
    Block() {
      rows = MaxRows;
      cols = MaxCols;
      start_row = 0;
      start_col = 0;
    }
  };

public:
  /**
   * @brief 构造函数
   */
  ComplexMatrix() {
  }

  /**
   * @brief 拷贝构造函数
   * @param[in] other 其它矩阵
   */
  ComplexMatrix(const ComplexMatrix& other) {
    block_ = other.block_;
    com_memcpy(storage_, other.storage_, sizeof(storage_));
  }

  /**
   * @brief 构造函数(1维向量形式)
   * @param[in] x 向量第一个元素的值
   */
  ComplexMatrix(const ComplexScalar& x) {
    COM_CHECK(MaxRows * MaxCols >= 1);
    storage_[0] = x;
  }

  /**
   * @brief 构造函数(2维向量形式)
   * @param[in] x 向量第一个元素的值
   * @param[in] y 向量第二个元素的值
   */
  ComplexMatrix(const ComplexScalar& x, const ComplexScalar& y) {
    COM_CHECK(MaxRows*MaxCols >= 2);
    storage_[0] = x;
    storage_[1] = y;
  }

  /**
   * @brief 构造函数(3维向量形式)
   * @param[in] x 向量第一个元素的值
   * @param[in] y 向量第二个元素的值
   * @param[in] z 向量第三个元素的值
   */
  ComplexMatrix(const ComplexScalar& x, const ComplexScalar& y,
                const ComplexScalar& z) {
    COM_CHECK(MaxRows*MaxCols >= 3);
    storage_[0] = x;
    storage_[1] = y;
    storage_[2] = z;
  }

  /**
   * @brief 构造函数(4维向量形式)
   * @param[in] x 向量第一个元素的值
   * @param[in] y 向量第二个元素的值
   * @param[in] z 向量第三个元素的值
   * @param[in] w 向量第四个元素的值
   */
  ComplexMatrix(const ComplexScalar& x, const ComplexScalar& y,
                const ComplexScalar& z, const ComplexScalar& w) {
    COM_CHECK(MaxRows*MaxCols >= 4);
    storage_[0] = x;
    storage_[1] = y;
    storage_[2] = z;
    storage_[3] = w;
  }

  /**
   * @brief 拷贝函数
   * @param[in] other 其它矩阵
   */
  ComplexMatrix& operator =(const ComplexMatrix& other) {
    block_ = other.block_;
    com_memcpy(storage_, other.storage_, sizeof(storage_));
    return (*this);
  }

  /**
   * @brief 获取矩阵分块信息
   * @return 矩阵分块信息
   */
  const Block& GetBlock() const {
    return (block_);
  }

  /**
   * @brief 设置矩阵分块信息
   * @param[in] block 矩阵分块信息
   */
  void SetBlock(const Block& block) {
    COM_CHECK((block.start_row + block.rows) <= MaxRows);
    COM_CHECK((block.start_col + block.cols) <= MaxCols);

    block_ = block;
  }

  /**
   * @brief 设置矩阵分块信息
   * @param[in] start_row 分块矩阵的起始行号(从0开始)
   * @param[in] start_col 分块矩阵的起始列号(从0开始)
   * @param[in] rows 分块矩阵的行数
   * @param[in] cols 分块矩阵的列数
   */
  void SetBlock(Int32_t start_row, Int32_t start_col,
                Int32_t rows, Int32_t cols) {
    COM_CHECK((start_row >= 0) && (start_col >=0) && (rows > 0) && (cols > 0));
    COM_CHECK((start_row + rows) <= MaxRows);
    COM_CHECK((start_col + cols) <= MaxCols);

    block_.start_row = start_row;
    block_.start_col = start_col;
    block_.rows = rows;
    block_.cols = cols;
  }

  /**
   * @brief 设置矩阵分块信息
   * @param[in] rows 分块矩阵的行数
   * @param[in] cols 分块矩阵的列数
   */
  void SetBlockSize(Int32_t rows, Int32_t cols) {
    COM_CHECK((rows > 0) && (cols > 0));
    COM_CHECK((block_.start_row + rows) <= MaxRows);
    COM_CHECK((block_.start_col + cols) <= MaxCols);

    block_.rows = rows;
    block_.cols = cols;
  }

  /**
   * @brief 获取当前矩阵的行数
   * @return 当前矩阵的行数
   */
  inline Int32_t rows() const {
    return (block_.rows);
  }

  /**
   * @brief 获取当前矩阵的列数
   * @return 当前矩阵的列数
   */
  inline Int32_t cols() const {
    return (block_.cols);
  }

  /**
   * @brief 以向量索引的方式访问矩阵元素 \n
   *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
   * @param[in] index 矩阵元素索引(列主序)
   * @return 矩阵元素索引所指向的数据
   */
  inline const ComplexScalar& operator [](Int32_t index) const {
    COM_CHECK(index >= 0);

    Int32_t real_index =
        GetStorageIndex(index % block_.rows, index / block_.rows);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 以向量索引的方式访问矩阵元素 \n
   *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
   * @param[in] index 矩阵元素索引(列主序)
   * @return 矩阵元素索引所指向的数据
   */
  inline const ComplexScalar& operator ()(Int32_t index) const {
    COM_CHECK(index >= 0);

    Int32_t real_index =
        GetStorageIndex(index % block_.rows, index / block_.rows);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 以矩阵行、列索引的形式访问矩阵元素
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 矩阵元素索引所指向的数据
   */
  inline const ComplexScalar& operator ()(Int32_t row, Int32_t col) const {
    COM_CHECK((row >= 0) && (col >= 0));

    Int32_t real_index = GetStorageIndex(row, col);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 以向量索引的方式访问矩阵元素 \n
   *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
   * @param[in] index 矩阵元素索引(列主序)
   * @return 矩阵元素索引所指向的数据
   */
  inline ComplexScalar& operator [](Int32_t index) {
    COM_CHECK(index >= 0);

    Int32_t real_index =
        GetStorageIndex(index % block_.rows, index / block_.rows);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 以向量索引的方式访问矩阵元素 \n
   *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
   * @param[in] index 矩阵元素索引(列主序)
   * @return 矩阵元素索引所指向的数据
   */
  inline ComplexScalar& operator ()(Int32_t index) {
    COM_CHECK(index >= 0);

    Int32_t real_index =
        GetStorageIndex(index % block_.rows, index / block_.rows);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 以矩阵行、列索引的形式访问矩阵元素
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 矩阵元素索引所指向的数据
   */
  inline ComplexScalar& operator ()(Int32_t row, Int32_t col) {
    COM_CHECK((row >= 0) && (col >= 0));
    Int32_t real_index = GetStorageIndex(row, col);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 将矩阵在内部转置
   * @warning {不会改变矩阵的存储结构，但必须留有足够的行和列 \n
   *           来进行转置操作，否则将导致严重错误} 
   */
  void TransposeInPlace() {
    COM_CHECK((block_.start_row + block_.cols) <= MaxRows);
    COM_CHECK((block_.start_col + block_.rows) <= MaxCols);

    if (block_.cols > block_.rows) {
      for (Int32_t j = 1; j < block_.rows; ++j) {
        for (Int32_t i = 0; i < j; ++i) {
          ComplexScalar tmp = GetData(i, j);
          GetData(i, j) = GetData(j, i);
          GetData(j, i) = tmp;
        }
      }
      for (Int32_t j = block_.rows; j < block_.cols; ++j) {
        for (Int32_t i = 0; i < block_.rows; ++i) {
          GetData(j, i) = GetData(i, j);
        }
      }
    } else if (block_.cols < block_.rows) {
      for (Int32_t j = 1; j < block_.cols; ++j) {
        for (Int32_t i = 0; i < j; ++i) {
          ComplexScalar tmp = GetData(i, j);
          GetData(i, j) = GetData(j, i);
          GetData(j, i) = tmp;
        }
      }
      for (Int32_t j = 0; j < block_.cols; ++j) {
        for (Int32_t i = block_.cols; i < block_.rows; ++i) {
          GetData(j, i) = GetData(i, j);
        }
      }
    } else {
      for (Int32_t j = 1; j < block_.rows; ++j) {
        for (Int32_t i = 0; i < j; ++i) {
          ComplexScalar tmp = GetData(i, j);
          GetData(i, j) = GetData(j, i);
          GetData(j, i) = tmp;
        }
      }
    }

    Int32_t tmp = block_.rows;
    block_.rows = block_.cols;
    block_.cols = tmp;
  }


protected:
  /*
   * @brief 获取矩阵元素的真实的存储索引
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 矩阵元素的真实的存储索引
   */
  inline Int32_t GetStorageIndex(Int32_t row, Int32_t col) const {
    return ((block_.start_col + col) * MaxRows + block_.start_row + row);
  }

  /*
   * @brief 获取存储矩阵元素的地址的引用
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 存储矩阵元素的地址的引用
   */
  inline ComplexScalar& GetData(Int32_t row, Int32_t col) {
    return (storage_[GetStorageIndex(row, col)]);
  }

  /*
   * @brief 获取存储矩阵元素的地址的引用
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 存储矩阵元素的地址的引用
   */
  inline const ComplexScalar& GetData(Int32_t row, Int32_t col) const {
    return (storage_[GetStorageIndex(row, col)]);
  }

protected:
  // 矩阵分块信息
  Block block_;
  // 存储矩阵元素的数组
  ComplexScalar storage_[MaxRows*MaxCols];
};

/**
 * @brief 将复数矩阵内部数据输出到字符流
 * @param[in] s 字符流
 * @param[in] mat 需要输出的复数矩阵信息
 * @param[in] precision 需要输出的数值的精度
 * @param[in] align_cols 是否按列对齐
 */
template<typename T, Int32_t Rows, Int32_t Cols>
std::ostream& PrintComplexMatrix(
    std::ostream& s, const ComplexMatrix<T, Rows, Cols>& mat,
    Int32_t precision = 0, bool align_cols = true) {
  Int32_t width = 0;

  std::streamsize explicit_precision;
  if(0 == precision) {
    explicit_precision = 0;
  } else {
    explicit_precision = precision;
  }

  std::streamsize old_precision = 0;
  if(explicit_precision) {
    old_precision = s.precision(explicit_precision);
  }

  if(align_cols) {
    // compute the largest width
    for(Int32_t j = 0; j < mat.cols(); ++j) {
      for(Int32_t i = 0; i < mat.rows(); ++i) {
        std::stringstream sstr;
        sstr.copyfmt(s);
        //sstr << "(" << mat(i, j).real() << ", " << mat(i, j).imag() << ")";
        sstr << mat(i, j).real();
        width = Max<Int32_t>(width, Int32_t(sstr.str().length()));
        sstr.clear();
        sstr << mat(i, j).imag();
        width = Max<Int32_t>(width, Int32_t(sstr.str().length()));
      }
    }
  }

  for(Int32_t i = 0; i < mat.rows(); ++i) {
    // s << fmt.rowPrefix;
#if 0
    s << " (";
    if(width) {
      s.width(width);
    }
    s << mat(i, 0).real();
    s << ", ";
    //if(width) {
    //  s.width(width);
    //}
    s << mat(i, 0).imag() << ") ";
#else
    s << " (" << mat(i, 0).real() << ", " << mat(i, 0).imag() << ") ";
#endif
    for(Int32_t j = 1; j < mat.cols(); ++j) {
#if 0
      s << "(";
      if(width) {
        s.width(width);
      }
      s << mat(i, j).real();
      s << ", ";
      //if(width) {
      //  s.width(width);
      //}
      s << mat(i, j).imag() << ") ";
#else
      s << "(" << mat(i, j).real() << ", " << mat(i, j).imag() << ") ";
#endif
    }
    // s << fmt.rowSeparator;
    if(i < (mat.rows() - 1)) {
      s << '\n';
    }
  }

  if(explicit_precision) {
    s.precision(old_precision);
  }

  return s;
}

/**
 * @brief 对<<的重载，用来将复数矩阵内部数据输出到字符流
 * @param[in] s 字符流
 * @param[in] mat 需要输出的复数矩阵信息
 */
template<typename T, Int32_t Rows, Int32_t Cols>
std::ostream& operator << (std::ostream& s,
                           const ComplexMatrix<T, Rows, Cols>& mat) {
  return PrintComplexMatrix(s, mat);
}

/**
 * @brief 求实数矩阵的特征值及特征向量(特征值及特征向量可能存在复数的形式)
 * @param[in&out] mat_coef 待特征值分解的矩阵，之后矩阵的Schur的形式
 * @param[out] mat_orth 用于内部计算的临时变量，保存矩阵的Schur的形式的正交分解向量
 * @param[out] mat_eigen_values 特征值
 * @param[out] mat_eigen_vectors 特征向量
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 * @endcode
 */
template<typename MatType1, typename MatType2,
         typename ComplexMatType1, typename ComplexMatType2>
bool Mat_CalcEigenValue(MatType1& mat_coef, MatType2& mat_orth,
                        ComplexMatType1& mat_eigen_values,
                        ComplexMatType2& mat_eigen_vectors) {
  COM_CHECK(mat_coef.rows() == mat_coef.cols() &&
            mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows() &&
            (mat_eigen_values.rows() == mat_coef.rows() ||
             mat_eigen_values.cols() == mat_coef.rows()) &&
            mat_eigen_vectors.rows() == mat_eigen_vectors.cols() &&
            mat_eigen_vectors.rows() == mat_coef.rows());

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
  std::cout << "########## Mat_CalcEigenValue ###########>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;
  typedef ComplexNum<Scalar> ComplexScalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();
  const Scalar consider_as_zero = NumLimits<Scalar>::min();

  Int32_t mat_rows = mat_coef.rows();
  Int32_t mat_cols = mat_coef.cols();

  bool ret = Mat_RealSchur(mat_coef, mat_orth);
  if (false == ret) {
    LOG_ERR << "Failed to reduce matrix to real Schur form.";
    return false;
  }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
  std::cout << "After Schur decomposition:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif

  // Compute eigenvalues
  Int32_t index = 0;
  while (index < mat_cols) {
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
    std::cout << "index=" << index << std::endl;
#endif

    if ((index == (mat_cols - 1)) ||
        (com_abs(mat_coef(index+1, index)) <= consider_as_zero)) {
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "real eigen value" << std::endl;
#endif

      // real eigen value
      mat_eigen_values(index) =
          ComplexScalar(mat_coef(index, index), Scalar(0));
      if(!com_isfinite(mat_coef(index, index))) {
        LOG_ERR << "mat_coef(" << index << "," << index << ") = "
                << mat_coef(index, index) << " is not finite number.";
        return false;
      }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "mat_eigen_values(" << index << ") = ("
                << mat_eigen_values(index).real()
                << ", " << mat_eigen_values(index).imag()
                << ")" << std::endl;
#endif

      ++index;
    } else {
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "complex eigen value" << std::endl;
#endif

      // complex eigen value
      Scalar p = Scalar(0.5) *
          (mat_coef(index, index) - mat_coef(index+1, index+1));
      Scalar z = 0;
      // Compute z =
      //     sqrt(abs(p * p + m_matT.coeff(i+1, i) * m_matT.coeff(i, i+1)));
      // without overflow
      {
        Scalar t0 = mat_coef(index+1, index);
        Scalar t1 = mat_coef(index, index+1);
        Scalar maxval = Max(com_abs(p), Max(com_abs(t0), com_abs(t1)));
        t0 /= maxval;
        t1 /= maxval;
        Scalar p0 = p / maxval;
        z = maxval * com_sqrt(com_abs(p0 * p0 + t0 * t1));
      }

      mat_eigen_values(index) =
          ComplexScalar(mat_coef(index+1, index+1) + p, z);
      mat_eigen_values(index+1) =
          ComplexScalar(mat_coef(index+1, index+1) + p, -z);
      if(!(com_isfinite(mat_coef(index+1, index+1)) &&
           com_isfinite(p) && com_isfinite(z))) {
        LOG_ERR << "mat_coef(" << index+1 << "," << index+1 << ") = "
                << mat_coef(index+1, index+1)
                << " or p = " << p << " or z = " << z
                << " is not finite number.";
        return false;
      }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "mat_eigen_values(" << index << ") = ("
                << mat_eigen_values(index).real()
                << ", " << mat_eigen_values(index).imag()
                << ")" << std::endl;
      std::cout << "mat_eigen_values(" << index+1 << ") = ("
                << mat_eigen_values(index+1).real()
                << ", " << mat_eigen_values(index+1).imag()
                << ")" << std::endl;
#endif

      index += 2;
    }
  }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
  std::cout << "The eigen values are:\n" << mat_eigen_values << std::endl;
#endif

  // Compute eigenvectors
  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      mat_eigen_vectors(i, j) = ComplexScalar(mat_orth(i, j), Scalar(0));
    }
  }

  Scalar norm(0);
  for (Int32_t i = 0; i < mat_rows; ++i) {
    Int32_t start_index = Max(i-1, 0);
    for (Int32_t j = start_index; j < mat_cols; ++j) {
      norm += com_abs(mat_coef(i, j));
    }
  }
  if (norm <= consider_as_zero) {
    return (true);
  }

  // Backsubstitute to find vectors of upper triangular form
  for (Int32_t n = (mat_cols-1); n >= 0; n--) {
    Scalar p = mat_eigen_values(n).real();
    Scalar q = mat_eigen_values(n).imag();
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
    std::cout << ">>> n=" << n << ", p=" << p << ", q=" << q << std::endl;
#endif

    if (com_abs(q) <= consider_as_zero) {
      // Scalar vector
      Scalar lastr(0), lastw(0);
      Int32_t l = n;
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "Scalar vector" << std::endl;
#endif

      mat_coef(n, n) = Scalar(1);
      for (Int32_t i = (n-1); i >= 0; i--) {
        Scalar w = mat_coef(i, i) - p;
        Scalar r = 0;
        for (Int32_t k = l; k < (n+1); ++k) {
          r += mat_coef(i, k) * mat_coef(k, n);
        }

        if (mat_eigen_values(i).imag() < Scalar(0)) {
          lastw = w;
          lastr = r;
        } else {
          l = i;
          if (com_abs(mat_eigen_values(i).imag()) <= consider_as_zero) {
            if (com_abs(w) > eps) {
              mat_coef(i, n) = -r / w;
            } else {
              mat_coef(i, n) = -r / (eps * norm);
            }
          } else {
            // Solve real equations
            Scalar x = mat_coef(i, i+1);
            Scalar y = mat_coef(i+1, i);
            Scalar denom = Square((mat_eigen_values(i).real() - p)) +
                Square(mat_eigen_values(i).imag());
            Scalar t = (x * lastr - lastw * r) / denom;
            mat_coef(i, n) = t;
            if (com_abs(x) > com_abs(lastw)) {
              mat_coef(i+1, n) = (-r - w * t) / x;
            } else {
              mat_coef(i+1, n) = (-lastr - y * t) / lastw;
            }
          }

          // Overflow control
          Scalar t = com_abs(mat_coef(i, n));
          if ((eps * t) * t > Scalar(1)) {
            //m_matT.col(n).tail(size-i) /= t;
            for (Int32_t k = i; k < mat_rows; ++k) {
              mat_coef(k, n) /= t;
            }
          }
        }
      }
    } else if ((q < Scalar(0)) && (n > 0)) {
      // Complex vector
      Scalar lastra(0), lastsa(0), lastw(0);
      Int32_t l = n-1;
#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
      std::cout << "Complex vector" << std::endl;
#endif

      // Last vector component imaginary so matrix is triangular
      if (com_abs(mat_coef(n, n-1)) > com_abs(mat_coef(n-1, n))) {
        mat_coef(n-1, n-1) = q / mat_coef(n, n-1);
        mat_coef(n-1, n) = -(mat_coef(n, n) - p) / mat_coef(n, n-1);
      } else {
        ComplexScalar cc = ComplexScalar(Scalar(0), -mat_coef(n-1, n)) /
            ComplexScalar(mat_coef(n-1, n-1)-p, q);
        mat_coef(n-1, n-1) = cc.real();
        mat_coef(n-1, n) = cc.imag();
      }
      mat_coef(n, n-1) = Scalar(0);
      mat_coef(n, n) = Scalar(1);

      for (Int32_t i = (n-2); i >= 0; i--) {
        Scalar ra = 0;
        Scalar sa = 0;
        for (Int32_t k = l; k < (n+1); ++k) {
          ra += mat_coef(i, k) * mat_coef(k, n-1);
          sa += mat_coef(i, k) * mat_coef(k, n);
        }
        Scalar w = mat_coef(i, i) - p;

        if (mat_eigen_values(i).imag() < Scalar(0)) {
          lastw = w;
          lastra = ra;
          lastsa = sa;
        } else {
          l = i;
          if (com_abs(mat_eigen_values(i).imag()) <= consider_as_zero) {
            ComplexScalar cc = ComplexScalar(-ra,-sa) / ComplexScalar(w,q);
            mat_coef(i, n-1) = cc.real();
            mat_coef(i, n) = cc.imag();
          } else {
            // Solve complex equations
            Scalar x = mat_coef(i, i+1);
            Scalar y = mat_coef(i+1, i);
            Scalar vr = Square(mat_eigen_values(i).real() - p) +
                Square(mat_eigen_values(i).imag()) - q * q;
            Scalar vi = (mat_eigen_values(i).real() - p) * Scalar(2) * q;
            if ((com_abs(vr) <= consider_as_zero) &&
                (com_abs(vi) <= consider_as_zero)) {
              vr = eps * norm *
                  (com_abs(w) + com_abs(q) +
                   com_abs(x) + com_abs(y) + com_abs(lastw));
            }

            ComplexScalar cc =
                ComplexScalar(x*lastra-lastw*ra+q*sa, x*lastsa-lastw*sa-q*ra) /
                ComplexScalar(vr, vi);
            mat_coef(i, n-1) = cc.real();
            mat_coef(i, n) = cc.imag();
            if (com_abs(x) > (com_abs(lastw) + com_abs(q))) {
              mat_coef(i+1, n-1) =
                  (-ra - w * mat_coef(i, n-1) + q * mat_coef(i, n)) / x;
              mat_coef(i+1, n) =
                  (-sa - w * mat_coef(i, n) - q * mat_coef(i, n-1)) / x;
            } else {
              cc = ComplexScalar(
                    -lastra-y*mat_coef(i, n-1), -lastsa-y*mat_coef(i, n)) /
                  ComplexScalar(lastw, q);
              mat_coef(i+1, n-1) = cc.real();
              mat_coef(i+1, n) = cc.imag();
            }
          }

          // Overflow control
          Scalar t = Max(com_abs(mat_coef(i, n-1)), com_abs(mat_coef(i, n)));
          if ((eps * t) * t > Scalar(1)) {
            //m_matT.block(i, n-1, size-i, 2) /= t;
            for (Int32_t k = i; k < mat_rows; ++k) {
              mat_coef(k, n-1) /= t;
              mat_coef(k, n) /= t;
            }
          }
        }
      }

      // We handled a pair of complex conjugate eigenvalues,
      // so need to skip them both
      n--;
    } else {
      LOG_ERR << "Detected internal error (Get INF or NaN value).";
    }
  }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
  std::cout << "After Backsubstituting to find vectors of "
               "upper triangular form:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  // Back transformation to get eigenvectors of original matrix
  for (Int32_t j = (mat_cols-1); j >= 0; j--) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      Scalar sum = 0;
      for (Int32_t k = 0; k < (j+1); ++k) {
        sum += mat_eigen_vectors(i, k).real() * mat_coef(k, j);
      }
      mat_eigen_vectors(i, j).real() = sum;
    }
  }

  const Scalar precision = Scalar(2)*NumLimits<Scalar>::epsilon();
  for (Int32_t j = 0; j < mat_cols; ++j) {
    if ((com_abs(mat_eigen_values(j).imag()) <=
         (com_abs(mat_eigen_values(j).real()) * precision)) ||
        ((j+1) == mat_cols)) {
      // we have a real eigen value
      Scalar vec_norm = 0;
      for (Int32_t i = 0; i < mat_rows; ++i) {
        vec_norm += Square(mat_eigen_vectors(i, j).real());
      }
      vec_norm = com_sqrt(vec_norm);
      for (Int32_t i = 0; i < mat_rows; ++i) {
        mat_eigen_vectors(i, j).real() /= vec_norm;
      }
    } else {
      // we have a pair of complex eigen values
      for (Int32_t i = 0; i < mat_rows; ++i) {
        ComplexScalar tmp = mat_eigen_vectors(i, j);
        mat_eigen_vectors(i, j)   =
            ComplexScalar(tmp.real(),  mat_eigen_vectors(i, j+1).real());
        mat_eigen_vectors(i, j+1) =
            ComplexScalar(tmp.real(), -mat_eigen_vectors(i, j+1).real());
      }
      Scalar vec_norm = 0;
      for (Int32_t i = 0; i < mat_rows; ++i) {
        vec_norm += Square(mat_eigen_vectors(i, j).real()) +
            Square(mat_eigen_vectors(i, j).imag());
      }
      vec_norm = com_sqrt(vec_norm);
      for (Int32_t i = 0; i < mat_rows; ++i) {
        mat_eigen_vectors(i, j) /= vec_norm;
        mat_eigen_vectors(i, j+1) /= vec_norm;
      }

      ++j;
    }
  }

#if ENABLE_MATH_COMPLEX_MATRIX_TRACE
  std::cout << "mat_eigen_vectors=\n" << mat_eigen_vectors << std::endl;
  std::cout << "<######### Mat_CalcEigenValue ############" << std::endl;
#endif

  return true;
}


} // common
} // phoenix


#endif  // PHOENIX_COMMON_MATH_COMPLEX_MATRIX_H_

