/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       matrix.h
 * @brief      矩阵运算
 * @details    定义相关矩阵运算算法
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

#ifndef PHOENIX_COMMON_MATH_MATRIX_H_
#define PHOENIX_COMMON_MATH_MATRIX_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"

namespace phoenix {
namespace common {


// Enable to output debug information
#define ENABLE_MATH_MATRIX_TRACE (0)


/**
 * @class Matrix
 * @brief 定义了矩阵的存储结构(列主序矩阵)
 * @param Scalar 矩阵中元素的数据类型
 * @param Rows 最大行数
 * @param Cols 最大列数
 */
template<typename Scalar, Int32_t MaxRows, Int32_t MaxCols>
class Matrix {
public:
  enum {
    RowsAtCompileTime = MaxRows,
    ColsAtCompileTime = MaxCols
  };
  typedef Scalar value_type;

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

  /**
   * @class CommaInitializer
   * @brief Helper class used by the comma initializer operator
   * @details This class is internally used to implement the      \n
   *          comma initializer feature. It is the return type of \n
   *          Matrix::operator<<, and most of the time this is    \n
   *          the only way it is used.
   */
  struct CommaInitializer {
    /**
     * @brief 构造函数
     * @param[in] xpr 需要赋值的矩阵的引用
     * @param[in] s 需要赋值给矩阵的数值
     */
    inline CommaInitializer(Matrix& xpr, const Scalar& s)
      : xpr_(xpr), row_(0), col_(1) {
      xpr_(0, 0) = s;
    }

    /**
     * @brief 拷贝构造函数
     * @param[in] o 另一个CommaInitializer的实例
     */
    inline CommaInitializer(const CommaInitializer& o)
      : xpr_(o.xpr_), row_(o.row_), col_(o.col_) {
      // Mark original object as finished.
      // In absence of R-value references we need to const_cast:
      const_cast<CommaInitializer&>(o).row_ = xpr_.rows();
      const_cast<CommaInitializer&>(o).col_ = xpr_.cols();
    }

    /**
     * @brief 对逗号操作符的重载
     * @param[in] s 需要赋值给矩阵的数值
     */
    CommaInitializer& operator ,(const Scalar& s) {
      if (col_ >= xpr_.cols()) {
        row_++;
        col_ = 0;
        COM_CHECK(row_ < xpr_.rows());
      }
      COM_CHECK(col_ < xpr_.cols());
      xpr_(row_, col_) = s;
      col_++;
      return *this;
    }

    // target expression
    Matrix& xpr_;
    // current row id
    Int32_t row_;
    // current col id
    Int32_t col_;
  };

public:
  /**
   * @brief 构造函数
   */
  Matrix() {
  }

  /**
   * @brief 拷贝构造函数
   * @param[in] other 其它矩阵
   */
  Matrix(const Matrix& other) {
    block_ = other.block_;
    com_memcpy(storage_, other.storage_, sizeof(storage_));
  }

  /**
   * @brief 构造函数(1维向量形式)
   * @param[in] x 向量第一个元素的值
   */
  Matrix(const Scalar& x) {
    COM_CHECK(MaxRows * MaxCols >= 1);
    storage_[0] = x;
  }

  /**
   * @brief 构造函数(2维向量形式)
   * @param[in] x 向量第一个元素的值
   * @param[in] y 向量第二个元素的值
   */
  Matrix(const Scalar& x, const Scalar& y) {
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
  Matrix(const Scalar& x, const Scalar& y, const Scalar& z) {
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
  Matrix(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w) {
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
  Matrix& operator =(const Matrix& other) {
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
  inline const Scalar& operator [](Int32_t index) const {
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
  inline const Scalar& operator ()(Int32_t index) const {
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
  inline const Scalar& operator ()(Int32_t row, Int32_t col) const {
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
  inline Scalar& operator [](Int32_t index) {
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
  inline Scalar& operator ()(Int32_t index) {
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
  inline Scalar& operator ()(Int32_t row, Int32_t col) {
    COM_CHECK((row >= 0) && (col >= 0));
    Int32_t real_index = GetStorageIndex(row, col);
    COM_CHECK(real_index < MaxRows*MaxCols);
    return storage_[real_index];
  }

  /**
   * @brief 将矩阵所有元素设置为0
   */
  void SetZeros() {
    for (Int32_t j = 0; j < block_.cols; ++j) {
      for (Int32_t i = 0; i < block_.rows; ++i) {
        Int32_t real_index = GetStorageIndex(i, j);
        storage_[real_index] = 0;
      }
    }
  }

  /**
   * @brief 将矩阵所有元素设置为1
   */
  void SetOnes() {
    for (Int32_t j = 0; j < block_.cols; ++j) {
      for (Int32_t i = 0; i < block_.rows; ++i) {
        Int32_t real_index = GetStorageIndex(i, j);
        storage_[real_index] = 1;
      }
    }
  }

  /**
   * @brief 将矩阵设置为单位矩阵
   */
  void SetIdentity() {
    for (Int32_t j = 0; j < block_.cols; ++j) {
      for (Int32_t i = 0; i < block_.rows; ++i) {
        Int32_t real_index = GetStorageIndex(i, j);
        if (i == j) {
          storage_[real_index] = 1;
        } else {
          storage_[real_index] = 0;
        }
      }
    }
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
          Scalar tmp = GetData(i, j);
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
          Scalar tmp = GetData(i, j);
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
          Scalar tmp = GetData(i, j);
          GetData(i, j) = GetData(j, i);
          GetData(j, i) = tmp;
        }
      }
    }

    Int32_t tmp = block_.rows;
    block_.rows = block_.cols;
    block_.cols = tmp;
  }

  /**
   * @brief 找出绝对值最大的元素
   * @return 绝对值最大的元素的值
   */
  Scalar FindMaxAbsCoeff() const {
    Scalar max_abs_coeff = com_abs(GetData(0, 0));
    for (Int32_t j = 0; j < block_.cols; ++j) {
      for (Int32_t i = 0; i < block_.rows; ++i) {
        Scalar abs_coeff = com_abs(GetData(i, j));
        if (abs_coeff > max_abs_coeff) {
          max_abs_coeff = abs_coeff;
        }
      }
    }

    return (max_abs_coeff);
  }

  /**
   * @brief 找出绝对值最大的对角线上的元素
   * @return 绝对值最大的对角线上的元素的值
   */
  Scalar FindMaxAbsDiagCoeff() const {
    Int32_t diag_size = Min(block_.rows, block_.cols);
    Scalar max_abs_coeff = com_abs(GetData(0, 0));

    for (Int32_t i = 0; i < diag_size; ++i) {
      Scalar abs_coeff = com_abs(GetData(i, i));
      if (abs_coeff > max_abs_coeff) {
        max_abs_coeff = abs_coeff;
      }
    }

    return (max_abs_coeff);
  }

  /**
   * @brief 对<<的重载，用来给矩阵赋值
   * @param[in] s 需要赋值给矩阵的数值
   * @return 保存矩阵信息及访问索引信息的CommaInitializer类的实例
   */
  inline CommaInitializer operator << (const Scalar& s) {
    return CommaInitializer(*this, s);
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
  inline Scalar& GetData(Int32_t row, Int32_t col) {
    return (storage_[GetStorageIndex(row, col)]);
  }

  /*
   * @brief 获取存储矩阵元素的地址的引用
   * @param[in] row 行号
   * @param[in] col 列号
   * @return 存储矩阵元素的地址的引用
   */
  inline const Scalar& GetData(Int32_t row, Int32_t col) const {
    return (storage_[GetStorageIndex(row, col)]);
  }

protected:
  // 矩阵分块信息
  Block block_;
  // 存储矩阵元素的数组
  Scalar storage_[MaxRows*MaxCols];
};


/// 定义2维列向量
typedef Matrix<Float32_t, 2, 1> Vector2f;
/// 定义2维行向量
typedef Matrix<Float32_t, 1, 2> RowVector2f;
/// 定义2*2矩阵
typedef Matrix<Float32_t, 2, 2> Matrix2f;
/// 定义3维列向量
typedef Matrix<Float32_t, 3, 1> Vector3f;
/// 定义3维行向量
typedef Matrix<Float32_t, 1, 3> RowVector3f;
/// 定义3*3矩阵
typedef Matrix<Float32_t, 3, 3> Matrix3f;

/// 定义2维列向量
typedef Matrix<Float64_t, 2, 1> Vector2d;
/// 定义2维行向量
typedef Matrix<Float64_t, 1, 2> RowVector2d;
/// 定义2*2矩阵
typedef Matrix<Float64_t, 2, 2> Matrix2d;
/// 定义3维列向量
typedef Matrix<Float64_t, 3, 1> Vector3d;
/// 定义3维行向量
typedef Matrix<Float64_t, 1, 3> RowVector3d;
/// 定义3*3矩阵
typedef Matrix<Float64_t, 3, 3> Matrix3d;


/**
 * @brief 将矩阵内部数据输出到字符流
 * @param[in] s 字符流
 * @param[in] mat 需要输出的矩阵信息
 * @param[in] precision 需要输出的数值的精度
 * @param[in] align_cols 是否按列对齐
 */
template<typename T, Int32_t Rows, Int32_t Cols>
std::ostream& PrintMatrix(
    std::ostream& s, const Matrix<T, Rows, Cols>& mat,
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
        sstr << mat(i, j);
        width = Max<Int32_t>(width, Int32_t(sstr.str().length()));
      }
    }
  }

  for(Int32_t i = 0; i < mat.rows(); ++i) {
    // s << fmt.rowPrefix;
    s << "  ";
    if(width) {
      s.width(width);
    }

    s << mat(i, 0);
    for(Int32_t j = 1; j < mat.cols(); ++j) {
      s << " ";
      if (width) {
        s.width(width);
      }
      s << mat(i, j);
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
 * @brief 对<<的重载，用来将矩阵内部数据输出到字符流
 * @param[in] s 字符流
 * @param[in] mat 需要输出的矩阵信息
 */
template<typename T, Int32_t Rows, Int32_t Cols>
std::ostream& operator << (std::ostream& s, const Matrix<T, Rows, Cols>& mat) {
  return PrintMatrix(s, mat);
}

/**
 * @brief 将矩阵1中的数据拷贝到矩阵2中
 * @param[in] m1 矩阵1
 * @param[out] m2 矩阵2
 */
template<typename MatType1, typename MatType2>
void Mat_Copy(MatType1& m1/*in*/,
              MatType2& m2/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m2(i, j) = m1(i, j);
    }
  }
}

/**
 * @brief 改变矩阵中所有元素的符号
 * @param[in] m1 待改变符号的矩阵
 * @param[out] m2 改变符号后的矩阵
 * @par Note:
 * @code
 *     m1和m2可以指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2>
void Mat_ChangeSign(MatType1& m1/*in*/,
                    MatType2& m2/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m2(i, j) = -m1(i, j);
    }
  }
}


/**
 * @brief 标量和矩阵相乘
 * @param[in] num 标量
 * @param[in] m1 被乘矩阵
 * @param[out] m2 相乘后的矩阵
 * @par Note:
 * @code
 *     m1和m2可以指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2>
void Mat_MulNum(const typename MatType1::value_type& num/*in*/,
                MatType1& m1/*in*/,
                MatType2& m2/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m2(i, j) = m1(i, j) * num;
    }
  }
}

/**
 * @brief 矩阵除以标量
 * @param[in] num 标量
 * @param[in] m1 被除矩阵
 * @param[out] m2 除以之后的矩阵
 * @par Note:
 * @code
 *     m1和m2可以指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2>
void Mat_DivNum(const typename MatType1::value_type& num/*in*/,
                MatType1& m1/*in*/,
                MatType2& m2/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m2(i, j) = m1(i, j) / num;
    }
  }
}

/**
 * @brief 两个矩阵相乘
 * @param[in] m1 矩阵乘法中左边的矩阵
 * @param[in] m2 矩阵乘法中右边的矩阵
 * @param[out] m3 矩阵相乘后的结果
 * @par Note:
 * @code
 *     m3不能和m1或m2指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
void Mat_Mul(const MatType1& m1/*in*/,
             const MatType2& m2/*in*/,
             MatType3& m3/*out*/) {
  // std::cout << "m1.rows()=" << m1.rows()
  //           << ", m1.cols()=" << m1.cols()
  //           << ", m2.rows()=" << m2.rows()
  //           << ", m2.cols()=" << m2.cols()
  //           << ", m3.rows()=" << m3.rows()
  //           << ", m3.cols()=" << m3.cols()
  //           << std::endl;

  COM_CHECK(m1.cols() == m2.rows() &&
            m1.rows() == m3.rows() &&
            m2.cols() == m3.cols());

  Int32_t m1_rows = m1.rows();
  Int32_t m1_cols = m1.cols();
  Int32_t m2_cols = m2.cols();

  Int32_t row = 0;
  Int32_t col = 0;

  for (col = 0; col < m2_cols; col++) {
    for (row = 0; row < m1_rows; row++) {
      m3(row, col) = 0;
      for (Int32_t i = 0; i < m1_cols; ++i) {
        m3(row, col) += m1(row, i) * m2(i, col);
      }
    }
  }
}

/**
 * @brief 矩阵转置
 * @param[in] m1 需要转置的矩阵
 * @param[out] m2 转置后的矩阵
 * @par Note:
 * @code
 *     m2不能和m1指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2>
void Mat_Transpose(const MatType1& m1/*in*/,
                   MatType2& m2/*out*/) {
  COM_CHECK(m1.cols() == m2.rows() && m1.rows() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m2(j, i) = m1(i, j);
    }
  }
}

/**
 * @brief 两个矩阵相加
 * @param[in] m1 矩阵加法中左边的矩阵
 * @param[in] m2 矩阵加法中右边的矩阵
 * @param[out] m3 矩阵相加后的结果
 * @par Note:
 * @code
 *     m3可以和m1或m2指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
void Mat_Add(MatType1& m1/*in*/,
             MatType2& m2/*in*/,
             MatType3& m3/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m3(i, j) = m1(i, j) + m2(i, j);
    }
  }
}

/**
 * @brief 两个矩阵相减
 * @param[in] m1 矩阵减法中左边的矩阵
 * @param[in] m2 矩阵减法中右边的矩阵
 * @param[out] m3 矩阵相减后的结果
 * @par Note:
 * @code
 *     m3可以和m1或m2指向同一个矩阵
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
void Mat_Sub(MatType1& m1/*in*/,
             MatType2& m2/*in*/,
             MatType3& m3/*out*/) {
  COM_CHECK(m1.rows() == m2.rows() && m1.cols() == m2.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  for (Int32_t j = 0; j < cols; ++j) {
    for (Int32_t i = 0; i < rows; ++i) {
      m3(i, j) = m1(i, j) - m2(i, j);
    }
  }
}

/**
 * @brief 将矩阵按列重新排序
 * @param[in] m1 待排序的矩阵
 * @param[in] vec_col_pert 排序信息(保存了需要进行列交换的信息)
 */
template<typename MatType1, typename MatType2>
void Mat_PerturbCols(MatType1& m1, MatType2& vec_col_pert) {
  COM_CHECK(m1.cols() == vec_col_pert.rows() ||
            m1.cols() == vec_col_pert.cols());

  Int32_t rows = m1.rows();
  Int32_t cols = m1.cols();
  Int32_t real_col = 0;
  typename MatType1::value_type tmp = 0;
  for (Int32_t j = 0; j < cols; ++j) {
    if (vec_col_pert(j) != j) {
      real_col = vec_col_pert(j);
      for (Int32_t i = 0; i < rows; ++i) {
        tmp = m1(i, j);
        m1(i, j) = m1(i, real_col);
        m1(i, real_col) = tmp;
      }
    }
  }
}


} // common
} // phoenix

#endif // PHOENIX_COMMON_MATH_MATRIX_H_
