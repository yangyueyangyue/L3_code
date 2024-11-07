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


#if USING_STD_IO
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
#endif

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

/**
 * @brief 使用高斯消元法解线性方程组(Partial Pivoting)
 * @param[in] mat_coef 线性方程组的系数矩阵
 * @param[in] vec_b 线性方程组等号右边的向量
 * @param[out] vec_x 线性方程组求解的结果
 * @return true - 成功, false - 失败
 * @par Note:
 * @code
 *     输入的系数矩阵和向量在运算的过程中会被修改，所以如果必要的，他们需要在运算前做好备份。
 *     The approximate number of multiplications/divisions required to reduce
 *     an n × n matrix to an upper-triangular form is n^3/3.
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
bool Mat_ColPivGaussElimination(MatType1& mat_coef,
                                MatType2& vec_b,
                                MatType3& vec_x) {
  COM_CHECK(mat_coef.cols() == mat_coef.rows() &&
            vec_b.rows() == mat_coef.rows() &&
            vec_x.rows() == mat_coef.rows());

  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();

  const Int32_t dim = mat_coef.rows();
#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########### ColPivGaussElimination ############>" << std::endl;
  std::cout << "The inputted augmented matrix is:" << std::endl;
  std::cout << "    ";
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << "[" << i << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < dim; ++j) {
      std::cout << mat_coef(i, j) << "\t";
    }
    std::cout << vec_b(i);
    std::cout << std::endl;
  }
#endif

  Int32_t row_index_list[MatType1::ColsAtCompileTime];
  for (Int32_t i = 0; i < dim; ++i) {
    row_index_list[i] = i;
  }

  for (Int32_t index = 0; index < dim-1; ++index) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << std::endl << "*****Loop " << index << " :" << std::endl;
#endif

    // Search the positions on and below the pivotal position for
    // the coefficient of maximum magnitude.
    Scalar max_abs_coef = com_abs(mat_coef(row_index_list[index], index));
    Int32_t max_abs_coef_row = index;
    for (Int32_t i = index; i < dim; ++i) {
      Scalar cur_abs_coef = com_abs(mat_coef(row_index_list[i], index));
      if (cur_abs_coef > max_abs_coef) {
        max_abs_coef = cur_abs_coef;
        max_abs_coef_row = i;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "The max absolute value is " << max_abs_coef
              << " in [" << max_abs_coef_row
              << ", " << index
              << "]." << std::endl;
#endif

    // If necessary perform the appropriate row interchange to bring this
    // maximal coefficient into the pivotal position.
    if (index != max_abs_coef_row) {
      Int32_t tmp = row_index_list[max_abs_coef_row];
      row_index_list[max_abs_coef_row] = row_index_list[index];
      row_index_list[index] = tmp;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After changing pivotal position:" << std::endl;
    std::cout << "The augmented matrix is:" << std::endl;
    std::cout << "    ";
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "]\t";
    }
    std::cout << std::endl;
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "] ";
      for (Int32_t j = 0; j < dim; ++j) {
        std::cout << mat_coef(row_index_list[i], j) << "\t";
      }
      std::cout << vec_b(row_index_list[i]);
      std::cout << std::endl;
    }
#endif

    // WARNING: If the max absolute value in the pivotal position is zero,
    // then this system may have not solution.
    if (max_abs_coef <= eps) {
      LOG_WARN << "The max absolute value in the pivotal position is zero.";
      continue;
    }

    // Eliminate all terms below the this pivot
    for (Int32_t i = index+1; i < dim; ++i) {
      Scalar eli_value = mat_coef(row_index_list[i], index)
          / mat_coef(row_index_list[index], index);
      for (Int32_t j = index; j < dim; ++j) {
        mat_coef(row_index_list[i], j) -= eli_value
            * mat_coef(row_index_list[index], j);
      }
      vec_b(row_index_list[i]) -= eli_value
          * vec_b(row_index_list[index]);
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After eliminating the elements below row[" << index
              << "]:" << std::endl;
    std::cout << "The augmented matrix is:" << std::endl;
    std::cout << "    ";
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "]\t";
    }
    std::cout << std::endl;
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "] ";
      for (Int32_t j = 0; j < dim; ++j) {
        std::cout << mat_coef(row_index_list[i], j) << "\t";
      }
      std::cout << vec_b(row_index_list[i]);
      std::cout << std::endl;
    }
#endif
  }

  // At this point, we say that the system has been triangularized.
  // Solve the last equation for the value of the last unknown.
  bool ret = true;
  if (com_abs(mat_coef(row_index_list[dim-1], dim-1)) > eps) {
    vec_x(dim-1) = vec_b(row_index_list[dim-1])
        / mat_coef(row_index_list[dim-1], dim-1);
  } else {
    LOG_WARN << "This system may have not unique solution.";
    vec_x(dim-1) = 0;
    if (com_abs(vec_b(row_index_list[dim-1])) > eps) {
      // This system have not solution.
      LOG_ERR << "This system have not solution.";
      ret = false;
    }
  }
  // Determine all of the unknowns using back substitution.
  for (Int32_t i = dim-1; i > 0; --i) {
    Scalar value = vec_b(row_index_list[i-1]);
    for (Int32_t j = dim-1; j > i-1; --j) {
      value -= mat_coef(row_index_list[i-1], j) * vec_x(j);
    }
    if (com_abs(mat_coef(row_index_list[i-1], i-1)) > eps) {
      vec_x(i-1) = value / mat_coef(row_index_list[i-1], i-1);
    } else {
      LOG_WARN << "This system may have not unique solution.";
      vec_x(i-1) = 0;
      if (com_abs(value) > eps) {
        // This system have not solution.
        LOG_ERR << "This system have not solution.";
        ret = false;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << std::endl << "The solution vector is: " << std::endl;
  std::cout << "   ";
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << vec_x(i) << "  ";
  }
  std::cout << std::endl;

  std::cout << "<########### ColPivGaussElimination ############" << std::endl;
#endif

  return (ret);
}


/**
 * @brief 使用高斯消元法解线性方程组(Complete Pivoting)
 * @param[in] mat_coef 线性方程组的系数矩阵
 * @param[in] vec_b 线性方程组等号右边的向量
 * @param[out] vec_x 线性方程组求解的结果
 * @return true - 成功, false - 失败
 * @par Note:
 * @code
 *     输入的系数矩阵和向量在运算的过程中会被修改，所以如果必要，他们需要在运算前做好备份。
 *     The approximate number of multiplications/divisions required to reduce
 *     an n × n matrix to an upper-triangular form is n^3/3. But when searching
 *     the pivotal position, a lots of float comparison is needed.
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
bool Mat_FullPivGaussElimination(MatType1& mat_coef,
                                 MatType2& vec_b,
                                 MatType3& vec_x) {
  COM_CHECK(mat_coef.cols() == mat_coef.rows() &&
            vec_b.rows() == mat_coef.rows() &&
            vec_x.rows() == mat_coef.rows());

  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();

  const Int32_t dim = mat_coef.rows();
#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########### FullPivGaussElimination ############>" << std::endl;
  std::cout << "The inputted augmented matrix is:" << std::endl;
  std::cout << "    ";
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << "[" << i << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < dim; ++j) {
      std::cout << mat_coef(i, j) << "\t";
    }
    std::cout << vec_b(i);
    std::cout << std::endl;
  }
#endif

  Int32_t row_index_list[MatType1::RowsAtCompileTime];
  Int32_t col_index_list[MatType1::ColsAtCompileTime];
  for (Int32_t i = 0; i < dim; ++i) {
    row_index_list[i] = i;
    col_index_list[i] = i;
  }

  for (Int32_t index = 0; index < dim-1; ++index) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << std::endl << "*****Loop " << index << " :" << std::endl;
#endif

    // Search the pivotal position together with every position in coefficient
    // matrix that is below or to the right of the pivotal position for the
    // coefficient of maximum magnitude.
    Scalar max_abs_coef = com_abs(mat_coef(row_index_list[index],
                                      col_index_list[index]));
    Int32_t max_abs_coef_col = index;
    Int32_t max_abs_coef_row = index;
    for (Int32_t j = index; j < dim; ++j) {
      for (Int32_t i = index; i < dim; ++i) {
        Scalar cur_abs_coef = com_abs(mat_coef(row_index_list[i],
                                          col_index_list[j]));
        if (cur_abs_coef > max_abs_coef) {
          max_abs_coef = cur_abs_coef;
          max_abs_coef_col = j;
          max_abs_coef_row = i;
        }
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "The max absolute value is " << max_abs_coef
              << " in [" << max_abs_coef_row
              << ", " << max_abs_coef_col
              << "]." << std::endl;
#endif

    // If necessary, perform the appropriate row and column interchanges
    // to bring the coefficient of maximum magnitude into the pivotal position.
    if (index != max_abs_coef_col) {
      Int32_t tmp = col_index_list[max_abs_coef_col];
      col_index_list[max_abs_coef_col] = col_index_list[index];
      col_index_list[index] = tmp;
    }
    if (index != max_abs_coef_row) {
      Int32_t tmp = row_index_list[max_abs_coef_row];
      row_index_list[max_abs_coef_row] = row_index_list[index];
      row_index_list[index] = tmp;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After changing pivotal position:" << std::endl;
    std::cout << "The augmented matrix is:" << std::endl;
    std::cout << "    ";
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "]\t";
    }
    std::cout << std::endl;
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "] ";
      for (Int32_t j = 0; j < dim; ++j) {
        std::cout << mat_coef(row_index_list[i], col_index_list[j]) << "\t";
      }
      std::cout << vec_b(row_index_list[i]);
      std::cout << std::endl;
    }
#endif

    // WARNING: If the max absolute value in the pivotal position is zero,
    // then this system may have not solution.
    if (max_abs_coef <= eps) {
      LOG_WARN << "The max absolute value in the pivotal position is zero.";
      continue;
    }

    // Eliminate all terms below the this pivot
    for (Int32_t i = index+1; i < dim; ++i) {
      Scalar eli_value = mat_coef(row_index_list[i], col_index_list[index])
          / mat_coef(row_index_list[index], col_index_list[index]);
      for (Int32_t j = index; j < dim; ++j) {
        mat_coef(row_index_list[i], col_index_list[j]) -= eli_value
            * mat_coef(row_index_list[index], col_index_list[j]);
      }
      vec_b(row_index_list[i]) -= eli_value
          * vec_b(row_index_list[index]);
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After eliminating the elements below row[" << index
              << "]:" << std::endl;
    std::cout << "The augmented matrix is:" << std::endl;
    std::cout << "    ";
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "]\t";
    }
    std::cout << std::endl;
    for (Int32_t i = 0; i < dim; ++i) {
      std::cout << "[" << i << "] ";
      for (Int32_t j = 0; j < dim; ++j) {
        std::cout << mat_coef(row_index_list[i], col_index_list[j]) << "\t";
      }
      std::cout << vec_b(row_index_list[i]);
      std::cout << std::endl;
    }
#endif
  }


  // At this point, we say that the system has been triangularized.
  // Solve the last equation for the value of the last unknown.
  bool ret = true;
  if (com_abs(mat_coef(row_index_list[dim-1], col_index_list[dim-1])) > eps) {
    vec_x(col_index_list[dim-1]) = vec_b(row_index_list[dim-1])
        / mat_coef(row_index_list[dim-1], col_index_list[dim-1]);
  } else {
    LOG_WARN << "This system may have not unique solution.";
    vec_x(col_index_list[dim-1]) = 0;
    if (com_abs(vec_b(row_index_list[dim-1])) > eps) {
      // This system have not solution.
      LOG_ERR << "This system have not solution.";
      ret = false;
    }
  }
  // Determine all of the unknowns using back substitution.
  for (Int32_t i = dim-1; i > 0; --i) {
    Scalar value = vec_b(row_index_list[i-1]);
    for (Int32_t j = dim-1; j > i-1; --j) {
      value -= mat_coef(row_index_list[i-1], col_index_list[j])
          * vec_x(col_index_list[j]);
    }
    if (com_abs(mat_coef(row_index_list[i-1], col_index_list[i-1])) > eps) {
      vec_x(col_index_list[i-1]) = value
          / mat_coef(row_index_list[i-1], col_index_list[i-1]);
    } else {
      LOG_WARN << "This system may have not unique solution.";
      vec_x(col_index_list[i-1]) = 0;
      if (com_abs(value) > eps) {
        // This system have not solution.
        LOG_ERR << "This system have not solution.";
        ret = false;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << std::endl << "The solution vector is: " << std::endl;
  std::cout << "   ";
  for (Int32_t i = 0; i < dim; ++i) {
    std::cout << vec_x(i) << "  ";
  }
  std::cout << std::endl;

  std::cout << "<########### FullPivGaussElimination ############" << std::endl;
#endif

  return (ret);
}


/**
 * @brief 求矩阵的正交分解矩阵(A = Q*R, 使用Householder QR方法)
 * @param[in&out] mat_coef 系数矩阵，运算完成后它将保存分解后的上三角矩阵R
 * @param[out] mat_orth 保存分解后的单位正交矩阵的转置矩阵
 * @return true - 成功, false - 失败
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     The approximate number of multiplications/divisions required to reduce
 *     an n × n matrix to an upper-triangular form is 2n^3/3, which is greater
 *     then Gaussian elimination.
 * @endcode
 */
template<typename MatType1, typename MatType2>
bool Mat_HouseholderQR(MatType1& mat_coef,
                       MatType2& mat_orth) {
  COM_CHECK(mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows());

  typedef typename MatType1::value_type Scalar;

  const Int32_t mat_rows = mat_coef.rows();
  const Int32_t mat_cols = mat_coef.cols();
  Int32_t max_loop = (mat_rows < mat_cols) ? mat_rows : mat_cols;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########### HouseholderQR ############>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  const Scalar tol = NumLimits<Scalar>::min();
  Scalar tail_sq_norm = 0;
  Scalar c0 = 0;
  Scalar tau = 0;
  Scalar beta = 0;
  mat_orth.SetIdentity();

  for (Int32_t col = 0; col < max_loop; ++col) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << std::endl << "*****Loop " << col << " :" << std::endl;
#endif
    // Compute the euclidean norm of the columns vector in coeffient matrix
    // below the pivot position.
    c0 = mat_coef(col, col);
    tail_sq_norm = 0;
    for (Int32_t i = col+1; i < mat_rows; ++i) {
      tail_sq_norm += Square(mat_coef(i,col));
    }
    // Compute the Householder vector.
    if (tail_sq_norm <= tol) {
      tau = 0;
      beta = c0;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) = 0;
      }
    } else {
      beta = com_sqrt(c0*c0 + tail_sq_norm);
      if (c0 >= 0) {
        beta = -beta;
      }
      Scalar ratio = c0 - beta;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) /= ratio;
      }
      tau = -ratio / beta;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "The square norm of Householder vector is "
              << beta << std::endl;
    std::cout << "The Householder vector is: ";
    for (Int32_t i = 0; i < mat_rows; ++i) {
      std::cout << mat_coef(i, col) << " ";
    }
    std::cout << std::endl;
#endif

    // Update the product of elementary reflectors
    for (Int32_t j = 0; j < mat_rows; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm += mat_orth(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_orth(col, j);
      mat_orth(col, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_orth(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

    // Compute a reflector by matrix product
    mat_coef(col, col) = beta;
    for (Int32_t j = col+1; j < mat_cols; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm += mat_coef(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(col, j);
      mat_coef(col, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_coef(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction:" << std::endl;
    std::cout << "The matrix is:" << std::endl;
    std::cout << mat_coef << std::endl;

    std::cout << "The orthogonal matrix is:" << std::endl;
    std::cout << mat_orth << std::endl;
#endif
  }

  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      if (i > j) {
        mat_coef(i, j) = 0;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "<########## HouseholderQR #############" << std::endl;
#endif

  return (true);
}

/**
 * @brief 求矩阵的正交分解矩阵(AP = Q*R, 使用Householder QR \n
 *                         (Column Partial Pivoting)方法)
 * @param[in&out] mat_coef 系数矩阵，运算完成后它将保存分解后的上三角矩阵R
 * @param[out] mat_orth 保存分解后的单位正交矩阵的转置矩阵
 * @param[out] vec_col_pert 列排序信息(内部进行了列交换的操作)
 * @return true - 成功, false - 失败
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     The approximate number of multiplications/divisions required to reduce
 *     an n × n matrix to an upper-triangular form is 2n^3/3, which is greater
 *     then Gaussian elimination.
 * @endcode
 */
template<typename MatType1, typename MatType2,typename MatType3>
bool Mat_ColPivHouseholderQR(MatType1& mat_coef,
                             MatType2& mat_orth,
                             MatType3& vec_col_pert) {
  COM_CHECK(mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows() &&
            vec_col_pert.cols() == mat_coef.cols());

  typedef typename MatType1::value_type Scalar;

  const Int32_t mat_rows = mat_coef.rows();
  const Int32_t mat_cols = mat_coef.cols();
  Int32_t max_loop = (mat_rows < mat_cols) ? mat_rows : mat_cols;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########### Mat_ColPivHouseholderQR ############>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  const Scalar tol = NumLimits<Scalar>::min();
  Scalar tail_sq_norm = 0;
  Scalar c0 = 0;
  Scalar tau = 0;
  Scalar beta = 0;
  Scalar norm = 0;
  Scalar max_norm = 0;
  Int32_t max_norm_col = 0;
  mat_orth.SetIdentity();
  for (Int32_t i = 0; i < mat_cols; ++i) {
    vec_col_pert(i) = i;
  }

  for (Int32_t col = 0; col < max_loop; ++col) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << std::endl << "*****Loop " << col << " :" << std::endl;
#endif

    // std::cout << "Befor perturbation:" << std::endl;
    // std::cout << "vec_col_pert=" << vec_col_pert << std::endl;

    // first, we look up in our matrix mat_coef
    // which column has the biggest norm
    // Using 1-Norm instead of 2-Norm to speed up calculation
    max_norm_col = col;
    max_norm = 0;
    for (Int32_t i = col; i < mat_rows; ++i) {
#if 0
      max_norm += Square(mat_coef(i, col));
#else
      max_norm += com_abs(mat_coef(i, col));
#endif
    }
    for (Int32_t j = col+1; j < mat_cols; ++j) {
      norm = 0;
      for (Int32_t i = col; i < mat_rows; ++i) {
#if 0
        norm += Square(mat_coef(i, j));
#else
        norm += com_abs(mat_coef(i, j));
#endif
      }
      if (norm > max_norm) {
        max_norm = norm;
        max_norm_col = j;
      }
    }
    if (max_norm_col != col) {
      // Int32_t tmp = vec_col_pert(col);
      // vec_col_pert(col) = vec_col_pert(max_norm_col);
      // vec_col_pert(max_norm_col) = tmp;
      vec_col_pert(col) = max_norm_col;
      for (Int32_t i = 0; i < mat_rows; ++i) {
        Scalar tmp = mat_coef(i, col);
        mat_coef(i, col) = mat_coef(i, max_norm_col);
        mat_coef(i, max_norm_col) = tmp;
      }
    }

    // std::cout << "After perturbation:" << std::endl;
    // std::cout << "max_norm_col=" << max_norm_col << std::endl;
    // std::cout << "vec_col_pert=" << vec_col_pert << std::endl;

    // Compute the euclidean norm of the columns vector in coeffient matrix
    // below the pivot position.
    //Int32_t true_mat_coef_col = vec_col_pert(col);
    c0 = mat_coef(col, col);
    tail_sq_norm = 0;
    for (Int32_t i = col+1; i < mat_rows; ++i) {
      tail_sq_norm += Square(mat_coef(i, col));
    }
    // Compute the Householder vector.
    if (tail_sq_norm <= tol) {
      tau = 0;
      beta = c0;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) = 0;
      }
    } else {
      beta = com_sqrt(c0*c0 + tail_sq_norm);
      if (c0 >= 0) {
        beta = -beta;
      }
      Scalar ratio = c0 - beta;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) /= ratio;
      }
      tau = -ratio / beta;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "The square norm of Householder vector is "
              << beta << std::endl;
    std::cout << "The Householder vector is: ";
    for (Int32_t i = 0; i < mat_rows; ++i) {
      std::cout << mat_coef(i, col) << " ";
    }
    std::cout << std::endl;
#endif

    // Update the product of elementary reflectors
    for (Int32_t j = 0; j < mat_rows; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm += mat_orth(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_orth(col, j);
      mat_orth(col, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_orth(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

    // Compute a reflector by matrix product
    mat_coef(col, col) = beta;
    for (Int32_t j = col+1; j < mat_cols; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm +=
            mat_coef(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(col, j);
      mat_coef(col, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_coef(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction:" << std::endl;
    std::cout << "The matrix is:" << std::endl;
    std::cout << mat_coef << std::endl;
    std::cout << "The orthogonal matrix is:" << std::endl;
    std::cout << mat_orth << std::endl;
#endif
  }

  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      if (i > j) {
        mat_coef(i, j) = 0;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "<########## Mat_ColPivHouseholderQR #############" << std::endl;
#endif

  return (true);
}

/**
 * @brief 求矩阵的正交分解矩阵(AP = Q*R, 使用Householder QR(Complete Pivoting)方法)
 * @param[in&out] mat_coef 系数矩阵，运算完成后它将保存分解后的上三角矩阵R
 * @param[out] mat_orth 保存分解后的单位正交矩阵的转置矩阵
 * @param[out] vec_col_pert 列排序信息(内部进行了列交换的操作)
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     The approximate number of multiplications/divisions required to reduce
 *     an n × n matrix to an upper-triangular form is 2n^3/3, which is greater
 *     then Gaussian elimination.
 * @endcode
 */
template<typename MatType1, typename MatType2, typename MatType3>
bool Mat_FullPivHouseholderQR(MatType1& mat_coef,
                              MatType2& mat_orth,
                              MatType3& vec_col_pert) {
  COM_CHECK(mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows() &&
            vec_col_pert.cols() == mat_coef.cols());

  typedef typename MatType1::value_type Scalar;

  const Int32_t mat_rows = mat_coef.rows();
  const Int32_t mat_cols = mat_coef.cols();
  Int32_t max_loop = (mat_rows < mat_cols) ? mat_rows : mat_cols;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########## Mat_FullPivHouseholderQR ###########>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  const Scalar tol = NumLimits<Scalar>::min();
  const Scalar precision = NumLimits<Scalar>::epsilon() * max_loop;
  Scalar tail_sq_norm = 0;
  Scalar c0 = 0;
  Scalar tau = 0;
  Scalar beta = 0;
  Scalar elem = 0;
  Scalar biggest = 0;
  Scalar max_elem = 0;
  Int32_t max_elem_row = 0;
  Int32_t max_elem_col = 0;
  mat_orth.SetIdentity();
  for (Int32_t i = 0; i < mat_cols; ++i) {
    vec_col_pert(i) = i;
  }

  for (Int32_t col = 0; col < max_loop; ++col) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << std::endl << "*****Loop " << col << " :" << std::endl;
#endif

    // std::cout << "Befor perturbation:" << std::endl;
    // std::cout << "vec_col_pert=" << vec_col_pert << std::endl;
    // std::cout << "mat_coef=\n" << mat_coef_pert << std::endl;

    // first, we look up in our matrix mat_coef,
    // which element is the biggest one
    max_elem_row = col;
    max_elem_col = col;
    max_elem = com_abs(mat_coef(col, col));
    for (Int32_t j = col; j < mat_cols; ++j) {
      for (Int32_t i = col; i < mat_rows; ++i) {
        elem = com_abs(mat_coef(i, j));
        if (elem > max_elem) {
          max_elem = elem;
          max_elem_row = i;
          max_elem_col = j;
        }
      }
    }
    if(0 == col) {
      biggest = max_elem;
    }

    // std::cout << "After perturbation:" << std::endl;
    // std::cout << "max_elem_row=" << max_elem_row << std::endl;
    // std::cout << "max_elem_col=" << max_elem_col << std::endl;
    // std::cout << "max_elem=" << max_elem << std::endl;

    // if the corner is negligible, then we have less than full rank,
    // and we can finish early
    if (com_abs(max_elem) <= (com_abs(biggest) * precision)) {
      break;
    }

    if (max_elem_row != col) {
      for (Int32_t j = col; j < mat_cols; ++j) {
        Scalar tmp = mat_coef(col, j);
        mat_coef(col, j) = mat_coef(max_elem_row, j);
        mat_coef(max_elem_row, j) = tmp;
      }
    }
    if (max_elem_col != col) {
      // Int32_t tmp = vec_col_pert(col);
      // vec_col_pert(col) = vec_col_pert(max_elem_col);
      // vec_col_pert(max_elem_col) = tmp;
      vec_col_pert(col) = max_elem_col;
      for (Int32_t i = 0; i < mat_rows; ++i) {
        Scalar tmp = mat_coef(i, col);
        mat_coef(i, col) = mat_coef(i, max_elem_col);
        mat_coef(i, max_elem_col) = tmp;
      }
    }

    // std::cout << "vec_row_pert=" << vec_row_pert << std::endl;
    // std::cout << "vec_col_pert=" << vec_col_pert << std::endl;
    // std::cout << "mat_coef=\n" << mat_coef_pert << std::endl;

    // Compute the euclidean norm of the columns vector in coeffient matrix
    // below the pivot position.
    c0 = mat_coef(col, col);
    tail_sq_norm = 0;
    for (Int32_t i = col+1; i < mat_rows; ++i) {
      tail_sq_norm += Square(mat_coef(i, col));
    }
    // Compute the Householder vector.
    if (tail_sq_norm <= tol) {
      tau = 0;
      beta = c0;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) = 0;
      }
    } else {
      beta = com_sqrt(c0*c0 + tail_sq_norm);
      if (c0 >= 0) {
        beta = -beta;
      }
      Scalar ratio = c0 - beta;
      for (Int32_t i = col+1; i < mat_rows; ++i) {
        mat_coef(i, col) /= ratio;
      }
      tau = -ratio / beta;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "The square norm of Householder vector is "
              << beta << std::endl;
    std::cout << "The Householder vector is: ";
    for (Int32_t i = 0; i < mat_rows; ++i) {
      std::cout << mat_coef(i, col) << " ";
    }
    std::cout << std::endl;
    std::cout << std::setprecision(20) << "tau=" << tau << std::endl;
#endif

    // Update the product of elementary reflectors
    if ((0 < col) && (col < (mat_rows-1)) && (max_elem_row != col)) {
      for (Int32_t j = 0; j < mat_rows; ++j) {
        Scalar tmp = mat_orth(col, j);
        mat_orth(col, j) = mat_orth(max_elem_row, j);
        mat_orth(max_elem_row, j) = tmp;
      }
    }
    for (Int32_t j = 0; j < mat_rows; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm +=
            mat_orth(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_orth(col, j);
      mat_orth(col, j) -= tau * tail_sq_norm;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_orth(k, j) -=
            tau * mat_coef(k, col) * tail_sq_norm;
      }
    }
    if ((0 == col) && (max_elem_row != col)) {
      for (Int32_t i = col; i < mat_rows; ++i) {
        Scalar tmp = mat_orth(i, col);
        mat_orth(i, col) = mat_orth(i, max_elem_row);
        mat_orth(i, max_elem_row) = tmp;
      }
    }

    // Compute a reflector by matrix product
    mat_coef(col, col) = beta;
    // std::cout << "Before reduction:" << std::endl;
    // std::cout << "The matrix is:" << std::endl;
    // std::cout << mat_coef << std::endl;
    for (Int32_t j = col+1; j < mat_cols; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        tail_sq_norm += mat_coef(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(col, j);
      mat_coef(col, j) -= tau * tail_sq_norm;
      for (Int32_t k = col+1; k < mat_rows; ++k) {
        mat_coef(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction:" << std::endl;
    std::cout << "The matrix is:" << std::endl;
    std::cout << mat_coef << std::endl;
    std::cout << "The orthogonal matrix is:" << std::endl;
    std::cout << mat_orth << std::endl;
#endif
  }

  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      if (i > j) {
        mat_coef(i, j) = 0;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "<######### Mat_FullPivHouseholderQR ############" << std::endl;
#endif

  return (true);
}

/**
 * @brief 使用QR分解后的正交矩阵及上三角矩阵(A=QR)求解线性方程组(Ax=b) \n
 *        其中Q为正交矩阵，R为上三角矩阵
 * @param[in] mat_tria 上三角矩阵R
 * @param[in] mat_orth 正交矩阵Q的转置矩阵
 * @param[in] vec_b 线性方程组（Ax=b）的右边的向量
 * @param[out] vec_x 线性方程组（Ax=b）的解
 * @return true 成功 \n
 *         false 失败
 */
template<typename MatType1, typename MatType2,
         typename MatType3, typename MatType4>
bool Mat_CalcLinearEquationFromQR(const MatType1& mat_tria,
                                  const MatType2& mat_orth,
                                  const MatType3& vec_b,
                                  MatType4& vec_x) {

  // std::cout << "mat_tria.rows()=" << mat_tria.rows()
  //           << ", mat_tria.cols()=" << mat_tria.cols()
  //           << ", mat_orth.rows()=" << mat_orth.rows()
  //           << ", mat_orth.cols()=" << mat_orth.cols()
  //           << ", vec_b.rows()=" << vec_b.rows()
  //           << ", vec_x.rows()=" << vec_x.rows()
  //           << std::endl;

  COM_CHECK(mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_tria.rows() &&
            vec_b.rows() == mat_orth.rows() &&
            vec_x.rows() == mat_orth.rows());


  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();

  Mat_Mul(mat_orth, vec_b, vec_x);

  bool ret = true;
  Int32_t dim = mat_tria.rows() < mat_tria.cols() ?
        mat_tria.rows() : mat_tria.cols();

  // Solve the last equation for the value of the last unknown.
  if (com_abs(mat_tria(dim-1, dim-1)) > eps) {
    vec_x(dim-1) = vec_x(dim-1) / mat_tria(dim-1, dim-1);
  } else {
    LOG_WARN << "This system may have not unique solution.";
    vec_x(dim-1) = 0;
    if (com_abs(vec_b(dim-1)) > eps) {
      // This system have not solution.
      LOG_ERR << "This system have not solution.";
      ret = false;
    }
  }
  // Determine all of the unknowns using back substitution.
  for (Int32_t i = dim-1; i > 0; --i) {
    Scalar value = vec_x(i-1);
    for (Int32_t j = dim-1; j > i-1; --j) {
      value -= mat_tria(i-1, j) * vec_x(j);
    }
    if (com_abs(mat_tria(i-1, i-1)) > eps) {
      vec_x(i-1) = value / mat_tria(i-1, i-1);
    } else {
      LOG_WARN << "This system may have not unique solution.";
      vec_x(i-1) = 0;
      if (com_abs(value) > eps) {
        // This system have not solution.
        LOG_ERR << "This system have not solution.";
        ret = false;
      }
    }
  }

  return (ret);
}

/**
 * @enum
 * @brief 使用矩阵SVD分解函数时的选项
 */
enum {
  /// 使用Householder QR方法将非方阵的矩阵转化为方阵
  MAT_SVD_OPT_HOUSEHOLDER = 0,
  /// 使用Householder QR(Column Partial Pivoting)方法将非方阵的矩阵转化为方阵
  MAT_SVD_OPT_COL_PIV_HOUSEHOLDER = 1,
  /// 使用Householder QR(Complete Pivoting)方法将非方阵的矩阵转化为方阵
  MAT_SVD_OPT_FULL_PIV_HOUSEHOLDER = 2,
  /// 使用Householder QR选项的掩码
  MAT_SVD_OPT_HOUSEHOLDER_MASK = 0x03
};

/**
 * @brief 将矩阵进行SVD(A = U D V^T)分解（使用Jacobi方法）
 * @param[in] mat_coef 待分解的系数矩阵
 * @param[out] mat_left 分解后的左边的正交矩阵
 * @param[out] mat_diag 分解后的对角矩阵
 * @param[out] mat_right 分解后的右边的正交矩阵
 * @param[out] options 操作选项
 * @return true - 成功, false - 失败
 */
template<typename MatType1, typename MatType2,
         typename MatType3, typename MatType4>
bool Mat_JacobiSVD(const MatType1& mat_coef,
                   MatType2& mat_left,
                   MatType3& mat_diag,
                   MatType4& mat_right,
                   const Uint32_t options = MAT_SVD_OPT_COL_PIV_HOUSEHOLDER) {
  COM_CHECK(mat_left.rows() == mat_left.cols() &&
            mat_left.rows() == mat_coef.rows() &&
            mat_right.rows() == mat_right.cols() &&
            mat_right.rows() == mat_coef.cols() &&
            mat_diag.cols() >= mat_coef.cols() &&
            mat_diag.rows() >= Max(mat_coef.rows(), mat_coef.cols()));

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########## Mat_JacobiSVD ###########>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;

  // currently we stop when we reach precision 2*epsilon as the last
  // bit of precision can require an unreasonable number of iterations,
  // only worsening the precision of U and V as we accumulate more rotations
  const Scalar precision = 2 * NumLimits<Scalar>::epsilon();
  // limit for denormal numbers to be considered zero in order
  // to avoid infinite loops
  const Scalar consider_as_zero = NumLimits<Scalar>::min();

  Uint32_t opt_householder = options & MAT_SVD_OPT_HOUSEHOLDER_MASK;
  Int32_t mat_rows = mat_coef.rows();
  Int32_t mat_cols = mat_coef.cols();
  Int32_t diag_size = Min(mat_rows, mat_cols);

  Scalar scale = mat_coef.FindMaxAbsCoeff();
  if(scale < NumLimits<Scalar>::epsilon()) {
    scale = 1;
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "\nEnter step 1. (Use a QR decomposition "
               "to reduce to the case of a square matrix)" << std::endl;
#endif

  /*** step 1. The R-SVD step: we use a QR decomposition to reduce to
   ***         the case of a square matrix */
  if (mat_rows != mat_cols) {
    if (mat_rows > mat_cols) {
      mat_diag.SetBlockSize(mat_rows, mat_cols);
      Mat_Copy(mat_coef, mat_diag);
      Mat_DivNum(scale, mat_diag, mat_diag);

      mat_right.SetIdentity();
      Matrix<Int32_t, 1, MatType1::ColsAtCompileTime> vec_col_pert;
      if (MAT_SVD_OPT_COL_PIV_HOUSEHOLDER == opt_householder) {
        Mat_ColPivHouseholderQR(mat_diag, mat_left, vec_col_pert);
        Mat_PerturbCols(mat_right, vec_col_pert);
      } else if (MAT_SVD_OPT_FULL_PIV_HOUSEHOLDER == opt_householder) {
        Mat_FullPivHouseholderQR(mat_diag, mat_left, vec_col_pert);
        Mat_PerturbCols(mat_right, vec_col_pert);
      } else {
        Mat_HouseholderQR(mat_diag, mat_left);
      }

      mat_diag.SetBlockSize(mat_cols, mat_cols);
      mat_left.TransposeInPlace();
    } else {
      mat_diag.SetBlockSize(mat_cols, mat_rows);
      Mat_Transpose(mat_coef, mat_diag);
      Mat_DivNum(scale, mat_diag, mat_diag);

      mat_left.SetIdentity();
      Matrix<Int32_t, 1, MatType1::RowsAtCompileTime> vec_col_pert;
      if (MAT_SVD_OPT_COL_PIV_HOUSEHOLDER == opt_householder) {
        Mat_ColPivHouseholderQR(mat_diag, mat_right, vec_col_pert);
        Mat_PerturbCols(mat_left, vec_col_pert);
      } else if (MAT_SVD_OPT_FULL_PIV_HOUSEHOLDER == opt_householder) {
        Mat_FullPivHouseholderQR(mat_diag, mat_right, vec_col_pert);
        Mat_PerturbCols(mat_left, vec_col_pert);
      } else {
        Mat_HouseholderQR(mat_diag, mat_right);
      }

      mat_diag.SetBlockSize(mat_rows, mat_rows);
      mat_diag.TransposeInPlace();
      mat_right.TransposeInPlace();
    }
  } else {
    mat_diag.SetBlockSize(mat_rows, mat_cols);
    Mat_Copy(mat_coef, mat_diag);
    Mat_DivNum(scale, mat_diag, mat_diag);
    mat_left.SetIdentity();
    mat_right.SetIdentity();
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "\nAfter step 1. (Use a QR decomposition "
               "to reduce to the case of a square matrix)"
            << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
  std::cout << "mat_left=\n" << mat_left << std::endl;
  std::cout << "mat_right=\n" << mat_right << std::endl;
#endif

  Scalar max_diag_entry = mat_diag.FindMaxAbsDiagCoeff();

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "\nEnter step 2. (The main Jacobi SVD iteration)" << std::endl;
  std::cout << "diag_size=" << diag_size << std::endl;
  std::cout << "max_diag_entry=" << max_diag_entry << std::endl;
#endif

  /*** step 2. The main Jacobi SVD iteration. ***/
  bool finished = false;
  Int32_t loop_count = 0;
  while(!finished) {
    loop_count++;

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "\n >>> loop_count=" << loop_count << std::endl;
#endif

    finished = true;
    // do a sweep: for all index pairs (p,q),
    // perform SVD of the corresponding 2x2 sub-matrix
    for(Int32_t p = 1; p < diag_size; ++p) {
      for(Int32_t q = 0; q < p; ++q) {
#if ENABLE_MATH_MATRIX_TRACE
        std::cout << "\n   >>> Loop (p=" << p
                  << ", q=" << q << ")" << std::endl;
        std::cout << "Handle coeff(" << p << "," << q << ") = "
                  << mat_diag(p, q)
                  << ", and coeff(" << q << "," << p << ") = "
                  << mat_diag(q, p)
                  << std::endl;
#endif

        // if this 2x2 sub-matrix is not diagonal already...
        // notice that this comparison will evaluate to false
        // if any NaN is involved, ensuring that NaN's don't
        // keep us iterating forever. Similarly, small denormal
        // numbers are considered zero.
        Scalar threshold = Max(consider_as_zero, precision * max_diag_entry);
        if ((com_abs(mat_diag(p, q)) > threshold) ||
            (com_abs(mat_diag(q, p)) > threshold)) {
#if ENABLE_MATH_MATRIX_TRACE
          std::cout << "Perform SVD decomposition:" << std::endl;
#endif

          finished = false;

          // Using Givens rotations
          // perform SVD decomposition of 2x2 sub-matrix
          // corresponding to indices p,q to make it diagonal
          Scalar t = mat_diag(p, p) + mat_diag(q, q);
          Scalar d = mat_diag(q, p) - mat_diag(p, q);
          Scalar s = 0;
          Scalar c = 0;
          if (com_abs(d) < NumLimits<Scalar>::min()) {
            s = 0;
            c = 1;
          } else {
            // If d!=0, then t/d cannot overflow because the magnitude of the
            // entries forming d are not too small compared to
            // the ones forming t.
            Scalar u = t / d;
            Scalar tmp = com_sqrt(1 + u*u);
            s = 1 / tmp;
            c = u / tmp;
          }

          Scalar m[2][2];
          m[0][0] =  c * mat_diag(p, p) + s * mat_diag(q, p);
          m[1][0] = -s * mat_diag(p, p) + c * mat_diag(q, p);
          m[0][1] =  c * mat_diag(p, q) + s * mat_diag(q, q);
          m[1][1] = -s * mat_diag(p, q) + c * mat_diag(q, q);

          Scalar right_jacobi[2] = { 1, 0 };
          Scalar deno = 2 * com_abs(m[0][1]);
          if (deno >= NumLimits<Scalar>::min()) {
            Scalar tau = (m[0][0] - m[1][1]) / deno;
            Scalar w = com_sqrt(tau*tau + 1);
            if(tau > 0) {
              t = 1 / (tau + w);
            } else {
              t = 1 / (tau - w);
            }
            Scalar sign_t = t > 0 ? 1 : -1;
            Scalar n = 1 / com_sqrt(t*t + 1);
            right_jacobi[0] = n;
            right_jacobi[1] = -sign_t * (m[0][1] > 0 ? 1 : -1) * com_abs(t) * n;
          }

          Scalar left_jacobi[2];
          left_jacobi[0] = c * right_jacobi[0] + s * right_jacobi[1];
          left_jacobi[1] = -c * right_jacobi[1] + s * right_jacobi[0];

          // accumulate resulting Jacobi rotations
          for (Int32_t j = 0; j < diag_size; ++j) {
            t = mat_diag(p, j);
            mat_diag(p, j) =
                left_jacobi[0] * t + left_jacobi[1] * mat_diag(q, j);
            mat_diag(q, j) =
                -left_jacobi[1] * t + left_jacobi[0] * mat_diag(q, j);
          }
          for (Int32_t i = 0; i < diag_size; ++i) {
            t = mat_diag(i, p);
            mat_diag(i, p) =
                right_jacobi[0] * t - right_jacobi[1] * mat_diag(i, q);
            mat_diag(i, q) =
                right_jacobi[1] * t + right_jacobi[0] * mat_diag(i, q);
          }
          for (Int32_t i = 0; i < mat_rows; ++i) {
            t = mat_left(i, p);
            mat_left(i, p) =
                left_jacobi[0] * t + left_jacobi[1] * mat_left(i, q);
            mat_left(i, q) =
                -left_jacobi[1] * t + left_jacobi[0] * mat_left(i, q);
          }
          for (Int32_t i = 0; i < mat_cols; ++i) {
            t = mat_right(i, p);
            mat_right(i, p) =
                right_jacobi[0] * t - right_jacobi[1] * mat_right(i, q);
            mat_right(i, q) =
                right_jacobi[1] * t + right_jacobi[0] * mat_right(i, q);
          }

          // keep track of the largest diagonal coefficient
          max_diag_entry = Max(max_diag_entry,
                               Max(com_abs(mat_diag(p, p)),
                                   com_abs(mat_diag(q, q))));

#if ENABLE_MATH_MATRIX_TRACE
          std::cout << "max_diag_entry=" << max_diag_entry << std::endl;
#endif
        }
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << std::endl << "After step 2:" << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
  std::cout << "mat_left=\n" << mat_left << std::endl;
  std::cout << "mat_right=\n" << mat_right << std::endl;

  std::cout << "\nEnter step 3. (The work matrix is now diagonal, "
               "so ensure it's positive so its diagonal entries "
               "are the singular values)"
            << std::endl;
#endif

  /*** step 3. The work matrix is now diagonal, so ensure it's positive
   ***         so its diagonal entries are the singular values ***/
  for (Int32_t i = 0; i < diag_size; ++i) {
    if (mat_diag(i, i) < 0) {
      mat_diag(i, i) = -mat_diag(i, i);
      for (Int32_t k = 0; k < mat_rows; ++k) {
        mat_left(k, i) = -mat_left(k, i);
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << std::endl << "After step 3:" << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
  std::cout << "mat_left=\n" << mat_left << std::endl;
  std::cout << "mat_right=\n" << mat_right << std::endl;

  std::cout << "\nEnter step 4. (Sort singular values in descending order "
               "and compute the number of nonzero singular values)"
            << std::endl;
#endif

  /*** step 4. Sort singular values in descending order and
   ***         compute the number of nonzero singular values ***/
  for (Int32_t i = 0; i < diag_size; ++i) {
    Int32_t max_singular_index = i;
    Scalar max_singular = mat_diag(i, i);
    for (Int32_t j = (i+1); j < diag_size; ++j) {
      Scalar value = mat_diag(j, j);
      if (value > max_singular) {
        max_singular = value;
        max_singular_index = j;
      }
    }

    if (max_singular_index != i) {
      Scalar tmp = mat_diag(i, i);
      mat_diag(i, i) = mat_diag(max_singular_index, max_singular_index);
      mat_diag(max_singular_index, max_singular_index) = tmp;

      for (Int32_t k = 0; k < mat_rows; ++k) {
        tmp = mat_left(k, i);
        mat_left(k, i) = mat_left(k, max_singular_index);
        mat_left(k, max_singular_index) = tmp;
      }
      for (Int32_t k = 0; k < mat_cols; ++k) {
        tmp = mat_right(k, i);
        mat_right(k, i) = mat_right(k, max_singular_index);
        mat_right(k, max_singular_index) = tmp;
      }
    }

    mat_diag(i, i) *= scale;
  }

  mat_diag.SetBlockSize(mat_rows, mat_cols);
  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      if (i != j) {
        mat_diag(i, j) = 0;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << std::endl << "After step 4:" << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
  std::cout << "mat_left=\n" << mat_left << std::endl;
  std::cout << "mat_right=\n" << mat_right << std::endl;
#endif

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "<######### Mat_JacobiSVD ############" << std::endl;
#endif

  return (true);
}

/**
 * @brief 求方阵的伪逆(Moore–Penrose Pseudoinverse)
 * @param[in] mat_coef 待求逆的方阵
 * @param[out] mat_inverse 方阵的伪逆
 *
 * @par Note:
 * @code
 *     函数内部有4个相同维数的矩阵用来作为局部变量，
 *     故矩阵的维数不宜过大（局部变量位于程序的栈空间中）
 * @endcode
 */
template<typename Scalar, Int32_t N>
void Mat_CalcPseudoInverse(const Matrix<Scalar, N, N>& mat_coef,
                           Matrix<Scalar, N, N>& mat_inverse) {
  const Scalar precision = NumLimits<Scalar>::epsilon();

  Matrix<Scalar, N, N> mat_left;
  Matrix<Scalar, N, N> mat_diag;
  Matrix<Scalar, N, N> mat_right;
  Matrix<Scalar, N, N> mat_tmp;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  Mat_JacobiSVD(mat_coef, mat_left, mat_diag, mat_right);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After SVD:" << std::endl;
  std::cout << "mat_left=\n" << mat_left << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
  std::cout << "mat_right=\n" << mat_right << std::endl;
#endif

  for (Int32_t i = 0; i < N; ++i) {
    if (mat_diag(i, i) > precision) {
      mat_diag(i, i) = Scalar(1) / mat_diag(i, i);
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After inversing diag matrix:" << std::endl;
  std::cout << "mat_diag=\n" << mat_diag << std::endl;
#endif

  mat_left.TransposeInPlace();
  Mat_Mul(mat_right, mat_diag, mat_tmp);
  Mat_Mul(mat_tmp, mat_left, mat_inverse);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "mat_inverse=\n" << mat_inverse << std::endl;
#endif
}

/**
 * @brief 将矩阵分解为Schur的形式(A = U T U^T，相似变换)
 * @param[in&out] mat_coef 待分解的系数矩阵
 * @param[out] mat_left 分解后的左边的正交矩阵
 * @param[out] mat_diag 分解后的对角矩阵
 * @param[out] mat_right 分解后的右边的正交矩阵
 * @param[out] options 操作选项
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     T is a real quasi-triangular matrix(a block-triangular matrix whose 
 *         diagonal consists of 1-by-1 blocks and 2-by-2 blocks with 
 *         complex eigenvalues)
 *     U is a real orthogonal matrix
 * @endcode
 */
template<typename MatType1, typename MatType2>
bool Mat_RealSchur(MatType1& mat_coef, MatType2& mat_orth) {
  COM_CHECK(mat_coef.rows() == mat_coef.cols() &&
            mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows());

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "########## Mat_RealSchur ###########>" << std::endl;
  std::cout << "The input matrix is:" << std::endl;
  std::cout << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;

  const Scalar consider_as_zero = NumLimits<Scalar>::min();
  Int32_t mat_rows = mat_coef.rows();
  Int32_t mat_cols = mat_coef.cols();
  mat_orth.SetIdentity();

  Scalar scale = mat_coef.FindMaxAbsCoeff();
  if (scale <= consider_as_zero) {
    mat_coef.SetZeros();
    return (true);
  }
  Mat_DivNum(scale, mat_coef, mat_coef);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "\nStep 1. Reduce to Hessenberg form" << std::endl;
#endif

  // Step 1. Reduce to Hessenberg form
  Scalar tail_sq_norm = 0;
  Scalar c0 = 0;
  Scalar tau = 0;
  Scalar beta = 0;
  for (Int32_t col = 0; col < (mat_rows-1); ++col) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "\n*****Loop " << col << " :" << std::endl;
#endif
    // Compute the euclidean norm of the columns vector in coeffient matrix
    // below the subdiagonal position.
    c0 = mat_coef(col+1, col);
    tail_sq_norm = 0;
    for (Int32_t i = col+2; i < mat_rows; ++i) {
      tail_sq_norm += Square(mat_coef(i, col));
    }
    // Compute the Householder vector.
    if (tail_sq_norm <= NumLimits<Scalar>::min()) {
      tau = 0;
      beta = c0;
      for (Int32_t i = col+2; i < mat_rows; ++i) {
        mat_coef(i, col) = 0;
      }
    } else {
      beta = com_sqrt(c0*c0 + tail_sq_norm);
      if (c0 >= 0) {
        beta = -beta;
      }
      Scalar ratio = c0 - beta;
      for (Int32_t i = col+2; i < mat_rows; ++i) {
        mat_coef(i, col) /= ratio;
      }
      tau = -ratio / beta;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "beta=" << beta << ", tau=" << tau << std::endl;
    std::cout << "The Householder vector is: ";
    for (Int32_t i = col+2; i < mat_rows; ++i) {
      std::cout << mat_coef(i, col) << " ";
    }
    std::cout << std::endl;
#endif

    // Update the product of elementary reflectors
    for (Int32_t i = 0; i < mat_rows; ++i) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        tail_sq_norm += mat_orth(i, k) * mat_coef(k, col);
      }
      tail_sq_norm += mat_orth(i, col+1);
      mat_orth(i, col+1) -= tau * tail_sq_norm;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        mat_orth(i, k) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

    // Apply similarity transformation to remaining columns,
    // i.e., compute A = H A H'
    // A = H A
    mat_coef(col+1, col) = beta;
    for (Int32_t j = col+1; j < mat_cols; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        tail_sq_norm += mat_coef(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(col+1, j);
      mat_coef(col+1, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        mat_coef(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

    // std::cout << "After reduction on left:" << std::endl;
    // std::cout << "mat_coef=\n" << mat_coef << std::endl;

    // A = A H'
    for (Int32_t i = 0; i < mat_rows; ++i) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        tail_sq_norm += mat_coef(i, k) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(i, col+1);
      mat_coef(i, col+1) -= tau * tail_sq_norm;;
      for (Int32_t k = col+2; k < mat_rows; ++k) {
        mat_coef(i, k) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction:" << std::endl;
    std::cout << "The matrix is:" << std::endl;
    std::cout << mat_coef << std::endl;
    // std::cout << "The orthogonal matrix is:" << std::endl;
    // std::cout << mat_orth << std::endl;
#endif
  }
  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      if (i > (j+1)) {
        mat_coef(i, j) = 0;
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After Hessenberg reduction:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_orth=\n" << mat_orth << std::endl;
  std::cout << "\nStep 2. Reduce to real Schur form" << std::endl;
#endif

  // Step 2. Reduce to real Schur form
  Int32_t max_iters = 40 * mat_rows;
  // Int32_t max_iters = 12;
  // The matrix m_matT is divided in three parts.
  // Rows 0,...,il-1 are decoupled from the rest because m_matT(il,il-1) is zero.
  // Rows il,...,iu is the part we are working on (the active window).
  // Rows iu+1,...,end are already brought in triangular form.
  Int32_t iu = mat_cols - 1;
  Int32_t iter = 0;       // iteration count for current eigenvalue
  Int32_t total_iter = 0; // iteration count for whole matrix
  Scalar exshift = 0;     // sum of exceptional shifts
  Scalar norm = 0;
  for (Int32_t j = 0; j < mat_cols; ++j) {
    Int32_t rows = Min(j+2, mat_rows);
    for (Int32_t i = 0; i < rows; ++i) {
      norm += com_abs(mat_coef(i, j));
    }
  }
  if (norm <= NumLimits<Scalar>::epsilon()) {
    return true;
  }

#if ENABLE_MATH_MATRIX_TRACE
  Int32_t loop_count = 0;
#endif
  while (iu >= 0) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << ">>> loop_count=" << loop_count++ << std::endl;
#endif

    Int32_t il = iu;
    while (il > 0) {
      Scalar s = com_abs(mat_coef(il-1, il-1)) + com_abs(mat_coef(il, il));
      if (com_abs(mat_coef(il, il-1)) <= NumLimits<Scalar>::epsilon() * s)
        break;
      il--;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "iu=" << iu << ", il=" << il
               << ", iter=" << iter << std::endl;
#endif

    // Check for convergence
    if (il == iu) {
#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "One roots found" << std::endl;
#endif

      // One root found
      mat_coef(iu, iu) += exshift;
      if (iu > 0) {
        mat_coef(iu, iu-1) = 0;
      }
      iu--;
      iter = 0;
    } else if (il == (iu - 1)) {
#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "Two roots found" << std::endl;
#endif

      // Two roots found
      // The eigenvalues of the 2x2 matrix [a b; c d] are
      // trace +/- sqrt(discr/4) where discr = tr^2 - 4*det, tr = a + d, det = ad - bc
      Scalar p = Scalar(0.5) * (mat_coef(iu-1, iu-1) - mat_coef(iu, iu));
      // q = tr^2 / 4 - det = discr/4
      Scalar q = p * p + mat_coef(iu, iu-1) * mat_coef(iu-1, iu);
      mat_coef(iu, iu) += exshift;
      mat_coef(iu-1, iu-1) += exshift;

      // std::cout << "p=" << p << ", q=" << q << std::endl;

      if (q >= Scalar(0)) {
        // std::cout << "Two real eigenvalues" << std::endl;

        // Two real eigenvalues
        Scalar z = com_sqrt(com_abs(q));
        Scalar coeff[2];
        if (p >= Scalar(0)) {
          coeff[0] = p + z;
        } else {
          coeff[0] = p - z;
        }
        coeff[1] = mat_coef(iu, iu-1);

        Scalar c = 0;
        Scalar s = 0;
        if (com_abs(coeff[1]) <= NumLimits<Scalar>::epsilon()) {
          c = (coeff[0] < Scalar(0)) ? Scalar(-1) : Scalar(1);
          s = Scalar(0);
        } else if(com_abs(coeff[0]) <= NumLimits<Scalar>::epsilon()) {
          c = Scalar(0);
          s = (coeff[1] < Scalar(0)) ? Scalar(1) : Scalar(-1);
        } else if(com_abs(coeff[0]) > com_abs(coeff[1])) {
          Scalar t = coeff[1] / coeff[0];
          Scalar u = com_sqrt(Scalar(1) + t*t);
          if(coeff[0] < Scalar(0)) {
            u = -u;
          }
          c = Scalar(1) / u;
          s = -t * c;
        } else {
          Scalar t = coeff[0] / coeff[1];
          Scalar u = com_sqrt(Scalar(1) + t*t);
          if(coeff[1] < Scalar(0)) {
            u = -u;
          }
          s = -Scalar(1) / u;
          c = -t * s;
        }
        for (Int32_t j = iu-1; j < mat_cols; ++j) {
          Scalar t = mat_coef(iu-1, j);
          mat_coef(iu-1, j) = c * t - s * mat_coef(iu, j);
          mat_coef(iu, j) = s * t + c * mat_coef(iu, j);
        }
        for (Int32_t i = 0; i < iu+1; ++i) {
          Scalar t = mat_coef(i, iu-1);
          mat_coef(i, iu-1) = c * t - s * mat_coef(i, iu);
          mat_coef(i, iu) = s * t + c * mat_coef(i, iu);
        }
        mat_coef(iu, iu-1) = Scalar(0);

        for (Int32_t i = 0; i < mat_rows; ++i) {
          Scalar t = mat_orth(i, iu-1);
          mat_orth(i, iu-1) = c * t - s * mat_orth(i, iu);
          mat_orth(i, iu) = s * t + c * mat_orth(i, iu);
        }
      }

      if (iu > 1) {
        mat_coef(iu-1, iu-2) = Scalar(0);
      }

      iu -= 2;
      iter = 0;
    } else {
#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "No convergence yet" << std::endl;
#endif

      // No convergence yet
      total_iter++;
      if (total_iter > max_iters) {
        break;
      }

      // compute shift
      Scalar shift_info[3];
      shift_info[0] = mat_coef(iu, iu);
      shift_info[1] = mat_coef(iu-1, iu-1);
      shift_info[2] = mat_coef(iu, iu-1) * mat_coef(iu-1, iu);
      // Wilkinson's original ad hoc shift
      if (10 == iter) {
        // std::cout << "Wilkinson's original ad hoc shift" << std::endl;

        exshift += shift_info[0];
        for (Int32_t i = 0; i <= iu; ++i) {
          mat_coef(i, i) -= shift_info[0];
        }
        Scalar s = com_abs(mat_coef(iu, iu-1)) + com_abs(mat_coef(iu-1, iu-2));
        shift_info[0] = Scalar(0.75) * s;
        shift_info[1] = Scalar(0.75) * s;
        shift_info[2] = Scalar(-0.4375) * s * s;
      }
      // MATLAB's new ad hoc shift
      if (30 == iter) {
        // std::cout << "MATLAB's new ad hoc shift" << std::endl;

        Scalar s = (shift_info[1] - shift_info[0]) / Scalar(2.0);
        s = s * s + shift_info[2];
        if (s > Scalar(0)) {
          s = com_sqrt(s);
          if (shift_info[1] < shift_info[0]) {
            s = -s;
          }
          s = s + (shift_info[1] - shift_info[0]) / Scalar(2.0);
          s = shift_info[0] - shift_info[2] / s;
          exshift += s;
          for (Int32_t i = 0; i <= iu; ++i) {
            mat_coef(i, i) -= s;
          }
          shift_info[0] = (Scalar(0.964));
          shift_info[1] = (Scalar(0.964));
          shift_info[2] = (Scalar(0.964));
        }
      }
      iter = iter + 1;

#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "shift_info= " << shift_info[0] << ", " << shift_info[1]
                << ", " << shift_info[2] << std::endl;
#endif

      // Francis QR
      // Compute index im at which Francis QR step starts
      // and the first Householder vector.
      Int32_t im = 0;
      Scalar first_householder_vector[3];
      for (im = (iu - 2); im >= il; --im) {
        const Scalar tmm = mat_coef(im,im);
        const Scalar r = shift_info[0] - tmm;
        const Scalar s = shift_info[1] - tmm;
        first_householder_vector[0] = (r * s - shift_info[2]) /
            mat_coef(im+1, im) + mat_coef(im, im+1);
        first_householder_vector[1] = mat_coef(im+1, im+1) - tmm - r - s;
        first_householder_vector[2] = mat_coef(im+2, im+1);
        if (im == il) {
          break;
        }
        const Scalar lhs = mat_coef(im, im-1) *
            (com_abs(first_householder_vector[1]) +
            com_abs(first_householder_vector[2]));
        const Scalar rhs = first_householder_vector[0] *
            (com_abs(mat_coef(im-1, im-1)) + com_abs(tmm) +
             com_abs(mat_coef(im+1, im+1)));
        if (com_abs(lhs) < NumLimits<Scalar>::epsilon() * rhs) {
          break;
        }
      }

#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "first_householder_vector= "
                << first_householder_vector[0]
                << ", " << first_householder_vector[1]
                << ", " << first_householder_vector[2]
                << std::endl;
      std::cout << "im=" << im << ", iu=" << iu << std::endl;
#endif

      // Perform a Francis QR step involving rows il:iu and columns im:iu
      for (Int32_t k = im; k <= (iu-2); ++k) {
#if ENABLE_MATH_MATRIX_TRACE
        std::cout << "   >>> k=" << k << std::endl;
#endif
        bool first_iteration = (k == im);

        if (first_iteration) {
          c0 = first_householder_vector[0];
          tail_sq_norm = Square(first_householder_vector[1]) +
              Square(first_householder_vector[2]);
        } else {
          c0 = mat_coef(k, k-1);
          tail_sq_norm = Square(mat_coef(k+1, k-1)) +
              Square(mat_coef(k+2, k-1));
        }

        Scalar ess[2];
        if (tail_sq_norm <= NumLimits<Scalar>::min()) {
          tau = 0;
          beta = c0;
          ess[0] = 0;
          ess[1] = 0;
        } else {
          beta = com_sqrt(c0*c0 + tail_sq_norm);
          if (c0 >= 0) {
            beta = -beta;
          }
          Scalar ratio = c0 - beta;
          if (first_iteration) {
            ess[0] = first_householder_vector[1] / ratio;
            ess[1] = first_householder_vector[2] / ratio;
          } else {
            ess[0] = mat_coef(k+1, k-1) / ratio;
            ess[1] = mat_coef(k+2, k-1) / ratio;
          }
          tau = -ratio / beta;
        }

        if (com_abs(beta) > consider_as_zero) {
          if (first_iteration && (k > il)) {
            mat_coef(k, k-1) = -mat_coef(k, k-1);
          } else if (!first_iteration) {
            mat_coef(k, k-1) = beta;
          }

          // These Householder transformations form the O(n^3)
          // part of the algorithm
          for (Int32_t j = k; j < mat_cols; ++j) {
            tail_sq_norm = mat_coef(k, j) + mat_coef(k+1, j) * ess[0] +
                mat_coef(k+2, j) * ess[1];
            mat_coef(k, j) -= tau * tail_sq_norm;
            mat_coef(k+1, j) -= tau * ess[0] * tail_sq_norm;
            mat_coef(k+2, j) -= tau * ess[1] * tail_sq_norm;
          }

#if ENABLE_MATH_MATRIX_TRACE
          std::cout << "After reduction on left:" << std::endl;
          std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

          Int32_t rows = Min(iu, k+3) + 1;
          for (Int32_t i = 0; i < rows; ++i) {
            tail_sq_norm = mat_coef(i, k) + mat_coef(i, k+1) * ess[0] +
                mat_coef(i, k+2) * ess[1];
            mat_coef(i, k) -= tau * tail_sq_norm;;
            mat_coef(i, k+1) -= tau * ess[0] * tail_sq_norm;
            mat_coef(i, k+2) -= tau * ess[1] * tail_sq_norm;
          }

#if ENABLE_MATH_MATRIX_TRACE
          std::cout << "After reduction on right:" << std::endl;
          std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

          for (Int32_t i = 0; i < mat_rows; ++i) {
            tail_sq_norm = mat_orth(i, k) + mat_orth(i, k+1) * ess[0] +
                mat_orth(i, k+2) * ess[1];
            mat_orth(i, k) -= tau * tail_sq_norm;;
            mat_orth(i, k+1) -= tau * ess[0] * tail_sq_norm;
            mat_orth(i, k+2) -= tau * ess[1] * tail_sq_norm;
          }

#if ENABLE_MATH_MATRIX_TRACE
          std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif
        }
      }

      c0 = mat_coef(iu-1, iu-2);
      tail_sq_norm = Square(mat_coef(iu, iu-2));
      Scalar ess;
      if (tail_sq_norm <= NumLimits<Scalar>::min()) {
        tau = 0;
        beta = c0;
        ess = 0;
      } else {
        beta = com_sqrt(c0*c0 + tail_sq_norm);
        if (c0 >= 0) {
          beta = -beta;
        }
        Scalar ratio = c0 - beta;
        ess = mat_coef(iu, iu-2) / ratio;
        tau = -ratio / beta;
      }

#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "beta=" << beta << std::endl;
#endif

      if (com_abs(beta) > consider_as_zero) {
        mat_coef(iu-1, iu-2) = beta;

        for (Int32_t j = (iu-1); j < mat_cols; ++j) {
          tail_sq_norm = mat_coef(iu-1, j) + mat_coef(iu, j) * ess;
          mat_coef(iu-1, j) -= tau * tail_sq_norm;;
          mat_coef(iu, j) -= tau * ess * tail_sq_norm;
        }
        for (Int32_t i = 0; i < (iu+1); ++i) {
          tail_sq_norm = mat_coef(i, iu-1) + mat_coef(i, iu) * ess;
          mat_coef(i, iu-1) -= tau * tail_sq_norm;;
          mat_coef(i, iu) -= tau * ess * tail_sq_norm;
        }
        for (Int32_t i = 0; i < mat_rows; ++i) {
          tail_sq_norm = mat_orth(i, iu-1) + mat_orth(i, iu) * ess;
          mat_orth(i, iu-1) -= tau * tail_sq_norm;;
          mat_orth(i, iu) -= tau * ess * tail_sq_norm;
        }
      }

#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "After all househoudler:" << std::endl;
      std::cout << "mat_coef=\n" << mat_coef << std::endl;
      std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif

      // clean up pollution due to round-off errors
      for (Int32_t i = (im+2); i <= iu; ++i) {
        mat_coef(i, i-2) = Scalar(0);
        if (i > (im+2)) {
          mat_coef(i, i-3) = Scalar(0);
        }
      }
#if ENABLE_MATH_MATRIX_TRACE
      std::cout << "After all handling:" << std::endl;
      std::cout << "mat_coef=\n" << mat_coef << std::endl;
      std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After reducing to real Schur form:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif

  for (Int32_t j = 0; j < mat_cols; ++j) {
    for (Int32_t i = 0; i < mat_rows; ++i) {
      mat_coef(i, j) *= scale;
    }
  }

  if (total_iter > max_iters) {
    LOG_ERR << "Failed to reduce matrix to real Schur form "
               "(Timeout), total_iter=" << total_iter;
    return false;
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "<######### Mat_RealSchur ############" << std::endl;
#endif

  return (true);
}

/**
 * @brief 求实对称矩阵的特征值及特征向量(必定有特征值及特征向量，\n
          且都是实数的，特征向量可以是正交的)
 * @param[in&out] mat_coef 待特征值分解的矩阵，之后保存特征值(位于矩阵对角线上)
 * @param[out] mat_orth 特征向量(相互正交)
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 * @endcode
 */
template<typename MatType1, typename MatType2>
bool Mat_CalcEigenValueOfSymmetricMatrix(MatType1& mat_coef,
                                         MatType2& mat_orth) {

  COM_CHECK(mat_coef.rows() == mat_coef.cols() &&
            mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows());

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix (Begin) ###"
            << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();
  const Scalar consider_as_zero = NumLimits<Scalar>::min();

  const Int32_t dims = mat_coef.cols();
  mat_orth.SetIdentity();

  if (1 == dims) {
    return true;
  }

  // map the matrix coefficients to [-1:1] to avoid over- and underflow.
  Scalar scale = com_abs(mat_coef(0, 0));
  for (Int32_t j = 0; j < dims; ++j) {
    for (Int32_t i = j; i < dims; ++i) {
      Scalar abs_coef = com_abs(mat_coef(i, j));
      if (abs_coef > scale) {
        scale = abs_coef;
      }
    }
  }
  if (scale <= consider_as_zero) {
    scale = Scalar(1);
  }
  for (Int32_t j = 0; j < dims; ++j) {
    for (Int32_t i = j; i < dims; ++i) {
      mat_coef(i, j) /= scale;
    }
  }
  for (Int32_t j = 1; j < dims; ++j) {
    for (Int32_t i = 0; i < j; ++i) {
      mat_coef(i, j) = mat_coef(j, i);
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After scaling, mat_coef=\n" << mat_coef << std::endl;
#endif

  // Performs a tridiagonal decomposition of the selfadjoint matrix
  Scalar tail_sq_norm = 0;
  Scalar c0 = 0;
  Scalar tau = 0;
  Scalar beta = 0;
  for (Int32_t col = 0; col < (dims-1); ++col) {
#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "\n*****Loop " << col << " :" << std::endl;
#endif

    // Compute the Householder vector.
    c0 = mat_coef(col+1, col);
    tail_sq_norm = 0;
    for (Int32_t i = col+2; i < dims; ++i) {
      tail_sq_norm += Square(mat_coef(i, col));
    }
    if (tail_sq_norm <= consider_as_zero) {
      tau = 0;
      beta = c0;
      for (Int32_t i = col+2; i < dims; ++i) {
        mat_coef(i, col) = 0;
      }
    } else {
      beta = com_sqrt(c0*c0 + tail_sq_norm);
      if (c0 >= 0) {
        beta = -beta;
      }
      Scalar ratio = c0 - beta;
      for (Int32_t i = col+2; i < dims; ++i) {
        mat_coef(i, col) /= ratio;
      }
      tau = -ratio / beta;
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "beta=" << beta << ", tau=" << tau << std::endl;
    std::cout << "The Householder vector is: ";
    for (Int32_t i = col+2; i < dims; ++i) {
      std::cout << mat_coef(i, col) << " ";
    }
    std::cout << std::endl;
#endif

    // Apply similarity transformation to remaining columns,
    // i.e., A = H A H' where H = I - h v v' and v = matA.col(i).tail(n-i-1)
    // Update the product of elementary reflectors
    for (Int32_t i = 0; i < dims; ++i) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < dims; ++k) {
        tail_sq_norm += mat_orth(i, k) * mat_coef(k, col);
      }
      tail_sq_norm += mat_orth(i, col+1);
      mat_orth(i, col+1) -= tau * tail_sq_norm;
      for (Int32_t k = col+2; k < dims; ++k) {
        mat_orth(i, k) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

    // Apply similarity transformation to remaining columns,
    // i.e., compute A = H A H'
    // A = H A
    mat_coef(col+1, col) = beta;
    for (Int32_t j = col+1; j < dims; ++j) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < dims; ++k) {
        tail_sq_norm += mat_coef(k, j) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(col+1, j);
      mat_coef(col+1, j) -= tau * tail_sq_norm;;
      for (Int32_t k = col+2; k < dims; ++k) {
        mat_coef(k, j) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction on left:" << std::endl;
    std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

    // A = A H'
    mat_coef(col, col+1) = mat_coef(col+1, col);
    for (Int32_t k = col+2; k < dims; ++k) {
      mat_coef(col, k) = 0;
    }
    for (Int32_t i = (col+1); i < dims; ++i) {
      tail_sq_norm = 0;
      for (Int32_t k = col+2; k < dims; ++k) {
        tail_sq_norm += mat_coef(i, k) * mat_coef(k, col);
      }
      tail_sq_norm += mat_coef(i, col+1);
      mat_coef(i, col+1) -= tau * tail_sq_norm;;
      for (Int32_t k = col+2; k <= i; ++k) {
        mat_coef(i, k) -= tau * mat_coef(k, col) * tail_sq_norm;
      }
    }
    for (Int32_t i = (col+1); i < dims; ++i) {
      for (Int32_t k = i+1; k < dims; ++k) {
        mat_coef(i, k) = mat_coef(k, i);
      }
    }

#if ENABLE_MATH_MATRIX_TRACE
    std::cout << "After reduction:" << std::endl;
    std::cout << "The matrix is:" << std::endl;
    std::cout << mat_coef << std::endl;
    std::cout << "The orthogonal matrix is:" << std::endl;
    std::cout << mat_orth << std::endl;
#endif
  }

  for (Int32_t j = 0; j < (dims-1); ++j) {
    mat_coef(j, j+1) = 0;
    for (Int32_t i = (j+2); i < dims; ++i) {
      mat_coef(i, j) = 0;
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After tridiagonalization:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif

  // Compute the eigendecomposition from a tridiagonal matrix
  const Int32_t max_iterations = 30 * dims;
  const Scalar precision = Scalar(2)*NumLimits<Scalar>::epsilon();
  Int32_t end = dims-1;
  Int32_t start = 0;
  Int32_t iter = 0; // total number of iterations

  while (end > 0) {
    for (Int32_t i = start; i < end; ++i) {
      if ((com_abs(mat_coef(i+1, i)) <=
           (com_abs(mat_coef(i, i))+com_abs(mat_coef(i+1, i+1))) * precision) ||
          (com_abs(mat_coef(i+1, i)) <= consider_as_zero)) {
        mat_coef(i+1, i) = 0;
      }
    }

    // find the largest unreduced block
    while ((end > 0) && (com_abs(mat_coef(end, end-1)) <= consider_as_zero)) {
      end--;
    }
    if (end <= 0) {
      break;
    }

    // if we spent too many iterations, we give up
    iter++;
    if (iter > max_iterations) {
      break;
    }

    start = end - 1;
    while ((start > 0) &&
           (com_abs(mat_coef(start, start-1)) > consider_as_zero)) {
      start--;
    }

    Scalar td = (mat_coef(end-1, end-1) - mat_coef(end, end))*Scalar(0.5);
    Scalar e = mat_coef(end, end-1);
    /// Note that thanks to scaling, e^2 or td^2 cannot overflow,
    /// however they can still underflow thus leading to inf/NaN values
    /// when using the following commented code:
    ///     RealScalar e2 = numext::abs2(subdiag[end-1]);
    ///     RealScalar mu = diag[end] - e2 / (td + (td>0 ? 1 : -1) *
    ///                                                    sqrt(td*td + e2));
    /// This explain the following, somewhat more complicated, version:
    Scalar mu = mat_coef(end, end);
    if (com_abs(td) <= consider_as_zero) {
      mu -= com_abs(e);
    } else {
      Scalar e2 = Square(mat_coef(end, end-1));
      Scalar h = com_hypot(td, e);
      if(e2 <= consider_as_zero) {
        mu -= (e / (td + (td > Scalar(0) ? Scalar(1) : Scalar(-1)))) * (e / h);
      } else {
        mu -= e2 / (td + (td > Scalar(0) ? h : -h));
      }
    }

    Scalar x = mat_coef(start, start) - mu;
    Scalar z = mat_coef(start+1, start);
    Scalar c = 0;
    Scalar s = 0;
    for (Int32_t k = start; k < end; ++k) {
      if (com_abs(z) <= consider_as_zero) {
        c = x < Scalar(0) ? Scalar(-1) : Scalar(1);
        s = Scalar(0);
      } else if(com_abs(x) <= consider_as_zero) {
        c = Scalar(0);
        s = z < Scalar(0) ? Scalar(1) : Scalar(-1);
      } else if(com_abs(x) > com_abs(z)) {
        Scalar t = z / x;
        Scalar u = com_sqrt(Scalar(1) + t*t);
        if (x < Scalar(0)) {
          u = -u;
        }
        c = Scalar(1) / u;
        s = -t * c;
      } else {
        Scalar t = x / z;
        Scalar u = com_sqrt(Scalar(1) + t*t);
        if (z < Scalar(0)) {
          u = -u;
        }
        s = -Scalar(1) / u;
        c = -t * s;
      }

      // do T = G' T G
      Scalar sdk = s * mat_coef(k, k) + c * mat_coef(k+1, k);
      Scalar dkp1 = s * mat_coef(k+1, k) + c * mat_coef(k+1, k+1);

      mat_coef(k, k) = c * (c * mat_coef(k, k) - s * mat_coef(k+1, k)) -
          s * (c * mat_coef(k+1, k) - s * mat_coef(k+1, k+1));
      mat_coef(k+1, k+1) = s * sdk + c * dkp1;
      mat_coef(k+1, k) = c * sdk - s * dkp1;

      if (k > start) {
        mat_coef(k, k-1) = c * mat_coef(k, k-1) - s * z;
      }

      x = mat_coef(k+1, k);

      if (k < (end - 1)) {
        z = -s * mat_coef(k+2, k+1);
        mat_coef(k+2, k+1) = c * mat_coef(k+2, k+1);
      }

      // apply the givens rotation to the unit matrix Q = Q * G
      for (Int32_t i = 0; i < dims; ++i) {
        Scalar t = mat_orth(i, k);
        mat_orth(i, k) = c * t - s * mat_orth(i, k+1);
        mat_orth(i, k+1) = s * t + c * mat_orth(i, k+1);
      }
    }
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After computing eigens:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_orth=\n" << mat_orth << std::endl;
#endif

  /// TODO: Sort eigenvalues and corresponding vectors.

  // scale back the eigen values
  for (Int32_t i = 0; i < dims; ++i) {
    mat_coef(i, i) *= scale;
  }

  if (iter > max_iterations) {
    LOG_ERR << "Failed to calculate eigen vectors (Timeout).";
    return (false);
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix (End) ###"
            << std::endl;
#endif

  return (true);
}

/**
 * @brief 求实对称矩阵的特征值及特征向量(2维矩阵，使用特征多项式求解)
 * @param[in&out] mat_coef 待特征值分解的矩阵，之后保存特征值(位于矩阵对角线上)
 * @param[out] mat_orth 特征向量(相互正交)
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     因为使用特征多项式直接求解，所以运算速度比使用迭代的方法快很多，但精度要
 *     差些。
 *     For the 3x3 case, we observed the following worst case relative 
 *     error regarding the eigenvalues:
 *       - double: 1e-8
 *       - float:  1e-3
 * @endcode
 */
template<typename MatType1, typename MatType2>
bool Mat_CalcEigenValueOfSymmetricMatrix_2D(MatType1& mat_coef,
                                            MatType2& mat_orth) {
  COM_CHECK(mat_coef.rows() == mat_coef.cols() &&
            mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows() &&
            mat_coef.rows() == 2);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix_2D (Begin) ###"
            << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();
  const Scalar consider_as_zero = NumLimits<Scalar>::min();

  mat_orth.SetIdentity();

  // Shift the matrix to the mean eigenvalue and map the matrix coefficients
  // to [-1:1] to avoid over- and underflow.
  Scalar shift = (mat_coef(0, 0) + mat_coef(1, 1)) / Scalar(2);
  mat_coef(0, 0) -= shift;
  mat_coef(1, 1) -= shift;
  Scalar scale = Max(com_abs(mat_coef(0, 0)),
                     Max(com_abs(mat_coef(1, 0)), com_abs(mat_coef(1, 1))));
  if (scale > Scalar(0)) {
    mat_coef(0, 0) /= scale;
    mat_coef(1, 0) /= scale;
    mat_coef(1, 1) /= scale;
  }
  mat_coef(0, 1) = mat_coef(1, 0);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After scaling matrix:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  // Compute the eigenvalues
  // computeRoots(const MatrixType& m, VectorType& roots)
  Scalar eivals[2];
  const Scalar t0 =
      Scalar(0.5) * com_sqrt(Square(mat_coef(0, 0)-mat_coef(1, 1)) +
                             Scalar(4)*Square(mat_coef(1, 0)));
  const Scalar t1 = Scalar(0.5) * (mat_coef(0,0) + mat_coef(1,1));
  eivals[0] = t1 - t0;
  eivals[1] = t1 + t0;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After computing eigenvalues:" << std::endl;
  std::cout << "eivals = " << eivals[0] << ", " << eivals[1] << std::endl;
#endif

  // compute the eigen vectors
  if((eivals[1] - eivals[0]) <= com_abs(eivals[1])*eps) {
    // Already Set eigen vectors to identity
  } else {
    mat_coef(0, 0) -= eivals[1];
    mat_coef(1, 1) -= eivals[1];
    Scalar a2 = Square(mat_coef(0, 0));
    Scalar c2 = Square(mat_coef(1, 1));
    Scalar b2 = Square(mat_coef(1, 0));

    if (a2 > c2) {
      Scalar tmp = com_sqrt(a2 + b2);
      mat_orth(0, 1) = -mat_coef(1, 0) / tmp;
      mat_orth(1, 1) = mat_coef(0, 0) / tmp;
    } else {
      Scalar tmp = com_sqrt(c2 + b2);
      mat_orth(0, 1) = -mat_coef(1, 1) / tmp;
      mat_orth(1, 1) = mat_coef(1, 0) / tmp;
    }

    mat_orth(0, 0) = -mat_orth(1, 1);
    mat_orth(1, 0) = mat_orth(0, 1);
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After computing eigenvectors:" << std::endl;
  std::cout << "eigenvectors = \n" << mat_orth << std::endl;
#endif

  // Rescale back to the original size.
  mat_coef(0, 0) = eivals[0] * scale + shift;
  mat_coef(1, 1) = eivals[1] * scale + shift;
  mat_coef(1, 0) = 0;
  mat_coef(0, 1) = 0;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix_2D (End) ###"
            << std::endl;
#endif

  return true;
}

/**
 * @brief 求实对称矩阵的特征值及特征向量(3维矩阵，使用特征多项式求解)
 * @param[in&out] mat_coef 待特征值分解的矩阵，之后保存特征值(位于矩阵对角线上)
 * @param[out] mat_orth 特征向量(相互正交)
 * @return true - 成功, false - 失败
 *
 * @par Note:
 * @code
 *     输入的系数矩阵在运算的过程中会被修改，所以如果必要，需要在运算前做好备份。
 *     因为使用特征多项式直接求解，所以运算速度比使用迭代的方法快很多，但精度要
 *     差些。
 *     For the 3x3 case, we observed the following worst case relative 
 *     error regarding the eigenvalues:
 *       - double: 1e-8
 *       - float:  1e-3
 * @endcode
 */
template<typename MatType1, typename MatType2>
bool Mat_CalcEigenValueOfSymmetricMatrix_3D(MatType1& mat_coef,
                                            MatType2& mat_orth) {
  COM_CHECK(mat_coef.rows() == mat_coef.cols() &&
            mat_orth.rows() == mat_orth.cols() &&
            mat_orth.rows() == mat_coef.rows() &&
            mat_coef.rows() == 3);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix_3D (Begin) ###"
            << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  typedef typename MatType1::value_type Scalar;
  const Scalar eps = NumLimits<Scalar>::epsilon();
  const Scalar consider_as_zero = NumLimits<Scalar>::min();

  mat_orth.SetIdentity();

  // Shift the matrix to the mean eigenvalue and map the matrix coefficients
  // to [-1:1] to avoid over- and underflow.
  Scalar shift = (mat_coef(0, 0) + mat_coef(1, 1) + mat_coef(2, 2)) / Scalar(3);
  mat_coef(0, 0) -= shift;
  mat_coef(1, 1) -= shift;
  mat_coef(2, 2) -= shift;
  Scalar scale = Max(com_abs(mat_coef(0, 0)),
                     Max(com_abs(mat_coef(1, 0)),
                         Max(com_abs(mat_coef(2, 0)),
                             Max(com_abs(mat_coef(1, 1)),
                                 Max(com_abs(mat_coef(1, 2)),
                                     com_abs(mat_coef(2, 2)))))));
  if (scale > Scalar(0)) {
    mat_coef(0, 0) /= scale;
    mat_coef(1, 0) /= scale;
    mat_coef(2, 0) /= scale;
    mat_coef(1, 1) /= scale;
    mat_coef(2, 1) /= scale;
    mat_coef(2, 2) /= scale;
  }
  mat_coef(0, 1) = mat_coef(1, 0);
  mat_coef(0, 2) = mat_coef(2, 0);
  mat_coef(1, 2) = mat_coef(2, 1);

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After scaling matrix:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  // compute the eigenvalues
  Scalar eivals[3];
  const Scalar s_inv3 = Scalar(1) / Scalar(3);
  const Scalar s_sqrt3 = com_sqrt(Scalar(3));

  // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
  // eigenvalues are the roots to this equation, all guaranteed to be
  // real-valued, because the matrix is symmetric.
  Scalar c0 = mat_coef(0, 0)*mat_coef(1, 1)*mat_coef(2, 2) +
      Scalar(2)*mat_coef(1, 0)*mat_coef(2, 0)*mat_coef(2, 1) -
      mat_coef(0, 0)*mat_coef(2, 1)*mat_coef(2, 1) -
      mat_coef(1, 1)*mat_coef(2, 0)*mat_coef(2, 0) -
      mat_coef(2, 2)*mat_coef(1, 0)*mat_coef(1, 0);
  Scalar c1 = mat_coef(0, 0)*mat_coef(1, 1) - mat_coef(1, 0)*mat_coef(1, 0) +
      mat_coef(0, 0)*mat_coef(2, 2) - mat_coef(2, 0)*mat_coef(2, 0) +
      mat_coef(1, 1)*mat_coef(2, 2) - mat_coef(2, 1)*mat_coef(2, 1);
  Scalar c2 = mat_coef(0, 0) + mat_coef(1, 1) + mat_coef(2, 2);

  // Construct the parameters used in classifying the roots of the equation
  // and in solving the equation for the roots in closed form.
  Scalar c2_over_3 = c2*s_inv3;
  Scalar a_over_3 = (c2*c2_over_3 - c1)*s_inv3;
  a_over_3 = Max(a_over_3, Scalar(0));

  Scalar half_b = Scalar(0.5)*
      (c0 + c2_over_3*(Scalar(2)*c2_over_3*c2_over_3 - c1));

  Scalar q = a_over_3*a_over_3*a_over_3 - half_b*half_b;
  q = Max(q, Scalar(0));

  // Compute the eigenvalues by solving for the roots of the polynomial.
  Scalar rho = com_sqrt(a_over_3);
  // since sqrt(q) > 0, atan2 is in [0, pi] and theta is in [0, pi/3]
  Scalar theta = com_atan2(com_sqrt(q), half_b)*s_inv3;
  Scalar cos_theta = com_cos(theta);
  Scalar sin_theta = com_sin(theta);
  // roots are already sorted, since cos is monotonically decreasing on [0, pi]
  // == 2*rho*cos(theta+2pi/3)
  eivals[0] = c2_over_3 - rho*(cos_theta + s_sqrt3*sin_theta);
  // == 2*rho*cos(theta+ pi/3)
  eivals[1] = c2_over_3 - rho*(cos_theta - s_sqrt3*sin_theta);
  eivals[2] = c2_over_3 + Scalar(2)*rho*cos_theta;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After computing eigenvalues:" << std::endl;
  std::cout << "eivals = " << eivals[0]
            << ", " << eivals[1]
            << ", " << eivals[2]
            << std::endl;
#endif

  // compute the eigenvectors
  if ((eivals[2] - eivals[0]) <= eps) {
    // All three eigenvalues are numerically the same
    // Already Set eigen vectors to identity
  } else {
    // Compute the eigenvector of the most distinct eigenvalue
    Scalar d0 = eivals[2] - eivals[1];
    Scalar d1 = eivals[1] - eivals[0];
    Int32_t k(0), l(2);
    if (d0 > d1) {
      Swap(k, l);
      d0 = d1;
    }

    // Compute the eigenvector of index k
    {
      mat_coef(0, 0) -= eivals[k];
      mat_coef(1, 1) -= eivals[k];
      mat_coef(2, 2) -= eivals[k];

      // By construction, 'tmp' is of rank 2, and its kernel corresponds to
      // the respective eigenvector.

      // Find non-zero column i0 (by construction, there must exist a non
      // zero coefficient on the diagonal):
      Int32_t i0 = com_abs(mat_coef(0, 0)) < com_abs(mat_coef(1, 1)) ? 1 : 0;
      i0 = com_abs(mat_coef(i0, i0)) < com_abs(mat_coef(2, 2)) ? 2 : i0;
      // mat.col(i0) is a good candidate for an orthogonal vector to the
      // current eigenvector, so let's save it:
      mat_orth(0, l) = mat_coef(0, i0);
      mat_orth(1, l) = mat_coef(1, i0);
      mat_orth(2, l) = mat_coef(2, i0);

      Int32_t index = (i0 + 1) % 3;
      Scalar c0[3];
      c0[0] = mat_coef(1, i0)*mat_coef(2, index) -
          mat_coef(2, i0)*mat_coef(1, index);
      c0[1] = mat_coef(2, i0)*mat_coef(0, index) -
          mat_coef(0, i0)*mat_coef(2, index);
      c0[2] = mat_coef(0, i0)*mat_coef(1, index) -
          mat_coef(1, i0)*mat_coef(0, index);
      Scalar n0 = com_sqrt(Square(c0[0]) + Square(c0[1]) + Square(c0[2]));
      Scalar c1[3];
      index = (i0 + 2) % 3;
      c1[0] = mat_coef(1, i0)*mat_coef(2, index) -
          mat_coef(2, i0)*mat_coef(1, index);
      c1[1] = mat_coef(2, i0)*mat_coef(0, index) -
          mat_coef(0, i0)*mat_coef(2, index);
      c1[2] = mat_coef(0, i0)*mat_coef(1, index) -
          mat_coef(1, i0)*mat_coef(0, index);
      Scalar n1 = com_sqrt(Square(c1[0]) + Square(c1[1]) + Square(c1[2]));
      if(n0 > n1) {
        mat_orth(0, k) = c0[0] / n0;
        mat_orth(1, k) = c0[1] / n0;
        mat_orth(2, k) = c0[2] / n0;
      } else {
        mat_orth(0, k) = c1[0] / n1;
        mat_orth(1, k) = c1[1] / n1;
        mat_orth(2, k) = c1[2] / n1;
      }
    }

    // Compute eigenvector of index l
    if(d0 <= 2*eps*d1) {
      // If d0 is too small, then the two other eigenvalues
      // are numerically the same, and thus we only have to ortho-normalize
      // the near orthogonal vector we saved above.
      Scalar tmp = mat_orth(0, k)*mat_orth(0, l) +
          mat_orth(1, k)*mat_orth(1, l) + mat_orth(2, k)*mat_orth(2, l);
      mat_orth(0, l) -= tmp*mat_orth(0, l);
      mat_orth(1, l) -= tmp*mat_orth(1, l);
      mat_orth(2, l) -= tmp*mat_orth(2, l);
      tmp = com_sqrt(Square(mat_orth(0, l)) +
                     Square(mat_orth(1, l)) + Square(mat_orth(2, l)));
      mat_orth(0, l) /= tmp;
      mat_orth(1, l) /= tmp;
      mat_orth(2, l) /= tmp;
    } else {
      mat_coef(0, 0) -= eivals[l] - eivals[k];
      mat_coef(1, 1) -= eivals[l] - eivals[k];
      mat_coef(2, 2) -= eivals[l] - eivals[k];

      // Find non-zero column i0 (by construction, there must exist a non
      // zero coefficient on the diagonal):
      Int32_t i0 = com_abs(mat_coef(0, 0)) < com_abs(mat_coef(1, 1)) ? 1 : 0;
      i0 = com_abs(mat_coef(i0, i0)) < com_abs(mat_coef(2, 2)) ? 2 : i0;

      Int32_t index = (i0 + 1) % 3;
      Scalar c0[3];
      c0[0] = mat_coef(1, i0)*mat_coef(2, index) -
          mat_coef(2, i0)*mat_coef(1, index);
      c0[1] = mat_coef(2, i0)*mat_coef(0, index) -
          mat_coef(0, i0)*mat_coef(2, index);
      c0[2] = mat_coef(0, i0)*mat_coef(1, index) -
          mat_coef(1, i0)*mat_coef(0, index);
      Scalar n0 = com_sqrt(Square(c0[0]) + Square(c0[1]) + Square(c0[2]));
      Scalar c1[3];
      index = (i0 + 2) % 3;
      c1[0] = mat_coef(1, i0)*mat_coef(2, index) -
          mat_coef(2, i0)*mat_coef(1, index);
      c1[1] = mat_coef(2, i0)*mat_coef(0, index) -
          mat_coef(0, i0)*mat_coef(2, index);
      c1[2] = mat_coef(0, i0)*mat_coef(1, index) -
          mat_coef(1, i0)*mat_coef(0, index);
      Scalar n1 = com_sqrt(Square(c1[0]) + Square(c1[1]) + Square(c1[2]));
      if(n0 > n1) {
        mat_orth(0, l) = c0[0] / n0;
        mat_orth(1, l) = c0[1] / n0;
        mat_orth(2, l) = c0[2] / n0;
      } else {
        mat_orth(0, l) = c1[0] / n1;
        mat_orth(1, l) = c1[1] / n1;
        mat_orth(2, l) = c1[2] / n1;
      }
    }

    // Compute last eigenvector from the other two
    mat_orth(0, 1) = mat_orth(1, 2)*mat_orth(2, 0) -
        mat_orth(2, 2)*mat_orth(1, 0);
    mat_orth(1, 1) = mat_orth(2, 2)*mat_orth(0, 0) -
        mat_orth(0, 2)*mat_orth(2, 0);
    mat_orth(2, 1) = mat_orth(0, 2)*mat_orth(1, 0) -
        mat_orth(1, 2)*mat_orth(0, 0);
    Scalar tmp = com_sqrt(Square(mat_orth(0, 1)) +
                          Square(mat_orth(1, 1)) + Square(mat_orth(2, 1)));
    mat_orth(0, 1) /= tmp;
    mat_orth(1, 1) /= tmp;
    mat_orth(2, 1) /= tmp;
  }

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "After computing eigenvectors:" << std::endl;
  std::cout << "eigenvectors = \n" << mat_orth << std::endl;
#endif

  // Rescale back to the original size.
  mat_coef(0, 0) = eivals[0] * scale + shift;
  mat_coef(1, 1) = eivals[1] * scale + shift;
  mat_coef(2, 2) = eivals[2] * scale + shift;
  mat_coef(1, 0) = 0;
  mat_coef(2, 0) = 0;
  mat_coef(0, 1) = 0;
  mat_coef(2, 1) = 0;
  mat_coef(0, 2) = 0;
  mat_coef(1, 2) = 0;

#if ENABLE_MATH_MATRIX_TRACE
  std::cout << "### Mat_CalcEigenValueOfSymmetricMatrix_3D (End) ###"
            << std::endl;
#endif

  return true;
}


} // common
} // phoenix

#endif // PHOENIX_COMMON_MATH_MATRIX_H_
