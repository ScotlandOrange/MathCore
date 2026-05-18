#pragma once

/// @file
/// @brief Eigen 与 STL / C 数组转换辅助函数。

#include <array>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QJsonArray>
#include <QStringList>

#include <Core/TypeTraits.h>
#include <Core/ZFAssert.h>

#include <tuple>
#include <vector>

namespace ZF
{

enum class MatrixToStdVectorOrder
{
    RowMajor,
    ColMajor,
};

namespace detail
{

template <typename Out, typename Derived>
using ConvertOutputScalar_t = std::conditional_t<std::is_void<Out>{}, eigen_scalar_t<Derived>, Out>;

template <typename Out, std::size_t N, typename Derived>
using ConvertStdArray_t = std::array<ConvertOutputScalar_t<Out, Derived>, N>;

template <typename Out, std::size_t Rows, std::size_t Cols, typename Derived>
using ConvertStdMatrixArray_t = std::array<ConvertStdArray_t<Out, Cols, Derived>, Rows>;

template <typename Out, typename Derived>
using ConvertStdVector_t = std::vector<ConvertOutputScalar_t<Out, Derived>>;

template <typename Out, typename Derived>
using ConvertStdMatrixVector_t = std::vector<ConvertStdVector_t<Out, Derived>>;

} // namespace detail

QStringList              toStringList(const std::vector<std::string>& vec);
std::vector<std::string> toVector    (const QStringList& list);

// 将固定尺寸 Eigen 向量 / 矩阵 / 四元数转换为 std::array。
// - 形状（N 或 Rows×Cols）须与输入一致。
// - 元素类型可显式指定 Out，省略时沿用输入标量类型。
// - 四元数顺序固定为 {x, y, z, w}。
// 示例：
//   auto a1 = ZF::toStdArray<double, 3>   (Eigen::Vector3f(1, 2, 3));        // std::array<double, 3>
//   auto a2 = ZF::toStdArray<3>           (Eigen::Vector3f(1, 2, 3));        // std::array<float, 3>
//   auto a3 = ZF::toStdArray<double, 2, 2>(Eigen::Matrix2f::Identity());     // std::array<std::array<double, 2>, 2>
//   auto a4 = ZF::toStdArray<double, 4>   (Eigen::Quaternionf::Identity());  // std::array<double, 4>
template <typename Out, std::size_t N, typename Derived,
          enable_if_eigen_vector_t<Derived> = 0, enable_if_arithmetic_t<Out> = 0>
inline detail::ConvertStdArray_t<Out, N, Derived>
toStdArray(const Eigen::MatrixBase<Derived>& vec);

template <std::size_t N, typename Derived,
          enable_if_eigen_vector_t<Derived> = 0>
inline detail::ConvertStdArray_t<void, N, Derived>
toStdArray(const Eigen::MatrixBase<Derived>& vec);

template <typename Out, std::size_t Rows, std::size_t Cols, typename Derived,
          enable_if_eigen_matrix_t<Derived> = 0, enable_if_arithmetic_t<Out> = 0>
inline detail::ConvertStdMatrixArray_t<Out, Rows, Cols, Derived>
toStdArray(const Eigen::MatrixBase<Derived>& m);

template <std::size_t Rows, std::size_t Cols, typename Derived,
          enable_if_eigen_matrix_t<Derived> = 0>
inline detail::ConvertStdMatrixArray_t<void, Rows, Cols, Derived>
toStdArray(const Eigen::MatrixBase<Derived>& m);

template <typename Out, std::size_t N, typename Derived,
          enable_if_eigen_quaternion_t<Derived> = 0, enable_if_arithmetic_t<Out> = 0>
inline detail::ConvertStdArray_t<Out, N, Derived>
toStdArray(const Eigen::QuaternionBase<Derived>& q);

template <std::size_t N, typename Derived,
          enable_if_eigen_quaternion_t<Derived> = 0>
inline detail::ConvertStdArray_t<void, N, Derived>
toStdArray(const Eigen::QuaternionBase<Derived>& q);

// 将固定尺寸 Eigen 向量 / 矩阵 / 四元数转换为 std::vector。
// - 元素类型可显式指定 Out，省略时沿用输入标量类型。
// - 矩阵重载返回 [row][col] 二维 vector，不暴露底层存储布局。
// - 四元数顺序固定为 {x, y, z, w}。
// 示例：
//   auto v1 = ZF::toStdVector        (Eigen::Vector3f(1, 2, 3));        // std::vector<float>
//   auto v2 = ZF::toStdVector<double>(Eigen::Matrix2f::Identity());     // std::vector<std::vector<double>>
//   auto v3 = ZF::toStdVector        (Eigen::Quaternionf::Identity());  // std::vector<float>，{x,y,z,w}
// Vector / quaternion to std::vector.
// - Vectors keep the original `toStdVector(...)` API.
// - Quaternions keep `{x, y, z, w}` order.
template <typename Out = void, typename Derived,
          enable_if_eigen_vector_t<Derived> = 0,
          enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>> = 0>
inline detail::ConvertStdVector_t<Out, Derived>
toStdVector(const Eigen::MatrixBase<Derived>& vec);

template <typename Out = void, typename Derived,
          enable_if_eigen_quaternion_t<Derived> = 0,
          enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>> = 0>
inline detail::ConvertStdVector_t<Out, Derived>
toStdVector(const Eigen::QuaternionBase<Derived>& q);

// Matrix-to-std::vector conversions.
// - `MatrixToStdVector1D` returns a flattened `std::vector<T>`.
// - `MatrixToStdVector2D` returns `std::vector<std::vector<T>>`.
// - `RowMajor` uses row-first traversal.
// - `ColMajor` uses column-first traversal; for `2D`, the outer dimension is columns.
template <typename Out = void, typename Derived,
          enable_if_eigen_matrix_t<Derived> = 0,
          enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>> = 0>
inline detail::ConvertStdVector_t<Out, Derived>
MatrixToStdVector1D(const Eigen::MatrixBase<Derived>& m,
                    MatrixToStdVectorOrder order = MatrixToStdVectorOrder::RowMajor);

template <typename Out = void, typename Derived,
          enable_if_eigen_matrix_t<Derived> = 0,
          enable_if_arithmetic_t<detail::ConvertOutputScalar_t<Out, Derived>> = 0>
inline detail::ConvertStdMatrixVector_t<Out, Derived>
MatrixToStdVector2D(const Eigen::MatrixBase<Derived>& m,
                    MatrixToStdVectorOrder order = MatrixToStdVectorOrder::RowMajor);

// 将固定尺寸 Eigen 向量 / 矩阵 / 四元数写入 C 风格数组。
// - 输出元素类型从 out 自动推导，数组形状必须与输入一致。
// - 四元数顺序固定为 {x, y, z, w}。
// 示例：
//   double a[3];     ZF::toCArray(Eigen::Vector3f(1, 2, 3),       a);
//   double m[2][2];  ZF::toCArray(Eigen::Matrix2f::Identity(),    m);
//   double q[4];     ZF::toCArray(Eigen::Quaternionf::Identity(), q);
template <typename Derived, typename U, std::size_t N,
          enable_if_eigen_vector_t<Derived> = 0, enable_if_arithmetic_t<U> = 0>
inline void toCArray(const Eigen::MatrixBase<Derived>& vec, U (&out)[N]);

template <typename Derived, typename U, std::size_t Rows, std::size_t Cols,
          enable_if_eigen_matrix_t<Derived> = 0, enable_if_arithmetic_t<U> = 0>
inline void toCArray(const Eigen::MatrixBase<Derived>& m, U (&out)[Rows][Cols]);

template <typename Derived, typename U, std::size_t N,
          enable_if_eigen_quaternion_t<Derived> = 0, enable_if_arithmetic_t<U> = 0>
inline void toCArray(const Eigen::QuaternionBase<Derived>& q, U (&out)[N]);

// 将各种线性容器转换为目标 Eigen 向量类型。
// - 目标类型必须显式指定为固定尺寸 Eigen 向量。
// - 输入元素数必须等于目标维度（指针重载假定调用方已保证）。
// - std::vector<std::string> / QStringList 重载将每个字符串按浮点解析后转换。
// 示例：
//   auto v1 = ZF::toEigenVector<Eigen::Vector3f>(std::array<double, 3>{1, 2, 3});
//   double  raw[3] = {1, 2, 3};
//   auto v2 = ZF::toEigenVector<Eigen::Vector3f>(raw);
//   auto v3 = ZF::toEigenVector<Eigen::Vector3f>(std::vector<double>{1, 2, 3});
//   auto v4 = ZF::toEigenVector<Eigen::Vector3f>(std::vector<std::string>{"1.0", "2.0", "3.0"});
//   auto v5 = ZF::toEigenVector<Eigen::Vector3f>(QStringList{"1.0", "2.0", "3.0"});
template <typename Target, typename U, std::size_t N,
          enable_if_eigen_vector_t<Target> = 0, enable_if_arithmetic_t<U> = 0>
inline Target toEigenVector(const std::array<U, N>& arr);

template <typename Target, typename U, std::size_t N,
          enable_if_eigen_vector_t<Target> = 0, enable_if_arithmetic_t<U> = 0>
inline Target toEigenVector(const U (&arr)[N]);

template <typename Target, typename U,
          enable_if_eigen_vector_t<Target> = 0,
          std::enable_if_t<!std::is_void<std::remove_cv_t<U>>{} && std::is_arithmetic<std::remove_cv_t<U>>{}, int> = 0>
inline Target toEigenVector(const U* const& arr);

template <typename Target, typename U,
          enable_if_eigen_vector_t<Target> = 0, enable_if_arithmetic_t<U> = 0>
inline Target toEigenVector(const std::vector<U>& arr);

template <typename Target, enable_if_eigen_vector_t<Target> = 0>
inline Target toEigenVector(const std::vector<std::string>& arr);

template <typename Target, enable_if_eigen_vector_t<Target> = 0>
inline Target toEigenVector(const QStringList& arr);

// 将固定尺寸 Eigen 向量 / 矩阵 / 四元数转换为 QJsonArray。
// - 矩阵按 [row][col] 嵌套；四元数顺序固定为 {x, y, z, w}。
// 示例：
//   QJsonArray j1 = ZF::toQJsonArray(Eigen::Vector3f(1, 2, 3));
//   QJsonArray j2 = ZF::toQJsonArray(Eigen::Matrix2f::Identity());
//   QJsonArray j3 = ZF::toQJsonArray(Eigen::Quaternionf::Identity());
template <typename Derived, enable_if_eigen_vector_t<Derived> = 0>
inline QJsonArray toQJsonArray(const Eigen::MatrixBase<Derived>& vec);

template <typename Derived, enable_if_eigen_matrix_t<Derived> = 0>
inline QJsonArray toQJsonArray(const Eigen::MatrixBase<Derived>& m);

template <typename Derived, enable_if_eigen_quaternion_t<Derived> = 0>
inline QJsonArray toQJsonArray(const Eigen::QuaternionBase<Derived>& q);

// 用 Qt 占位符模板拼接固定尺寸 Eigen 向量 / 四元数为 QString。
// - format 中的 %1、%2 ... 按顺序对应每个元素，占位符个数须与元素数一致。
// - 四元数顺序固定为 {x, y, z, w}；元素一律先 static_cast<double> 再走 QString::arg。
// 示例：
//   QString s = ZF::toQString(Eigen::Vector3f(1, 2, 3), "%1 %2 %3");
//   // 等价于：QString("%1 %2 %3").arg(1.0).arg(2.0).arg(3.0)
template <typename Derived, enable_if_eigen_vector_t<Derived> = 0>
inline QString toQString(const Eigen::MatrixBase<Derived>& vec, const char* format);

template <typename Derived, enable_if_eigen_quaternion_t<Derived> = 0>
inline QString toQString(const Eigen::QuaternionBase<Derived>& q, const char* format);

// 将 Eigen 矩阵拼接为 QString：
// 示例：对 Eigen::Matrix3f m =
//   1 2 3
//   4 5 6
//   7 8 9
// 调用：
//   ZF::toQString(m, ", ", "\n  ", "[\n  ", "\n]");
// 得到：
//   [
//     1, 2, 3
//     4, 5, 6
//     7, 8, 9
//   ]
//
// 其它示例：
//   QString m1 = ZF::toQString(Eigen::Matrix2f::Identity());                           // "1 0\n0 1"
//   QString m2 = ZF::toQString(Eigen::Matrix2f::Identity(), ", ", "; ");               // "1, 0; 0, 1"
//   QString m3 = ZF::toQString(Eigen::Matrix2f::Identity(), ", ", "], [", "[[", "]]"); // "[[1, 0], [0, 1]]"
// 同时支持动态尺寸矩阵（如 Eigen::MatrixXd），按运行期 rows()/cols() 遍历。
template <typename Derived, enable_if_eigen_matrix_any_t<Derived> = 0>
inline QString toQString(const Eigen::MatrixBase<Derived>& m,
                         const QString& elementSeparator = QStringLiteral(" "),
                         const QString& rowSeparator     = QStringLiteral("\n"),
                         const QString& prefix           = QString(),
                         const QString& suffix           = QString());

// std::string 版本：接口与 toQString 对齐，不依赖 Qt。
// 示例：
//   std::string s1 = ZF::toStdString(Eigen::Vector3f(1, 2, 3));                       // "1 2 3"
//   std::string s2 = ZF::toStdString(Eigen::Vector3f(1, 2, 3), ", ", "(", ")");      // "(1, 2, 3)"
//   std::string s3 = ZF::toStdString(Eigen::Quaternionf::Identity());                  // "0 0 0 1"
//   std::string s4 = ZF::toStdString(Eigen::Matrix2f::Identity(), ", ", "; ");        // "1, 0; 0, 1"
template <typename Derived, enable_if_eigen_vector_t<Derived> = 0>
inline std::string toStdString(const Eigen::MatrixBase<Derived>& vec,
                               const std::string& elementSeparator = " ",
                               const std::string& prefix           = std::string(),
                               const std::string& suffix           = std::string());

template <typename Derived, enable_if_eigen_quaternion_t<Derived> = 0>
inline std::string toStdString(const Eigen::QuaternionBase<Derived>& q,
                               const std::string& elementSeparator = " ",
                               const std::string& prefix           = std::string(),
                               const std::string& suffix           = std::string());

template <typename Derived, enable_if_eigen_matrix_any_t<Derived> = 0>
inline std::string toStdString(const Eigen::MatrixBase<Derived>& m,
                               const std::string& elementSeparator = " ",
                               const std::string& rowSeparator     = "\n",
                               const std::string& prefix           = std::string(),
                               const std::string& suffix           = std::string());

} // namespace ZF

#include <Core/Utils/Convert.inl>
