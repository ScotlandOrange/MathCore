#pragma once

/// @file
/// @brief Eigen 相关模板特质与 SFINAE 辅助类型。

#include <array>

#include <Eigen/Core>
#include <type_traits>
#include <vector>

namespace ZF
{
template <bool Cond, typename = void>
struct static_bool
{
    static constexpr bool value = false;
};

template <bool Cond>
struct static_bool<Cond, std::enable_if_t<Cond>>
{
    static constexpr bool value = true;
};

template <bool Cond>
constexpr bool static_bool_v = static_bool<Cond>::value;

template <typename T>
struct is_vector : std::false_type
{
};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type
{
};

template <typename Derived>
using eigen_scalar_t = typename Derived::Scalar;

/// @brief 判断 Derived 是否为固定尺寸 Eigen 列向量，且标量为算术类型。
template <typename Derived>
using is_eigen_vector = std::integral_constant<bool,
                                               Derived::IsVectorAtCompileTime &&
                                               (Derived::SizeAtCompileTime != Eigen::Dynamic) &&
                                               std::is_arithmetic<eigen_scalar_t<Derived>>{}>;

/// @brief 判断 Derived 是否为固定尺寸 Eigen 矩阵（非向量），且标量为算术类型。
template <typename Derived>
using is_eigen_matrix = std::integral_constant<bool,
                                               (Derived::IsVectorAtCompileTime == false) &&
                                               (Derived::RowsAtCompileTime != Eigen::Dynamic) &&
                                               (Derived::ColsAtCompileTime != Eigen::Dynamic) &&
                                               std::is_arithmetic<eigen_scalar_t<Derived>>{}>;

/// @brief 判断 Derived 是否为 Eigen 矩阵（允许动态尺寸，非向量），且标量为算术类型。
template <typename Derived>
using is_eigen_matrix_any = std::integral_constant<bool,
                                                   (Derived::IsVectorAtCompileTime == false) &&
                                                   std::is_arithmetic<eigen_scalar_t<Derived>>{}>;

/// @brief 判断 Derived 是否为固定尺寸 3x3 Eigen 矩阵，且标量为算术类型。
template <typename Derived>
struct is_eigen_matrix3 : std::false_type
{
};

template <typename Scalar, int Options, int MaxRows, int MaxCols>
struct is_eigen_matrix3<Eigen::Matrix<Scalar, 3, 3, Options, MaxRows, MaxCols>>
    : std::integral_constant<bool, std::is_arithmetic<Scalar>{}>
{
};

/// @brief 判断 Derived 是否为固定尺寸 4x4 Eigen 矩阵，且标量为算术类型。
template <typename Derived>
struct is_eigen_matrix4 : std::false_type
{
};

template <typename Scalar, int Options, int MaxRows, int MaxCols>
struct is_eigen_matrix4<Eigen::Matrix<Scalar, 4, 4, Options, MaxRows, MaxCols>>
    : std::integral_constant<bool, std::is_arithmetic<Scalar>{}>
{
};

/// @brief 判断 Derived 是否为 Eigen 四元数，且标量为算术类型。
template <typename Derived>
struct is_eigen_quaternion : std::false_type
{
};

template <typename Scalar, int Options>
struct is_eigen_quaternion<Eigen::Quaternion<Scalar, Options>>
    : std::integral_constant<bool, std::is_arithmetic<Scalar>{}>
{
};

template <typename Derived>
constexpr bool is_eigen_vector_v = is_eigen_vector<Derived>{};

template <typename Derived>
constexpr bool is_eigen_matrix_v = is_eigen_matrix<Derived>{};

template <typename Derived>
constexpr bool is_eigen_matrix_any_v = is_eigen_matrix_any<Derived>{};

template <typename Derived>
constexpr bool is_eigen_matrix3_v = is_eigen_matrix3<Derived>{};

template <typename Derived>
constexpr bool is_eigen_matrix4_v = is_eigen_matrix4<Derived>{};

template <typename Derived>
constexpr bool is_eigen_quaternion_v = is_eigen_quaternion<Derived>{};

/// @brief SFINAE 约束：仅对固定尺寸 Eigen 向量启用重载。
template <typename Derived>
using enable_if_eigen_vector_t = std::enable_if_t<is_eigen_vector_v<Derived>, int>;

/// @brief SFINAE 约束：仅对固定尺寸 Eigen 矩阵启用重载。
template <typename Derived>
using enable_if_eigen_matrix_t = std::enable_if_t<is_eigen_matrix_v<Derived>, int>;

/// @brief SFINAE 约束：对 Eigen 矩阵启用重载（允许动态尺寸，非向量）。
template <typename Derived>
using enable_if_eigen_matrix_any_t = std::enable_if_t<is_eigen_matrix_any_v<Derived>, int>;

/// @brief SFINAE 约束：仅对固定尺寸 3x3 Eigen 矩阵启用重载。
template <typename Derived>
using enable_if_eigen_matrix3_t = std::enable_if_t<is_eigen_matrix3_v<Derived>, int>;

/// @brief SFINAE 约束：仅对固定尺寸 4x4 Eigen 矩阵启用重载。
template <typename Derived>
using enable_if_eigen_matrix4_t = std::enable_if_t<is_eigen_matrix4_v<Derived>, int>;

/// @brief SFINAE 约束：仅对 Eigen 四元数启用重载。
template <typename Derived>
using enable_if_eigen_quaternion_t = std::enable_if_t<is_eigen_quaternion_v<Derived>, int>;

/// @brief SFINAE 约束：仅对算术标量类型启用重载。
template <typename U>
using enable_if_arithmetic_t = std::enable_if_t<std::is_arithmetic<U>{}, int>;

/// @brief 判断 T 是否为 `std::array`。
template <typename T>
struct is_std_array : std::false_type
{
};

template <typename U, std::size_t N>
struct is_std_array<std::array<U, N>> : std::true_type
{
};

template <typename T>
constexpr bool is_std_array_v = is_std_array<T>{};

/// @brief 判断 T 是否为元素为算术类型的 `std::array`。
template <typename T>
struct is_arithmetic_std_array : std::false_type
{
};

template <typename U, std::size_t N>
struct is_arithmetic_std_array<std::array<U, N>> : std::integral_constant<bool, std::is_arithmetic<U>{}>
{
};

template <typename T>
constexpr bool is_arithmetic_std_array_v = is_arithmetic_std_array<T>{};

/// @brief 判断 T 是否为二维嵌套且最内层为算术类型的 `std::array`。
template <typename T>
struct is_nested_arithmetic_std_array : std::false_type
{
};

template <typename Row, std::size_t N>
struct is_nested_arithmetic_std_array<std::array<Row, N>> : is_arithmetic_std_array<Row>
{
};

template <typename T>
constexpr bool is_nested_arithmetic_std_array_v = is_nested_arithmetic_std_array<T>{};

} // namespace ZF