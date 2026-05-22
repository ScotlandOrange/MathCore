#pragma once

/// @file
/// @brief Eigen 相关模板特质与 SFINAE 辅助类型。

#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <type_traits>
#include <vector>

namespace ZF
{
/// @brief C++14 版本的 std::void_t。
template <typename...>
using void_t = void;

/// @brief 移除引用和 cv 限定，等价于 C++20 std::remove_cvref_t。
template <typename T>
using uncvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

/// @brief C++14 remove_pointer_t 别名，保持项目内模板写法统一。
template <typename T>
using remove_pointer_t = typename std::remove_pointer<T>::type;

/// @brief SFINAE 约束短写，默认产生 int 类型，适合作为非类型模板参数。
template <bool Cond>
using enable_if_t = typename std::enable_if<Cond, int>::type;

/// @brief 判断 T 是否为完整类型。
template <typename T, typename = std::size_t>
struct is_complete : std::false_type
{
};

template <typename T>
struct is_complete<T, decltype(sizeof(T))> : std::true_type
{
};

template <typename T>
constexpr bool is_complete_v = is_complete<T>{};

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

/// @brief 判断 Derived 是否为固定尺寸 3x3 Eigen 矩阵或表达式，且标量为算术类型。
template <typename Derived>
using is_eigen_matrix3 = std::integral_constant<bool,
                                                (Derived::IsVectorAtCompileTime == false) &&
                                                (Derived::RowsAtCompileTime == 3) &&
                                                (Derived::ColsAtCompileTime == 3) &&
                                                std::is_arithmetic<eigen_scalar_t<Derived>>{}>;

/// @brief 判断 Derived 是否为固定尺寸 4x4 Eigen 矩阵或表达式，且标量为算术类型。
template <typename Derived>
using is_eigen_matrix4 = std::integral_constant<bool,
                                                (Derived::IsVectorAtCompileTime == false) &&
                                                (Derived::RowsAtCompileTime == 4) &&
                                                (Derived::ColsAtCompileTime == 4) &&
                                                std::is_arithmetic<eigen_scalar_t<Derived>>{}>;

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
