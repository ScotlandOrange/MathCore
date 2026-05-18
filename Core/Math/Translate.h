#pragma once

/// @file
/// @brief 平移相关变换辅助函数。

#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

// 由平移分量生成 4x4 平移矩阵。
// 示例：
//   Matrix4f T1 = makeTranslateMatrix4x4(Vector3f(10.0f, 20.0f, 30.0f));
//   Matrix4f T2 = makeTranslateMatrix4x4(10.0f, 20.0f, 30.0f);
template <typename DTranslation>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTranslateMatrix4x4(const Eigen::MatrixBase<DTranslation>& translation);

inline Eigen::Matrix4f makeTranslateMatrix4x4(float x, float y, float z);

} // namespace Math
} // namespace ZF

#include <Core/Math/detail/Translate.inl>
