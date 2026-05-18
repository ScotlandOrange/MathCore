#pragma once

/// @file
/// @brief 缩放相关变换辅助函数。

#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

// 由缩放分量生成缩放矩阵。
// 示例：
//   Vector3f s(2.0f, 3.0f, 4.0f);
//   Matrix3f S3 = makeScaleMatrix3x3(s); // diag(2,3,4)
//   Matrix4f S4 = makeScaleMatrix4x4(s); // 4x4，平移分量为 0
template <typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DScale>, 3, 3>
makeScaleMatrix3x3(const Eigen::MatrixBase<DScale>& scale);

template <typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DScale>, 4, 4>
makeScaleMatrix4x4(const Eigen::MatrixBase<DScale>& scale);

} // namespace Math
} // namespace ZF

#include <Core/Math/detail/Scale.inl>
