#pragma once

/// @file
/// @brief 二维与三维向量夹角计算（无符号 / 有符号）。

#include <Core/TypeTraits.h>

#include <Eigen/Core>

namespace ZF
{
namespace Math
{

/// @brief 三维有符号夹角的叉积顺序约定，仅决定符号。
enum class CrossProductOrder
{
    SrcCrossDst,
    DstCrossSrc,
};

// 二维向量夹角：无符号 [0, pi]，有符号 [-pi, pi]（src 逆时针到 dst 为正）。
// 示例：
//   double a = angleBetween2D       (Vector2d::UnitX(), Vector2d::UnitY()); // pi/2
//   double s = signedAngleBetween2D (Vector2d::UnitX(), Vector2d::UnitY()); // +pi/2
template <typename DA, typename DB>
inline eigen_scalar_t<DA> angleBetween2D      (const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst);

template <typename DA, typename DB>
inline eigen_scalar_t<DA> signedAngleBetween2D(const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst);

// 三维向量夹角：无符号 [0, pi]，有符号 [-pi, pi]（叉积顺序仅决定符号）。
// 示例：
//   double a  = angleBetween3D      (Vector3d::UnitX(), Vector3d::UnitY());                                    // pi/2
//   double s1 = signedAngleBetween3D(Vector3d::UnitX(), Vector3d::UnitY(), CrossProductOrder::SrcCrossDst);    // +pi/2
//   double s2 = signedAngleBetween3D(Vector3d::UnitX(), Vector3d::UnitY(), CrossProductOrder::DstCrossSrc);    // -pi/2
template <typename DA, typename DB>
inline eigen_scalar_t<DA> angleBetween3D      (const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst);

template <typename DA, typename DB>
inline eigen_scalar_t<DA> signedAngleBetween3D(const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst,
                                               CrossProductOrder            crossProductOrder);

} // namespace Math
} // namespace ZF

#include <Core/Math/detail/Angle.inl>
