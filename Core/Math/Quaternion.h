#pragma once

/// @file
/// @brief 四元数转换辅助函数（从四元数转换到其他表示）。

#include <Core/Math/Matrix.h>
#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

/// @brief 将四元数转换为 4x4 齐次旋转矩阵。
///
/// 示例：
///   Quaternionf q(AngleAxisf(0.5f, Vector3f::UnitZ()));
///   Matrix4f M = QuaternionToMatrix4x4(q);
template <typename DQuaternion, enable_if_eigen_quaternion_t<DQuaternion> = 0>
inline Eigen::Matrix<eigen_scalar_t<DQuaternion>, 4, 4>
QuaternionToMatrix4x4(const Eigen::QuaternionBase<DQuaternion>& rotation)
{
    Eigen::Matrix<eigen_scalar_t<DQuaternion>, 4, 4> result =
        Eigen::Matrix<eigen_scalar_t<DQuaternion>, 4, 4>::Identity();
    result.template topLeftCorner<3, 3>() = rotation.toRotationMatrix();
    return result;
}

/// @brief 从四元数提取 XYZ 顺序的弧度制欧拉角。
///
/// 示例：
///   Vector3f euler = QuaternionToEulerXYZRadians(q);
template <typename DQuaternion, enable_if_eigen_quaternion_t<DQuaternion> = 0>
inline Eigen::Matrix<eigen_scalar_t<DQuaternion>, 3, 1>
QuaternionToEulerXYZRadians(const Eigen::QuaternionBase<DQuaternion>& rotation)
{
    return Matrix3x3ToEulerXYZRadians(rotation.toRotationMatrix());
}

} // namespace Math
} // namespace ZF
