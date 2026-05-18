#pragma once

/// @file
/// @brief 矩阵转换辅助函数（从矩阵转换到其他表示）。

#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

/// @brief 将任意 3x3 矩阵扩展为 4x4 齐次矩阵。
///
/// 示例：
///   Matrix3f R = makeRotationMatrix3x3(Vector3f::UnitZ(), 0.5f);
///   Matrix4f M = Matrix3x3ToMatrix4x4(R); // 左上 3x3 = R，其余为单位阵
template <typename DMatrix3, enable_if_eigen_matrix3_t<DMatrix3> = 0>
inline Eigen::Matrix<eigen_scalar_t<DMatrix3>, 4, 4>
Matrix3x3ToMatrix4x4(const Eigen::MatrixBase<DMatrix3>& matrix)
{
    Eigen::Matrix<eigen_scalar_t<DMatrix3>, 4, 4> result =
        Eigen::Matrix<eigen_scalar_t<DMatrix3>, 4, 4>::Identity();
    result.template topLeftCorner<3, 3>() = matrix;
    return result;
}

/// @brief 提取 4x4 矩阵左上角的 3x3 线性部分。
///
/// 示例：
///   Matrix3f L = Matrix4x4ToMatrix3x3(M);
template <typename DMatrix4, enable_if_eigen_matrix4_t<DMatrix4> = 0>
inline Eigen::Matrix<eigen_scalar_t<DMatrix4>, 3, 3>
Matrix4x4ToMatrix3x3(const Eigen::MatrixBase<DMatrix4>& matrix)
{
    return matrix.template topLeftCorner<3, 3>();
}

/// @brief 将 3x3 旋转矩阵转换为四元数。
///
/// 示例：
///   Quaternionf q = Matrix3x3ToQuaternion(R);
template <typename DRotation, enable_if_eigen_matrix3_t<DRotation> = 0>
inline Eigen::Quaternion<eigen_scalar_t<DRotation>>
Matrix3x3ToQuaternion(const Eigen::MatrixBase<DRotation>& rotation)
{
    return Eigen::Quaternion<eigen_scalar_t<DRotation>>(rotation);
}

/// @brief 从 3x3 旋转矩阵提取 XYZ 顺序的弧度制欧拉角。
///
/// 示例：
///   Vector3f euler = Matrix3x3ToEulerXYZRadians(R); // (rx, ry, rz)
template <typename DRotation, enable_if_eigen_matrix3_t<DRotation> = 0>
inline Eigen::Matrix<eigen_scalar_t<DRotation>, 3, 1>
Matrix3x3ToEulerXYZRadians(const Eigen::MatrixBase<DRotation>& matrix)
{
    const Eigen::Matrix<eigen_scalar_t<DRotation>, 3, 1> zyx = matrix.eulerAngles(2, 1, 0);
    return Eigen::Matrix<eigen_scalar_t<DRotation>, 3, 1>(zyx.z(), zyx.y(), zyx.x());
}

/// @brief 从 4x4 齐次旋转矩阵提取 XYZ 顺序的弧度制欧拉角。
///
/// 示例：
///   Vector3f euler = Matrix4x4ToEulerXYZRadians(M);
template <typename DRotation, enable_if_eigen_matrix4_t<DRotation> = 0>
inline Eigen::Matrix<eigen_scalar_t<DRotation>, 3, 1>
Matrix4x4ToEulerXYZRadians(const Eigen::MatrixBase<DRotation>& matrix)
{
    const Eigen::Matrix<eigen_scalar_t<DRotation>, 3, 3> rotation = matrix.template topLeftCorner<3, 3>();
    return Matrix3x3ToEulerXYZRadians(rotation);
}

} // namespace Math
} // namespace ZF
