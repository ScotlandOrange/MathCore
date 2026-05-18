#pragma once

/// @file
/// @brief 旋转、朝向与欧拉角辅助函数。

#include <Core/Math/Matrix.h>
#include <Core/Math/Numbers.h>
#include <Core/Math/Quaternion.h>
#include <Core/ZFAssert.h>
#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace ZF
{
namespace Math
{
// clang-format off
/// @brief 指定局部哪根轴与目标方向对齐。
enum class OrientationAxis
{
    X,
    Y,
    Z,
};

// 围绕任意轴旋转指定弧度。
// 示例：
//   auto R = makeRotationMatrix3x3(Vector3f::UnitZ(), Numbersf::PI_2()); // 绕 Z 转 90°
inline Eigen::Matrix3f    makeRotationMatrix3x3 (const Eigen::Vector3f& axis, float radians);
inline Eigen::Matrix4f    makeRotationMatrix4x4 (const Eigen::Vector3f& axis, float radians);
inline Eigen::Quaternionf makeRotationQuaternion(const Eigen::Vector3f& axis, float radians);

// 根据目标方向、参考上方向和局部对齐轴构造旋转。
// 示例：让相机的 -Z（OrientationAxis::Z）指向 target，世界 Y 作为参考上方向
//   auto R = makeRotationFromDirectionMatrix3x3(target - eye, Vector3f::UnitY(), OrientationAxis::Z);
inline Eigen::Matrix3f    makeRotationFromDirectionMatrix3x3 (const Eigen::Vector3f& direction,
                                                              const Eigen::Vector3f& referenceUp,
                                                              OrientationAxis orientationAxis);
inline Eigen::Matrix4f    makeRotationFromDirectionMatrix4x4 (const Eigen::Vector3f& direction,
                                                              const Eigen::Vector3f& referenceUp,
                                                              OrientationAxis orientationAxis);
inline Eigen::Quaternionf makeRotationFromDirectionQuaternion(const Eigen::Vector3f& direction,
                                                              const Eigen::Vector3f& referenceUp,
                                                              OrientationAxis orientationAxis);

// 由三条单位化且两两正交的右手基轴构造旋转。
// 示例：
//   auto R = makeRotationFromOrthoAxesMatrix3x3(xAxis, yAxis, zAxis); // 列向量依次为局部 x/y/z
inline Eigen::Matrix3f    makeRotationFromOrthoAxesMatrix3x3 (const Eigen::Vector3f& x,
                                                              const Eigen::Vector3f& y,
                                                              const Eigen::Vector3f& z);
inline Eigen::Matrix4f    makeRotationFromOrthoAxesMatrix4x4 (const Eigen::Vector3f& x,
                                                              const Eigen::Vector3f& y,
                                                              const Eigen::Vector3f& z);
inline Eigen::Quaternionf makeRotationFromOrthoAxesQuaternion(const Eigen::Vector3f& x,
                                                              const Eigen::Vector3f& y,
                                                              const Eigen::Vector3f& z);

// 让局部 X 轴对齐到指定方向（其余两轴自动取垂直方向）。
// 示例：
//   auto R = makeRotationFromXAxisMatrix3x3(Vector3f::UnitY()); // 局部 X 对齐到世界 Y
inline Eigen::Matrix3f    makeRotationFromXAxisMatrix3x3 (const Eigen::Vector3f& axis);
inline Eigen::Matrix4f    makeRotationFromXAxisMatrix4x4 (const Eigen::Vector3f& axis);
inline Eigen::Quaternionf makeRotationFromXAxisQuaternion(const Eigen::Vector3f& axis);

// 按 XYZ 顺序的弧度制欧拉角构造旋转（先绕 X，再绕 Y，最后绕 Z）。
// 示例：
//   auto R = makeRotationFromEulerRadiansMatrix3x3(Vector3f(0.3f, -0.2f, 0.1f));
inline Eigen::Matrix3f    makeRotationFromEulerRadiansMatrix3x3 (const Eigen::Vector3f& angles);
inline Eigen::Matrix4f    makeRotationFromEulerRadiansMatrix4x4 (const Eigen::Vector3f& angles);
inline Eigen::Quaternionf makeRotationFromEulerRadiansQuaternion(const Eigen::Vector3f& angles);

// 按 XYZ 顺序的角度制欧拉角构造旋转。
// 示例：
//   auto R = makeRotationFromEulerDegreesQuaternion(Vector3f(30.0f, -45.0f, 90.0f));
inline Eigen::Matrix3f    makeRotationFromEulerDegreesMatrix3x3 (const Eigen::Vector3f& angles);
inline Eigen::Matrix4f    makeRotationFromEulerDegreesMatrix4x4 (const Eigen::Vector3f& angles);
inline Eigen::Quaternionf makeRotationFromEulerDegreesQuaternion(const Eigen::Vector3f& angles);

/// @brief 围绕过 point 且方向为 axis 的轴旋转指定弧度，生成 4x4 变换矩阵。
///
/// 示例：绕世界点 (1,0,0) 沿 Z 轴旋转 90°
///   auto T = makeRotationMatrixAroundPoint(Vector3f(1,0,0), Vector3f::UnitZ(), Numbersf::PI_2());
inline Eigen::Matrix4f makeRotationMatrixAroundPoint(const Eigen::Vector3f& point,
                                                     const Eigen::Vector3f& axis,
                                                     float radians);

} // namespace Math
} // namespace ZF

#include <Core/Math/detail/Rotation.inl>

// clang-format on