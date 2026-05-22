#pragma once

/// @file
/// @brief 综合仿射变换辅助函数，并聚合平移、旋转、缩放相关能力。

#include <Core/Math/Matrix.h>
#include <Core/Math/Numbers.h>
#include <Core/Math/Rotation.h>
#include <Core/Math/Scale.h>
#include <Core/Math/Translate.h>
#include <Core/Sphere.h>
#include <Core/TypeTraits.h>

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

// 由平移、旋转、缩放组合 4x4 仿射变换；线性部分按 rotation * scale.asDiagonal() 构造。
// 示例：
//   Vector3f t(1, 2, 3);
//   Matrix3f R = makeRotationMatrix3x3(Vector3f::UnitZ(), 0.5f);
//   Quaternionf q(AngleAxisf(0.5f, Vector3f::UnitZ()));
//   Vector3f s(2, 3, 4);
//   Matrix4f M1 = makeTransform(t, R, s);
//   Matrix4f M2 = makeTransform(t, q, s);
template <typename DTranslation, typename DRotation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransform(const Eigen::MatrixBase<DTranslation>& translation,
              const Eigen::MatrixBase<DRotation>&    rotation,
              const Eigen::MatrixBase<DScale>&       scale);

template <typename DTranslation, typename DQuaternion, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransform(const Eigen::MatrixBase<DTranslation>&     translation,
              const Eigen::QuaternionBase<DQuaternion>&  rotation,
              const Eigen::MatrixBase<DScale>&           scale);

// 由两两组合的分量构造 4x4 仿射变换。
// 示例：
//   Matrix4f TR = makeTransformFromTransAndRot(t, R);    // 也可传 Quaternionf
//   Matrix4f TS = makeTransformFromTransAndScale(t, s);
//   Matrix4f RS = makeTransformFromRotAndScale(R, s);    // 也可传 Quaternionf
template <typename DTranslation, typename DRotation>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndRot(const Eigen::MatrixBase<DTranslation>& translation,
                             const Eigen::MatrixBase<DRotation>&    rotation);

template <typename DTranslation, typename DQuaternion>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndRot(const Eigen::MatrixBase<DTranslation>&    translation,
                             const Eigen::QuaternionBase<DQuaternion>& rotation);

template <typename DTranslation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndScale(const Eigen::MatrixBase<DTranslation>& translation,
                               const Eigen::MatrixBase<DScale>&       scale);

template <typename DRotation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DRotation>, 4, 4>
makeTransformFromRotAndScale(const Eigen::MatrixBase<DRotation>& rotation,
                             const Eigen::MatrixBase<DScale>&    scale);

template <typename DQuaternion, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DQuaternion>, 4, 4>
makeTransformFromRotAndScale(const Eigen::QuaternionBase<DQuaternion>& rotation,
                             const Eigen::MatrixBase<DScale>&          scale);

// 用 4x4 矩阵变换三维点（带齐次除法）、方向向量（取左上 3x3 后归一化）和包围球。
// 示例：
//   Vector3f p2 = transformPoint(M, Vector3f(1, 2, 3));
//   Vector3f d2 = transformDirection(M, Vector3f::UnitX());
//   Sphere    s2 = transformSphere(M, sphere);
template <typename DMatrix, typename DPoint,
          enable_if_eigen_matrix4_t<DMatrix> = 0,
          enable_if_eigen_vector_t<DPoint> = 0>
inline Eigen::Matrix<eigen_scalar_t<DMatrix>, 3, 1>
transformPoint(const Eigen::MatrixBase<DMatrix>& matrix,
               const Eigen::MatrixBase<DPoint>&  point)
{
    static_assert(DPoint::SizeAtCompileTime == 3, "transformPoint expects a 3D point.");

    using Scalar = eigen_scalar_t<DMatrix>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

    const Vector3 localPoint = point.template cast<Scalar>();
    const Vector4 transformed = matrix.derived() * localPoint.homogeneous();
    const Scalar invW = (std::abs(transformed.w()) > Numbers<Scalar>::epsilon())
        ? (Scalar(1) / transformed.w())
        : Scalar(1);

    return transformed.template head<3>() * invW;
}

template <typename DMatrix, typename DDirection,
          enable_if_eigen_matrix4_t<DMatrix> = 0,
          enable_if_eigen_vector_t<DDirection> = 0>
inline Eigen::Matrix<eigen_scalar_t<DMatrix>, 3, 1>
transformDirection(const Eigen::MatrixBase<DMatrix>&    matrix,
                   const Eigen::MatrixBase<DDirection>& direction)
{
    static_assert(DDirection::SizeAtCompileTime == 3, "transformDirection expects a 3D direction.");

    using Scalar = eigen_scalar_t<DMatrix>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Vector3 transformed = matrix.template topLeftCorner<3, 3>() * direction.template cast<Scalar>();
    const Scalar length = transformed.norm();
    if (length > Numbers<Scalar>::epsilon())
    {
        transformed /= length;
    }
    return transformed;
}

template <typename DMatrix, typename T,
          enable_if_eigen_matrix4_t<DMatrix> = 0>
inline TSphere<eigen_scalar_t<DMatrix>>
transformSphere(const Eigen::MatrixBase<DMatrix>& matrix,
                const TSphere<T>&                 sphere)
{
    using Scalar = eigen_scalar_t<DMatrix>;
    using SphereType = TSphere<Scalar>;

    if (!sphere.valid()) return SphereType();

    const Eigen::Matrix<Scalar, 3, 1> transformedCenter = transformPoint(matrix, sphere.center());
    const auto linear = matrix.template topLeftCorner<3, 3>();
    const Scalar maxScale = std::max(linear.col(0).norm(),
                                     std::max(linear.col(1).norm(), linear.col(2).norm()));

    return SphereType(transformedCenter, static_cast<Scalar>(sphere.radius()) * maxScale);
}

} // namespace Math
} // namespace ZF

#include <Core/Math/detail/Transform.inl>
