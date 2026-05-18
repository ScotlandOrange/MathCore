#pragma once

namespace ZF
{
namespace Math
{

template <typename DTranslation, typename DRotation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransform(const Eigen::MatrixBase<DTranslation>& translation,
              const Eigen::MatrixBase<DRotation>& rotation,
              const Eigen::MatrixBase<DScale>& scale)
{
    auto transform = Eigen::Transform<eigen_scalar_t<DTranslation>, 3, Eigen::Affine>::Identity();
    transform.linear() = rotation * scale.asDiagonal();
    transform.translation() = translation;
    return transform.matrix();
}

template <typename DTranslation, typename DQuaternion, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransform(const Eigen::MatrixBase<DTranslation>& translation,
              const Eigen::QuaternionBase<DQuaternion>& rotation,
              const Eigen::MatrixBase<DScale>& scale)
{
    return makeTransform(translation, rotation.toRotationMatrix(), scale);
}

template <typename DTranslation, typename DRotation>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndRot(const Eigen::MatrixBase<DTranslation>& translation,
                             const Eigen::MatrixBase<DRotation>& rotation)
{
    auto transform = Eigen::Transform<eigen_scalar_t<DTranslation>, 3, Eigen::Affine>::Identity();
    transform.linear() = rotation;
    transform.translation() = translation;
    return transform.matrix();
}

template <typename DTranslation, typename DQuaternion>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndRot(const Eigen::MatrixBase<DTranslation>& translation,
                             const Eigen::QuaternionBase<DQuaternion>& rotation)
{
    return makeTransformFromTransAndRot(translation, rotation.toRotationMatrix());
}

template <typename DTranslation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTransformFromTransAndScale(const Eigen::MatrixBase<DTranslation>& translation,
                               const Eigen::MatrixBase<DScale>& scale)
{
    auto transform = Eigen::Transform<eigen_scalar_t<DTranslation>, 3, Eigen::Affine>::Identity();
    transform.linear() = scale.asDiagonal();
    transform.translation() = translation;
    return transform.matrix();
}

template <typename DRotation, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DRotation>, 4, 4>
makeTransformFromRotAndScale(const Eigen::MatrixBase<DRotation>& rotation,
                             const Eigen::MatrixBase<DScale>& scale)
{
    auto transform = Eigen::Transform<eigen_scalar_t<DRotation>, 3, Eigen::Affine>::Identity();
    transform.linear() = rotation * scale.asDiagonal();
    return transform.matrix();
}

template <typename DQuaternion, typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DQuaternion>, 4, 4>
makeTransformFromRotAndScale(const Eigen::QuaternionBase<DQuaternion>& rotation,
                             const Eigen::MatrixBase<DScale>& scale)
{
    return makeTransformFromRotAndScale(rotation.toRotationMatrix(), scale);
}

inline Eigen::Matrix4f Coord3(const Eigen::Vector3f& translation,
                              const Eigen::Matrix3f& rotation,
                              const Eigen::Vector3f& scale)
{
    return makeTransform(translation, rotation, scale);
}

inline Eigen::Vector3f MatrixMulPoint(const Eigen::Matrix4f& matrix,
                                      const Eigen::Vector3f& point)
{
    const Eigen::Vector4f homogeneous = matrix * point.homogeneous();
    if (std::abs(homogeneous.w()) <= Numbersf::epsilon())
    {
        return homogeneous.head<3>();
    }

    return homogeneous.hnormalized();
}

inline Eigen::Vector3f MatrixMulDir(const Eigen::Matrix4f& matrix,
                                    const Eigen::Vector3f& dir)
{
    const Eigen::Vector3f transformed = matrix.topLeftCorner<3, 3>() * dir;
    return transformed.normalized();
}

} // namespace Math
} // namespace ZF