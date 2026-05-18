#pragma once

namespace ZF
{
namespace Math
{

template <typename DTranslation>
inline Eigen::Matrix<eigen_scalar_t<DTranslation>, 4, 4>
makeTranslateMatrix4x4(const Eigen::MatrixBase<DTranslation>& translation)
{
    auto transform = Eigen::Transform<eigen_scalar_t<DTranslation>, 3, Eigen::Affine>::Identity();
    transform.translation() = translation;
    return transform.matrix();
}

inline Eigen::Matrix4f makeTranslateMatrix4x4(float x, float y, float z)
{
    return makeTranslateMatrix4x4(Eigen::Vector3f(x, y, z));
}

} // namespace Math
} // namespace ZF