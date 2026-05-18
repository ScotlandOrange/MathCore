#pragma once

namespace ZF
{
namespace Math
{

template <typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DScale>, 3, 3>
makeScaleMatrix3x3(const Eigen::MatrixBase<DScale>& scale)
{
    return scale.asDiagonal();
}

template <typename DScale>
inline Eigen::Matrix<eigen_scalar_t<DScale>, 4, 4>
makeScaleMatrix4x4(const Eigen::MatrixBase<DScale>& scale)
{
    Eigen::Matrix<eigen_scalar_t<DScale>, 4, 4> result = Eigen::Matrix<eigen_scalar_t<DScale>, 4, 4>::Identity();
    result.template topLeftCorner<3, 3>() = makeScaleMatrix3x3(scale);
    return result;
}

} // namespace Math
} // namespace ZF