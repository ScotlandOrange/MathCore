#pragma once

#include <Core/Math/Numbers.h>

#include <cmath>

namespace ZF
{
namespace Math
{

template <typename DA, typename DB>
inline eigen_scalar_t<DA> angleBetween2D(const Eigen::MatrixBase<DA>& src,
                                         const Eigen::MatrixBase<DB>& dst)
{
    const Eigen::Vector<eigen_scalar_t<DA>, 2> srcv = src;
    const Eigen::Vector<eigen_scalar_t<DA>, 2> dstv = dst.template cast<eigen_scalar_t<DA>>();
    if (srcv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon()
        || dstv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon())
        return Numbers<eigen_scalar_t<DA>>::zero();

    const auto cross = srcv.x() * dstv.y() - srcv.y() * dstv.x();
    return std::atan2(std::abs(cross), srcv.dot(dstv));
}

template <typename DA, typename DB>
inline eigen_scalar_t<DA> signedAngleBetween2D(const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst)
{
    const Eigen::Vector<eigen_scalar_t<DA>, 2> srcv = src;
    const Eigen::Vector<eigen_scalar_t<DA>, 2> dstv = dst.template cast<eigen_scalar_t<DA>>();
    if (srcv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon()
        || dstv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon())
        return Numbers<eigen_scalar_t<DA>>::zero();

    const auto cross = srcv.x() * dstv.y() - srcv.y() * dstv.x();
    return std::atan2(cross, srcv.dot(dstv));
}

template <typename DA, typename DB>
inline eigen_scalar_t<DA> angleBetween3D(const Eigen::MatrixBase<DA>& src,
                                         const Eigen::MatrixBase<DB>& dst)
{
    const Eigen::Vector<eigen_scalar_t<DA>, 3> srcv = src;
    const Eigen::Vector<eigen_scalar_t<DA>, 3> dstv = dst.template cast<eigen_scalar_t<DA>>();
    if (srcv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon()
        || dstv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon())
        return Numbers<eigen_scalar_t<DA>>::zero();

    return std::atan2(srcv.cross(dstv).norm(), srcv.dot(dstv));
}

template <typename DA, typename DB>
inline eigen_scalar_t<DA> signedAngleBetween3D(const Eigen::MatrixBase<DA>& src,
                                               const Eigen::MatrixBase<DB>& dst,
                                               CrossProductOrder            crossProductOrder)
{
    const Eigen::Vector<eigen_scalar_t<DA>, 3> srcv = src;
    const Eigen::Vector<eigen_scalar_t<DA>, 3> dstv = dst.template cast<eigen_scalar_t<DA>>();
    if (srcv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon()
        || dstv.squaredNorm() <= Numbers<eigen_scalar_t<DA>>::epsilon() * Numbers<eigen_scalar_t<DA>>::epsilon())
        return Numbers<eigen_scalar_t<DA>>::zero();

    const auto dot = srcv.dot(dstv);
    const auto crossNorm = srcv.cross(dstv).norm();
    if (crossNorm <= Numbers<eigen_scalar_t<DA>>::epsilon())
    {
        if (dot < Numbers<eigen_scalar_t<DA>>::zero())
        {
            return (crossProductOrder == CrossProductOrder::SrcCrossDst)
                ? Numbers<eigen_scalar_t<DA>>::PI()
                : -Numbers<eigen_scalar_t<DA>>::PI();
        }
        return Numbers<eigen_scalar_t<DA>>::zero();
    }

    const auto signedCross = (crossProductOrder == CrossProductOrder::SrcCrossDst) ? crossNorm : -crossNorm;
    return std::atan2(signedCross, dot);
}

} // namespace Math
} // namespace ZF
