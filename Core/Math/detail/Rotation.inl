#pragma once

namespace ZF
{
namespace Math
{

inline Eigen::Vector3f orthogonalizedUp(const Eigen::Vector3f& axis,
                                        const Eigen::Vector3f& preferredUp)
{
    Eigen::Vector3f up = preferredUp - preferredUp.dot(axis) * axis;
    if (up.squaredNorm() > Numbersf::epsilon2())
    {
        return up.normalized();
    }

    up = Eigen::Vector3f::UnitX() - Eigen::Vector3f::UnitX().dot(axis) * axis;
    if (up.squaredNorm() <= Numbersf::epsilon2())
    {
        up = Eigen::Vector3f::UnitZ() - Eigen::Vector3f::UnitZ().dot(axis) * axis;
    }
    return up.normalized();
}

// ---- 围绕任意轴旋转指定弧度 ----

inline Eigen::Matrix3f makeRotationMatrix3x3(const Eigen::Vector3f& axis, float radians)
{
    ZF_ASSERT_MSG(axis.squaredNorm() > Numbersf::epsilon2(),
                  "makeRotation requires a non-zero rotation axis.");
    return Eigen::AngleAxisf(radians, axis.normalized()).toRotationMatrix();
}

inline Eigen::Matrix4f makeRotationMatrix4x4(const Eigen::Vector3f& axis, float radians)
{
    return Matrix3x3ToMatrix4x4(makeRotationMatrix3x3(axis, radians));
}

inline Eigen::Quaternionf makeRotationQuaternion(const Eigen::Vector3f& axis, float radians)
{
    ZF_ASSERT_MSG(axis.squaredNorm() > Numbersf::epsilon2(),
                  "makeRotation requires a non-zero rotation axis.");
    return Eigen::Quaternionf(Eigen::AngleAxisf(radians, axis.normalized()));
}

// ---- 由三条单位正交基轴构造旋转 ----

inline Eigen::Matrix3f makeRotationFromOrthoAxesMatrix3x3(const Eigen::Vector3f& x,
                                                          const Eigen::Vector3f& y,
                                                          const Eigen::Vector3f& z)
{
    const float tolerance = Numbersf::epsilon() * 100.0f;

    ZF_ASSERT_MSG(std::abs(x.squaredNorm() - 1.0f) <= tolerance,
                  "makeRotationFromOrthoAxes requires x to be unit length.");
    ZF_ASSERT_MSG(std::abs(y.squaredNorm() - 1.0f) <= tolerance,
                  "makeRotationFromOrthoAxes requires y to be unit length.");
    ZF_ASSERT_MSG(std::abs(z.squaredNorm() - 1.0f) <= tolerance,
                  "makeRotationFromOrthoAxes requires z to be unit length.");
    ZF_ASSERT_MSG(std::abs(x.dot(y)) <= tolerance,
                  "makeRotationFromOrthoAxes requires x and y to be orthogonal.");
    ZF_ASSERT_MSG(std::abs(x.dot(z)) <= tolerance,
                  "makeRotationFromOrthoAxes requires x and z to be orthogonal.");
    ZF_ASSERT_MSG(std::abs(y.dot(z)) <= tolerance,
                  "makeRotationFromOrthoAxes requires y and z to be orthogonal.");
    ZF_ASSERT_MSG(std::abs(x.cross(y).dot(z) - 1.0f) <= tolerance,
                  "makeRotationFromOrthoAxes requires a right-handed orthonormal basis.");

    Eigen::Matrix3f result;
    result.col(0) = x;
    result.col(1) = y;
    result.col(2) = z;
    return result;
}

inline Eigen::Matrix4f makeRotationFromOrthoAxesMatrix4x4(const Eigen::Vector3f& x,
                                                          const Eigen::Vector3f& y,
                                                          const Eigen::Vector3f& z)
{
    return Matrix3x3ToMatrix4x4(makeRotationFromOrthoAxesMatrix3x3(x, y, z));
}

inline Eigen::Quaternionf makeRotationFromOrthoAxesQuaternion(const Eigen::Vector3f& x,
                                                              const Eigen::Vector3f& y,
                                                              const Eigen::Vector3f& z)
{
    return Matrix3x3ToQuaternion(makeRotationFromOrthoAxesMatrix3x3(x, y, z));
}

// ---- 根据目标方向 + 参考上方向 + 局部对齐轴构造旋转 ----

inline Eigen::Matrix3f makeRotationFromDirectionMatrix3x3(const Eigen::Vector3f& direction,
                                                          const Eigen::Vector3f& referenceUp,
                                                          OrientationAxis orientationAxis)
{
    const Eigen::Vector3f primaryAxis = direction.normalized();

    Eigen::Vector3f xAxis;
    Eigen::Vector3f yAxis;
    Eigen::Vector3f zAxis;

    switch (orientationAxis)
    {
    case OrientationAxis::X:
        xAxis = primaryAxis;
        yAxis = orthogonalizedUp(xAxis, referenceUp);
        zAxis = xAxis.cross(yAxis).normalized();
        break;
    case OrientationAxis::Y:
        yAxis = primaryAxis;
        zAxis = orthogonalizedUp(yAxis, referenceUp);
        xAxis = yAxis.cross(zAxis).normalized();
        break;
    case OrientationAxis::Z:
    default:
        zAxis = primaryAxis;
        yAxis = orthogonalizedUp(zAxis, referenceUp);
        xAxis = yAxis.cross(zAxis).normalized();
        break;
    }

    return makeRotationFromOrthoAxesMatrix3x3(xAxis, yAxis, zAxis);
}

inline Eigen::Matrix4f makeRotationFromDirectionMatrix4x4(const Eigen::Vector3f& direction,
                                                          const Eigen::Vector3f& referenceUp,
                                                          OrientationAxis orientationAxis)
{
    return Matrix3x3ToMatrix4x4(makeRotationFromDirectionMatrix3x3(direction, referenceUp, orientationAxis));
}

inline Eigen::Quaternionf makeRotationFromDirectionQuaternion(const Eigen::Vector3f& direction,
                                                              const Eigen::Vector3f& referenceUp,
                                                              OrientationAxis orientationAxis)
{
    return Matrix3x3ToQuaternion(makeRotationFromDirectionMatrix3x3(direction, referenceUp, orientationAxis));
}

// ---- 让局部 X 轴对齐到指定方向 ----

inline Eigen::Matrix3f makeRotationFromXAxisMatrix3x3(const Eigen::Vector3f& axis)
{
    const Eigen::Vector3f xAxis = axis.normalized();
    const Eigen::Vector3f reference =
        (std::abs(xAxis.dot(Eigen::Vector3f::UnitX())) > 1.0f - Numbersf::epsilon())
            ? Eigen::Vector3f::UnitY()
            : Eigen::Vector3f::UnitX();
    const Eigen::Vector3f yAxis = xAxis.cross(reference);
    const Eigen::Vector3f zAxis = xAxis.cross(yAxis);

    return makeRotationFromOrthoAxesMatrix3x3(xAxis.normalized(), yAxis.normalized(), zAxis.normalized());
}

inline Eigen::Matrix4f makeRotationFromXAxisMatrix4x4(const Eigen::Vector3f& axis)
{
    return Matrix3x3ToMatrix4x4(makeRotationFromXAxisMatrix3x3(axis));
}

inline Eigen::Quaternionf makeRotationFromXAxisQuaternion(const Eigen::Vector3f& axis)
{
    return Matrix3x3ToQuaternion(makeRotationFromXAxisMatrix3x3(axis));
}

// ---- 由 XYZ 弧度欧拉角构造旋转 ----

inline Eigen::Matrix3f makeRotationFromEulerRadiansMatrix3x3(const Eigen::Vector3f& angles)
{
    const Eigen::Quaternionf q =
        Eigen::AngleAxisf(angles.z(), Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(angles.y(), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(angles.x(), Eigen::Vector3f::UnitX());
    return q.toRotationMatrix();
}

inline Eigen::Matrix4f makeRotationFromEulerRadiansMatrix4x4(const Eigen::Vector3f& angles)
{
    return Matrix3x3ToMatrix4x4(makeRotationFromEulerRadiansMatrix3x3(angles));
}

inline Eigen::Quaternionf makeRotationFromEulerRadiansQuaternion(const Eigen::Vector3f& angles)
{
    return Matrix3x3ToQuaternion(makeRotationFromEulerRadiansMatrix3x3(angles));
}

// ---- 由 XYZ 角度欧拉角构造旋转 ----

inline Eigen::Matrix3f makeRotationFromEulerDegreesMatrix3x3(const Eigen::Vector3f& angles)
{
    return makeRotationFromEulerRadiansMatrix3x3(angles * Numbersf::DEG_TO_RAD());
}

inline Eigen::Matrix4f makeRotationFromEulerDegreesMatrix4x4(const Eigen::Vector3f& angles)
{
    return makeRotationFromEulerRadiansMatrix4x4(angles * Numbersf::DEG_TO_RAD());
}

inline Eigen::Quaternionf makeRotationFromEulerDegreesQuaternion(const Eigen::Vector3f& angles)
{
    return makeRotationFromEulerRadiansQuaternion(angles * Numbersf::DEG_TO_RAD());
}

// ---- 围绕过 point 沿 axis 的轴旋转，得到 4x4 变换矩阵 ----

inline Eigen::Matrix4f makeRotationMatrixAroundPoint(const Eigen::Vector3f& point,
                                                     const Eigen::Vector3f& axis,
                                                     float radians)
{
    const Eigen::Affine3f transform =
        Eigen::Translation3f(point) *
        Eigen::AngleAxisf(radians, axis.normalized()) *
        Eigen::Translation3f(-point);
    return transform.matrix();
}

} // namespace Math
} // namespace ZF
