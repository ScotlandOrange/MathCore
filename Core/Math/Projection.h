#pragma once

// 投影矩阵辅助函数 (Eigen 版本)。
//
// 约定与 glm::ortho / OpenGL 右手系保持一致：
//   - 视空间相机看向 -Z。
//   - NDC 深度范围为 [-1, 1]。
//   - near / far 传入正向距离 n / f，对应视空间 z = -n / -f。

#include <Core/Math/Numbers.h>

#include <Eigen/Core>

#include <cmath>

namespace ZF
{
namespace Math
{

template <typename T>
inline constexpr T zeroGuard(T x)
{
    const T abs_x = (x < Numbers<T>::zero()) ? -x : x;
    if (abs_x < Numbers<T>::epsilon())
    {
        return (x < Numbers<T>::zero()) ? -Numbers<T>::epsilon() : Numbers<T>::epsilon();
    }
    return x;
}

/// OpenGL 风格正交投影矩阵，行为对齐 glm::ortho(left, right, bottom, top, near, far)。
template <typename T>
inline Eigen::Matrix<T, 4, 4> ortho(T left, T right,
                                    T bottom, T top,
                                    T nearPlane, T farPlane)
{
    const T rl = zeroGuard(right - left);
    const T tb = zeroGuard(top - bottom);
    const T fn = zeroGuard(farPlane - nearPlane);

    Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Zero();
    m(0, 0) = Numbers<T>::two() / rl;
    m(1, 1) = Numbers<T>::two() / tb;
    m(2, 2) = -Numbers<T>::two() / fn;
    m(3, 3) = Numbers<T>::one();

    m(0, 3) = -(right + left) / rl;
    m(1, 3) = -(top + bottom) / tb;
    m(2, 3) = -(farPlane + nearPlane) / fn;
    return m;
}

/// 4 参数重载：深度保持在 OpenGL 默认裁剪范围 [-1, 1]。
template <typename T>
inline Eigen::Matrix<T, 4, 4> ortho(T left, T right,
                                    T bottom, T top)
{
    return ortho(left, right, bottom, top, Numbers<T>::minus_one(), Numbers<T>::one());
}

/// OpenGL 风格透视投影矩阵，行为对齐 glm::perspective(fovyRadians, aspect, near, far)。
///
/// 约定：
///   - 右手系，相机看 -Z。
///   - 深度区间 [-1, 1]。
///   - fovy 使用弧度，且是垂直视角。
template <typename T>
inline Eigen::Matrix<T, 4, 4> perspective(T verticalFovRadians,
                                          T aspect,
                                          T nearPlane,
                                          T farPlane)
{
    const T maxHalf = Numbers<T>::PI_2() - Numbers<T>::epsilon();
    T half = verticalFovRadians * Numbers<T>::half();
    if (half >  maxHalf) half =  maxHalf;
    if (half < -maxHalf) half = -maxHalf;

    const T tanHalfV = std::tan(half);
    const T fn = zeroGuard(farPlane - nearPlane);

    Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Zero();
    m(0, 0) = Numbers<T>::one() / zeroGuard(aspect * tanHalfV);
    m(1, 1) = Numbers<T>::one() / zeroGuard(tanHalfV);
    m(2, 2) = -(farPlane + nearPlane) / fn;
    m(2, 3) = -(Numbers<T>::two() * farPlane * nearPlane) / fn;
    m(3, 2) = -Numbers<T>::one();
    return m;
}

/// OpenGL 风格透视投影矩阵，行为对齐 glm::frustum(left, right, bottom, top, near, far)。
///
/// 当 left/right/bottom/top 来自同一组 fovy/aspect/near 时，结果与
/// perspective(fovy, aspect, near, far) 等价。
template <typename T>
inline Eigen::Matrix<T, 4, 4> perspective(T left,
                                          T right,
                                          T bottom,
                                          T top,
                                          T nearPlane,
                                          T farPlane)
{
    const T rl = zeroGuard(right - left);
    const T tb = zeroGuard(top - bottom);
    const T fn = zeroGuard(farPlane - nearPlane);

    Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Zero();
    m(0, 0) = (Numbers<T>::two() * nearPlane) / rl;
    m(1, 1) = (Numbers<T>::two() * nearPlane) / tb;
    m(0, 2) = (right + left) / rl;
    m(1, 2) = (top + bottom) / tb;
    m(2, 2) = -(farPlane + nearPlane) / fn;
    m(2, 3) = -(Numbers<T>::two() * farPlane * nearPlane) / fn;
    m(3, 2) = -Numbers<T>::one();
    return m;
}

} // namespace Math
} // namespace ZF
