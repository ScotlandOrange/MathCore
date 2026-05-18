#pragma once

// 射线与几何图元的相交测试 (Eigen 版本)。
//
// 提供与 glm::intersectRayTriangle 行为对齐的实现，使用 Möller–Trumbore 算法：
//   - 双面三角形 (不做背面剔除)。
//   - 仅返回前向命中 (沿 dir 正方向)。
//   - 输出重心坐标 (u, v)，命中点 P = (1 - u - v) * v0 + u * v1 + v * v2。
//   - 输出参数 t，使得 P = orig + t * dir。dir 不要求归一化；当 dir 已归一化时 t 即距离。

#include <Eigen/Geometry> // Eigen::Vector<T, 3>::cross 依赖 (MSVC 上仅 Eigen/Core 不够)

#include <Core/Math/Numbers.h>

#include <cmath>

namespace ZF
{
namespace Math
{

/// 射线 / 三角形相交 (Möller–Trumbore)。
///
/// @param orig         射线起点。
/// @param dir          射线方向 (无需归一化)。
/// @param v0, v1, v2   三角形三个顶点 (顺序决定 (u, v) 的方向)。
/// @param baryPosition 输出：命中点的重心坐标 (u, v)。
/// @param distance     输出：命中点的射线参数 t (P = orig + t * dir)。
/// @return             命中且 t >= 0 时返回 true；否则 baryPosition / distance 不修改。
template <typename T>
inline bool intersectRayTriangle(
    const Eigen::Vector<T, 3>& orig, const Eigen::Vector<T, 3>& dir,
    const Eigen::Vector<T, 3>& v0,   const Eigen::Vector<T, 3>& v1, const Eigen::Vector<T, 3>& v2,
    Eigen::Vector<T, 2>& baryPosition, T& distance)
{
    // 行列式接近 0 视为退化 (射线与三角形平面平行)。
    // 用类型相关的 epsilon，避免 float 阈值在 double 下过松 / 在 float 下过紧。
    const Eigen::Vector<T, 3> e1 = v1 - v0;
    const Eigen::Vector<T, 3> e2 = v2 - v0;
    const Eigen::Vector<T, 3> p  = dir.cross(e2);

    T det = e1.dot(p);

    // 双面三角形：当 det < 0 时翻转 T 与 det 符号，等效成 det > 0 的情形。
    Eigen::Vector<T, 3> tvec;
    if (det > Numbers<T>::zero())
    {
        tvec = orig - v0;
    }
    else
    {
        tvec = v0 - orig;
        det  = -det;
    }

    if (det < Numbers<T>::epsilon())
        return false;

    // u = T·P / det，先用未除法版本与 det 比较，省掉一次除法并避免数值噪声。
    const T u = tvec.dot(p);
    if (u < Numbers<T>::zero() || u > det)
        return false;

    const Eigen::Vector<T, 3> q = tvec.cross(e1);
    const T v = dir.dot(q);
    if (v < Numbers<T>::zero() || (u + v) > det)
        return false;

    const T rayStep = e2.dot(q);

    // 仅前向命中 (t >= 0)。det 此处必为正，所以与 rayStep 同号。
    if (rayStep < Numbers<T>::zero())
        return false;

    const T invDet = Numbers<T>::one() / det;
    distance       = rayStep * invDet;
    baryPosition   = Eigen::Vector<T, 2>(u * invDet, v * invDet);
    return true;
}

} // namespace Math
} // namespace ZF
