#pragma once

// 平面 (Hessian Normal Form)：dot(n, x) + p = 0
//
// 设计目标：保持与 vsg::t_plane 的接口/命名风格对齐，但底层不再使用匿名 union，
// 向量与矩阵全部基于 Eigen 原生类型。
//
// 与 VSG 的差异点：
//   - 类名采用驼峰风格 TPlane (对应 vsg::t_plane)。
//   - 法线/系数通过函数 norm() / d() / vec() 暴露 (对应 VSG 的 n / p)。
//   - 提供 Plane / Planed / Planeld 别名。

#include <Core/Sphere.h>

#include <cstddef>
#include <iterator>

#include <Eigen/Core>

namespace ZF
{
namespace Math
{

template <typename T>
class TPlane
{
public:
    using value_type = T;
    using vec_type = Eigen::Vector<T, 4>;
    using normal_type = Eigen::Vector<T, 3>;

    constexpr TPlane() : m_vec(vec_type::Zero()) {}

    constexpr TPlane(const TPlane&) = default;
    constexpr TPlane& operator=(const TPlane&) = default;

    explicit TPlane(const vec_type& v) : m_vec(v) {}

    TPlane(value_type nx, value_type ny, value_type nz, value_type in_d)
        : m_vec(nx, ny, nz, in_d)
    {
    }

    TPlane(const normal_type& normal, value_type in_d)
        : m_vec(normal.x(), normal.y(), normal.z(), in_d)
    {
    }

    // 给平面上的一点和法线，自动推出 d = -dot(position, normal)
    TPlane(const normal_type& position, const normal_type& normal)
        : m_vec(normal.x(),
                normal.y(),
                normal.z(),
                -position.dot(normal))
    {
    }

    // 跨标量类型的显式转换构造
    template <typename R>
    explicit TPlane(const TPlane<R>& other)
        : m_vec(static_cast<T>(other[0]),
                static_cast<T>(other[1]),
                static_cast<T>(other[2]),
                static_cast<T>(other[3]))
    {
    }

    constexpr std::size_t size() const { return 4; }

    value_type& operator[](std::size_t i) { return m_vec[static_cast<Eigen::Index>(i)]; }
    value_type operator[](std::size_t i) const { return m_vec[static_cast<Eigen::Index>(i)]; }

    template <typename R>
    TPlane& operator=(const TPlane<R>& rhs)
    {
        m_vec[0] = static_cast<value_type>(rhs[0]);
        m_vec[1] = static_cast<value_type>(rhs[1]);
        m_vec[2] = static_cast<value_type>(rhs[2]);
        m_vec[3] = static_cast<value_type>(rhs[3]);
        return *this;
    }

    void set(value_type in_x, value_type in_y, value_type in_z, value_type in_d)
    {
        m_vec << in_x, in_y, in_z, in_d;
    }

    // 法线非零即为有效平面，与 vsg::t_plane::valid() 行为一致。
    bool valid() const
    {
        return m_vec[0] != value_type(0)
               || m_vec[1] != value_type(0)
               || m_vec[2] != value_type(0);
    }

    explicit operator bool() const noexcept { return valid(); }

    T* data() { return m_vec.data(); }
    const T* data() const { return m_vec.data(); }

    // 访问器 (对应 VSG 的 union 成员 n / p / vec)
    auto norm() { return m_vec.template head<3>(); }
    auto norm() const { return m_vec.template head<3>(); }

    value_type& d() { return m_vec[3]; }
    value_type d() const { return m_vec[3]; }

    vec_type& vec() { return m_vec; }
    const vec_type& vec() const { return m_vec; }

private:
    vec_type m_vec;
};

using Plane = TPlane<float>;
using Planed = TPlane<double>;
using Planeld = TPlane<long double>;

template <typename T>
inline bool operator==(const TPlane<T>& lhs, const TPlane<T>& rhs)
{
    return lhs[0] == rhs[0] && lhs[1] == rhs[1]
           && lhs[2] == rhs[2] && lhs[3] == rhs[3];
}

template <typename T>
inline bool operator!=(const TPlane<T>& lhs, const TPlane<T>& rhs)
{
    return !(lhs == rhs);
}

template <typename T>
inline bool operator<(const TPlane<T>& lhs, const TPlane<T>& rhs)
{
    if (lhs[0] < rhs[0]) return true;
    if (lhs[0] > rhs[0]) return false;
    if (lhs[1] < rhs[1]) return true;
    if (lhs[1] > rhs[1]) return false;
    if (lhs[2] < rhs[2]) return true;
    if (lhs[2] > rhs[2]) return false;
    return lhs[3] < rhs[3];
}

// 点到平面的有符号距离
template <typename T>
inline T distance(const TPlane<T>& pl, const Eigen::Vector<T, 3>& v)
{
    return pl.norm().dot(v) + pl.d();
}

// 不同标量类型的距离 (点会先转换到平面的标量类型)
template <typename T, typename R>
inline T distance(const TPlane<T>& pl, const Eigen::Vector<R, 3>& v)
{
    return pl.norm().dot(v.template cast<T>()) + pl.d();
}

// 凸多面体 (一组平面，法线指向多面体内部) 与包围球的相交测试。
template <class PlaneIterator, typename T>
inline bool intersect(PlaneIterator first,
                      PlaneIterator last,
                      const TSphere<T>& s)
{
    const T negative_radius = -s.radius();
    const Eigen::Vector<T, 3> center = s.center();
    for (auto it = first; it != last; ++it)
    {
        if (distance(*it, center) < negative_radius) return false;
    }
    return true;
}

template <class Polytope, typename T>
inline bool intersect(const Polytope& polytope, const TSphere<T>& s)
{
    using std::begin;
    using std::end;
    return intersect(begin(polytope), end(polytope), s);
}

// 判断点是否在凸多面体内部 (允许一个 epsilon 容差)。
template <class PlaneIterator, typename T>
inline bool inside(PlaneIterator first,
                   PlaneIterator last,
                   const Eigen::Vector<T, 3>& v,
                   T epsilon = T(1e-10))
{
    const T negative_epsilon = -epsilon;
    for (auto it = first; it != last; ++it)
    {
        if (distance(*it, v) < negative_epsilon) return false;
    }
    return true;
}

template <class Polytope, typename T>
inline bool inside(const Polytope& polytope,
                   const Eigen::Vector<T, 3>& v,
                   T epsilon = T(1e-10))
{
    using std::begin;
    using std::end;
    return inside(begin(polytope), end(polytope), v, epsilon);
}

// 矩阵 * 平面：把源空间 S 下的平面 p_S 变换到目标空间 D 下的 p_D。
//
// 记号 (列向量约定，下标即空间)：
//   p_S = (n_S, d_S)^T   是 4x1 列向量, n_S 是 3x1 法线
//   p_D = (n_D, d_D)^T
//   x   是齐次点 (4x1 列向量)
//   M   是 4x4 矩阵, 约定为 D->S 的点变换:    x_S = M * x_D
//
// 注意 M 的方向: 这里 M 是把目标空间的点变到源空间, 不是常见的 "源->目标"。
// 这样写是因为 p_D = M^T * p_S, 只用一次转置, 不需要求逆 (求逆开销大)。
// 上层 API 想用直观的 S->D 写法, 应该自己调一次 .inverse() 再传进来。
//
// 推导:
//   1. 平面方程 (行向量 . 列向量):     p_S^T * x_S = 0
//   2. 代入 x_S = M * x_D:             p_S^T * M * x_D = 0
//   3. 合并前两项:                     (M^T * p_S)^T * x_D = 0
//   4. 与 p_D^T * x_D = 0 对照:        p_D = M^T * p_S
//   5. 归一化: p_D /= |n_D|, 让法线变成单位向量
template <typename Derived, typename T>
inline TPlane<T> operator*(const Eigen::MatrixBase<Derived>& lhs, const TPlane<T>& rhs)
{
    static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
                  "Matrix * TPlane requires a 4x4 matrix");

    // 步骤 1-4: p_D = M^T * p_S
    Eigen::Vector<T, 4> p_D = (lhs.template cast<T>().transpose() * rhs.vec()).eval();

    // 步骤 5: 按法线长度归一化。退化情形 (变换后法线为 0) 不除，
    // 避免 0/0 出 NaN；这种 p_D 本身已是退化平面，valid() 会返回 false。
    const T n_len = p_D.template head<3>().norm();
    if (n_len > T(0)) p_D /= n_len;

    return TPlane<T>(p_D);
}

} // namespace Math
} // namespace ZF
