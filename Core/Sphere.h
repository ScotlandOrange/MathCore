#pragma once

// 包围球 (中心 + 半径)
//
// 设计目标：保持与 vsg::t_sphere 的接口/命名风格对齐，但底层不再使用匿名 union，
// 4 个分量统一存放在 Eigen 4D 向量中。
//
// 与 VSG 的差异点：
//   - 类名采用驼峰风格 TSphere (对应 vsg::t_sphere)。
//   - 中心和半径通过函数 center() / radius() / vec() 暴露。
//   - 默认构造的 sphere 半径为 -1，与 VSG 行为一致 (代表 invalid)。

#include <cstddef>

#include <Eigen/Core>

namespace ZF
{
namespace Math
{

template <typename T>
class TSphere
{
public:
    using value_type = T;
    using vec_type = Eigen::Vector<T, 4>;
    using center_type = Eigen::Vector<T, 3>;

    // 默认构造：center = (0, 0, 0)，radius = -1，valid() == false
    constexpr TSphere() : m_vec(value_type(0), value_type(0), value_type(0), value_type(-1)) {}

    constexpr TSphere(const TSphere&) = default;
    constexpr TSphere& operator=(const TSphere&) = default;

    explicit TSphere(const vec_type& v) : m_vec(v) {}

    TSphere(value_type sx, value_type sy, value_type sz, value_type sr)
        : m_vec(sx, sy, sz, sr)
    {
    }

    template <typename R>
    TSphere(const Eigen::Vector<R, 3>& c, value_type rad)
        : m_vec(static_cast<value_type>(c.x()),
                static_cast<value_type>(c.y()),
                static_cast<value_type>(c.z()),
                rad)
    {
    }

    // 跨标量类型的显式转换构造
    template <typename R>
    explicit TSphere(const TSphere<R>& other)
        : m_vec(static_cast<value_type>(other[0]),
                static_cast<value_type>(other[1]),
                static_cast<value_type>(other[2]),
                static_cast<value_type>(other[3]))
    {
    }

    constexpr std::size_t size() const { return 4; }

    value_type& operator[](std::size_t i) { return m_vec[static_cast<Eigen::Index>(i)]; }
    value_type operator[](std::size_t i) const { return m_vec[static_cast<Eigen::Index>(i)]; }

    template <typename R>
    TSphere& operator=(const TSphere<R>& rhs)
    {
        m_vec[0] = static_cast<value_type>(rhs[0]);
        m_vec[1] = static_cast<value_type>(rhs[1]);
        m_vec[2] = static_cast<value_type>(rhs[2]);
        m_vec[3] = static_cast<value_type>(rhs[3]);
        return *this;
    }

    void set(value_type in_x, value_type in_y, value_type in_z, value_type in_r)
    {
        m_vec << in_x, in_y, in_z, in_r;
    }

    template <typename R>
    void set(const Eigen::Vector<R, 3>& c, value_type rad)
    {
        m_vec << static_cast<value_type>(c.x()),
                 static_cast<value_type>(c.y()),
                 static_cast<value_type>(c.z()),
                 rad;
    }

    // radius >= 0 视为有效，与 vsg::t_sphere::valid() 行为一致。
    bool valid() const { return m_vec[3] >= value_type(0); }

    explicit operator bool() const noexcept { return valid(); }

    T* data() { return m_vec.data(); }
    const T* data() const { return m_vec.data(); }

    // 访问器 (对应 VSG 的 union 成员 center / radius / vec)
    auto center() { return m_vec.template head<3>(); }
    auto center() const { return m_vec.template head<3>(); }

    value_type& radius() { return m_vec[3]; }
    value_type radius() const { return m_vec[3]; }

    vec_type& vec() { return m_vec; }
    const vec_type& vec() const { return m_vec; }

    // 重置为 invalid (radius = -1)
    void reset()
    {
        m_vec << value_type(0), value_type(0), value_type(0), value_type(-1);
    }

private:
    vec_type m_vec;
};

using Sphere = TSphere<float>;
using Sphered = TSphere<double>;
using Sphereld = TSphere<long double>;

template <typename T>
inline bool operator==(const TSphere<T>& lhs, const TSphere<T>& rhs)
{
    return lhs[0] == rhs[0] && lhs[1] == rhs[1]
           && lhs[2] == rhs[2] && lhs[3] == rhs[3];
}

template <typename T>
inline bool operator!=(const TSphere<T>& lhs, const TSphere<T>& rhs)
{
    return !(lhs == rhs);
}

template <typename T>
inline bool operator<(const TSphere<T>& lhs, const TSphere<T>& rhs)
{
    if (lhs[0] < rhs[0]) return true;
    if (lhs[0] > rhs[0]) return false;
    if (lhs[1] < rhs[1]) return true;
    if (lhs[1] > rhs[1]) return false;
    if (lhs[2] < rhs[2]) return true;
    if (lhs[2] > rhs[2]) return false;
    return lhs[3] < rhs[3];
}

} // namespace Math
} // namespace ZF
