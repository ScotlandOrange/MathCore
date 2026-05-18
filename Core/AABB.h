#pragma once

// Axis-aligned bounding box (AABB).
//
// 设计要点：
//   - 只存最小点 / 最大点，不缓存 extents，避免重复状态失真。
//   - 默认构造得到 invalid box，适合增量 merge。
//   - 仿射变换统一通过变换 8 个角点得到新的包围盒。

#include <Core/Macros.h>
#include <Core/Math/Transform.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <vector>

namespace ZF
{
namespace Math
{

/**
 * @brief 轴对齐包围盒。只保存最小点和最大点。
 *
 * 默认构造和 `null()` 返回无效包围盒。
 * 从最小值/最大值、结构化顶点、扁平顶点创建时使用命名工厂函数。
 * 从另一种标量类型的 AABB 转换时使用 `make(other)`。
 *
 * 创建方式总览：
 * @code{.cpp}
 * ZF::Math::TAABB<double> emptyBox;
 * ZF::Math::TAABB<double> nullBox = ZF::Math::TAABB<double>::null();
 *
 * const ZF::Math::TAABB<double> byScalarBounds =
 *     ZF::Math::TAABB<double>::makeFromMinMax(-1.0, 2.0);
 *
 * const ZF::Math::TAABB<double> byBounds = ZF::Math::TAABB<double>::makeFromMinMax(
 *     Eigen::Vector<double, 3>(-1.0, -2.0, -3.0),
 *     Eigen::Vector<double, 3>(4.0, 5.0, 6.0));
 *
 * // 从另一种标量类型的 AABB 转换创建。
 * const ZF::Math::TAABB<float> byOtherBox =
 *     ZF::Math::TAABB<float>::make(byBounds);
 *
 * const std::vector<Eigen::Vector<double, 3>> vertices = {
 *     Eigen::Vector<double, 3>(1.0, 2.0, 3.0),
 *     Eigen::Vector<double, 3>(-4.0, 5.0, -6.0),
 * };
 * const ZF::Math::TAABB<double> byVertices =
 *     ZF::Math::TAABB<double>::makeFromStructVertices(vertices);
 * const ZF::Math::TAABB<double> byPointer =
 *     ZF::Math::TAABB<double>::makeFromStructVertices(vertices.data(), vertices.size());
 * const Eigen::Vector<double, 3> vertexArray[] = {
 *     Eigen::Vector<double, 3>(1.0, 2.0, 3.0),
 *     Eigen::Vector<double, 3>(-4.0, 5.0, -6.0),
 * };
 * const ZF::Math::TAABB<double> byArray =
 *     ZF::Math::TAABB<double>::makeFromStructVertices(vertexArray);
 *
 * struct VertexRecord
 * {
 *     double x;
 *     double y;
 *     double z;
 * };
 * const std::vector<VertexRecord> customVertices = {
 *     {1.0, 2.0, 3.0},
 *     {-4.0, 5.0, -6.0},
 * };
 * const ZF::Math::TAABB<double> byCustomVertices = ZF::Math::TAABB<double>::makeFromStructVertices(
 *     customVertices,
 *     [](const VertexRecord& vertex) { return Eigen::Vector<double, 3>(vertex.x, vertex.y, vertex.z); });
 * const ZF::Math::TAABB<double> byCustomPointer = ZF::Math::TAABB<double>::makeFromStructVertices(
 *     customVertices.data(),
 *     customVertices.size(),
 *     [](const VertexRecord& vertex) { return Eigen::Vector<double, 3>(vertex.x, vertex.y, vertex.z); });
 * const VertexRecord customVertexArray[] = {
 *     {1.0, 2.0, 3.0},
 *     {-4.0, 5.0, -6.0},
 * };
 * const ZF::Math::TAABB<double> byCustomArray = ZF::Math::TAABB<double>::makeFromStructVertices(
 *     customVertexArray,
 *     [](const VertexRecord& vertex) { return Eigen::Vector<double, 3>(vertex.x, vertex.y, vertex.z); });
 *
 * const std::vector<double> flatVertices = {
 *     1.0, 2.0, 3.0,
 *     -4.0, 5.0, -6.0,
 * };
 * const ZF::Math::TAABB<double> byFlatVector =
 *     ZF::Math::TAABB<double>::makeFromFlatVertices(flatVertices);
 * const double flatVertexPointer[] = {
 *     1.0, 2.0, 3.0,
 *     -4.0, 5.0, -6.0,
 * };
 * const ZF::Math::TAABB<double> byFlatPointer =
 *     ZF::Math::TAABB<double>::makeFromFlatVertices(flatVertexPointer, 6);
 * const double flatVertexArray[] = {
 *     1.0, 2.0, 3.0,
 *     -4.0, 5.0, -6.0,
 * };
 * const ZF::Math::TAABB<double> byFlatArray =
 *     ZF::Math::TAABB<double>::makeFromFlatVertices(flatVertexArray);
 * @endcode
 *
 * 也可以先默认构造，再用 `merge(...)` 增量扩展。
 * @code{.cpp}
 * ZF::Math::TAABB<double> box;
 * box.merge(Eigen::Vector<double, 3>(1.0, 2.0, 3.0));
 * box.merge(Eigen::Vector<double, 3>(-4.0, 5.0, -6.0));
 *
 * if (box.valid())
 * {
 *     const Eigen::Vector<double, 3> center = box.center();
 *     const Eigen::Vector<double, 3> size = box.extents();
 * }
 * @endcode
 *
 * 常见变换：
 * @code{.cpp}
 * const ZF::Math::TAABB<double> moved =
 *     byBounds.translate(Eigen::Vector<double, 3>(1.0, 0.0, 0.0));
 * const ZF::Math::TAABB<double> rotated =
 *     byBounds.rotate(Eigen::Quaternion<double>::Identity());
 * const ZF::Math::TAABB<double> ndcBox =
 *     byBounds.viewProject(Eigen::Matrix<double, 4, 4>::Identity());
 * @endcode
 *
 * @tparam T 包围盒内部使用的标量类型。
 */
template <typename T>
class TAABB
{
public:
    using value_type = T;
    using vec_type = Eigen::Vector<T, 3>;
    using matrix_type = Eigen::Matrix<T, 4, 4>;
    using quaternion_type = Eigen::Quaternion<T>;

    /// 默认构造：invalid box，适合后续通过 merge 逐步扩展。
    TAABB()
        : m_min(vec_type::Constant(std::numeric_limits<value_type>::infinity()))
        , m_max(vec_type::Constant(-std::numeric_limits<value_type>::infinity()))
    {
    }

    TAABB(const TAABB&) = default;
    TAABB& operator=(const TAABB&) = default;

    template <typename R>
    TAABB& operator=(const TAABB<R>& rhs)
    {
        reset();
        if (rhs.valid())
        {
            m_min = rhs.getMin().template cast<value_type>();
            m_max = rhs.getMax().template cast<value_type>();
        }
        return *this;
    }

    /// 返回一个 invalid box，语义等价于默认构造。
    static TAABB null()
    {
        return TAABB();
    }

    /// 由统一标量上下界创建 AABB，若 `minValue > maxValue` 会自动归一化。
    static TAABB makeFromMinMax(value_type minValue, value_type maxValue)
    {
        return TAABB(minValue, maxValue);
    }

    /// 由最小点和最大点创建 AABB，内部会自动归一化各分量顺序。
    static TAABB makeFromMinMax(const vec_type& minValue, const vec_type& maxValue)
    {
        return TAABB(minValue, maxValue);
    }

    /// 由另一种标量类型的 AABB 创建当前类型的 AABB。
    template <typename R>
    static TAABB make(const TAABB<R>& other)
    {
        return TAABB(other);
    }

    /// 由结构化顶点容器创建 AABB。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromStructVertices(const std::vector<U>& vertices)
    {
        return TAABB(vertices);
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromStructVertices(const std::vector<U>& vertices, const Converter& toVec)
    {
        return TAABB(vertices, toVec);
    }

    /// 由扁平标量数组创建 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromFlatVertices(const std::vector<U>& scalarVertices)
    {
        return TAABB(scalarVertices);
    }

    /// 由结构化顶点指针和数量创建 AABB。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromStructVertices(const U* vertices, std::size_t count)
    {
        return TAABB(vertices, count);
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromStructVertices(const U* vertices, std::size_t count, const Converter& toVec)
    {
        return TAABB(vertices, count, toVec);
    }

    /// 由扁平标量裸指针创建 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromFlatVertices(const U* scalarVertices, std::size_t scalarCount)
    {
        return TAABB(scalarVertices, scalarCount);
    }

    /// 由结构化顶点 C 数组创建 AABB。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::size_t N, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromStructVertices(const U (&vertices)[N])
    {
        return TAABB(vertices);
    }

    template <typename U, std::size_t N, typename Converter,
              std::enable_if_t<!std::is_arithmetic<U>{} && !std::is_arithmetic<std::decay_t<Converter>>{}, int> = 0>
    static TAABB makeFromStructVertices(const U (&vertices)[N], const Converter& toVec)
    {
        return TAABB(vertices, toVec);
    }

    /// 由固定长度扁平标量 C 数组创建 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::size_t N, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    static TAABB makeFromFlatVertices(const U (&scalarVertices)[N])
    {
        return TAABB(scalarVertices);
    }

    /// 仅当 min/max 全部有限且 `max >= min` 时视为有效。
    ZF_FORCE_INLINE bool valid() const
    {
        return m_min.array().isFinite().all()
            && m_max.array().isFinite().all()
            && (m_max.array() >= m_min.array()).all();
    }

    explicit operator bool() const noexcept { return valid(); }

    /// 重置为 invalid box。
    void reset()
    {
        m_min = vec_type::Constant(std::numeric_limits<value_type>::infinity());
        m_max = vec_type::Constant(-std::numeric_limits<value_type>::infinity());
    }

    /// 直接设置最小/最大点，内部会自动归一化各分量顺序。
    void set(const vec_type& minValue, const vec_type& maxValue)
    {
        m_min = minValue.cwiseMin(maxValue);
        m_max = minValue.cwiseMax(maxValue);
    }

    /// 直接写入最小点；若与当前最大点顺序冲突，则 `valid()` 会返回 false。
    void setMin(const vec_type& minValue)
    {
        m_min = minValue;
    }

    /// 直接写入最大点；若与当前最小点顺序冲突，则 `valid()` 会返回 false。
    void setMax(const vec_type& maxValue)
    {
        m_max = maxValue;
    }

    const vec_type& getMin() const { return m_min; }
    const vec_type& getMax() const { return m_max; }

    /// 返回包围盒中心；invalid box 返回零向量。
    vec_type center() const
    {
        if (!valid()) return vec_type::Zero();
        return value_type(0.5) * (m_min + m_max);
    }

    /// 返回完整尺寸 `(max - min)`；invalid box 返回零向量。
    vec_type extents() const
    {
        if (!valid()) return vec_type::Zero();
        return m_max - m_min;
    }

    /// 返回半尺寸；invalid box 返回零向量。
    vec_type halfExtents() const
    {
        return value_type(0.5) * extents();
    }

    /// 兼容旧命名 `halfExtends()`。
    vec_type halfExtends() const
    {
        return halfExtents();
    }

    /// 返回包围盒对角线长度；invalid box 返回 0。
    value_type diagDistance() const
    {
        return extents().norm();
    }

    /// 判断一个点是否落在当前包围盒内；边界视为内部。
    bool contains(const vec_type& position) const
    {
        if (!valid() || !position.array().isFinite().all()) return false;

        return (position.array() >= m_min.array()).all()
            && (position.array() <= m_max.array()).all();
    }

    /// 判断另一个包围盒是否完全包含在当前包围盒内；边界重合视为包含。
    bool contains(const TAABB& otherAABB) const
    {
        if (!valid() || !otherAABB.valid()) return false;

        return contains(otherAABB.getMin()) && contains(otherAABB.getMax());
    }

    /// 将一个点并入包围盒；非有限点会被忽略。
    void merge(const vec_type& position)
    {
        if (!position.array().isFinite().all()) return;

        if (!valid())
        {
            m_min = position;
            m_max = position;
            return;
        }

        m_min = m_min.cwiseMin(position);
        m_max = m_max.cwiseMax(position);
    }

    /// 将另一个 AABB 并入当前包围盒；若输入无效则返回 false。
    bool merge(const TAABB& otherAABB)
    {
        if (!otherAABB.valid()) return false;

        merge(otherAABB.getMin());
        merge(otherAABB.getMax());
        return true;
    }

    /// 将顶点容器并入当前包围盒。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    bool merge(const std::vector<U>& vertices)
    {
        return merge(vertices, [](const U& vertex) { return vec_type(vertex); });
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    bool merge(const std::vector<U>& vertices, const Converter& toVec)
    {
        return merge(vertices.data(), vertices.size(), toVec);
    }

    /// 将扁平标量数组并入当前包围盒；每 3 个标量解释为一个顶点。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    bool merge(const std::vector<U>& scalarVertices)
    {
        return merge(scalarVertices.data(), scalarVertices.size());
    }

    /// 将顶点指针和数量并入当前包围盒。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    bool merge(const U* vertices, std::size_t count)
    {
        return merge(vertices, count, [](const U& vertex) { return vec_type(vertex); });
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    bool merge(const U* vertices, std::size_t count, const Converter& toVec)
    {
        if (vertices == nullptr || count == 0) return false;

        bool merged = false;
        for (std::size_t index = 0; index < count; ++index)
        {
            const vec_type vertex = toVec(vertices[index]);
            if (!vertex.array().isFinite().all()) continue;
            merge(vertex);
            merged = true;
        }
        return merged;
    }

    /// 将扁平标量裸指针并入当前包围盒；每 3 个标量解释为一个顶点。
    /// 尾部不足 3 个标量的残余数据会被忽略。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    bool merge(const U* scalarVertices, std::size_t scalarCount)
    {
        if (scalarVertices == nullptr || scalarCount < 3) return false;

        bool merged = false;
        const std::size_t vertexCount = scalarCount / 3;
        for (std::size_t index = 0; index < vertexCount; ++index)
        {
            const std::size_t base = index * 3;
            const vec_type vertex(static_cast<value_type>(scalarVertices[base]),
                                  static_cast<value_type>(scalarVertices[base + 1]),
                                  static_cast<value_type>(scalarVertices[base + 2]));
            if (!vertex.array().isFinite().all()) continue;
            merge(vertex);
            merged = true;
        }
        return merged;
    }

    /// 将顶点 C 数组并入当前包围盒。`toVec` 用于把元素转换为 `Eigen::Vector<T, 3>`。
    template <typename U, std::size_t N, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    bool merge(const U (&vertices)[N])
    {
        return merge(vertices, N, [](const U& vertex) { return vec_type(vertex); });
    }

    template <typename U, std::size_t N, typename Converter,
              std::enable_if_t<!std::is_arithmetic<U>{} && !std::is_arithmetic<std::decay_t<Converter>>{}, int> = 0>
    bool merge(const U (&vertices)[N], const Converter& toVec)
    {
        return merge(vertices, N, toVec);
    }

    /// 将固定长度扁平标量 C 数组并入当前包围盒；每 3 个标量解释为一个顶点。
    template <typename U, std::size_t N, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    bool merge(const U (&scalarVertices)[N])
    {
        return merge(scalarVertices, N);
    }

    /// 返回 8 个角点；invalid box 返回空数组。
    std::vector<vec_type> getPoints() const
    {
        if (!valid()) return {};

        std::vector<vec_type> points;
        points.reserve(8);
        points.emplace_back(m_min.x(), m_min.y(), m_min.z());
        points.emplace_back(m_max.x(), m_min.y(), m_min.z());
        points.emplace_back(m_max.x(), m_max.y(), m_min.z());
        points.emplace_back(m_min.x(), m_max.y(), m_min.z());
        points.emplace_back(m_min.x(), m_min.y(), m_max.z());
        points.emplace_back(m_max.x(), m_min.y(), m_max.z());
        points.emplace_back(m_max.x(), m_max.y(), m_max.z());
        points.emplace_back(m_min.x(), m_max.y(), m_max.z());
        return points;
    }

    /// 按包围盒中心进行非均匀缩放。
    TAABB scale(const vec_type& scale3) const
    {
        return scale(makeScaleMatrix4x4(scale3));
    }

    /// 按包围盒中心应用一个缩放矩阵。
    TAABB scale(const matrix_type& scaleMat) const
    {
        if (!valid()) return TAABB();

        const vec_type boxCenter = center();
        return translate(-boxCenter).transform(scaleMat).translate(boxCenter);
    }

    /// 用任意仿射矩阵把当前包围盒变换到另一空间。
    TAABB convertToSpace(const matrix_type& spaceMatrix) const
    {
        return transform(spaceMatrix);
    }

    /// 按包围盒中心旋转。
    TAABB rotate(const quaternion_type& quat) const
    {
        if (!valid()) return TAABB();

        const vec_type boxCenter = center();
        return translate(-boxCenter).transform(QuaternionToMatrix4x4(quat)).translate(boxCenter);
    }

    /// 平移包围盒。
    TAABB translate(const vec_type& trans) const
    {
        if (!valid()) return TAABB();
        return TAABB(m_min + trans, m_max + trans);
    }

    /// 将包围盒中心移动到原点。
    TAABB moveToZeroPoint() const
    {
        return translate(-center());
    }

    /// 对 8 个角点应用一般仿射矩阵并重新包围。
    TAABB transform(const matrix_type& transMat) const
    {
        if (!valid()) return TAABB();

        TAABB newAABB;
        for (const vec_type& point : getPoints())
        {
            const Eigen::Vector<value_type, 4> homoPoint = transMat * point.homogeneous();
            newAABB.merge(homoPoint.template head<3>());
        }
        return newAABB;
    }

    /// 先应用 viewProjection，再做齐次除法，返回 NDC 下的新 AABB。
    TAABB viewProject(const matrix_type& viewProjection) const
    {
        if (!valid()) return TAABB();

        TAABB newAABB;
        for (const vec_type& point : getPoints())
        {
            const Eigen::Vector<value_type, 4> homoPosition = viewProjection * point.homogeneous();
            if (std::abs(homoPosition.w()) <= std::numeric_limits<value_type>::epsilon()) continue;
            newAABB.merge(homoPosition.hnormalized());
        }
        return newAABB;
    }

private:
    /// 以统一标量构造包围盒，若 `minValue > maxValue` 会自动归一化。
    TAABB(value_type minValue, value_type maxValue)
        : TAABB(vec_type::Constant(minValue), vec_type::Constant(maxValue))
    {
    }

    /// 由最小点和最大点构造包围盒，内部会自动归一化各分量顺序。
    TAABB(const vec_type& minValue, const vec_type& maxValue)
        : m_min(minValue.cwiseMin(maxValue))
        , m_max(minValue.cwiseMax(maxValue))
    {
    }

    /// 由一组顶点直接构造 AABB。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    explicit TAABB(const std::vector<U>& vertices)
        : TAABB()
    {
        merge(vertices);
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    explicit TAABB(const std::vector<U>& vertices, const Converter& toVec)
        : TAABB()
    {
        merge(vertices, toVec);
    }

    /// 由扁平标量数组构造 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    explicit TAABB(const std::vector<U>& scalarVertices)
        : TAABB()
    {
        merge(scalarVertices);
    }

    /// 由裸指针和顶点数量构造 AABB。
    template <typename U, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    TAABB(const U* vertices, std::size_t count)
        : TAABB()
    {
        merge(vertices, count);
    }

    template <typename U, typename Converter, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    TAABB(const U* vertices, std::size_t count, const Converter& toVec)
        : TAABB()
    {
        merge(vertices, count, toVec);
    }

    /// 由扁平标量裸指针和标量数量构造 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    TAABB(const U* scalarVertices, std::size_t scalarCount)
        : TAABB()
    {
        merge(scalarVertices, scalarCount);
    }

    /// 由固定长度 C 数组构造 AABB。
    template <typename U, std::size_t N, std::enable_if_t<!std::is_arithmetic<U>{}, int> = 0>
    explicit TAABB(const U (&vertices)[N])
        : TAABB(static_cast<const U*>(vertices), N)
    {
    }

    template <typename U, std::size_t N, typename Converter,
              std::enable_if_t<!std::is_arithmetic<U>{} && !std::is_arithmetic<std::decay_t<Converter>>{}, int> = 0>
    explicit TAABB(const U (&vertices)[N], const Converter& toVec)
        : TAABB(static_cast<const U*>(vertices), N, toVec)
    {
    }

    /// 由固定长度扁平标量 C 数组构造 AABB；每 3 个标量解释为一个顶点。
    template <typename U, std::size_t N, std::enable_if_t<std::is_arithmetic<U>{}, int> = 0>
    explicit TAABB(const U (&scalarVertices)[N])
        : TAABB(scalarVertices, N)
    {
    }

    template <typename R>
    explicit TAABB(const TAABB<R>& other)
        : TAABB()
    {
        if (other.valid())
        {
            m_min = other.getMin().template cast<value_type>();
            m_max = other.getMax().template cast<value_type>();
        }
    }

    vec_type m_min;
    vec_type m_max;
};

using AABB = TAABB<float>;
using AABBd = TAABB<double>;
using AABBld = TAABB<long double>;

/// @brief 判断两个 AABB 的最小点和最大点是否逐分量完全相等。
template <typename T>
inline bool operator==(const TAABB<T>& lhs, const TAABB<T>& rhs)
{
    return lhs.getMin() == rhs.getMin() && lhs.getMax() == rhs.getMax();
}

/// @brief 判断两个 AABB 是否不相等。
template <typename T>
inline bool operator!=(const TAABB<T>& lhs, const TAABB<T>& rhs)
{
    return !(lhs == rhs);
}

/// @brief 按 `(min, max)` 的字典序比较两个 AABB。
///
/// 该比较只用于为关联容器和排序提供稳定顺序，
/// 不表示几何意义上的“包围盒更大/更小”。
template <typename T>
inline bool operator<(const TAABB<T>& lhs, const TAABB<T>& rhs)
{
    if (lhs.getMin().x() < rhs.getMin().x()) return true;
    if (lhs.getMin().x() > rhs.getMin().x()) return false;
    if (lhs.getMin().y() < rhs.getMin().y()) return true;
    if (lhs.getMin().y() > rhs.getMin().y()) return false;
    if (lhs.getMin().z() < rhs.getMin().z()) return true;
    if (lhs.getMin().z() > rhs.getMin().z()) return false;

    if (lhs.getMax().x() < rhs.getMax().x()) return true;
    if (lhs.getMax().x() > rhs.getMax().x()) return false;
    if (lhs.getMax().y() < rhs.getMax().y()) return true;
    if (lhs.getMax().y() > rhs.getMax().y()) return false;
    return lhs.getMax().z() < rhs.getMax().z();
}

} // namespace Math
} // namespace ZF