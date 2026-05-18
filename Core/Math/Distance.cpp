#include <Core/Math/Distance.h>
#include <Core/Math/Numbers.h>

#include <algorithm>
#include <cmath>

namespace ZF
{
namespace Math
{

// ---- 点到无限直线 ----

Eigen::Vector3f closestPointOnLine(const Eigen::Vector3f& linePoint1,
                                   const Eigen::Vector3f& linePoint2,
                                   const Eigen::Vector3f& point)
{
    const Eigen::Vector3f dir  = linePoint2 - linePoint1;
    const float           len2 = dir.squaredNorm();
    if (len2 <= Numbersf::epsilon2())
    {
        return linePoint1; // 退化：两点重合
    }
    const float t = (point - linePoint1).dot(dir) / len2;
    return linePoint1 + t * dir;
}

float distancePointToLine(const Eigen::Vector3f& point,
                          const Eigen::Vector3f& linePoint1,
                          const Eigen::Vector3f& linePoint2)
{
    return (point - closestPointOnLine(linePoint1, linePoint2, point)).norm();
}

// ---- 点到线段 ----

Eigen::Vector3f closestPointOnSegment(const Eigen::Vector3f& segStart,
                                      const Eigen::Vector3f& segEnd,
                                      const Eigen::Vector3f& point)
{
    const Eigen::Vector3f dir  = segEnd - segStart;
    const float           len2 = dir.squaredNorm();
    if (len2 <= Numbersf::epsilon2())
    {
        return segStart;
    }
    const float t = std::min(1.0f, std::max(0.0f, (point - segStart).dot(dir) / len2));
    return segStart + t * dir;
}

float distancePointToSegment(const Eigen::Vector3f& point,
                             const Eigen::Vector3f& segStart,
                             const Eigen::Vector3f& segEnd)
{
    return (point - closestPointOnSegment(segStart, segEnd, point)).norm();
}

std::pair<float, Eigen::Vector3f>
distancePointToSegmentWithNearest(const Eigen::Vector3f& point,
                                  const Eigen::Vector3f& segStart,
                                  const Eigen::Vector3f& segEnd)
{
    const Eigen::Vector3f nearest = closestPointOnSegment(segStart, segEnd, point);
    return { (point - nearest).norm(), nearest };
}

static Eigen::Vector2f closestPointOnSegment2D(const Eigen::Vector2f& segStart,
                                               const Eigen::Vector2f& segEnd,
                                               const Eigen::Vector2f& point)
{
    const Eigen::Vector2f dir  = segEnd - segStart;
    const float           len2 = dir.squaredNorm();
    if (len2 <= Numbersf::epsilon2())
    {
        return segStart;
    }
    const float t = std::min(1.0f, std::max(0.0f, (point - segStart).dot(dir) / len2));
    return segStart + t * dir;
}

float distancePointToSegment2D(const Eigen::Vector2f& point,
                               const Eigen::Vector2f& segStart,
                               const Eigen::Vector2f& segEnd)
{
    return (point - closestPointOnSegment2D(segStart, segEnd, point)).norm();
}

std::pair<float, Eigen::Vector2f>
distancePointToSegment2DWithNearest(const Eigen::Vector2f& point,
                                    const Eigen::Vector2f& segStart,
                                    const Eigen::Vector2f& segEnd)
{
    const Eigen::Vector2f nearest = closestPointOnSegment2D(segStart, segEnd, point);
    return { (point - nearest).norm(), nearest };
}

// ---- 点到三角形（含退化处理）----

float distancePointToTriangle(const Eigen::Vector3f& point,
                              const Eigen::Vector3f& v0,
                              const Eigen::Vector3f& v1,
                              const Eigen::Vector3f& v2)
{
    return distancePointToTriangleWithNearest(point, v0, v1, v2).first;
}

std::pair<float, Eigen::Vector3f>
distancePointToTriangleWithNearest(const Eigen::Vector3f& point,
                                   const Eigen::Vector3f& v0,
                                   const Eigen::Vector3f& v1,
                                   const Eigen::Vector3f& v2)
{
    const Eigen::Vector3f e01 = v1 - v0;
    const Eigen::Vector3f e02 = v2 - v0;
    const Eigen::Vector3f n   = e01.cross(e02); // 法向（未归一化）
    const float           d   = n.squaredNorm();

    // 退化：三点共线，退回到三条边求距离取最小。
    if (d <= Numbersf::epsilon2())
    {
        Eigen::Vector3f best   = closestPointOnSegment(v0, v1, point);
        float           bestD2 = (point - best).squaredNorm();

        Eigen::Vector3f cand   = closestPointOnSegment(v1, v2, point);
        float           candD2 = (point - cand).squaredNorm();
        if (candD2 < bestD2) { best = cand; bestD2 = candD2; }

        cand   = closestPointOnSegment(v2, v0, point);
        candD2 = (point - cand).squaredNorm();
        if (candD2 < bestD2) { best = cand; bestD2 = candD2; }

        return { std::sqrt(bestD2), best };
    }

    // 非退化：参考 Real-Time Collision Detection 中三角形最近点算法。
    const float           invD = 1.0f / d;
    const Eigen::Vector3f e12  = v2 - v1;
    const Eigen::Vector3f v0p  = point - v0;
    const Eigen::Vector3f t    = v0p.cross(n);
    const float           a    = t.dot(e02) * -invD;
    const float           b    = t.dot(e01) *  invD;

    auto clampToEdge = [&](const Eigen::Vector3f& a0, const Eigen::Vector3f& edge,
                           const Eigen::Vector3f& vEnd0, const Eigen::Vector3f& vEnd1) -> Eigen::Vector3f
    {
        const float s = (point - a0).dot(edge) / edge.squaredNorm();
        if (s <= 0.0f) return vEnd0;
        if (s >= 1.0f) return vEnd1;
        return a0 + s * edge;
    };

    Eigen::Vector3f result;
    if (a < 0.0f)
    {
        const float s02v = e02.dot(v0p) / e02.squaredNorm();
        if      (s02v < 0.0f) result = clampToEdge(v0, e01, v0, v1);
        else if (s02v > 1.0f) result = clampToEdge(v1, e12, v1, v2);
        else                  result = v0 + s02v * e02;
    }
    else if (b < 0.0f)
    {
        const float s01v = e01.dot(v0p) / e01.squaredNorm();
        if      (s01v < 0.0f) result = clampToEdge(v0, e02, v0, v2);
        else if (s01v > 1.0f) result = clampToEdge(v1, e12, v1, v2);
        else                  result = v0 + s01v * e01;
    }
    else if (a + b > 1.0f)
    {
        const float s12v = e12.dot(point - v1) / e12.squaredNorm();
        if      (s12v < 0.0f) result = clampToEdge(v0, e01, v0, v1);
        else if (s12v > 1.0f) result = clampToEdge(v0, e02, v0, v2);
        else                  result = v1 + s12v * e12;
    }
    else
    {
        // 投影在三角形内部
        result = point - n * (n.dot(v0p) * invD);
    }

    return { (point - result).norm(), result };
}

} // namespace Math
} // namespace ZF
