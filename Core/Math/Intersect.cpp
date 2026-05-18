#include <Core/Math/Intersect.h>
#include <Core/Math/Numbers.h>

#include <cmath>

namespace ZF
{
namespace Math
{

// ---- 点是否在三角形内 ----

bool pointInTriangle(const Eigen::Vector3f& A,
                     const Eigen::Vector3f& B,
                     const Eigen::Vector3f& C,
                     const Eigen::Vector3f& P)
{
    const Eigen::Vector3f v0 = C - A;
    const Eigen::Vector3f v1 = B - A;
    const Eigen::Vector3f v2 = P - A;

    const float dot00 = v0.dot(v0);
    const float dot01 = v0.dot(v1);
    const float dot02 = v0.dot(v2);
    const float dot11 = v1.dot(v1);
    const float dot12 = v1.dot(v2);

    const float denom = dot00 * dot11 - dot01 * dot01;
    if (std::abs(denom) <= Numbersf::epsilon2())
    {
        return false; // 退化（共线）
    }

    const float invDeno = 1.0f / denom;
    const float u       = (dot11 * dot02 - dot01 * dot12) * invDeno;
    if (u < 0.0f || u > 1.0f) return false;

    const float v = (dot00 * dot12 - dot01 * dot02) * invDeno;
    if (v < 0.0f || v > 1.0f) return false;

    return (u + v) <= 1.0f;
}

bool pointInTriangle2D(const Eigen::Vector2f& P,
                       const Eigen::Vector2f& A,
                       const Eigen::Vector2f& B,
                       const Eigen::Vector2f& C)
{
    const float ab = (B.x() - A.x()) * (P.y() - A.y()) - (B.y() - A.y()) * (P.x() - A.x());
    const float bc = (C.x() - B.x()) * (P.y() - B.y()) - (C.y() - B.y()) * (P.x() - B.x());
    const float ca = (A.x() - C.x()) * (P.y() - C.y()) - (A.y() - C.y()) * (P.x() - C.x());
    return (ab >= 0.0f && bc >= 0.0f && ca >= 0.0f)
        || (ab <= 0.0f && bc <= 0.0f && ca <= 0.0f);
}

// ---- 裁剪空间可见性 ----

bool insideClipSpace(const Eigen::Vector4f& clipVertex)
{
    const float w = clipVertex.w();
    return clipVertex.x() >= -w && clipVertex.x() <= w
        && clipVertex.y() >= -w && clipVertex.y() <= w
        && clipVertex.z() >= -w && clipVertex.z() <= w;
}

bool triangleTouchesClipSpace(const Eigen::Vector4f& v1Clip,
                              const Eigen::Vector4f& v2Clip,
                              const Eigen::Vector4f& v3Clip)
{
    return insideClipSpace(v1Clip) || insideClipSpace(v2Clip) || insideClipSpace(v3Clip);
}

} // namespace Math
} // namespace ZF
