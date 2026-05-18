// intersectRayTriangle (Möller–Trumbore) 的 GoogleTest 测试。
//
// 校验项：
//   - 命中三角形内部、边、顶点时返回 true，且重心坐标 / t 数值正确。
//   - 命中点重建：P = (1-u-v)*v0 + u*v1 + v*v2 == orig + t*dir。
//   - 双面命中（背面射入）。
//   - 反向射线 (t < 0) 视为未命中。
//   - 射线平行于三角形平面视为未命中。
//   - 击中三角形外（重心坐标越界）视为未命中。
//   - dir 未归一化时 t 为参数而非欧氏距离。
//   - float / double 模板均可用。

#include <gtest/gtest.h>

#include <Core/Math/RayIntersect.h>

#include <cmath>

using ZF::Math::intersectRayTriangle;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector3f;

namespace
{

// XY 平面上的单位三角形：v0 原点，v1 沿 +X，v2 沿 +Y。
// 用它做主要测试，命中点 (x, y, 0) 的重心坐标恰好是 (u, v) = (x, y)。
constexpr double kV0X = 0.0, kV0Y = 0.0;
constexpr double kV1X = 1.0, kV1Y = 0.0;
constexpr double kV2X = 0.0, kV2Y = 1.0;

const Vector3d kV0(kV0X, kV0Y, 0.0);
const Vector3d kV1(kV1X, kV1Y, 0.0);
const Vector3d kV2(kV2X, kV2Y, 0.0);

constexpr double kTol = 1e-12;

} // namespace

TEST(RayIntersectTriangle, HitsInteriorFromAbove)
{
    // 从 (0.25, 0.25, 1) 沿 -Z 射向三角形内部点 (0.25, 0.25, 0)。
    Vector3d orig(0.25, 0.25, 1.0);
    Vector3d dir(0.0, 0.0, -1.0);

    Vector2d bary;
    double t = 0.0;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    EXPECT_NEAR(bary.x(), 0.25, kTol);
    EXPECT_NEAR(bary.y(), 0.25, kTol);
    EXPECT_NEAR(t, 1.0, kTol);

    // 重心坐标重建命中点 P = (1-u-v)*v0 + u*v1 + v*v2，应与 orig + t*dir 一致。
    const double w = 1.0 - bary.x() - bary.y();
    Vector3d pBary    = w * kV0 + bary.x() * kV1 + bary.y() * kV2;
    Vector3d pRay     = orig + t * dir;
    EXPECT_NEAR((pBary - pRay).norm(), 0.0, kTol);
}

TEST(RayIntersectTriangle, HitsBackFaceIsAccepted)
{
    // 从 -Z 一侧射向三角形 (背面)：算法是双面的，应当命中。
    Vector3d orig(0.25, 0.25, -2.0);
    Vector3d dir(0.0, 0.0, 1.0);

    Vector2d bary;
    double t = 0.0;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    EXPECT_NEAR(bary.x(), 0.25, kTol);
    EXPECT_NEAR(bary.y(), 0.25, kTol);
    EXPECT_NEAR(t, 2.0, kTol);
}

TEST(RayIntersectTriangle, HitsAtVertices)
{
    Vector3d dir(0.0, 0.0, -1.0);

    // 顶点 v0 -> (u, v) = (0, 0)
    {
        Vector3d orig(kV0X, kV0Y, 1.0);
        Vector2d bary;
        double t = 0.0;
        ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));
        EXPECT_NEAR(bary.x(), 0.0, kTol);
        EXPECT_NEAR(bary.y(), 0.0, kTol);
        EXPECT_NEAR(t, 1.0, kTol);
    }
    // 顶点 v1 -> (u, v) = (1, 0)
    {
        Vector3d orig(kV1X, kV1Y, 1.0);
        Vector2d bary;
        double t = 0.0;
        ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));
        EXPECT_NEAR(bary.x(), 1.0, kTol);
        EXPECT_NEAR(bary.y(), 0.0, kTol);
    }
    // 顶点 v2 -> (u, v) = (0, 1)
    {
        Vector3d orig(kV2X, kV2Y, 1.0);
        Vector2d bary;
        double t = 0.0;
        ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));
        EXPECT_NEAR(bary.x(), 0.0, kTol);
        EXPECT_NEAR(bary.y(), 1.0, kTol);
    }
}

TEST(RayIntersectTriangle, HitsOnEdge)
{
    // 命中 v1-v2 边的中点 (0.5, 0.5, 0)，重心坐标应为 (0.5, 0.5)。
    Vector3d orig(0.5, 0.5, 1.0);
    Vector3d dir(0.0, 0.0, -1.0);

    Vector2d bary;
    double t = 0.0;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    EXPECT_NEAR(bary.x(), 0.5, kTol);
    EXPECT_NEAR(bary.y(), 0.5, kTol);
    EXPECT_NEAR(t, 1.0, kTol);
}

TEST(RayIntersectTriangle, MissesOutsideTriangle)
{
    Vector3d dir(0.0, 0.0, -1.0);
    Vector2d bary;
    double t = 0.0;

    // u + v > 1：在三角形 "斜边外侧"。
    EXPECT_FALSE(intersectRayTriangle(Vector3d(0.8, 0.8, 1.0), dir, kV0, kV1, kV2, bary, t));
    // u < 0
    EXPECT_FALSE(intersectRayTriangle(Vector3d(-0.1, 0.5, 1.0), dir, kV0, kV1, kV2, bary, t));
    // v < 0
    EXPECT_FALSE(intersectRayTriangle(Vector3d(0.5, -0.1, 1.0), dir, kV0, kV1, kV2, bary, t));
}

TEST(RayIntersectTriangle, MissesWhenRayPointsAway)
{
    // 射线起点在三角形上方，但方向也朝 +Z (远离)，应得 t < 0 -> 未命中。
    Vector3d orig(0.25, 0.25, 1.0);
    Vector3d dir(0.0, 0.0, 1.0);

    Vector2d bary;
    double t = 0.0;
    EXPECT_FALSE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));
}

TEST(RayIntersectTriangle, MissesWhenRayParallelToPlane)
{
    // 射线起点在 z = 1 平面，方向沿 +X，与 XY 平面平行 -> det ≈ 0 -> 未命中。
    Vector3d orig(0.25, 0.25, 1.0);
    Vector3d dir(1.0, 0.0, 0.0);

    Vector2d bary;
    double t = 0.0;
    EXPECT_FALSE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));
}

TEST(RayIntersectTriangle, NonUnitDirectionGivesParameterT)
{
    // dir 长度为 2 时，t 应为 0.5 (从 z=1 到 z=0 走半步)；命中点仍正确。
    Vector3d orig(0.25, 0.25, 1.0);
    Vector3d dir(0.0, 0.0, -2.0);

    Vector2d bary;
    double t = 0.0;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    EXPECT_NEAR(t, 0.5, kTol);

    Vector3d pRay = orig + t * dir;
    EXPECT_NEAR(pRay.z(), 0.0, kTol);
    EXPECT_NEAR(pRay.x(), 0.25, kTol);
    EXPECT_NEAR(pRay.y(), 0.25, kTol);
}

TEST(RayIntersectTriangle, ObliqueRay)
{
    // 倾斜射线：从 (-1, 0, 1) 朝 (1, 0.5, -1) 方向，命中点应在三角形内部。
    Vector3d orig(-1.0, 0.0, 1.0);
    Vector3d dir(1.0, 0.5, -1.0); // 未归一化

    Vector2d bary;
    double t = 0.0;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    // 命中点 = orig + t*dir，应满足 z = 0。
    Vector3d pRay = orig + t * dir;
    EXPECT_NEAR(pRay.z(), 0.0, kTol);

    // 重心坐标重建结果应一致。
    const double w = 1.0 - bary.x() - bary.y();
    Vector3d pBary    = w * kV0 + bary.x() * kV1 + bary.y() * kV2;
    EXPECT_NEAR((pBary - pRay).norm(), 0.0, kTol);

    // 落在三角形内部：u, v >= 0 且 u + v <= 1。
    EXPECT_GE(bary.x(), 0.0);
    EXPECT_GE(bary.y(), 0.0);
    EXPECT_LE(bary.x() + bary.y(), 1.0);
}

TEST(RayIntersectTriangle, FloatTemplateInstantiation)
{
    // 模板 T = float 也应能正常工作。
    Vector3f orig(0.25f, 0.25f, 1.0f);
    Vector3f dir(0.0f, 0.0f, -1.0f);
    Vector3f v0(0.0f, 0.0f, 0.0f);
    Vector3f v1(1.0f, 0.0f, 0.0f);
    Vector3f v2(0.0f, 1.0f, 0.0f);

    Vector2f bary;
    float t = 0.0f;
    ASSERT_TRUE(intersectRayTriangle(orig, dir, v0, v1, v2, bary, t));

    EXPECT_NEAR(bary.x(), 0.25f, 1e-5f);
    EXPECT_NEAR(bary.y(), 0.25f, 1e-5f);
    EXPECT_NEAR(t, 1.0f, 1e-5f);
}

TEST(RayIntersectTriangle, OutputsUnchangedOnMiss)
{
    // 未命中时不修改输出参数 (符合实现约定)。
    Vector3d dir(0.0, 0.0, 1.0); // 朝远离方向
    Vector3d orig(0.25, 0.25, 1.0);

    Vector2d bary(-7.0, 42.0);
    double t = 99.0;
    EXPECT_FALSE(intersectRayTriangle(orig, dir, kV0, kV1, kV2, bary, t));

    EXPECT_DOUBLE_EQ(bary.x(), -7.0);
    EXPECT_DOUBLE_EQ(bary.y(), 42.0);
    EXPECT_DOUBLE_EQ(t, 99.0);
}
