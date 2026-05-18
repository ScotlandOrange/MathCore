// Frustum 的 GoogleTest 测试 (OpenGL 坐标系，左乘约定)。
//
// 覆盖三个职责：
//   1. 视锥剔除 (单位视锥 + 球体相交 / 矩阵变换后的平面)
//   2. 投影矩阵 (正交 / 透视，OpenGL 风格)
//   3. 二维矩形操作 (width/height/center2D/shift2D/scale2DWithRatio/ndcToView 等)

#include <gtest/gtest.h>

#include <Core/Frustum.h>

#include <cmath>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

using ZF::Math::distance;
using ZF::Math::Frustum;
using ZF::Math::TSphere;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

// ============================================================
// 1. 默认 / 单位视锥与球体剔除
// ============================================================

TEST(FrustumUnit, AllSixFacesAreUnitDistanceFromOrigin)
{
    Frustum f;

    Vector3d origin(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < Frustum::kFaceCount; ++i)
    {
        EXPECT_NEAR(distance(f.face(i), origin), 1.0, 1e-12) << "face " << i;
    }
}

TEST(FrustumIntersect, OriginIsInsideUnitFrustum)
{
    Frustum f;
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, 0.0), 0.25)));
}

TEST(FrustumIntersect, SphereOutsideRightFaceIsCulled)
{
    Frustum f;
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(10.0, 0.0, 0.0), 0.5)));
}

TEST(FrustumIntersect, SphereTangentToRightFaceIsAccepted)
{
    Frustum f;
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(2.0, 0.0, 0.0), 1.0)));
}

TEST(FrustumIntersect, NearAndFarPlaneCullSpheresOutsideZRange)
{
    Frustum f;
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0,  5.0), 0.25)));
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, -5.0), 0.25)));
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0,  0.0), 0.25)));
}

// ============================================================
// 2. 由参考视锥经矩阵变换重建剔除平面 (左乘)
// ============================================================

TEST(FrustumTransform, IdentityMatrixPreservesPlanes)
{
    Frustum unit;
    Frustum transformed(unit, Matrix4d::Identity());

    for (std::size_t i = 0; i < Frustum::kFaceCount; ++i)
    {
        EXPECT_TRUE(transformed.face(i).vec().isApprox(unit.face(i).vec(), 1e-12))
            << "face " << i;
    }
}

TEST(FrustumTransform, RotationAroundZRotatesPlaneNormals)
{
    Frustum unit;
    // 绕 Z 轴 +90°：x → y, y → -x
    Matrix4d rot = Matrix4d::Identity();
    rot(0, 0) = 0.0; rot(0, 1) = -1.0;
    rot(1, 0) = 1.0; rot(1, 1) =  0.0;

    // setTransformed 接受 S->D 点变换，内部取逆后调 Plane operator*。
    // 平面公式化为 p_D = (rot^{-1})^T * p_S；正交矩阵下 rot^{-T} = rot，
    // 等价于把法线跟点一起绕 Z 轴转 +90°。
    Frustum r(unit, rot);

    // 原 LEFT 面 (法线 +X, d = 1) 转后法线 → +Y
    EXPECT_NEAR(r.face(Frustum::FACE_LEFT).norm()[0],  0.0, 1e-12);
    EXPECT_NEAR(r.face(Frustum::FACE_LEFT).norm()[1],  1.0, 1e-12);
    EXPECT_NEAR(r.face(Frustum::FACE_LEFT).d(),        1.0, 1e-12);

    // 原 BOTTOM 面 (法线 +Y) → 法线 -X
    EXPECT_NEAR(r.face(Frustum::FACE_BOTTOM).norm()[0], -1.0, 1e-12);
    EXPECT_NEAR(r.face(Frustum::FACE_BOTTOM).norm()[1],  0.0, 1e-12);
}

TEST(FrustumTransform, TranslationCorrectlyShiftsCullingRegion)
{
    // setTransformed 接受 S->D 点变换。这里 S = 单位视锥所在的局部空间，
    // D = 世界空间，所以传 localToWorld：把原点搬到世界 +5 处，
    // 进而把单位视锥沿 +X 平移 5。平移矩阵非正交，是验证
    // (M^{-1})^T 公式正确性的关键用例。
    Frustum unit;

    Matrix4d localToWorld = Matrix4d::Identity();
    localToWorld(0, 3) = 5.0;

    Frustum shifted(unit, localToWorld);

    // 原点位于平移后的视锥外侧 (右面距离应为 -4)
    EXPECT_FALSE(shifted.intersect(TSphere<double>(Vector3d(0.0, 0.0, 0.0), 0.5)));
    // 平移后的中心 (5, 0, 0) 应在视锥内
    EXPECT_TRUE(shifted.intersect(TSphere<double>(Vector3d(5.0, 0.0, 0.0), 0.1)));
    // 越过新右面 (x = 6)
    EXPECT_FALSE(shifted.intersect(TSphere<double>(Vector3d(7.0, 0.0, 0.0), 0.5)));
}

// ============================================================
// 3. setOrtho / setPerspective 参数与视空间剔除平面
// ============================================================

TEST(FrustumSetOrtho, BoundsMapToViewSpacePlanes)
{
    Frustum f = Frustum::makeOrtho(-2.0, 2.0, -1.0, 1.0, 0.1, 100.0);

    EXPECT_TRUE(f.isOrthographic());
    EXPECT_EQ(f.left(),   -2.0);
    EXPECT_EQ(f.right(),   2.0);
    EXPECT_EQ(f.bottom(), -1.0);
    EXPECT_EQ(f.top(),     1.0);
    EXPECT_EQ(f.nearZ(),   0.1);
    EXPECT_EQ(f.farZ(), 100.0);
    EXPECT_NEAR(f.aspect(), 4.0 / 2.0, 1e-12);

    // OpenGL：相机看 -Z，原点 (0,0,0) 在近平面前方 → 应被剔除
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, 0.0), 0.05)));
    // (0, 0, -10) 在视锥内
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, -10.0), 0.05)));
    // (3, 0, -10) 超出右侧
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(3.0, 0.0, -10.0), 0.5)));
}

TEST(FrustumSetPerspective, FovAndNearFarDriveAspect)
{
    Frustum f = Frustum::makePerspective(60.0, 30.0, 0.5, 50.0);

    EXPECT_TRUE(f.isPerspective());
    EXPECT_DOUBLE_EQ(f.aspect(), 2.0);
    EXPECT_DOUBLE_EQ(f.nearZ(), 0.5);
    EXPECT_DOUBLE_EQ(f.farZ(), 50.0);

    // 透视视锥：4 个侧面均过原点，近/远平面把它排除
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, 0.0), 0.01)));
    // -Z 方向 z = -10 在视锥内
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, -10.0), 0.05)));
    // 摄像机背后
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, 10.0), 0.05)));
}

// ============================================================
// 4. 投影矩阵 (OpenGL，深度 [-1, 1])
// ============================================================

TEST(FrustumProjection, OrthoMapsBoundsToOpenGLNdc)
{
    Frustum f = Frustum::makeOrtho(-2.0, 2.0, -1.0, 1.0, 1.0, 10.0);
    Matrix4d p = f.projection();

    // 视空间近平面右上角 (right, top, -near, 1) → (1, 1, -1, 1)
    Vector4d nearTopRight(2.0, 1.0, -1.0, 1.0);
    Vector4d ndc = p * nearTopRight;
    EXPECT_NEAR(ndc.x(),  1.0, 1e-12);
    EXPECT_NEAR(ndc.y(),  1.0, 1e-12);
    EXPECT_NEAR(ndc.z(), -1.0, 1e-12);
    EXPECT_NEAR(ndc.w(),  1.0, 1e-12);

    // (left, bottom, -far, 1) → (-1, -1, 1, 1)
    Vector4d farBottomLeft(-2.0, -1.0, -10.0, 1.0);
    Vector4d ndc2 = p * farBottomLeft;
    EXPECT_NEAR(ndc2.x(), -1.0, 1e-12);
    EXPECT_NEAR(ndc2.y(), -1.0, 1e-12);
    EXPECT_NEAR(ndc2.z(),  1.0, 1e-12);
}

TEST(FrustumProjection, PerspectiveMapsCenterToOriginAndDepthIntoNdc)
{
    Frustum f = Frustum::makePerspective(60.0, 60.0, 1.0, 100.0);
    Matrix4d p = f.projection();

    // 中心 (0, 0, -10, 1) → NDC (0, 0, *, *)，z 落在 [-1, 1] 内
    Vector4d center(0.0, 0.0, -10.0, 1.0);
    Vector4d clip = p * center;
    Vector4d ndc = clip / clip.w();

    EXPECT_NEAR(ndc.x(), 0.0, 1e-12);
    EXPECT_NEAR(ndc.y(), 0.0, 1e-12);
    EXPECT_GT(ndc.z(), -1.0);
    EXPECT_LT(ndc.z(),  1.0);

    // 近平面中心 z = -1，远平面 z = +1
    Vector4d nearCenter(0.0, 0.0, -1.0, 1.0);
    Vector4d nearClip = p * nearCenter;
    EXPECT_NEAR(nearClip.z() / nearClip.w(), -1.0, 1e-9);

    Vector4d farCenter(0.0, 0.0, -100.0, 1.0);
    Vector4d farClip = p * farCenter;
    EXPECT_NEAR(farClip.z() / farClip.w(), 1.0, 1e-9);
}

TEST(FrustumProjection, InverseProjectionRoundTripsForOrtho)
{
    Frustum f = Frustum::makeOrtho(-3.0, 3.0, -2.0, 2.0, 1.0, 10.0);
    Matrix4d round = f.projection() * f.inverseProjection();
    EXPECT_TRUE(round.isApprox(Matrix4d::Identity(), 1e-9));
}

// ============================================================
// 5. 二维矩形操作 (旧 OrthoFrustum)
// ============================================================

TEST(FrustumRect, WidthHeightCenter)
{
    Frustum f = Frustum::makeOrtho(0.0, 4.0, 0.0, 2.0, 0.0, 1.0);

    EXPECT_DOUBLE_EQ(f.width(),  4.0);
    EXPECT_DOUBLE_EQ(f.height(), 2.0);
    EXPECT_EQ(f.size(), Vector2d(4.0, 2.0));
    EXPECT_EQ(f.center2D(), Vector2d(2.0, 1.0));
    EXPECT_EQ(f.topLeft(), Vector2d(0.0, 2.0));
    EXPECT_EQ(f.center(), Vector3d(2.0, 1.0, 0.5));
}

TEST(FrustumRect, Shift2DTranslatesBoundsAndPlanes)
{
    Frustum f = Frustum::makeOrtho(-1.0, 1.0, -1.0, 1.0, 0.0, 1.0);

    f.shift2D(Vector2d(5.0, -3.0));

    EXPECT_DOUBLE_EQ(f.left(),    4.0);
    EXPECT_DOUBLE_EQ(f.right(),   6.0);
    EXPECT_DOUBLE_EQ(f.bottom(), -4.0);
    EXPECT_DOUBLE_EQ(f.top(),    -2.0);

    // 移位后中心应在视锥内 (z = -0.5，对应 OpenGL 视空间)
    EXPECT_TRUE(f.intersect(TSphere<double>(Vector3d(5.0, -3.0, -0.5), 0.1)));
    EXPECT_FALSE(f.intersect(TSphere<double>(Vector3d(0.0, 0.0, -0.5), 0.1)));
}

TEST(FrustumRect, Scale2DWithRatioKeepsAspect)
{
    Frustum f = Frustum::makeOrtho(-2.0, 2.0, -1.0, 1.0, 0.0, 1.0); // aspect = 2
    f.scale2DWithRatio(0.5); // 高度增量 0.5 → deltaH = 0.5*2 + 0.5 = 1.5

    // 中心 (0, 0)，新高度 = 3，aspect = 2 → 新宽度 = 6
    EXPECT_NEAR(f.height(), 3.0, 1e-12);
    EXPECT_NEAR(f.width(),  6.0, 1e-12);
    EXPECT_EQ(f.center2D(), Vector2d(0.0, 0.0));
}

TEST(FrustumRect, LocalPosToMapsNormalizedToPhysical)
{
    Frustum f = Frustum::makeOrtho(0.0, 100.0, 0.0, 50.0, 0.0, 1.0);

    Vector2d origin(10.0, 20.0);
    Vector2d mapped = f.localPosTo(Vector2d(0.5, 0.5), origin);
    EXPECT_EQ(mapped, Vector2d(60.0, 45.0));
}

TEST(FrustumRect, NdcToViewMapsCornersToBounds)
{
    Frustum f = Frustum::makeOrtho(-2.0, 2.0, -1.0, 1.0, 0.5, 5.0);

    // u=v=1, d=1 → (right, top, -far)
    Vector3d p = f.ndcToView(1.0, 1.0, 1.0);
    EXPECT_NEAR(p.x(),  2.0, 1e-12);
    EXPECT_NEAR(p.y(),  1.0, 1e-12);
    EXPECT_NEAR(p.z(), -5.0, 1e-12);

    // u=v=-1, d=-1 → (left, bottom, -near)
    Vector3d q = f.ndcToView(-1.0, -1.0, -1.0);
    EXPECT_NEAR(q.x(), -2.0, 1e-12);
    EXPECT_NEAR(q.y(), -1.0, 1e-12);
    EXPECT_NEAR(q.z(), -0.5, 1e-12);
}

// ============================================================
// 6. 有效性
// ============================================================

TEST(FrustumValid, RejectsDegenerateBounds)
{
    Frustum f = Frustum::makeOrtho(0.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    EXPECT_FALSE(f.valid());

    Frustum g = Frustum::makeOrtho(0.0, 1.0, 0.0, 1.0, 1.0, 0.5);
    EXPECT_FALSE(g.valid());

    Frustum h = Frustum::makeOrtho(-1.0, 1.0, -1.0, 1.0, 0.0, 10.0);
    EXPECT_TRUE(h.valid());
}

TEST(FrustumValid, RejectsZeroFovInPerspective)
{
    Frustum f = Frustum::makePerspective(0.0, 0.0, 0.1, 10.0);
    EXPECT_FALSE(f.valid());

    Frustum g = Frustum::makePerspective(60.0, 30.0, 0.1, 10.0);
    EXPECT_TRUE(g.valid());
}

// ============================================================
// 7. LOD 缩放
// ============================================================

TEST(FrustumLodScale, IdentityViewMatrixGivesProjectionDependentScale)
{
    Frustum f;
    Matrix4d proj = Matrix4d::Identity();
    proj(1, 1) = -2.0; // -proj(1,1) = 2

    Matrix4d mv = Matrix4d::Identity();

    f.computeLodScale(proj, mv);

    const double inv = 1.0 / std::sqrt(2.0);
    const Vector4d& s = f.lodScale();
    EXPECT_NEAR(s[0], 0.0,       1e-12);
    EXPECT_NEAR(s[1], 0.0,       1e-12);
    EXPECT_NEAR(s[2], 1.0 * inv, 1e-12);
    EXPECT_NEAR(s[3], 0.0,       1e-12);
}
