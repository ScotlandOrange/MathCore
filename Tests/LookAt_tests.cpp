// LookAt (Eigen) 的 GoogleTest 测试。
//
// 与 glm::lookAt 的行为对齐。校验项：
//   - 相机原点 (eye) 在视图空间映射到 (0, 0, 0)。
//   - 视点 (center) 在视图空间位于 -Z 轴上 (RH)，距离 = ||center - eye||。
//   - 上方向被映射到视图空间 +Y。
//   - 视图矩阵的旋转部分是正交的 (M^T * M = I)，行列式 = +1。
//   - 与已知数值情况对比 (相机沿 +X 看向原点)。
//   - LH 版本时视点位于 +Z 轴。
//   - float / double 模板均能编译运行。
//   - 接受 Eigen 表达式实参 (-v, Vector3f::Zero(), a + b)。

#include <gtest/gtest.h>

#include <Core/Math/LookAt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using ZF::Math::lookAt;
using ZF::Math::lookAtLH;
using ZF::Math::lookAtRH;

namespace
{

constexpr double kTol = 1e-12;

Eigen::Vector3d transformPoint(const Eigen::Matrix4d& m, const Eigen::Vector3d& p)
{
    Eigen::Vector4d h = m * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
    return Eigen::Vector3d(h.x(), h.y(), h.z());
}

} // namespace

TEST(LookAt, EyeMapsToOrigin)
{
    Eigen::Vector3d eye(2.0, 3.0, 5.0);
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAt(eye, center, up);
    Eigen::Vector3d p = transformPoint(m, eye);

    EXPECT_NEAR(p.x(), 0.0, kTol);
    EXPECT_NEAR(p.y(), 0.0, kTol);
    EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(LookAtRH, CenterIsOnNegativeZ)
{
    Eigen::Vector3d eye(2.0, 3.0, 5.0);
    Eigen::Vector3d center(1.0, 1.0, 1.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAtRH(eye, center, up);
    Eigen::Vector3d p = transformPoint(m, center);

    const double dist = (center - eye).norm();
    EXPECT_NEAR(p.x(), 0.0,   kTol);
    EXPECT_NEAR(p.y(), 0.0,   kTol);
    EXPECT_NEAR(p.z(), -dist, kTol);
}

TEST(LookAtLH, CenterIsOnPositiveZ)
{
    Eigen::Vector3d eye(2.0, 3.0, 5.0);
    Eigen::Vector3d center(1.0, 1.0, 1.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAtLH(eye, center, up);
    Eigen::Vector3d p = transformPoint(m, center);

    const double dist = (center - eye).norm();
    EXPECT_NEAR(p.x(), 0.0,  kTol);
    EXPECT_NEAR(p.y(), 0.0,  kTol);
    EXPECT_NEAR(p.z(), dist, kTol);
}

TEST(LookAt, UpDirectionMapsToPositiveY)
{
    Eigen::Vector3d eye(0.0, 0.0, 5.0);
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAt(eye, center, up);

    // up 是方向向量，用 w = 0 变换 (排除平移)。
    Eigen::Vector4d hUp = m * Eigen::Vector4d(up.x(), up.y(), up.z(), 0.0);
    EXPECT_NEAR(hUp.x(), 0.0, kTol);
    EXPECT_NEAR(hUp.y(), 1.0, kTol);
    EXPECT_NEAR(hUp.z(), 0.0, kTol);
}

TEST(LookAt, RotationBlockIsOrthonormal)
{
    Eigen::Vector3d eye(7.0, -2.0, 4.0);
    Eigen::Vector3d center(-1.0, 1.5, -3.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAt(eye, center, up);

    Eigen::Matrix3d R = m.block<3, 3>(0, 0);
    Eigen::Matrix3d shouldBeI = R.transpose() * R;
    EXPECT_NEAR((shouldBeI - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-10);
    EXPECT_NEAR(R.determinant(), 1.0, 1e-10);
}

TEST(LookAt, KnownCaseCameraOnPositiveX)
{
    // 相机站在 (5,0,0)，看向原点，up = +Y。
    // 期望视图矩阵旋转部分：
    //   row0 = ( 0,  0, -1)   (right)
    //   row1 = ( 0,  1,  0)   (up)
    //   row2 = ( 1,  0,  0)   (-forward)
    Eigen::Vector3d eye(5.0, 0.0, 0.0);
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    Eigen::Vector3d up(0.0, 1.0, 0.0);

    Eigen::Matrix4d m = lookAtRH(eye, center, up);

    EXPECT_NEAR(m(0, 0),  0.0, kTol); EXPECT_NEAR(m(0, 1), 0.0, kTol); EXPECT_NEAR(m(0, 2), -1.0, kTol);
    EXPECT_NEAR(m(1, 0),  0.0, kTol); EXPECT_NEAR(m(1, 1), 1.0, kTol); EXPECT_NEAR(m(1, 2),  0.0, kTol);
    EXPECT_NEAR(m(2, 0),  1.0, kTol); EXPECT_NEAR(m(2, 1), 0.0, kTol); EXPECT_NEAR(m(2, 2),  0.0, kTol);

    EXPECT_NEAR(m(0, 3),  0.0, kTol);
    EXPECT_NEAR(m(1, 3),  0.0, kTol);
    EXPECT_NEAR(m(2, 3), -5.0, kTol);
}

TEST(LookAt, FloatTemplateInstantiation)
{
    Eigen::Vector3f eye(0.0f, 0.0f, 3.0f);
    Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);

    Eigen::Matrix4f m = lookAt(eye, center, up);

    Eigen::Vector4f hEye = m * Eigen::Vector4f(eye.x(), eye.y(), eye.z(), 1.0f);
    EXPECT_NEAR(hEye.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(hEye.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(hEye.z(), 0.0f, 1e-5f);

    Eigen::Vector4f hCenter = m * Eigen::Vector4f(center.x(), center.y(), center.z(), 1.0f);
    EXPECT_NEAR(hCenter.x(),  0.0f, 1e-6f);
    EXPECT_NEAR(hCenter.y(),  0.0f, 1e-6f);
    EXPECT_NEAR(hCenter.z(), -3.0f, 1e-5f);
}

// 关键回归用例：Eigen 表达式 (-v, Zero(), a + b) 直接传入应能编译并得到正确结果。
TEST(LookAt, AcceptsEigenExpressions)
{
    Eigen::Vector3f forwardDir(1.0f, 0.0f, 0.0f);    // 相机看向 +X
    Eigen::Vector3f upDir(0.0f, 1.0f, 0.0f);

    // 模仿真实调用：eye = -forwardDir (表达式)，center = Zero() (表达式)。
    Eigen::Matrix4f look = lookAt(-forwardDir, Eigen::Vector3f::Zero(), upDir);

    // 与显式求值结果应当完全一致。
    Eigen::Vector3f eyeEval = -forwardDir;
    Eigen::Matrix4f ref     = lookAt(eyeEval, Eigen::Vector3f::Zero().eval(), upDir);

    EXPECT_NEAR((look - ref).norm(), 0.0f, 1e-6f);

    // 数学上 eye = (-1, 0, 0)，看向原点，所以 forward = +X，
    // -forward = -X 应当是视图空间第三行；变换 eye 应得 (0, 0, 0)。
    Eigen::Vector4f h = look * Eigen::Vector4f(-1.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(h.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(h.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(h.z(), 0.0f, 1e-6f);
}
