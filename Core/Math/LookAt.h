#pragma once

// LookAt 视图矩阵 (Eigen 版本)。
//
// 与 glm::lookAt / glm::lookAtRH 行为对齐：
//   - 右手坐标系，相机看向 -Z (OpenGL 风格)。
//   - 返回 world -> view 的 4x4 矩阵 (列主序)。
//   - 输入: eye 相机位置，center 视点目标，up 上方向 (无需归一化)。
//
// 数学定义：
//   f = normalize(center - eye)        // 相机看向方向
//   r = normalize(f × up)              // 相机右方向
//   u = r × f                          // 重新正交化的相机上方向
//
//   M = [  r.x   r.y   r.z   -r·eye ]
//       [  u.x   u.y   u.z   -u·eye ]
//       [ -f.x  -f.y  -f.z    f·eye ]
//       [  0     0     0      1     ]
//
// 等价于：把世界坐标变换到 "相机为原点、+X 为右、+Y 为上、-Z 为前" 的视图坐标系。
//
// 接口设计：
//   形参用 Eigen::MatrixBase<Derived>，可接受任意 Eigen 表达式
//   (-v, a + b, Vector3f::Zero(), block(...) 等)，无需调用方先 .eval()。
//   标量类型从 eye 推导，center / up 通过 cast<T> 统一到该类型。
//
// 退化情况：
//   - center == eye  -> f 为零向量 -> NaN。
//   - up 与 (center - eye) 平行 -> r 为零向量 -> NaN。
//   调用方应保证两者不发生。

#include <Core/TypeTraits.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ZF
{
namespace Math
{

/// 右手坐标系 LookAt (相机看 -Z)，与 glm::lookAtRH 一致。
template <typename DEye, typename DCenter, typename DUp>
inline Eigen::Matrix<eigen_scalar_t<DEye>, 4, 4>
lookAtRH(const Eigen::MatrixBase<DEye>&    eye,
         const Eigen::MatrixBase<DCenter>& center,
         const Eigen::MatrixBase<DUp>&     up)
{
    using Scalar = eigen_scalar_t<DEye>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

    const Vector3 e = eye;
    const Vector3 c = center.template cast<Scalar>();
    const Vector3 uIn = up.template cast<Scalar>();

    const auto f = (c - e).normalized();
    const auto r = f.cross(uIn).normalized();
    const auto u = r.cross(f);

    Matrix4 m = Matrix4::Identity();

    // 旋转部分按行写入：第 0 行 = r，第 1 行 = u，第 2 行 = -f。
    m(0, 0) =  r.x();  m(0, 1) =  r.y();  m(0, 2) =  r.z();
    m(1, 0) =  u.x();  m(1, 1) =  u.y();  m(1, 2) =  u.z();
    m(2, 0) = -f.x();  m(2, 1) = -f.y();  m(2, 2) = -f.z();

    // 平移部分 = -R · eye。
    m(0, 3) = -r.dot(e);
    m(1, 3) = -u.dot(e);
    m(2, 3) =  f.dot(e);
    return m;
}

/// 左手坐标系 LookAt (相机看 +Z)，与 glm::lookAtLH 一致。
template <typename DEye, typename DCenter, typename DUp>
inline Eigen::Matrix<eigen_scalar_t<DEye>, 4, 4>
lookAtLH(const Eigen::MatrixBase<DEye>&    eye,
         const Eigen::MatrixBase<DCenter>& center,
         const Eigen::MatrixBase<DUp>&     up)
{
    using Scalar = eigen_scalar_t<DEye>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

    const Vector3 e = eye;
    const Vector3 c = center.template cast<Scalar>();
    const Vector3 uIn = up.template cast<Scalar>();

    const auto f = (c - e).normalized();
    const auto r = uIn.cross(f).normalized();
    const auto u = f.cross(r);

    Matrix4 m = Matrix4::Identity();
    m(0, 0) = r.x();  m(0, 1) = r.y();  m(0, 2) = r.z();
    m(1, 0) = u.x();  m(1, 1) = u.y();  m(1, 2) = u.z();
    m(2, 0) = f.x();  m(2, 1) = f.y();  m(2, 2) = f.z();

    m(0, 3) = -r.dot(e);
    m(1, 3) = -u.dot(e);
    m(2, 3) = -f.dot(e);
    return m;
}

/// 默认 LookAt = 右手版本，与 glm::lookAt 默认一致。
template <typename DEye, typename DCenter, typename DUp>
inline Eigen::Matrix<eigen_scalar_t<DEye>, 4, 4>
lookAt(const Eigen::MatrixBase<DEye>&    eye,
       const Eigen::MatrixBase<DCenter>& center,
       const Eigen::MatrixBase<DUp>&     up)
{
    return lookAtRH(eye, center, up);
}

} // namespace Math
} // namespace ZF
