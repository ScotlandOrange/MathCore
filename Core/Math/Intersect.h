#pragma once

/// @file
/// @brief 点-三角形包含性、裁剪空间可见性等相交判断。

#include <Eigen/Core>

namespace ZF
{
namespace Math
{

// 点是否落在三角形内部。
// - 3D 重载假设 P 与 ABC 共面，按重心坐标判定（共线三角形返回 false）。
// - 2D 重载用三条边的有向面积，同号即在内部（不区分缠绕方向）。
// 示例：
//   bool in3 = pointInTriangle  (A, B, C, P);
//   bool in2 = pointInTriangle2D(Vector2f(pX, pY), Vector2f(aX, aY), Vector2f(bX, bY), Vector2f(cX, cY));
bool pointInTriangle  (const Eigen::Vector3f& A,
                       const Eigen::Vector3f& B,
                       const Eigen::Vector3f& C,
                       const Eigen::Vector3f& P);

bool pointInTriangle2D(const Eigen::Vector2f& P,
                       const Eigen::Vector2f& A,
                       const Eigen::Vector2f& B,
                       const Eigen::Vector2f& C);

// 裁剪空间（齐次坐标，divide-by-w 之前）的可见性测试：判断顶点是否在 [-w, w]^3 内，
// 三角形是否至少有一个顶点在内（粗筛，不替代严格的视锥剔除）。
// 示例：
//   bool show = insideClipSpace         (vClip);
//   bool any  = triangleTouchesClipSpace(v1Clip, v2Clip, v3Clip);
bool insideClipSpace         (const Eigen::Vector4f& clipVertex);
bool triangleTouchesClipSpace(const Eigen::Vector4f& v1Clip,
                              const Eigen::Vector4f& v2Clip,
                              const Eigen::Vector4f& v3Clip);

} // namespace Math
} // namespace ZF
