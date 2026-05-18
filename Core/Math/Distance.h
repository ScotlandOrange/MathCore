#pragma once

/// @file
/// @brief 点到直线、线段、三角形的最近点与距离。

#include <Eigen/Core>

#include <utility>

namespace ZF
{
namespace Math
{

// 点到无限直线的最近点 / 距离（直线由两点定义）。
// 示例：
//   Vector3f c = closestPointOnLine (linePoint1, linePoint2, point);
//   float    d = distancePointToLine(point, linePoint1, linePoint2);
Eigen::Vector3f closestPointOnLine (const Eigen::Vector3f& linePoint1,
                                    const Eigen::Vector3f& linePoint2,
                                    const Eigen::Vector3f& point);

float           distancePointToLine(const Eigen::Vector3f& point,
                                    const Eigen::Vector3f& linePoint1,
                                    const Eigen::Vector3f& linePoint2);

// 点到线段的最近点 / 距离（参数 t 在端点处钳到 [0, 1]，退化线段返回端点）。
// 同名WithNearest 重载返回 {距离, 最近点}。
// 示例：
//   Vector3f q  = closestPointOnSegment             (segStart, segEnd, point);
//   float    d3 = distancePointToSegment            (point, segStart, segEnd);
//   float    d2 = distancePointToSegment2D          (point, segStart2D, segEnd2D);
//   auto     pr = distancePointToSegmentWithNearest  (point, segStart, segEnd);     // pr.first=距离, pr.second=最近点
//   auto     p2 = distancePointToSegment2DWithNearest(point, segStart2D, segEnd2D);
Eigen::Vector3f closestPointOnSegment   (const Eigen::Vector3f& segStart,
                                         const Eigen::Vector3f& segEnd,
                                         const Eigen::Vector3f& point);

float           distancePointToSegment  (const Eigen::Vector3f& point,
                                         const Eigen::Vector3f& segStart,
                                         const Eigen::Vector3f& segEnd);

std::pair<float, Eigen::Vector3f>
                distancePointToSegmentWithNearest  (const Eigen::Vector3f& point,
                                                    const Eigen::Vector3f& segStart,
                                                    const Eigen::Vector3f& segEnd);

float           distancePointToSegment2D(const Eigen::Vector2f& point,
                                         const Eigen::Vector2f& segStart,
                                         const Eigen::Vector2f& segEnd);

std::pair<float, Eigen::Vector2f>
                distancePointToSegment2DWithNearest(const Eigen::Vector2f& point,
                                                    const Eigen::Vector2f& segStart,
                                                    const Eigen::Vector2f& segEnd);

// 点到三角形的最短距离及最近点。退化三角形（三点共线）退回到三条线段最小值。
// WithNearest 重载返回 {距离, 最近点}。
// 示例：
//   float d  = distancePointToTriangle            (point, v0, v1, v2);
//   auto  pr = distancePointToTriangleWithNearest (point, v0, v1, v2);  // pr.first=距离, pr.second=最近点
float distancePointToTriangle(const Eigen::Vector3f& point,
                              const Eigen::Vector3f& v0,
                              const Eigen::Vector3f& v1,
                              const Eigen::Vector3f& v2);

std::pair<float, Eigen::Vector3f>
distancePointToTriangleWithNearest(const Eigen::Vector3f& point,
                                   const Eigen::Vector3f& v0,
                                   const Eigen::Vector3f& v1,
                                   const Eigen::Vector3f& v2);

} // namespace Math
} // namespace ZF
