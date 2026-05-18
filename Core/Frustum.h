#pragma once

// 视锥体 (Frustum) —— 整合 OrthoFrustum / PerspectiveFrustum / Frustum 三个旧类的功能。
//
// 一个类，承载三类职责：
//   1. 视锥剔除：6 个剔除平面 (left/right/bottom/top/near/far)，法线指向内部，
//      支持包围球相交测试，以及由 "参考视锥 + 矩阵" 重建剔除平面 (用于场景遍历)。
//
//   2. 投影矩阵：根据 Type 分别生成正交 / 透视投影矩阵 (OpenGL 风格，深度 [-1, 1])。
//
//   3. 二维矩形操作：对正交投影的视口矩形提供
//      width/height/center2D/size/topLeft/shift2D/scale2DWithRatio/localPosTo/ndcToView。
//
// 与旧代码差异：
//   - 数据类型由 glm 切换为 Eigen (Vector2d/Vector3d/Vector4d/Matrix4d)。
//   - 删除三个独立 struct，统一为 Frustum + Frustum::Type 枚举。
//   - 投影矩阵采用 OpenGL 右手坐标系 (相机看 -Z)，深度区间 [-1, 1]。

#include <Core/Macros.h>
#include <Core/Math/Projection.h>
#include <Core/Plane.h>
#include <Core/Sphere.h>

#include <Eigen/Core>
#include <Eigen/LU> // 用于 Matrix::inverse()

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ZF
{
namespace Math
{

class Frustum
{
public:
    using value_type = double;
    using plane_type = Planed;
    using vec2_type = Eigen::Vector2d;
    using vec3_type = Eigen::Vector3d;
    using vec4_type = Eigen::Vector4d;
    using matrix_type = Eigen::Matrix4d;
    /// 投影类型
    enum class Type
    {
        Orthographic,   ///< 正交：由 (l, r, b, t, n, f) 完整定义
        Perspective     ///< 透视：由 (hFov, vFov, n, f) 完整定义
    };

    /// 剔除平面在数组中的索引
    enum FaceIndex : std::size_t
    {
        FACE_LEFT   = 0,
        FACE_RIGHT  = 1,
        FACE_BOTTOM = 2,
        FACE_TOP    = 3,
        FACE_NEAR   = 4,
        FACE_FAR    = 5
    };

    static constexpr std::size_t kFaceCount = 6;

    // ============================================================
    //  构造 / 工厂
    // ============================================================

    /// 默认构造：得到 OpenGL NDC 单位视锥 ([-1, 1]³)。
    /// 该 "单位视锥" 用作场景遍历中剔除平面的模板，可被 setTransformed()
    /// 配合 (proj * modelview) 推出局部空间下的剔除视锥。
    Frustum()
    {
        setUnit();
    }

    // 由参考视锥 reference 经矩阵 srcToDst 变换到当前视锥。
    // 等价于默认构造后调用 setTransformed(reference, srcToDst)。
    template <typename M>
    Frustum(const Frustum& reference, const M& srcToDst)
    {
        setTransformed(reference, srcToDst);
    }

    /// 工厂：正交投影。l/r/b/t 为视空间矩形边界，n/f 为正向距离 (近/远)。
    static Frustum makeOrtho(value_type l, value_type r,
                             value_type b, value_type t,
                             value_type n, value_type f)
    {
        Frustum out;
        out.setOrtho(l, r, b, t, n, f);
        return out;
    }

    /// 工厂：透视投影。fov 单位为角度。
    static Frustum makePerspective(value_type horizontalFov, value_type verticalFov,
                                   value_type n, value_type f)
    {
        Frustum out;
        out.setPerspective(horizontalFov, verticalFov, n, f);
        return out;
    }

    // ============================================================
    //  访问器
    // ============================================================

    Type type() const { return mType; }
    bool isOrthographic() const { return mType == Type::Orthographic; }
    bool isPerspective()  const { return mType == Type::Perspective; }

    value_type left()   const { return mLeft; }
    value_type right()  const { return mRight; }
    value_type bottom() const { return mBottom; }
    value_type top()    const { return mTop; }
    value_type nearZ()  const { return mNear; }
    value_type farZ()   const { return mFar; }

    value_type horizontalFov() const { return mHorizontalFov; }
    value_type verticalFov()   const { return mVerticalFov; }
    value_type aspect()        const { return mAspect; }

    /// 按索引访问 6 个剔除平面 (索引使用 FaceIndex 枚举更直观)
    plane_type&       face(std::size_t i)       { return mFaces[i]; }
    const plane_type& face(std::size_t i) const { return mFaces[i]; }

    const vec4_type& lodScale() const { return mLodScale; }

    // ============================================================
    //  设置
    // ============================================================

    /// 重置为 OpenGL NDC 立方体 ([-1, 1]³) 对应的剔除平面 (法线朝内)，元数据回到默认。
    void setUnit()
    {
        mType   = Type::Orthographic;
        mLeft   = -1.0; mRight  = 1.0;
        mBottom = -1.0; mTop    = 1.0;
        mNear   = -1.0; mFar    = 1.0;
        mHorizontalFov = 0.0;
        mVerticalFov   = 0.0;
        mAspect        = 1.0;

        mFaces[FACE_LEFT  ].set( 1.0,  0.0,  0.0,  1.0);
        mFaces[FACE_RIGHT ].set(-1.0,  0.0,  0.0,  1.0);
        mFaces[FACE_BOTTOM].set( 0.0,  1.0,  0.0,  1.0);
        mFaces[FACE_TOP   ].set( 0.0, -1.0,  0.0,  1.0);
        mFaces[FACE_NEAR  ].set( 0.0,  0.0,  1.0,  1.0);
        mFaces[FACE_FAR   ].set( 0.0,  0.0, -1.0,  1.0);
    }

    /// 设置为正交视锥；同时重建视空间下的剔除平面。
    void setOrtho(value_type l, value_type r,
                  value_type b, value_type t,
                  value_type n, value_type f)
    {
        mType   = Type::Orthographic;
        mLeft   = l; mRight = r;
        mBottom = b; mTop   = t;
        mNear   = n; mFar   = f;
        mHorizontalFov = 0.0;
        mVerticalFov   = 0.0;
        refreshDerived();
    }

    /// 设置为透视视锥；同时重建视空间下的剔除平面。
    /// hFov/vFov 单位为角度；同步推出近平面边界 (left/right/bottom/top) 与 aspect。
    void setPerspective(value_type horizontalFov, value_type verticalFov,
                        value_type n, value_type f)
    {
        mType = Type::Perspective;
        mHorizontalFov = horizontalFov;
        mVerticalFov   = verticalFov;
        mNear = n;
        mFar  = f;
        refreshDerived();
    }

    // ------------------------------------------------------------
    //  单字段 setter：修改一个字段并自动刷新派生量与剔除平面。
    //
    //  正交模式下，l/r/b/t 是自变量；mAspect 由 (r-l)/(t-b) 派生。
    //  透视模式下，hFov/vFov/near 是自变量；l/r/b/t 与 aspect 都由它们派生，
    //  所以 setLeft / setRight / setBottom / setTop 在透视模式下不会动 fov，
    //  仅修改近平面边界 (一般你也用不到，建议改 fov)。
    // ------------------------------------------------------------

    /// 设置左边界。正交模式直接生效，透视模式只改近平面左边界。
    void setLeft  (value_type v) { mLeft   = v; refreshDerived(); }
    /// 设置右边界。语义同 setLeft。
    void setRight (value_type v) { mRight  = v; refreshDerived(); }
    /// 设置下边界。语义同 setLeft。
    void setBottom(value_type v) { mBottom = v; refreshDerived(); }
    /// 设置上边界。语义同 setLeft。
    void setTop   (value_type v) { mTop    = v; refreshDerived(); }

    /// 设置近平面距离 (正向值)。两种模式都生效。
    void setNear(value_type v) { mNear = v; refreshDerived(); }
    /// 设置远平面距离 (正向值)。两种模式都生效。
    void setFar (value_type v) { mFar  = v; refreshDerived(); }

    /// 设置水平 FoV (角度)。仅在透视模式下有意义。
    void setHorizontalFov(value_type degrees) { mHorizontalFov = degrees; refreshDerived(); }
    /// 设置垂直 FoV (角度)。仅在透视模式下有意义。
    void setVerticalFov  (value_type degrees) { mVerticalFov   = degrees; refreshDerived(); }

    // 把参考视锥的 6 个剔除平面经矩阵变换，写到当前视锥上。
    //
    // 矩阵 srcToDst：把Frustum 从空间 S 变换到 空间 D
    //
    // 示例：把 NDC 单位视锥变到世界空间，用于场景遍历时的包围球剔除
    //     Eigen::Matrix4d viewProj   = projection * view;       // world -> NDC
    //     Eigen::Matrix4d ndcToWorld = viewProj.inverse();      // NDC   -> world
    //     Frustum worldFrustum;
    //     worldFrustum.setTransformed(Frustum(), ndcToWorld);
    //     worldFrustum.intersect(sphere) 世界坐标相交判断包围球。
    template <typename M>
    void setTransformed(const Frustum& reference, const M& srcToDst)
    {
        const matrix_type dstToSrc = srcToDst.template cast<value_type>().inverse();
        for (std::size_t i = 0; i < kFaceCount; ++i)
        {
            //see Plane.h 中的 operator * (M, plane)
            mFaces[i] = dstToSrc * reference.mFaces[i]; 
        }

        // 拷贝参考视锥的其他字段 (Type, l/r/b/t/n/f, fov, aspect)，
        // 这些字段描述的是视锥本身的形状，不会因变换而变
        mType   = reference.mType;
        mLeft   = reference.mLeft;   mRight  = reference.mRight;
        mBottom = reference.mBottom; mTop    = reference.mTop;
        mNear   = reference.mNear;   mFar    = reference.mFar;
        mHorizontalFov = reference.mHorizontalFov;
        mVerticalFov   = reference.mVerticalFov;
        mAspect        = reference.mAspect;
    }

    // ============================================================
    //  二维矩形操作 (旧 OrthoFrustum)
    // ============================================================

    /// 矩形宽度 (right - left)
    ZF_FORCE_INLINE value_type width()  const { return mRight - mLeft; }
    /// 矩形高度 (top - bottom)
    ZF_FORCE_INLINE value_type height() const { return mTop - mBottom; }
    /// 矩形尺寸 (width, height)
    ZF_FORCE_INLINE vec2_type  size()   const { return vec2_type(width(), height()); }

    /// 矩形中心 (xy)
    ZF_FORCE_INLINE vec2_type center2D() const
    {
        return vec2_type(0.5 * (mLeft + mRight), 0.5 * (mBottom + mTop));
    }

    /// 视锥包围盒中心 (xyz)
    ZF_FORCE_INLINE vec3_type center() const
    {
        return vec3_type(0.5 * (mLeft + mRight),
                         0.5 * (mBottom + mTop),
                         0.5 * (mNear + mFar));
    }

    /// 屏幕局部坐标系下，矩形左上角的相对偏移 (与旧 OrthoFrustum::topLeft 一致)
    ZF_FORCE_INLINE vec2_type topLeft() const
    {
        return vec2_type(0.0, height());
    }

    /// 整体平移矩形 (Δx, Δy)；正交模式下同步刷新剔除平面。
    void shift2D(const vec2_type& step)
    {
        mLeft   += step.x();
        mRight  += step.x();
        mBottom += step.y();
        mTop    += step.y();
        if (mType == Type::Orthographic) recomputeOrthoPlanes();
    }

    /// 以中心为原点，按 hStep 为高度增量、保持宽高比缩放 (与旧 scale2DWithRatio 一致)。
    void scale2DWithRatio(value_type hStep)
    {
        const vec2_type c = center2D();
        value_type deltaH = 0.5 * height() + hStep;
        deltaH = std::max((value_type)0.0001, deltaH);

        mLeft   = c.x() - mAspect * deltaH;
        mRight  = c.x() + mAspect * deltaH;
        mBottom = c.y() - deltaH;
        mTop    = c.y() + deltaH;
        if (mType == Type::Orthographic) recomputeOrthoPlanes();
    }

    /// 把单位矩形内 ([0, 1]²) 的归一化点映射到以 origin 为左下角的物理坐标。
    ZF_FORCE_INLINE vec2_type localPosTo(const vec2_type& normalizedPoint,
                                         const vec2_type& origin) const
    {
        return vec2_type(normalizedPoint.x() * width()  + origin.x(),
                         normalizedPoint.y() * height() + origin.y());
    }

    // 把 NDC 坐标 (u, v, d ∈ [-1, 1]) 线性反推回视空间（仅适用于正交视锥）。
    //   x: u ∈ [-1, 1]  ->  [left,   right]
    //   y: v ∈ [-1, 1]  ->  [bottom, top]
    //   z: d ∈ [-1, 1]  ->  [-near,  -far]   (OpenGL 相机看 -Z)
    // 以 x 为例的二点插值：
    //   x = (u+1)/2 * right + (1-u)/2 * left
    ZF_FORCE_INLINE vec3_type ndcToView(value_type u, value_type v, value_type d) const
    {
        return vec3_type(
            mRight * (u + 1.0) / 2.0 + (1.0 - u) / 2.0 * mLeft,
            mTop   * (v + 1.0) / 2.0 + (1.0 - v) / 2.0 * mBottom,
            (d - 1.0) / 2.0 * mNear - mFar * (d + 1.0) / 2.0);
    }

    // ============================================================
    //  有效性
    // ============================================================

    /// 各参数有限、且 left<right、bottom<top、near<far (透视模式额外要求 fov>0)。
    bool valid() const
    {
        const value_type values[6] = {mLeft, mRight, mBottom, mTop, mNear, mFar};
        for (value_type v : values)
        {
            if (std::isnan(v) || std::isinf(v)) return false;
        }
        if (mLeft   >= mRight) return false;
        if (mBottom >= mTop)   return false;
        if (mNear   >= mFar)   return false;
        if (mType == Type::Perspective)
        {
            if (mHorizontalFov <= 0.0 || mVerticalFov <= 0.0) return false;
        }
        return true;
    }

    // ============================================================
    //  投影矩阵 (OpenGL 风格，深度区间 [-1, 1])
    // ============================================================

    /// 当前 type 对应的投影矩阵。
    matrix_type projection() const
    {
        if (mType == Type::Perspective) return makePerspectiveMatrix();
        return makeOrthoMatrix();
    }

    // 投影矩阵的逆。从裁剪空间 (未除以 w) 反推视空间。
    matrix_type inverseProjection() const
    {
        return projection().inverse();
    }

    // ============================================================
    //  剔除
    // ============================================================

    // 包围球与视锥相交测试 (保守)。
    //
    // 原理：6 个剔除平面的法线都指向内部。点到平面的有符号距离为
    //   dist = norm . center + d。只要球心到某个平面的距离 < -radius，
    //   说明整个包围球都在该平面外侧，立刻返回 false。
    //
    // 返回值：
    //   true  - 可能相交 (只能证明“包围球不在任何单一平面外侧”，不能证明一定相交)
    //   false - 一定不相交
    template <typename T>
    bool intersect(const TSphere<T>& s) const
    {
        const value_type negative_radius = -static_cast<value_type>(s.radius());
        const vec3_type c = s.center().template cast<value_type>();
        for (std::size_t i = 0; i < kFaceCount; ++i)
        {
            if (distance(mFaces[i], c) < negative_radius) return false;
        }
        return true;
    }

    // 计算 LOD 缩放向量 (与 vsg::Frustum::computeLodScale 一致)。
    //
    // 用途：后续在场景遍历中，用 dot(lodScale, nodeCenter) 估计某个节点中心到相机的
    // “屏幕上的视觉大小”，从而选择合适的 LOD 层。
    //
    // 公式：
    //   f      = -proj(1, 1)                                   // 与垂直 FoV 相关的缩放因子
    //   sc     = f * sqrt(∑ mv(0..1, 0..2)^2) / 2              // mv 前两行的均方根
    //   lodScale = mv 的第 3 行 / sc
template <typename M>
    void computeLodScale(const M& proj, const M& mv)
    {
        const value_type f = -proj(1, 1);
        const value_type sc = f * std::sqrt(
            mv(0, 0) * mv(0, 0) + mv(0, 1) * mv(0, 1) + mv(0, 2) * mv(0, 2)
            + mv(1, 0) * mv(1, 0) + mv(1, 1) * mv(1, 1) + mv(1, 2) * mv(1, 2)) * 0.5;
        // 退化 proj/mv 下 sc 可能为 0，加 safeDenom 避免产生 inf。
        const value_type inv_scale = 1.0 / safeDenom(sc);
        mLodScale = vec4_type(mv(2, 0) * inv_scale,
                              mv(2, 1) * inv_scale,
                              mv(2, 2) * inv_scale,
                              mv(2, 3) * inv_scale);
    }

private:
    static value_type radians(value_type degrees)
    {
        return degrees * (3.14159265358979323846 / 180.0);
    }

    // 除法安全护栏：分母绝对值小于 eps 时用同号的 eps 代替，
    // 避免出现 inf/-inf。适用于 "几何上本不应为 0 但数值上可能退化" 的场景。
    static value_type safeDenom(value_type x)
    {
        constexpr value_type eps = value_type(1e-12);
        if (std::abs(x) < eps) return (x < value_type(0)) ? -eps : eps;
        return x;
    }

    // tan(°/2)：在输入接近 180° 时 tan 会变 inf，这里把半角夹到 (-π/2 + ε, π/2 - ε)。
    static value_type clampedTanHalfDegrees(value_type degrees)
    {
        constexpr value_type maxHalf = value_type(1.5707963); // ≈ π/2 - 1e-7
        value_type half = value_type(0.5) * radians(degrees);
        if (half >  maxHalf) half =  maxHalf;
        if (half < -maxHalf) half = -maxHalf;
        return std::tan(half);
    }

    // 根据当前 mType 与自变量字段，刷新派生量 (aspect / 近平面边界) 与 6 个剔除平面。
    //   - 正交：自变量 = l/r/b/t/n/f；mAspect 由 (r-l)/(t-b) 派生。
    //   - 透视：自变量 = hFov/vFov/n/f；l/r/b/t 与 aspect 均由它们派生。
    void refreshDerived()
    {
        constexpr value_type eps = value_type(1e-12);
        if (mType == Type::Orthographic)
        {
            mAspect = (mRight - mLeft) / safeDenom(mTop - mBottom);
            recomputeOrthoPlanes();
        }
        else
        {
            // 透视下 vFov 应严格 > 0；这里只防几乎为 0 的退化输入，避免 /0 产生 inf。
            mAspect = (std::abs(mVerticalFov) > eps) ? (mHorizontalFov / mVerticalFov) : value_type(1.0);
            const value_type tanHalfH = clampedTanHalfDegrees(mHorizontalFov);
            const value_type tanHalfV = clampedTanHalfDegrees(mVerticalFov);
            mRight =  mNear * tanHalfH; mLeft   = -mRight;
            mTop   =  mNear * tanHalfV; mBottom = -mTop;
            recomputePerspectivePlanes(tanHalfH, tanHalfV);
        }
    }

    // OpenGL 正交投影矩阵 (右手系，相机看 -Z，深度区间 [-1, 1])。
    //
    // 要求的线性映射：
    //   x ∈ [l, r]   ->  x' ∈ [-1, 1]
    //   y ∈ [b, t]   ->  y' ∈ [-1, 1]
    //   z ∈ [-f, -n] ->  z' ∈ [-1, 1]   (近平面 z=-n -> -1, 远平面 z=-f -> +1)
    //
    // 逆推：设 x' = a*x + b, 代入两个端点解出 a = 2/(r-l), b = -(r+l)/(r-l)。
    // y、z 同理。w 通道不变。
    matrix_type makeOrthoMatrix() const
    {
        return Math::ortho(mLeft, mRight, mBottom, mTop, mNear, mFar);
    }

    // OpenGL 透视投影矩阵 (右手系，相机看 -Z，深度区间 [-1, 1])。
    //
    // 记 t = tan(vFov / 2)。
    //
    //   x 通道：视空间 (x, *, z) 的左右边界为 x = ± t * aspect * (-z)。
    //            令 x_c = x / (t * aspect), w_c = -z, NDC x = x_c / w_c ∈ [-1, 1]。
    //   y 通道：同理, y_c = y / t。
    //   z 通道：要求 z=-n -> -1，z=-f -> +1 (除以 w_c = -z 后)。
    //            设 z_c = A*z + B*w, 解得 A = -(f+n)/(f-n), B = -2fn/(f-n)。
    //   w 通道：w_c = -z, 即 m(3, 2) = -1。
    matrix_type makePerspectiveMatrix() const
    {
        return Math::perspective(mLeft, mRight, mBottom, mTop, mNear, mFar);
    }

    // 重建正交视锥在视空间下的 6 个剔除平面 (法线指向内部)。
    //
    // OpenGL 右手系，相机看 -Z，近平面位于 z = -n，远平面位于 z = -f。
    // 平面以 Hessian 法线形式 norm . x + d = 0 表示，内部侧为 norm . x + d >= 0。
    //
    // 例：
    //   LEFT  要求 x >=  l, 改写成  x - l >= 0, 即 norm = (1, 0, 0),  d = -l
    //   NEAR  要求 z <= -n, 改写成 -z - n >= 0, 即 norm = (0, 0, -1), d = -n
    void recomputeOrthoPlanes()
    {
        mFaces[FACE_LEFT  ].set( 1.0,  0.0,  0.0, -mLeft);    // x ≥ l
        mFaces[FACE_RIGHT ].set(-1.0,  0.0,  0.0,  mRight);   // x ≤ r
        mFaces[FACE_BOTTOM].set( 0.0,  1.0,  0.0, -mBottom);  // y ≥ b
        mFaces[FACE_TOP   ].set( 0.0, -1.0,  0.0,  mTop);     // y ≤ t
        mFaces[FACE_NEAR  ].set( 0.0,  0.0, -1.0, -mNear);    // z ≤ -n
        mFaces[FACE_FAR   ].set( 0.0,  0.0,  1.0,  mFar);     // z ≥ -f
    }

    // 重建透视视锥在视空间下的 6 个剔除平面 (法线指向内部)。
    //
    // OpenGL 右手系，相机看 -Z。记 th = tan(hFov/2)。4 个侧面都过原点 (d = 0)。
    //
    // 以 RIGHT 面为例：
    //   右边界射线上点满足 x = th * (-z)，即 x + th * z = 0。
    //   代入内部点 (0, 0, -n)：-th * n < 0，不满足 "内部侧 >= 0" 的约定，
    //   所以取反法线：norm = (-1, 0, -th)。
    //   再除以 sqrt(1 + th^2) 归一化。
    //
    // LEFT 同理取 norm = (1, 0, -th)；BOTTOM/TOP 同理以 tv = tan(vFov/2) 代 th。
    // NEAR/FAR 与正交一致：z = -n 与 z = -f。
    void recomputePerspectivePlanes(value_type tanHalfH, value_type tanHalfV)
    {
        // 侧面法线归一化因子
        const value_type invHx = 1.0 / std::sqrt(1.0 + tanHalfH * tanHalfH);
        const value_type invVy = 1.0 / std::sqrt(1.0 + tanHalfV * tanHalfV);
        // 4 个侧面：过原点，d = 0
        mFaces[FACE_LEFT  ].set( invHx, 0.0,   -tanHalfH * invHx, 0.0);
        mFaces[FACE_RIGHT ].set(-invHx, 0.0,   -tanHalfH * invHx, 0.0);
        mFaces[FACE_BOTTOM].set( 0.0,    invVy, -tanHalfV * invVy, 0.0);
        mFaces[FACE_TOP   ].set( 0.0,   -invVy, -tanHalfV * invVy, 0.0);
        // 近/远平面：与正交一致
        mFaces[FACE_NEAR  ].set( 0.0,    0.0,  -1.0, -mNear);
        mFaces[FACE_FAR   ].set( 0.0,    0.0,   1.0,  mFar);
    }

private:
    Type mType = Type::Orthographic;

    // 视锥包围盒 (正交) / 近平面边界 (透视)
    value_type mLeft   = -1.0;
    value_type mRight  =  1.0;
    value_type mBottom = -1.0;
    value_type mTop    =  1.0;
    value_type mNear   =  0.0;
    value_type mFar    =  1.0;

    // 透视专属 (单位：角度)
    value_type mHorizontalFov = 0.0;
    value_type mVerticalFov   = 0.0;

    // 派生：宽高比
    value_type mAspect = 1.0;

    // 剔除平面 (顺序见 FaceIndex)
    plane_type mFaces[kFaceCount];

    // LOD 缩放向量
    vec4_type mLodScale = vec4_type::Zero();
};

} // namespace Math
} // namespace ZF
