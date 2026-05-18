// TAABB 的 GoogleTest 测试。

#include <gtest/gtest.h>

#include <Core/AABB.h>

#include <array>
#include <cmath>

using ZF::Math::AABB;
using ZF::Math::AABBd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace
{

template <typename DerivedA, typename DerivedB>
::testing::AssertionResult VecNearlyEqual(const Eigen::MatrixBase<DerivedA>& lhs,
                                          const Eigen::MatrixBase<DerivedB>& rhs,
                                          double tol = 1e-9)
{
    const double diff = (lhs.template cast<double>() - rhs.template cast<double>()).norm();
    if (diff <= tol)
    {
        return ::testing::AssertionSuccess();
    }

    return ::testing::AssertionFailure() << "vector diff = " << diff;
}

template <typename T>
::testing::AssertionResult NearlyEqual(T lhs, T rhs, T tol = T(1e-9))
{
    if (std::abs(lhs - rhs) <= tol)
    {
        return ::testing::AssertionSuccess();
    }

    return ::testing::AssertionFailure()
           << "expected " << lhs << " ~= " << rhs
           << " (diff = " << std::abs(lhs - rhs) << ")";
}

} // namespace

TEST(AABBBehavior, DefaultConstructedIsInvalidAndNullMatches)
{
    AABBd box;
    AABBd nullBox = AABBd::null();

    EXPECT_FALSE(box.valid());
    EXPECT_FALSE(static_cast<bool>(box));
    EXPECT_TRUE(std::isinf(box.getMin().x()));
    EXPECT_TRUE(std::isinf(box.getMax().x()));
    EXPECT_TRUE(box == nullBox);
}

TEST(AABBBehavior, ConstructFromBoundsNormalizesOrder)
{
    const AABBd box = AABBd::makeFromMinMax(Vector3d(3.0, 2.0, 1.0), Vector3d(-1.0, 5.0, -2.0));

    EXPECT_TRUE(box.valid());
    EXPECT_TRUE(VecNearlyEqual(box.getMin(), Vector3d(-1.0, 2.0, -2.0)));
    EXPECT_TRUE(VecNearlyEqual(box.getMax(), Vector3d(3.0, 5.0, 1.0)));
    EXPECT_TRUE(VecNearlyEqual(box.extents(), Vector3d(4.0, 3.0, 3.0)));
    EXPECT_TRUE(VecNearlyEqual(box.halfExtents(), Vector3d(2.0, 1.5, 1.5)));
    EXPECT_TRUE(VecNearlyEqual(box.halfExtends(), Vector3d(2.0, 1.5, 1.5)));
    EXPECT_TRUE(VecNearlyEqual(box.center(), Vector3d(1.0, 3.5, -0.5)));
    EXPECT_TRUE(NearlyEqual(box.diagDistance(), std::sqrt(34.0)));
}

TEST(AABBBehavior, ConstructFromVertexVectorBuildsBounds)
{
    const std::vector<Vector3d> vertices = {
        Vector3d(1.0, 2.0, 3.0),
        Vector3d(-4.0, 5.0, -6.0),
        Vector3d(2.0, -3.0, 7.0),
    };

    const AABBd byFactory = AABBd::makeFromStructVertices(vertices);

    EXPECT_TRUE(VecNearlyEqual(byFactory.getMin(), Vector3d(-4.0, -3.0, -6.0)));
    EXPECT_TRUE(VecNearlyEqual(byFactory.getMax(), Vector3d(2.0, 5.0, 7.0)));
}

TEST(AABBBehavior, ConstructFromVertexArrayAndPointerBuildsSameBounds)
{
    const Vector3d vertices[] = {
        Vector3d(1.0, 2.0, 3.0),
        Vector3d(-4.0, 5.0, -6.0),
        Vector3d(2.0, -3.0, 7.0),
    };

    const AABBd byArrayFactory = AABBd::makeFromStructVertices(vertices);
    const AABBd byPointerFactory = AABBd::makeFromStructVertices(vertices, 3);

    EXPECT_TRUE(VecNearlyEqual(byArrayFactory.getMin(), Vector3d(-4.0, -3.0, -6.0)));
    EXPECT_TRUE(VecNearlyEqual(byArrayFactory.getMax(), Vector3d(2.0, 5.0, 7.0)));
    EXPECT_TRUE(byArrayFactory == byPointerFactory);

    AABBd merged;
    EXPECT_TRUE(merged.merge(vertices));
    EXPECT_TRUE(merged == byArrayFactory);
}

TEST(AABBBehavior, ConstructFromCustomVertexElementsUsesConverter)
{
    struct VertexRecord
    {
        double x;
        double y;
        double z;
    };

    const VertexRecord vertices[] = {
        {1.0, 2.0, 3.0},
        {-4.0, 5.0, -6.0},
        {2.0, -3.0, 7.0},
    };
    const std::vector<VertexRecord> vertexVector(std::begin(vertices), std::end(vertices));
    const auto toVec = [](const VertexRecord& vertex) {
        return Vector3d(vertex.x, vertex.y, vertex.z);
    };

    const AABBd byArrayFactory = AABBd::makeFromStructVertices(vertices, toVec);
    const AABBd byPointerFactory = AABBd::makeFromStructVertices(vertices, std::size(vertices), toVec);
    const AABBd byVectorFactory = AABBd::makeFromStructVertices(vertexVector, toVec);

    EXPECT_TRUE(VecNearlyEqual(byArrayFactory.getMin(), Vector3d(-4.0, -3.0, -6.0)));
    EXPECT_TRUE(VecNearlyEqual(byArrayFactory.getMax(), Vector3d(2.0, 5.0, 7.0)));
    EXPECT_TRUE(byArrayFactory == byPointerFactory);
    EXPECT_TRUE(byArrayFactory == byVectorFactory);

    AABBd merged;
    EXPECT_TRUE(merged.merge(vertices, toVec));
    EXPECT_TRUE(merged == byArrayFactory);
}

TEST(AABBBehavior, ConstructFromFlatDoubleVerticesSupportsCrossPrecision)
{
    const std::vector<double> flatVertices = {
        1.0, 2.0, 3.0,
        -4.0, 5.0, -6.0,
        2.0, -3.0, 7.0,
    };

    const double flatArray[] = {
        1.0, 2.0, 3.0,
        -4.0, 5.0, -6.0,
        2.0, -3.0, 7.0,
    };

    const AABB byVectorFactory = AABB::makeFromFlatVertices(flatVertices);
    const AABB byPointerFactory = AABB::makeFromFlatVertices(flatArray, std::size(flatArray));
    const AABB byArrayFactory = AABB::makeFromFlatVertices(flatArray);

    EXPECT_TRUE(VecNearlyEqual(byVectorFactory.getMin(), Eigen::Vector3f(-4.0f, -3.0f, -6.0f), 1e-6));
    EXPECT_TRUE(VecNearlyEqual(byVectorFactory.getMax(), Eigen::Vector3f(2.0f, 5.0f, 7.0f), 1e-6));
    EXPECT_TRUE(byVectorFactory == byPointerFactory);
    EXPECT_TRUE(byVectorFactory == byArrayFactory);

    AABB merged;
    EXPECT_TRUE(merged.merge(flatVertices));
    EXPECT_TRUE(merged == byVectorFactory);
}

TEST(AABBBehavior, MergePointAndOtherBoxExpandBounds)
{
    AABBd box;
    box.merge(Vector3d(1.0, 2.0, 3.0));
    box.merge(Vector3d(-4.0, 5.0, -6.0));

    EXPECT_TRUE(VecNearlyEqual(box.getMin(), Vector3d(-4.0, 2.0, -6.0)));
    EXPECT_TRUE(VecNearlyEqual(box.getMax(), Vector3d(1.0, 5.0, 3.0)));

    const AABBd other = AABBd::makeFromMinMax(Vector3d(-2.0, -3.0, -1.0), Vector3d(10.0, 0.0, 4.0));
    EXPECT_TRUE(box.merge(other));
    EXPECT_TRUE(VecNearlyEqual(box.getMin(), Vector3d(-4.0, -3.0, -6.0)));
    EXPECT_TRUE(VecNearlyEqual(box.getMax(), Vector3d(10.0, 5.0, 4.0)));
}

TEST(AABBBehavior, ContainsPointAndNestedBoxUseInclusiveBounds)
{
    const AABBd outer = AABBd::makeFromMinMax(Vector3d(-2.0, -1.0, -3.0), Vector3d(5.0, 6.0, 7.0));
    const AABBd inner = AABBd::makeFromMinMax(Vector3d(-2.0, 0.0, -1.0), Vector3d(1.0, 6.0, 3.0));
    const AABBd overlapping = AABBd::makeFromMinMax(Vector3d(4.0, 0.0, 0.0), Vector3d(6.0, 2.0, 2.0));
    const AABBd invalid;

    EXPECT_TRUE(outer.contains(Vector3d(-2.0, -1.0, -3.0)));
    EXPECT_TRUE(outer.contains(Vector3d(0.0, 2.0, 1.0)));
    EXPECT_TRUE(outer.contains(Vector3d(5.0, 6.0, 7.0)));
    EXPECT_FALSE(outer.contains(Vector3d(5.01, 6.0, 7.0)));

    EXPECT_TRUE(outer.contains(inner));
    EXPECT_FALSE(outer.contains(overlapping));
    EXPECT_FALSE(outer.contains(invalid));
    EXPECT_FALSE(invalid.contains(inner));
}

TEST(AABBBehavior, TranslateScaleAndRotatePreserveAabbSemantics)
{
    const AABBd box = AABBd::makeFromMinMax(Vector3d(-2.0, -1.0, 0.0), Vector3d(2.0, 1.0, 0.0));

    const AABBd translated = box.translate(Vector3d(1.0, 2.0, 3.0));
    EXPECT_TRUE(VecNearlyEqual(translated.getMin(), Vector3d(-1.0, 1.0, 3.0)));
    EXPECT_TRUE(VecNearlyEqual(translated.getMax(), Vector3d(3.0, 3.0, 3.0)));

    const AABBd scaled = box.scale(Vector3d(2.0, 1.0, 1.0));
    EXPECT_TRUE(VecNearlyEqual(scaled.getMin(), Vector3d(-4.0, -1.0, 0.0)));
    EXPECT_TRUE(VecNearlyEqual(scaled.getMax(), Vector3d(4.0, 1.0, 0.0)));

    const Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2.0, Vector3d::UnitZ()));
    const AABBd rotated = box.rotate(rotation);
    EXPECT_TRUE(VecNearlyEqual(rotated.getMin(), Vector3d(-1.0, -2.0, 0.0), 1e-8));
    EXPECT_TRUE(VecNearlyEqual(rotated.getMax(), Vector3d(1.0, 2.0, 0.0), 1e-8));
}

TEST(AABBBehavior, TransformAndViewProjectRecomputeBoundsFromCorners)
{
    const AABBd box = AABBd::makeFromMinMax(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));

    const AABBd translated = box.transform(ZF::Math::makeTranslateMatrix4x4(Vector3d(2.0, 0.0, 0.0)));
    EXPECT_TRUE(VecNearlyEqual(translated.getMin(), Vector3d(1.0, -1.0, -1.0)));
    EXPECT_TRUE(VecNearlyEqual(translated.getMax(), Vector3d(3.0, 1.0, 1.0)));

    const AABBd sameBox = box.convertToSpace(Eigen::Matrix4d::Identity());
    EXPECT_TRUE(sameBox == box);

    const AABBd projected = box.viewProject(Eigen::Matrix4d::Identity());
    EXPECT_TRUE(projected == box);
}