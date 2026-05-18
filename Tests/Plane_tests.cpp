// TPlane 的 GoogleTest 测试。

#include <gtest/gtest.h>

#include <Core/Plane.h>

#include <array>
#include <vector>

using ZF::Math::Plane;
using ZF::Math::Planed;
using ZF::Math::TPlane;
using ZF::Math::TSphere;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector4d;

namespace
{

template <typename T>
::testing::AssertionResult NearlyEqual(T lhs, T rhs, T tol = T(1e-6))
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

TEST(PlaneUsage, BasicDistanceFromXyPlane)
{
    Planed pl(0.0, 0.0, 1.0, 0.0);

    Vector3d above(1.0, 2.0, 3.0);
    Vector3d below(1.0, 2.0, -4.5);

    EXPECT_TRUE(NearlyEqual(ZF::Math::distance(pl, above), 3.0));
    EXPECT_TRUE(NearlyEqual(ZF::Math::distance(pl, below), -4.5));
}

TEST(PlaneUsage, BuildFrustumLikePolytopeAndCullSphere)
{
    std::array<Planed, 6> box = {
        Planed(Vector3d(-1.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0)),
        Planed(Vector3d(1.0, 0.0, 0.0), Vector3d(-1.0, 0.0, 0.0)),
        Planed(Vector3d(0.0, -1.0, 0.0), Vector3d(0.0, 1.0, 0.0)),
        Planed(Vector3d(0.0, 1.0, 0.0), Vector3d(0.0, -1.0, 0.0)),
        Planed(Vector3d(0.0, 0.0, -1.0), Vector3d(0.0, 0.0, 1.0)),
        Planed(Vector3d(0.0, 0.0, 1.0), Vector3d(0.0, 0.0, -1.0)),
    };

    EXPECT_TRUE(ZF::Math::intersect(box, TSphere<double>(Vector3d(0.0, 0.0, 0.0), 0.5)));
    EXPECT_FALSE(ZF::Math::intersect(box, TSphere<double>(Vector3d(2.0, 0.0, 0.0), 0.5)));
    EXPECT_TRUE(ZF::Math::intersect(box, TSphere<double>(Vector3d(2.0, 0.0, 0.0), 1.5)));
}

TEST(PlaneBehavior, DefaultConstructedIsZeroAndInvalid)
{
    Planed pl;

    EXPECT_EQ(pl.size(), 4u);
    EXPECT_EQ(pl[0], 0.0);
    EXPECT_EQ(pl[1], 0.0);
    EXPECT_EQ(pl[2], 0.0);
    EXPECT_EQ(pl[3], 0.0);

    EXPECT_FALSE(pl.valid());
    EXPECT_FALSE(static_cast<bool>(pl));
}

TEST(PlaneBehavior, ConstructFromComponentsExposesAccessors)
{
    Planed pl(1.0, 2.0, 3.0, -4.0);

    EXPECT_EQ(pl.norm(), Vector3d(1.0, 2.0, 3.0));
    EXPECT_EQ(pl.d(), -4.0);
    EXPECT_EQ(pl.vec(), Vector4d(1.0, 2.0, 3.0, -4.0));
    EXPECT_TRUE(pl.valid());
}

TEST(PlaneBehavior, ConstructFromNormalAndDMatchesComponents)
{
    Planed pl(Vector3d(0.0, 1.0, 0.0), 7.0);

    EXPECT_EQ(pl.norm(), Vector3d(0.0, 1.0, 0.0));
    EXPECT_EQ(pl.d(), 7.0);
}

TEST(PlaneBehavior, ConstructFromPositionAndNormalSetsHessianForm)
{
    Vector3d position(2.0, 3.0, 5.0);
    Vector3d normal(0.0, 0.0, 1.0);

    Planed pl(position, normal);

    EXPECT_EQ(pl.norm(), normal);
    EXPECT_TRUE(NearlyEqual(pl.d(), -5.0));
    EXPECT_TRUE(NearlyEqual(ZF::Math::distance(pl, Vector3d(10.0, -7.0, 5.0)), 0.0));
}

TEST(PlaneBehavior, SetOverwritesAllCoefficients)
{
    Planed pl(1.0, 2.0, 3.0, 4.0);

    pl.set(-1.0, -2.0, -3.0, -4.0);

    EXPECT_EQ(pl.vec(), Vector4d(-1.0, -2.0, -3.0, -4.0));
}

TEST(PlaneBehavior, NormAccessorIsWritableAndAliasesVec)
{
    Planed pl(1.0, 0.0, 0.0, 0.0);

    pl.norm() = Vector3d(0.0, 0.0, 1.0);

    EXPECT_EQ(pl[0], 0.0);
    EXPECT_EQ(pl[1], 0.0);
    EXPECT_EQ(pl[2], 1.0);
    EXPECT_EQ(pl.vec().head<3>(), Vector3d(0.0, 0.0, 1.0));
}

TEST(PlaneBehavior, DataReturnsContiguousFourValues)
{
    Planed pl(1.0, 2.0, 3.0, 4.0);

    const double* d = pl.data();
    EXPECT_EQ(d[0], 1.0);
    EXPECT_EQ(d[1], 2.0);
    EXPECT_EQ(d[2], 3.0);
    EXPECT_EQ(d[3], 4.0);
}

TEST(PlaneBehavior, CrossScalarConstructorAndAssignment)
{
    Plane f(1.0f, 2.0f, 3.0f, 4.0f);
    Planed d(f);

    EXPECT_EQ(d.vec(), Vector4d(1.0, 2.0, 3.0, 4.0));

    Planed d2;
    d2 = f;
    EXPECT_EQ(d2.vec(), d.vec());
}

TEST(PlaneBehavior, EqualityComparesAllFourComponents)
{
    Planed a(1.0, 2.0, 3.0, 4.0);
    Planed b(1.0, 2.0, 3.0, 4.0);
    Planed c(1.0, 2.0, 3.0, 5.0);

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);
    EXPECT_TRUE(a < c);
    EXPECT_FALSE(c < a);
}

TEST(PlaneBehavior, FreeDistanceFunctionWorksAcrossScalarTypes)
{
    Plane pl(Vector3f(0.0f, 0.0f, 1.0f), -2.0f);
    Vector3d point(0.0, 0.0, 5.0);

    EXPECT_TRUE(NearlyEqual(ZF::Math::distance(pl, point), 3.0f, 1e-5f));
}

TEST(PlaneBehavior, InsideRespectsAllPlanesAndEpsilon)
{
    std::vector<Planed> half = {
        Planed(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 1.0)),
    };

    EXPECT_TRUE(ZF::Math::inside(half, Vector3d(0.0, 0.0, 1.0)));
    EXPECT_FALSE(ZF::Math::inside(half, Vector3d(0.0, 0.0, -1.0)));
    EXPECT_TRUE(ZF::Math::inside(half, Vector3d(0.0, 0.0, 0.0)));
}

TEST(PlaneBehavior, IntersectIteratorOverloadMatchesContainerOverload)
{
    std::vector<Planed> half = {
        Planed(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 1.0)),
    };

    TSphere<double> s(Vector3d(0.0, 0.0, -2.0), 1.0);

    bool by_iter = ZF::Math::intersect(half.begin(), half.end(), s);
    bool by_range = ZF::Math::intersect(half, s);

    EXPECT_EQ(by_iter, by_range);
    EXPECT_FALSE(by_iter);
}
