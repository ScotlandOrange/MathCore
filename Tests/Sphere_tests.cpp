// TSphere 的 GoogleTest 测试。

#include <gtest/gtest.h>

#include <Core/Sphere.h>

using ZF::Math::Sphere;
using ZF::Math::Sphered;
using ZF::Math::TSphere;
using Eigen::Vector3d;
using Eigen::Vector4d;

TEST(SphereBehavior, DefaultConstructedIsInvalidWithRadiusMinusOne)
{
    Sphered s;

    EXPECT_EQ(s.size(), 4u);
    EXPECT_EQ(s[0], 0.0);
    EXPECT_EQ(s[1], 0.0);
    EXPECT_EQ(s[2], 0.0);
    EXPECT_EQ(s.radius(), -1.0);

    EXPECT_FALSE(s.valid());
    EXPECT_FALSE(static_cast<bool>(s));
}

TEST(SphereBehavior, ConstructFromComponentsExposesAccessors)
{
    Sphered s(1.0, 2.0, 3.0, 4.0);

    EXPECT_EQ(s.center(), Vector3d(1.0, 2.0, 3.0));
    EXPECT_EQ(s.radius(), 4.0);
    EXPECT_EQ(s.vec(), Vector4d(1.0, 2.0, 3.0, 4.0));
    EXPECT_TRUE(s.valid());
}

TEST(SphereBehavior, ConstructFromCenterAndRadius)
{
    Vector3d c(1.0, 2.0, 3.0);
    Sphered s(c, 5.0);

    EXPECT_EQ(s.center(), c);
    EXPECT_EQ(s.radius(), 5.0);
}

TEST(SphereBehavior, SetOverwritesAllCoefficients)
{
    Sphered s(1.0, 2.0, 3.0, 4.0);

    s.set(-1.0, -2.0, -3.0, -4.0);
    EXPECT_EQ(s.vec(), Vector4d(-1.0, -2.0, -3.0, -4.0));

    s.set(Vector3d(7.0, 8.0, 9.0), 10.0);
    EXPECT_EQ(s.vec(), Vector4d(7.0, 8.0, 9.0, 10.0));
}

TEST(SphereBehavior, ValidRequiresNonNegativeRadius)
{
    Sphered s(0.0, 0.0, 0.0, 0.0);
    EXPECT_TRUE(s.valid());

    s.radius() = -0.001;
    EXPECT_FALSE(s.valid());
}

TEST(SphereBehavior, CenterAccessorIsWritableAndAliasesVec)
{
    Sphered s(1.0, 0.0, 0.0, 2.0);

    s.center() = Vector3d(0.0, 0.0, 1.0);

    EXPECT_EQ(s[0], 0.0);
    EXPECT_EQ(s[1], 0.0);
    EXPECT_EQ(s[2], 1.0);
    EXPECT_EQ(s.radius(), 2.0);
}

TEST(SphereBehavior, DataReturnsContiguousFourValues)
{
    Sphered s(1.0, 2.0, 3.0, 4.0);

    const double* d = s.data();
    EXPECT_EQ(d[0], 1.0);
    EXPECT_EQ(d[1], 2.0);
    EXPECT_EQ(d[2], 3.0);
    EXPECT_EQ(d[3], 4.0);
}

TEST(SphereBehavior, ResetMakesSphereInvalid)
{
    Sphered s(1.0, 2.0, 3.0, 4.0);
    s.reset();

    EXPECT_FALSE(s.valid());
    EXPECT_EQ(s.radius(), -1.0);
    EXPECT_EQ(s.center(), Vector3d::Zero());
}

TEST(SphereBehavior, CrossScalarConstructorAndAssignment)
{
    Sphere f(1.0f, 2.0f, 3.0f, 4.0f);
    Sphered d(f);

    EXPECT_EQ(d.vec(), Vector4d(1.0, 2.0, 3.0, 4.0));

    Sphered d2;
    d2 = f;
    EXPECT_EQ(d2.vec(), d.vec());
}

TEST(SphereBehavior, EqualityComparesAllFourComponents)
{
    Sphered a(1.0, 2.0, 3.0, 4.0);
    Sphered b(1.0, 2.0, 3.0, 4.0);
    Sphered c(1.0, 2.0, 3.0, 5.0);

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);
    EXPECT_TRUE(a < c);
    EXPECT_FALSE(c < a);
}
