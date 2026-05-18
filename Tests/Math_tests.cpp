// ZF::Math 与 Eigen 原生类型的功能性测试。
//
// 这里不再做类型对齐 (static_assert std::is_same)，而是验证常用的几何/线代运算结果是否正确：
//   - 向量基本运算 (加减、点积、叉积、范数、归一化)
//   - 矩阵乘法、转置、逆
//   - 仿射变换：旋转、平移、组合
//   - 四元数旋转，并与轴角矩阵相互一致

#include <gtest/gtest.h>

#include <Core/Math/Math.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

using namespace ZF::Math;
using ZF::MatrixToStdVectorOrder;

using Eigen::AngleAxisd;
using Eigen::AngleAxisf;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector3i;
using Eigen::Vector4d;
using Eigen::Vector4f;

static_assert(std::is_same<ZF::Int, std::int32_t>{},
              "ZF::Int should map to std::int32_t");
static_assert(std::is_same<ZF::UInt, std::uint32_t>{},
              "ZF::UInt should map to std::uint32_t");
static_assert(std::is_same<ZF::Float, float>{},
              "ZF::Float should map to float");
static_assert(std::is_same<ZF::eigen_scalar_t<Vector3i>, ZF::Int>{},
              "Vector3i should use the shared Int alias");
static_assert(std::is_same<ZF::eigen_scalar_t<Vector3d>, ZF::Float64>{},
              "Vector3d should use the shared Float64 alias");

static_assert(ZF::is_arithmetic_std_array_v<std::array<double, 3>>,
              "std::array<double, 3> should be accepted as a std-array conversion target");
static_assert(ZF::is_nested_arithmetic_std_array_v<std::array<std::array<double, 2>, 2>>,
              "nested std::array should be accepted as a matrix std-array conversion target");
static_assert(!ZF::is_arithmetic_std_array_v<std::vector<double>>,
              "std::vector should be rejected by the std-array conversion constraints");
static_assert(!ZF::is_nested_arithmetic_std_array_v<std::vector<std::vector<double>>>,
              "nested std::vector should be rejected by the std-array conversion constraints");

// 验证向量加减法和标量乘法的基础行为。
TEST(Vector, AddSubtractAndScalarMultiply)
{
    Vector3d a(1.0, 2.0, 3.0);
    Vector3d b(4.0, 5.0, 6.0);

    EXPECT_EQ(a + b, Vector3d(5.0, 7.0, 9.0));
    EXPECT_EQ(b - a, Vector3d(3.0, 3.0, 3.0));
    EXPECT_EQ(2.0 * a, Vector3d(2.0, 4.0, 6.0));
}

// 验证正交基向量的点积和叉积结果。
TEST(Vector, DotAndCrossProduct)
{
    Vector3d x(1.0, 0.0, 0.0);
    Vector3d y(0.0, 1.0, 0.0);

    EXPECT_DOUBLE_EQ(x.dot(y), 0.0);
    EXPECT_DOUBLE_EQ(x.dot(x), 1.0);

    // 右手系: x × y = z
    EXPECT_EQ(x.cross(y), Vector3d(0.0, 0.0, 1.0));
}

// 验证向量范数、平方范数和归一化结果。
TEST(Vector, NormAndNormalize)
{
    Vector3d v(3.0, 4.0, 0.0);

    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
    EXPECT_DOUBLE_EQ(v.squaredNorm(), 25.0);

    Vector3d n = v.normalized();
    EXPECT_NEAR(n.norm(), 1.0, 1e-12);
    EXPECT_NEAR(n.x(), 0.6, 1e-12);
    EXPECT_NEAR(n.y(), 0.8, 1e-12);
}

// 验证二维无符号夹角在垂直、同向和反向情况下的结果。
TEST(Vector, AngleBetween2DUnsigned)
{
    Vector2d x(1.0, 0.0);
    Vector2d y(0.0, 1.0);
    Vector2d negX(-1.0, 0.0);

    EXPECT_NEAR(angleBetween2D(x, y), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(angleBetween2D(x, x), 0.0, 1e-12);
    EXPECT_NEAR(angleBetween2D(x, negX), M_PI, 1e-12);
}

// 验证二维有符号夹角能反映向量顺序的方向性。
TEST(Vector, SignedAngleBetween2DTracksOrientation)
{
    Vector2d x(1.0, 0.0);
    Vector2d y(0.0, 1.0);

    EXPECT_NEAR(signedAngleBetween2D(x, y), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(signedAngleBetween2D(y, x), -M_PI / 2.0, 1e-12);
}

// 验证三维无符号夹角在垂直、同向和反向情况下的结果。
TEST(Vector, AngleBetween3DUnsigned)
{
    Vector3d x(1.0, 0.0, 0.0);
    Vector3d y(0.0, 1.0, 0.0);
    Vector3d negX(-1.0, 0.0, 0.0);

    EXPECT_NEAR(angleBetween3D(x, y), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(angleBetween3D(x, x), 0.0, 1e-12);
    EXPECT_NEAR(angleBetween3D(x, negX), M_PI, 1e-12);
}

// 验证三维有符号夹角遵循指定的叉积顺序。
TEST(Vector, SignedAngleBetween3DUsesCrossProductOrder)
{
    Vector3d x(1.0, 0.0, 0.0);
    Vector3d y(0.0, 1.0, 0.0);
    Vector3d negX(-1.0, 0.0, 0.0);

    EXPECT_NEAR(signedAngleBetween3D(x, y, CrossProductOrder::SrcCrossDst), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(signedAngleBetween3D(x, y, CrossProductOrder::DstCrossSrc), -M_PI / 2.0, 1e-12);
    EXPECT_NEAR(signedAngleBetween3D(x, negX, CrossProductOrder::SrcCrossDst), M_PI, 1e-12);
    EXPECT_NEAR(signedAngleBetween3D(x, negX, CrossProductOrder::DstCrossSrc), -M_PI, 1e-12);
}

// 验证角度辅助函数在退化输入下返回零。
TEST(Vector, AngleBetweenReturnsZeroForDegenerateInput)
{
    Vector2d zero2 = Vector2d::Zero();
    Vector2d x2(1.0, 0.0);
    Vector3d zero3 = Vector3d::Zero();
    Vector3d x3(1.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(angleBetween2D(zero2, x2), 0.0);
    EXPECT_DOUBLE_EQ(signedAngleBetween2D(zero2, x2), 0.0);
    EXPECT_DOUBLE_EQ(angleBetween3D(zero3, x3), 0.0);
    EXPECT_DOUBLE_EQ(signedAngleBetween3D(zero3, x3, CrossProductOrder::SrcCrossDst), 0.0);
    EXPECT_DOUBLE_EQ(signedAngleBetween3D(x3, zero3, CrossProductOrder::SrcCrossDst), 0.0);
}

// 验证固定尺寸列向量转换为 std::array 的结果。
TEST(Vector, ToStdArrayFromFixedSizeColumnVector)
{
    Vector3d v(1.5, -2.0, 3.25);

    const auto arr = ZF::toStdArray<double, 3>(v);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::array<double, 3>>{},
                  "toStdArray<double, 3>(Vector3d) should return std::array<double, 3>");

    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证转换为 std::array 时可以转换输出标量类型。
TEST(Vector, ToStdArrayCanCastOutputScalar)
{
    Vector3f v(1.5f, -2.0f, 3.25f);

    const auto arr = ZF::toStdArray<double, 3>(v);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::array<double, 3>>{},
                  "toStdArray<double, 3>(Vector3f) should return std::array<double, 3>");

    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证固定尺寸行向量转换为 std::array 的结果。
TEST(Vector, ToStdArrayFromFixedSizeRowVector)
{
    Eigen::RowVector4i v;
    v << 1, 2, 3, 4;

    const auto arr = ZF::toStdArray<int, 4>(v);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::array<int, 4>>{},
                  "toStdArray<int, 4>(RowVector4i) should return std::array<int, 4>");

    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
    EXPECT_EQ(arr[2], 3);
    EXPECT_EQ(arr[3], 4);
}

// 验证固定尺寸列向量转换为 std::vector 的结果。
TEST(Vector, ToStdVectorFromFixedSizeColumnVector)
{
    Vector3d v(1.5, -2.0, 3.25);

    const auto arr = ZF::toStdVector(v);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::vector<double>>{},
                  "toStdVector(Vector3d) should return std::vector<double>");

    ASSERT_EQ(arr.size(), 3u);
    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证转换为 std::vector 时可以转换输出标量类型。
TEST(Vector, ToStdVectorCanCastOutputScalar)
{
    Vector3f v(1.5f, -2.0f, 3.25f);

    const auto arr = ZF::toStdVector<double>(v);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::vector<double>>{},
                  "toStdVector<double>(Vector3f) should return std::vector<double>");

    ASSERT_EQ(arr.size(), 3u);
    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证转换结果会正确写入固定尺寸 C 数组。
TEST(Vector, ToCArrayWritesFixedSizeColumnCArray)
{
    Vector3d v(1.5, -2.0, 3.25);
    double arr[3] = {};

    ZF::toCArray(v, arr);

    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证写入 C 数组时能够自动推断并转换输出标量类型。
TEST(Vector, ToCArrayAutoDeducesOutputScalar)
{
    Vector3f v(1.5f, -2.0f, 3.25f);
    double arr[3] = {};

    ZF::toCArray(v, arr);

    EXPECT_DOUBLE_EQ(arr[0], 1.5);
    EXPECT_DOUBLE_EQ(arr[1], -2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.25);
}

// 验证 std::array 能转换为指定的 Eigen 向量类型。
TEST(Vector, ToEigenVectorFromStdArray)
{
    std::array<double, 3> arr{{1.5, -2.0, 3.25}};

    const auto v = ZF::toEigenVector<Vector3d>(arr);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3d>{},
                  "toEigenVector<Vector3d>(std::array<double, 3>) should return Vector3d");

    EXPECT_DOUBLE_EQ(v.x(), 1.5);
    EXPECT_DOUBLE_EQ(v.y(), -2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.25);
}

// 验证 std::array 转换时可以转换到目标 Eigen 向量类型。
TEST(Vector, ToEigenVectorFromStdArrayCanCastToTargetVector)
{
    std::array<float, 3> arr{{1.5f, -2.0f, 3.25f}};

    const auto v = ZF::toEigenVector<Vector3d>(arr);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3d>{},
                  "toEigenVector<Vector3d>(std::array<float, 3>) should return Vector3d");

    EXPECT_DOUBLE_EQ(v.x(), 1.5);
    EXPECT_DOUBLE_EQ(v.y(), -2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.25);
}

// 验证 C 数组能转换为指定的 Eigen 向量类型。
TEST(Vector, ToEigenVectorFromCArray)
{
    double arr[3] = {1.5, -2.0, 3.25};

    const auto v = ZF::toEigenVector<Vector3d>(arr);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3d>{},
                  "toEigenVector<Vector3d>(double[3]) should return Vector3d");

    EXPECT_DOUBLE_EQ(v.x(), 1.5);
    EXPECT_DOUBLE_EQ(v.y(), -2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.25);
}

// 验证 C 数组转换时可以转换到目标 Eigen 向量类型。
TEST(Vector, ToEigenVectorFromCArrayCanCastToTargetVector)
{
    float arr[3] = {1.5f, -2.0f, 3.25f};

    const auto v = ZF::toEigenVector<Vector3d>(arr);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3d>{},
                  "toEigenVector<Vector3d>(float[3]) should return Vector3d");

    EXPECT_DOUBLE_EQ(v.x(), 1.5);
    EXPECT_DOUBLE_EQ(v.y(), -2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.25);
}

// 验证 std::vector 转换时会使用指定的 Eigen 目标类型。
TEST(Vector, ToEigenVectorFromStdVectorUsesTargetType)
{
    const std::vector<double> arr{1.5, -2.0, 3.25};

    const auto v = ZF::toEigenVector<Vector3f>(arr);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3f>{},
                  "ZF::toEigenVector<Vector3f>(std::vector<double>) should return Vector3f");

    EXPECT_FLOAT_EQ(v.x(), 1.5f);
    EXPECT_FLOAT_EQ(v.y(), -2.0f);
    EXPECT_FLOAT_EQ(v.z(), 3.25f);
}

// 验证标量指针转换时会按目标 Eigen 维度读取数据。
TEST(Vector, ToEigenVectorFromScalarPointerUsesTargetDimension)
{
    const std::vector<double> arr{1.5, -2.0, 3.25, 8.0, 9.0, 10.0};

    const double* values = arr.data() + 3;

    const auto v = ZF::toEigenVector<Vector3f>(values);
    static_assert(std::is_same<std::decay_t<decltype(v)>, Vector3f>{},
                  "ZF::toEigenVector<Vector3f>(double*) should return Vector3f");

    EXPECT_FLOAT_EQ(v.x(), 8.0f);
    EXPECT_FLOAT_EQ(v.y(), 9.0f);
    EXPECT_FLOAT_EQ(v.z(), 10.0f);
}

// 验证转换为 QStringList 时保持元素顺序。
TEST(Convert, ToStringListPreservesOrder)
{
    const std::vector<std::string> values{"alpha", "beta", "gamma"};

    const QStringList list = ZF::toStringList(values);

    ASSERT_EQ(list.size(), 3);
    EXPECT_EQ(list.at(0), QStringLiteral("alpha"));
    EXPECT_EQ(list.at(1), QStringLiteral("beta"));
    EXPECT_EQ(list.at(2), QStringLiteral("gamma"));
}

// 验证从 QStringList 转回 vector 时保持元素顺序。
TEST(Convert, ToVectorPreservesOrder)
{
    const QStringList list{QStringLiteral("left"), QStringLiteral("right")};

    const auto values = ZF::toVector(list);

    ASSERT_EQ(values.size(), 2u);
    EXPECT_EQ(values[0], "left");
    EXPECT_EQ(values[1], "right");
}

// 验证矩阵字符串格式化会保留指定前缀和尾部分隔符。
TEST(Convert, ToStdStringMatrixWithMatPrefixAndTrailingSpace)
{
    Eigen::Matrix2f m;
    m << 1, 0, 0, 1;

    const std::string s = ZF::toStdString(m, " ", " \n", "mat: \n", " \n");

    EXPECT_EQ(s, std::string("mat: \n1 0 \n0 1 \n"));
}

// 验证矩阵字符串格式化同样适用于动态尺寸矩阵。
TEST(Convert, ToStdStringSupportsDynamicMatrix)
{
    Eigen::MatrixXd m(2, 2);
    m << 1, 0,
         0, 1;

    const std::string s = ZF::toStdString(m, " ", " \n", "mat: \n", " \n");

    EXPECT_EQ(s, std::string("mat: \n1 0 \n0 1 \n"));
}

// 验证向量转换为 QJsonArray 时保持分量顺序。
TEST(Vector, ToQJsonArrayFromVector)
{
    const Vector3f v(1.5f, -2.0f, 3.25f);

    const QJsonArray arr = ZF::toQJsonArray(v);

    ASSERT_EQ(arr.size(), 3);
    EXPECT_DOUBLE_EQ(arr.at(0).toDouble(), 1.5);
    EXPECT_DOUBLE_EQ(arr.at(1).toDouble(), -2.0);
    EXPECT_DOUBLE_EQ(arr.at(2).toDouble(), 3.25);
}

// 验证固定尺寸矩阵转换为嵌套 std::array 的结果。
TEST(Matrix, ToStdArrayFromFixedSizeMatrix)
{
    Eigen::Matrix<double, 2, 3> m;
    m << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0;

    const auto arr = ZF::toStdArray<double, 2, 3>(m);
    static_assert(std::is_same<std::decay_t<decltype(arr)>,
                               std::array<std::array<double, 3>, 2>>{},
                  "toStdArray<double, 2, 3>(Matrix<double, 2, 3>) should return std::array<std::array<double, 3>, 2>");

    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 2.0);
    EXPECT_DOUBLE_EQ(arr[0][2], 3.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 4.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 5.0);
    EXPECT_DOUBLE_EQ(arr[1][2], 6.0);
}

// 验证矩阵转换为 std::array 时可以转换输出标量类型。
TEST(Matrix, ToStdArrayCanCastOutputScalar)
{
    Eigen::Matrix<float, 2, 2> m;
    m << 1.0f, 2.0f,
         3.0f, 4.0f;

    const auto arr = ZF::toStdArray<double, 2, 2>(m);
    static_assert(std::is_same<std::decay_t<decltype(arr)>,
                               std::array<std::array<double, 2>, 2>>{},
                  "toStdArray<double, 2, 2>(Matrix<float, 2, 2>) should return std::array<std::array<double, 2>, 2>");

    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 2.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 3.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 4.0);
}

// 验证矩阵转换为 QJsonArray 时保持行列顺序。
TEST(Matrix, ToQJsonArrayFromMatrix)
{
    Eigen::Matrix<float, 2, 2> m;
    m << 1.0f, 2.0f,
         3.0f, 4.0f;

    const QJsonArray arr = ZF::toQJsonArray(m);

    ASSERT_EQ(arr.size(), 2);
    ASSERT_TRUE(arr.at(0).isArray());
    ASSERT_TRUE(arr.at(1).isArray());

    const QJsonArray row0 = arr.at(0).toArray();
    const QJsonArray row1 = arr.at(1).toArray();

    ASSERT_EQ(row0.size(), 2);
    ASSERT_EQ(row1.size(), 2);
    EXPECT_DOUBLE_EQ(row0.at(0).toDouble(), 1.0);
    EXPECT_DOUBLE_EQ(row0.at(1).toDouble(), 2.0);
    EXPECT_DOUBLE_EQ(row1.at(0).toDouble(), 3.0);
    EXPECT_DOUBLE_EQ(row1.at(1).toDouble(), 4.0);
}

// 验证按行主序展开时矩阵元素会逐行输出。
TEST(Matrix, MatrixToStdVector1DRowMajorFlattensByRows)
{
    Eigen::Matrix<float, 2, 3> m;
    m << 1.0f, 2.0f, 3.0f,
         4.0f, 5.0f, 6.0f;

    const auto arr = ZF::MatrixToStdVector1D<double>(m, MatrixToStdVectorOrder::RowMajor);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::vector<double>>{},
                  "MatrixToStdVector1D<double>(Matrix<float, 2, 3>, RowMajor) should return std::vector<double>");

    ASSERT_EQ(arr.size(), 6u);
    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
    EXPECT_DOUBLE_EQ(arr[4], 5.0);
    EXPECT_DOUBLE_EQ(arr[5], 6.0);
}

// 验证按列主序展开时矩阵元素会逐列输出。
TEST(Matrix, MatrixToStdVector1DColMajorFlattensByCols)
{
    Eigen::Matrix<float, 2, 3> m;
    m << 1.0f, 2.0f, 3.0f,
         4.0f, 5.0f, 6.0f;

    const auto arr = ZF::MatrixToStdVector1D<double>(m, MatrixToStdVectorOrder::ColMajor);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::vector<double>>{},
                  "MatrixToStdVector1D<double>(Matrix<float, 2, 3>, ColMajor) should return std::vector<double>");

    ASSERT_EQ(arr.size(), 6u);
    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 4.0);
    EXPECT_DOUBLE_EQ(arr[2], 2.0);
    EXPECT_DOUBLE_EQ(arr[3], 5.0);
    EXPECT_DOUBLE_EQ(arr[4], 3.0);
    EXPECT_DOUBLE_EQ(arr[5], 6.0);
}

// 验证按行主序转换二维 vector 时外层表示各行。
TEST(Matrix, MatrixToStdVector2DRowMajorReturnsRowOuterVector)
{
    Eigen::Matrix<float, 2, 3> m;
    m << 1.0f, 2.0f, 3.0f,
         4.0f, 5.0f, 6.0f;

    const auto arr = ZF::MatrixToStdVector2D<double>(m, MatrixToStdVectorOrder::RowMajor);
    static_assert(std::is_same<std::decay_t<decltype(arr)>,
                               std::vector<std::vector<double>>>{},
                  "MatrixToStdVector2D<double>(Matrix<float, 2, 3>, RowMajor) should return std::vector<std::vector<double>>");

    ASSERT_EQ(arr.size(), 2u);
    ASSERT_EQ(arr[0].size(), 3u);
    ASSERT_EQ(arr[1].size(), 3u);
    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 2.0);
    EXPECT_DOUBLE_EQ(arr[0][2], 3.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 4.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 5.0);
    EXPECT_DOUBLE_EQ(arr[1][2], 6.0);
}

// 验证按列主序转换二维 vector 时外层表示各列。
TEST(Matrix, MatrixToStdVector2DColMajorReturnsColOuterVector)
{
    Eigen::Matrix<float, 2, 3> m;
    m << 1.0f, 2.0f, 3.0f,
         4.0f, 5.0f, 6.0f;

    const auto arr = ZF::MatrixToStdVector2D<double>(m, MatrixToStdVectorOrder::ColMajor);
    static_assert(std::is_same<std::decay_t<decltype(arr)>,
                               std::vector<std::vector<double>>>{},
                  "MatrixToStdVector2D<double>(Matrix<float, 2, 3>, ColMajor) should return std::vector<std::vector<double>>");

    ASSERT_EQ(arr.size(), 3u);
    ASSERT_EQ(arr[0].size(), 2u);
    ASSERT_EQ(arr[1].size(), 2u);
    ASSERT_EQ(arr[2].size(), 2u);
    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 4.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 2.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 5.0);
    EXPECT_DOUBLE_EQ(arr[2][0], 3.0);
    EXPECT_DOUBLE_EQ(arr[2][1], 6.0);
}

// 验证矩阵转换结果会正确写入固定尺寸 C 矩阵。
TEST(Matrix, ToCArrayWritesFixedSizeCMatrix)
{
    Eigen::Matrix<double, 2, 3> m;
    m << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0;

    double arr[2][3] = {};
    ZF::toCArray(m, arr);

    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 2.0);
    EXPECT_DOUBLE_EQ(arr[0][2], 3.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 4.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 5.0);
    EXPECT_DOUBLE_EQ(arr[1][2], 6.0);
}

// 验证矩阵写入 C 数组时能够自动推断并转换输出标量类型。
TEST(Matrix, ToCArrayMatrixAutoDeducesOutputScalar)
{
    Eigen::Matrix<float, 2, 2> m;
    m << 1.0f, 2.0f,
         3.0f, 4.0f;

    double arr[2][2] = {};
    ZF::toCArray(m, arr);

    EXPECT_DOUBLE_EQ(arr[0][0], 1.0);
    EXPECT_DOUBLE_EQ(arr[0][1], 2.0);
    EXPECT_DOUBLE_EQ(arr[1][0], 3.0);
    EXPECT_DOUBLE_EQ(arr[1][1], 4.0);
}

// 验证单位矩阵不会改变向量。
TEST(Matrix, IdentityMultiplyVector)
{
    Vector3d v(1.0, 2.0, 3.0);

    EXPECT_EQ(Matrix3d::Identity() * v, v);
}

// 验证对角缩放矩阵会按坐标轴分别缩放向量。
TEST(Matrix, DiagonalScalingTransformsVector)
{
    Matrix3d m = Matrix3d::Identity();
    m(0, 0) = 2.0;
    m(1, 1) = 3.0;
    m(2, 2) = 4.0;

    Vector3d v(1.0, 2.0, 3.0);

    EXPECT_EQ(m * v, Vector3d(2.0, 6.0, 12.0));
}

// 验证转置和求逆操作满足预期的往返性质。
TEST(Matrix, TransposeAndInverseRoundTrip)
{
    Matrix3d m;
    m << 1.0, 2.0, 3.0,
         0.0, 1.0, 4.0,
         5.0, 6.0, 0.0;

    Matrix3d inv = m.inverse();
    Matrix3d round = m * inv;

    EXPECT_TRUE(round.isApprox(Matrix3d::Identity(), 1e-9));

    // 转置的转置仍是自身
    EXPECT_TRUE(m.transpose().transpose().isApprox(m));
}

// 验证 4x4 平移矩阵只影响点而不影响方向。
TEST(Matrix, Mat4TranslationOnHomogeneousPoint)
{
    Matrix4d m = Matrix4d::Identity();
    m(0, 3) = 10.0;
    m(1, 3) = 20.0;
    m(2, 3) = 30.0;

    // 齐次坐标 w = 1 -> 受到平移影响
    Vector4d p(1.0, 2.0, 3.0, 1.0);
    EXPECT_EQ(m * p, Vector4d(11.0, 22.0, 33.0, 1.0));

    // 齐次方向 w = 0 -> 不受平移影响
    Vector4d dir(1.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(m * dir, Vector4d(1.0, 0.0, 0.0, 0.0));
}

// 验证绕 Z 轴旋转矩阵会把 X 轴旋到 Y 轴。
TEST(Matrix, RotationMatrixAroundZ)
{
    AngleAxisd aa(M_PI / 2.0, Vector3d::UnitZ());
    Matrix3d rot = aa.toRotationMatrix();

    Vector3d v(1.0, 0.0, 0.0);
    Vector3d r = rot * v;

    EXPECT_NEAR(r.x(), 0.0, 1e-12);
    EXPECT_NEAR(r.y(), 1.0, 1e-12);
    EXPECT_NEAR(r.z(), 0.0, 1e-12);
}

// 验证四元数转换为 QJsonArray 时保持 XYZW 分量顺序。
TEST(Quaternion, ToQJsonArrayKeepsXyzwOrder)
{
    const Quaterniond q(4.0, 1.0, 2.0, 3.0);

    const QJsonArray arr = ZF::toQJsonArray(q);

    ASSERT_EQ(arr.size(), 4);
    EXPECT_DOUBLE_EQ(arr.at(0).toDouble(), 1.0);
    EXPECT_DOUBLE_EQ(arr.at(1).toDouble(), 2.0);
    EXPECT_DOUBLE_EQ(arr.at(2).toDouble(), 3.0);
    EXPECT_DOUBLE_EQ(arr.at(3).toDouble(), 4.0);
}

// 验证正交投影会把视盒角点和中心映射到 NDC。
TEST(Matrix, OrthoMapsViewBoxToNdc)
{
    const double left = -3.0;
    const double right = 5.0;
    const double bottom = -2.0;
    const double top = 6.0;
    const double nearPlane = 0.5;
    const double farPlane = 10.5;

    Matrix4d proj = ortho(left, right, bottom, top, nearPlane, farPlane);

    const Vector4d nearMin(left, bottom, -nearPlane, 1.0);
    const Vector4d farMax(right, top, -farPlane, 1.0);
    const Vector4d center((left + right) * 0.5,
                          (bottom + top) * 0.5,
                          -(nearPlane + farPlane) * 0.5,
                          1.0);

    EXPECT_TRUE((proj * nearMin).isApprox(Vector4d(-1.0, -1.0, -1.0, 1.0), 1e-12));
    EXPECT_TRUE((proj * farMax).isApprox(Vector4d(1.0, 1.0, 1.0, 1.0), 1e-12));
    EXPECT_TRUE((proj * center).isApprox(Vector4d(0.0, 0.0, 0.0, 1.0), 1e-12));
}

// 验证四参数正交投影重载保持默认深度范围。
TEST(Matrix, OrthoFourArgOverloadKeepsDefaultDepthRange)
{
    Matrix4d proj = ortho(-2.0, 2.0, -4.0, 4.0);

    EXPECT_TRUE((proj * Vector4d(-2.0, -4.0, 1.0, 1.0)).isApprox(Vector4d(-1.0, -1.0, -1.0, 1.0), 1e-12));
    EXPECT_TRUE((proj * Vector4d(2.0, 4.0, -1.0, 1.0)).isApprox(Vector4d(1.0, 1.0, 1.0, 1.0), 1e-12));
}

// 验证透视投影会把中心射线的近远平面深度映射到 NDC。
TEST(Matrix, PerspectiveMapsCenterRayDepthIntoNdc)
{
    const double fovy = M_PI / 2.0;
    const double aspect = 1.5;
    const double nearPlane = 1.0;
    const double farPlane = 11.0;

    Matrix4d proj = perspective(fovy, aspect, nearPlane, farPlane);

    const Vector4d nearCenter(0.0, 0.0, -nearPlane, 1.0);
    const Vector4d farCenter(0.0, 0.0, -farPlane, 1.0);

    const Vector4d nearClip = proj * nearCenter;
    const Vector4d farClip = proj * farCenter;
    const Vector3d nearNdc = nearClip.head<3>() / nearClip.w();
    const Vector3d farNdc = farClip.head<3>() / farClip.w();

    EXPECT_NEAR(nearNdc.x(), 0.0, 1e-12);
    EXPECT_NEAR(nearNdc.y(), 0.0, 1e-12);
    EXPECT_NEAR(nearNdc.z(), -1.0, 1e-12);

    EXPECT_NEAR(farNdc.x(), 0.0, 1e-12);
    EXPECT_NEAR(farNdc.y(), 0.0, 1e-12);
    EXPECT_NEAR(farNdc.z(), 1.0, 1e-12);
}

// 验证六平面透视重载在对称视锥下与 FOV 重载结果一致。
TEST(Matrix, PerspectiveSixPlanesMatchesFovPerspectiveForSymmetricFrustum)
{
    const double fovy = M_PI / 3.0;
    const double aspect = 1.5;
    const double nearPlane = 0.25;
    const double farPlane = 64.0;

    const double top = nearPlane * std::tan(fovy * 0.5);
    const double bottom = -top;
    const double right = top * aspect;
    const double left = -right;

    const Matrix4d fromFov = perspective(fovy, aspect, nearPlane, farPlane);
    const Matrix4d fromPlanes = perspective(left, right, bottom, top, nearPlane, farPlane);

    EXPECT_TRUE(fromPlanes.isApprox(fromFov, 1e-12));
}

// 验证六平面透视重载会把视锥边界映射到 NDC。
TEST(Matrix, PerspectiveSixPlanesMapsNearPlaneBoundsToNdc)
{
    const double left = -0.5;
    const double right = 1.0;
    const double bottom = -0.25;
    const double top = 0.75;
    const double nearPlane = 0.5;
    const double farPlane = 20.0;

    const Matrix4d proj = perspective(left, right, bottom, top, nearPlane, farPlane);

    const Vector4d nearBottomLeft(left, bottom, -nearPlane, 1.0);
    const Vector4d nearTopRight(right, top, -nearPlane, 1.0);
    const Vector4d farCenter(0.0, 0.0, -farPlane, 1.0);

    const Vector4d nearBottomLeftClip = proj * nearBottomLeft;
    const Vector4d nearTopRightClip = proj * nearTopRight;
    const Vector4d farCenterClip = proj * farCenter;

    const Vector3d nearBottomLeftNdc = nearBottomLeftClip.head<3>() / nearBottomLeftClip.w();
    const Vector3d nearTopRightNdc = nearTopRightClip.head<3>() / nearTopRightClip.w();
    const Vector3d farCenterNdc = farCenterClip.head<3>() / farCenterClip.w();

    EXPECT_TRUE(nearBottomLeftNdc.isApprox(Vector3d(-1.0, -1.0, -1.0), 1e-12));
    EXPECT_TRUE(nearTopRightNdc.isApprox(Vector3d(1.0, 1.0, -1.0), 1e-12));
    EXPECT_NEAR(farCenterNdc.z(), 1.0, 1e-12);
}


// 验证 makeTransform 返回的 4x4 仿射矩阵包含正确分量。
TEST(Transform, MakeAffineReturnsMatrix4Type)
{
    const Vector3d translation(1.0, 2.0, 3.0);
    const Matrix3d rotation = AngleAxisd(M_PI / 4.0, Vector3d::UnitY()).toRotationMatrix();
    const Quaterniond quaternion(AngleAxisd(M_PI / 4.0, Vector3d::UnitY()));
    const Vector3d scale(2.0, 3.0, 4.0);

    const auto transform = makeTransform(translation, rotation, scale);
    const auto transformFromQuaternion = makeTransform(translation, quaternion, scale);

    static_assert(std::is_same<std::decay_t<decltype(transform)>,
                               Eigen::Matrix<double, 4, 4>>{},
                  "makeTransform(Vector3d, Matrix3d, Vector3d) should return Eigen::Matrix<double, 4, 4>");

    EXPECT_TRUE((transform.topLeftCorner<3, 3>().isApprox(rotation * scale.asDiagonal(), 1e-12)));
    EXPECT_TRUE((transform.topRightCorner<3, 1>().isApprox(translation, 1e-12)));
    EXPECT_TRUE(transform.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-12));
    EXPECT_TRUE(transformFromQuaternion.isApprox(transform, 1e-12));
}

// 验证各类仿射辅助函数会正确填充平移、旋转和缩放分量。
TEST(Transform, MakeAffineVariantsSetExpectedComponents)
{
    const Vector3f translation(1.0f, 2.0f, 3.0f);
    const Matrix3f rotation = AngleAxisf(Numbersf::PI_2(), Vector3f::UnitZ()).toRotationMatrix();
    const Vector3f scale(2.0f, 3.0f, 4.0f);
    const Matrix3f expectedScaleMatrix = scale.asDiagonal();
    const Matrix3f scaleMatrix3x3 = makeScaleMatrix3x3(scale);

    const auto fromTranslation = makeTranslateMatrix4x4(translation);
    EXPECT_TRUE((fromTranslation.topLeftCorner<3, 3>().isApprox(Matrix3f::Identity(), 1e-5f)));
    EXPECT_TRUE((fromTranslation.topRightCorner<3, 1>().isApprox(translation, 1e-5f)));
    EXPECT_TRUE(fromTranslation.row(3).isApprox(Eigen::RowVector4f(0.0f, 0.0f, 0.0f, 1.0f), 1e-5f));

    const auto fromLinear = Matrix3x3ToMatrix4x4(rotation);
    EXPECT_TRUE((fromLinear.topLeftCorner<3, 3>().isApprox(rotation, 1e-5f)));
    EXPECT_TRUE((fromLinear.topRightCorner<3, 1>().isApprox(Vector3f::Zero(), 1e-5f)));
    EXPECT_TRUE(Matrix4x4ToMatrix3x3(fromLinear).isApprox(rotation, 1e-5f));

    const Quaternionf quaternion(AngleAxisf(Numbersf::PI_2(), Vector3f::UnitZ()));
    const auto fromQuaternion = QuaternionToMatrix4x4(quaternion);
    EXPECT_TRUE((fromQuaternion.topLeftCorner<3, 3>().isApprox(rotation, 1e-5f)));
    EXPECT_TRUE((fromQuaternion.topRightCorner<3, 1>().isApprox(Vector3f::Zero(), 1e-5f)));

    const auto fromScale = makeScaleMatrix4x4(scale);
    EXPECT_TRUE(scaleMatrix3x3.isApprox(expectedScaleMatrix, 1e-5f));
    EXPECT_TRUE((fromScale.topLeftCorner<3, 3>().isApprox(scaleMatrix3x3, 1e-5f)));
    EXPECT_TRUE((fromScale.topRightCorner<3, 1>().isApprox(Vector3f::Zero(), 1e-5f)));

    const auto fromTransAndRot = makeTransformFromTransAndRot(translation, rotation);
    EXPECT_TRUE((fromTransAndRot.topLeftCorner<3, 3>().isApprox(rotation, 1e-5f)));
    EXPECT_TRUE((fromTransAndRot.topRightCorner<3, 1>().isApprox(translation, 1e-5f)));

    const auto fromTransAndQuaternion = makeTransformFromTransAndRot(translation, quaternion);
    EXPECT_TRUE(fromTransAndQuaternion.isApprox(fromTransAndRot, 1e-5f));

    const auto fromTransAndScale = makeTransformFromTransAndScale(translation, scale);
    EXPECT_TRUE((fromTransAndScale.topLeftCorner<3, 3>().isApprox(expectedScaleMatrix, 1e-5f)));
    EXPECT_TRUE((fromTransAndScale.topRightCorner<3, 1>().isApprox(translation, 1e-5f)));

    const auto fromRotAndScale = makeTransformFromRotAndScale(rotation, scale);
    EXPECT_TRUE((fromRotAndScale.topLeftCorner<3, 3>().isApprox(rotation * scale.asDiagonal(), 1e-5f)));
    EXPECT_TRUE((fromRotAndScale.topRightCorner<3, 1>().isApprox(Vector3f::Zero(), 1e-5f)));

    const auto fromQuaternionAndScale = makeTransformFromRotAndScale(quaternion, scale);
    EXPECT_TRUE(fromQuaternionAndScale.isApprox(fromRotAndScale, 1e-5f));
}

// 验证点和方向的矩阵乘法遵循齐次坐标 W 的约定。
TEST(Transform, MatrixMulPointAndDirRespectHomogeneousW)
{
    const Matrix4f translation = makeTranslateMatrix4x4(10.0f, 20.0f, 30.0f);
    const Matrix4f scaling = makeScaleMatrix4x4(Vector3f(2.0f, 3.0f, 4.0f));

    EXPECT_TRUE(MatrixMulPoint(translation, Vector3f(1.0f, 2.0f, 3.0f)).isApprox(Vector3f(11.0f, 22.0f, 33.0f), 1e-5f));
    EXPECT_TRUE(MatrixMulDir(translation, Vector3f::UnitX()).isApprox(Vector3f::UnitX(), 1e-5f));
    EXPECT_TRUE(MatrixMulDir(scaling, Vector3f(1.0f, 1.0f, 0.0f)).isApprox(Vector3f(2.0f, 3.0f, 0.0f).normalized(), 1e-5f));
}

// 验证按轴构造朝向的辅助函数返回请求的矩阵和四元数类型。
TEST(Transform, OrientationFromAxisHelpersReturnRequestedTypes)
{
    const Matrix4f fromZ = makeRotationFromDirectionMatrix4x4(Vector3f::UnitZ(), Vector3f::UnitY(), OrientationAxis::Z);
    const Matrix3f fromX = makeRotationFromDirectionMatrix3x3(Vector3f::UnitX(), Vector3f::UnitY(), OrientationAxis::X);
    const Quaternionf fromY = makeRotationFromDirectionQuaternion(Vector3f::UnitY(), Vector3f::UnitZ(), OrientationAxis::Y);

    EXPECT_TRUE(fromZ.isApprox(Matrix4f::Identity(), 1e-5f));
    EXPECT_TRUE(fromX.isApprox(Matrix3f::Identity(), 1e-5f));
    EXPECT_TRUE(fromY.toRotationMatrix().isApprox(Matrix3f::Identity(), 1e-5f));
}

// 验证旋转辅助函数返回请求的矩阵和四元数类型。
TEST(Transform, RotationHelpersReturnRequestedTypes)
{
    const Matrix3f fromMatrix3 = makeRotationMatrix3x3(Vector3f::UnitZ(), Numbersf::PI_2());
    const Matrix4f fromMatrix4 = makeRotationMatrix4x4(Vector3f::UnitZ(), Numbersf::PI_2());
    const Quaternionf fromQuaternion = makeRotationQuaternion(Vector3f::UnitZ(), Numbersf::PI_2());
    const Matrix3f fromQuaternionMatrix3 = fromQuaternion.toRotationMatrix();
    const Matrix4f fromQuaternionMatrix4 = QuaternionToMatrix4x4(fromQuaternion);
    const Quaternionf fromMatrix3Quaternion = Matrix3x3ToQuaternion(fromMatrix3);
    const Quaternionf fromMatrix4Quaternion(fromMatrix4.topLeftCorner<3, 3>());

    EXPECT_TRUE((fromMatrix3 * Vector3f::UnitX()).isApprox(Vector3f::UnitY(), 1e-5f));
    EXPECT_TRUE((fromMatrix4.topLeftCorner<3, 3>().isApprox(fromMatrix3, 1e-5f)));
    EXPECT_TRUE(fromQuaternion.toRotationMatrix().isApprox(fromMatrix3, 1e-5f));
    EXPECT_TRUE(fromQuaternionMatrix3.isApprox(fromMatrix3, 1e-5f));
    EXPECT_TRUE(fromQuaternionMatrix4.isApprox(fromMatrix4, 1e-5f));
    EXPECT_TRUE(fromMatrix3Quaternion.toRotationMatrix().isApprox(fromMatrix3, 1e-5f));
    EXPECT_TRUE(fromMatrix4Quaternion.toRotationMatrix().isApprox(fromMatrix3, 1e-5f));
}

// 验证旋转辅助函数能构造预期的局部坐标系和绕点旋转结果。
TEST(Transform, RotationHelpersProduceExpectedFrames)
{
    const Matrix3f aroundZ = makeRotationMatrix3x3(Vector3f::UnitZ(), Numbersf::PI_2());
    EXPECT_TRUE((aroundZ * Vector3f::UnitX()).isApprox(Vector3f::UnitY(), 1e-5f));

    const Matrix3f basis = makeRotationFromOrthoAxesMatrix3x3(Vector3f::UnitX(), Vector3f::UnitY(), Vector3f::UnitZ());
    EXPECT_TRUE(basis.isApprox(Matrix3f::Identity(), 1e-5f));

    const Matrix3f alignX = makeRotationFromXAxisMatrix3x3(Vector3f::UnitY());
    EXPECT_TRUE(alignX.col(0).isApprox(Vector3f::UnitY(), 1e-5f));
    EXPECT_NEAR(alignX.col(0).dot(alignX.col(1)), 0.0f, 1e-5f);
    EXPECT_NEAR(alignX.col(0).dot(alignX.col(2)), 0.0f, 1e-5f);

    const Matrix4f aroundPivot = makeRotationMatrixAroundPoint(Vector3f(1.0f, 0.0f, 0.0f), Vector3f::UnitZ(), Numbersf::PI_2());
    EXPECT_TRUE(MatrixMulPoint(aroundPivot, Vector3f(1.0f, 0.0f, 0.0f)).isApprox(Vector3f(1.0f, 0.0f, 0.0f), 1e-5f));
    EXPECT_TRUE(MatrixMulPoint(aroundPivot, Vector3f(2.0f, 0.0f, 0.0f)).isApprox(Vector3f(1.0f, 1.0f, 0.0f), 1e-5f));
}

// 验证欧拉角辅助函数在角度、弧度、矩阵和四元数之间保持一致。
TEST(Transform, EulerHelpersStayConsistent)
{
    const Vector3f radians(0.3f, -0.2f, 0.1f);
    const Vector3f degrees = radians * Numbersf::RAD_TO_DEG();

    const Matrix3f matrixFromRadians = makeRotationFromEulerRadiansMatrix3x3(radians);
    const Matrix3f matrixFromDegrees = makeRotationFromEulerDegreesMatrix3x3(degrees);
    const Matrix4f matrix4FromRadians = makeRotationFromEulerRadiansMatrix4x4(radians);
    const Matrix4f matrix4FromDegrees = makeRotationFromEulerDegreesMatrix4x4(degrees);
    const Quaternionf quaternionFromRadians = makeRotationFromEulerRadiansQuaternion(radians);

    EXPECT_TRUE(matrixFromDegrees.isApprox(matrixFromRadians, 1e-5f));
    EXPECT_TRUE((matrix4FromRadians.topLeftCorner<3, 3>().isApprox(matrixFromRadians, 1e-5f)));
    EXPECT_TRUE((matrix4FromDegrees.topLeftCorner<3, 3>().isApprox(matrixFromRadians, 1e-5f)));
    EXPECT_TRUE(Matrix3x3ToEulerXYZRadians(matrixFromRadians).isApprox(radians, 1e-5f));
    EXPECT_TRUE(Matrix4x4ToEulerXYZRadians(matrix4FromRadians).isApprox(radians, 1e-5f));
    EXPECT_TRUE(QuaternionToEulerXYZRadians(quaternionFromRadians).isApprox(radians, 1e-5f));

    const Quaternionf quat = makeRotationFromEulerDegreesQuaternion(degrees);
    EXPECT_TRUE(quat.toRotationMatrix().isApprox(matrixFromRadians, 1e-5f));
}

// 验证四元数绕 Z 轴旋转 90 度会把 X 轴旋到 Y 轴。
TEST(Quaternion, RotateVectorAroundZBy90Degrees)
{
    Quaterniond q(AngleAxisd(M_PI / 2.0, Vector3d::UnitZ()));

    Vector3d r = q * Vector3d(1.0, 0.0, 0.0);

    EXPECT_NEAR(r.x(), 0.0, 1e-12);
    EXPECT_NEAR(r.y(), 1.0, 1e-12);
    EXPECT_NEAR(r.z(), 0.0, 1e-12);
}

// 验证四元数转换为 std::array 时保持 XYZW 分量顺序。
TEST(Quaternion, ToStdArrayKeepsXyzwOrder)
{
    Quaterniond q(4.0, 1.0, 2.0, 3.0);

    const auto arr = ZF::toStdArray<double, 4>(q);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::array<double, 4>>{},
                  "toStdArray<double, 4>(Quaterniond) should return std::array<double, 4>");

    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
}

// 验证四元数转换为 std::array 时可以转换输出标量类型。
TEST(Quaternion, ToStdArrayCanCastOutputScalar)
{
    Quaternionf q(4.0f, 1.0f, 2.0f, 3.0f);

    const auto arr = ZF::toStdArray<double, 4>(q);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::array<double, 4>>{},
                  "toStdArray<double, 4>(Quaternionf) should return std::array<double, 4>");

    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
}

// 验证四元数转换为 std::vector 时保持 XYZW 分量顺序。
TEST(Quaternion, ToStdVectorKeepsXyzwOrder)
{
    Quaternionf q(4.0f, 1.0f, 2.0f, 3.0f);

    const auto arr = ZF::toStdVector<double>(q);
    static_assert(std::is_same<std::decay_t<decltype(arr)>, std::vector<double>>{},
                  "toStdVector<double>(Quaternionf) should return std::vector<double>");

    ASSERT_EQ(arr.size(), 4u);
    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
}

// 验证四元数转换结果会按 XYZW 顺序写入 C 数组。
TEST(Quaternion, ToCArrayWritesQuaternionCArray)
{
    Quaterniond q(4.0, 1.0, 2.0, 3.0);
    double arr[4] = {};

    ZF::toCArray(q, arr);

    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
}

// 验证四元数写入 C 数组时能够自动推断并转换输出标量类型。
TEST(Quaternion, ToCArrayQuaternionAutoDeducesOutputScalar)
{
    Quaternionf q(4.0f, 1.0f, 2.0f, 3.0f);
    double arr[4] = {};

    ZF::toCArray(q, arr);

    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 2.0);
    EXPECT_DOUBLE_EQ(arr[2], 3.0);
    EXPECT_DOUBLE_EQ(arr[3], 4.0);
}

// 验证四元数组合旋转与对应矩阵组合结果一致。
TEST(Quaternion, ComposeRotationsEquivalentToMatrixComposition)
{
    Quaterniond qx(AngleAxisd(M_PI / 2.0, Vector3d::UnitX()));
    Quaterniond qy(AngleAxisd(M_PI / 2.0, Vector3d::UnitY()));

    // 先绕 Y 再绕 X (注意四元数乘法顺序与变换顺序的关系)
    Quaterniond q_combined = qx * qy;

    Matrix3d m_combined = qx.toRotationMatrix() * qy.toRotationMatrix();

    // 同一向量经四元数与矩阵作用后应得到相同结果
    Vector3d v(1.0, 2.0, 3.0);
    Vector3d r_quat = q_combined * v;
    Vector3d r_mat = m_combined * v;

    EXPECT_TRUE(r_quat.isApprox(r_mat, 1e-12));
}

// 验证单位四元数不会改变向量。
TEST(Quaternion, IdentityRotationKeepsVectorIntact)
{
    Quaterniond q = Quaterniond::Identity();

    Vector3d v(1.5, -2.0, 3.25);
    EXPECT_TRUE((q * v).isApprox(v, 1e-12));
}
