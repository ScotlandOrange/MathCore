#pragma once

// ZF::Math::Numbers<T> —— 数学常量模板，风格对齐 vsg::numbers<T>。
//
// 用法：
//   ZF::Math::Numbers<double>::PI()          // 3.14159...
//   ZF::Math::Numbers<float>::DEG_TO_RAD()   // (float)(π / 180)
//   ZF::Math::Numbers<double>::E()
//
// 所有常量均为 static constexpr 成员函数，通过 static_cast<T> 保证精度不丢失。

#include <limits>

namespace ZF
{
namespace Math
{

template <typename T>
struct Numbers
{
    // ---- 通用 ----
    static constexpr T zero()     { return static_cast<T>(0.0); }
    static constexpr T half()     { return static_cast<T>(0.5); }
    static constexpr T one()      { return static_cast<T>(1.0); }
    static constexpr T two()      { return static_cast<T>(2.0); }
    static constexpr T minus_one(){ return static_cast<T>(-1.0); }

    static constexpr T epsilon()  { return std::numeric_limits<T>::epsilon(); }
    static constexpr T epsilon2()  { return std::numeric_limits<T>::epsilon() * std::numeric_limits<T>::epsilon(); }
    static constexpr T infinity() { return std::numeric_limits<T>::infinity(); }

    // ---- 自然对数相关 ----
    /// 自然常数 e
    static constexpr T E()        { return static_cast<T>(2.71828182845904523536028747135266250); }
    /// log2(e)
    static constexpr T LOG2E()    { return static_cast<T>(1.44269504088896340735992468100189214); }
    /// log10(e)
    static constexpr T LOG10E()   { return static_cast<T>(0.43429448190325182765112891891660508); }
    /// ln(2)
    static constexpr T LN2()      { return static_cast<T>(0.69314718055994530941723212145817657); }
    /// ln(10)
    static constexpr T LN10()     { return static_cast<T>(2.30258509299404568401799145468436421); }

    // ---- 圆周率相关 ----
    /// π
    static constexpr T PI()       { return static_cast<T>(3.14159265358979323846264338327950288); }
    /// π / 2
    static constexpr T PI_2()     { return static_cast<T>(1.57079632679489661923132169163975144); }
    /// π / 4
    static constexpr T PI_4()     { return static_cast<T>(0.78539816339744830961566084581987572); }
    /// 1 / π
    static constexpr T ONE_OVER_PI()     { return static_cast<T>(0.31830988618379067153776675282489628); }
    /// 2 / π
    static constexpr T TWO_OVER_PI()     { return static_cast<T>(0.63661977236758134307553350564989316); }
    /// 2 / sqrt(π)
    static constexpr T TWO_OVER_SQRTPI(){ return static_cast<T>(1.12837916709551257389615890312154517); }
    /// 2π
    static constexpr T TAU()      { return PI() * static_cast<T>(2.0); }

    // ---- 根号相关 ----
    /// sqrt(2)
    static constexpr T SQRT2()    { return static_cast<T>(1.41421356237309504880168872420969808); }
    /// 1 / sqrt(2)  (= sqrt(2) / 2)
    static constexpr T SQRT1_2()  { return static_cast<T>(0.70710678118654752440084436210484904); }

    // ---- 角度换算 ----
    /// 角度 → 弧度系数 (π / 180)
    static constexpr T DEG_TO_RAD() { return PI() / static_cast<T>(180.0); }
    /// 弧度 → 角度系数 (180 / π)
    static constexpr T RAD_TO_DEG() { return static_cast<T>(180.0) / PI(); }
};

// 常用别名
using Numbersf = Numbers<float>;
using Numbersd = Numbers<double>;

} // namespace Math
} // namespace ZF
