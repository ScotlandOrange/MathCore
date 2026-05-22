#include <gtest/gtest.h>

#include <Core/Span.h>

#include <array>
#include <cstddef>
#include <initializer_list>
#include <type_traits>
#include <vector>

namespace
{

void expectConstSpanValues(ZF::span<const int> values)
{
    ASSERT_EQ(values.size(), 3u);
    EXPECT_EQ(values[0], 1);
    EXPECT_EQ(values[1], 2);
    EXPECT_EQ(values[2], 3);
}

void expectSingleConstSpanValue(ZF::span<const int> value)
{
    ASSERT_EQ(value.size(), 1u);
    EXPECT_EQ(value[0], 42);
}

int makeSingleSpanValue()
{
    return 42;
}

} // namespace

TEST(Span, DefaultConstructedSpanIsEmpty)
{
    constexpr ZF::span<int> values;

    EXPECT_EQ(values.data(), nullptr);
    EXPECT_EQ(values.size(), 0u);
    EXPECT_TRUE(values.empty());
    EXPECT_EQ(values.begin(), values.end());
}

TEST(Span, ViewsPointerAndCountWithoutOwningStorage)
{
    int values[] = {1, 2, 3};
    ZF::span<int> view(values, 3);

    static_assert(std::is_same<ZF::span<int>::element_type, int>::value, "element type mismatch");
    static_assert(std::is_same<ZF::span<int>::value_type, int>::value, "value type mismatch");

    EXPECT_EQ(view.data(), values);
    EXPECT_EQ(view.size(), 3u);
    EXPECT_EQ(view.size_bytes(), 3u * sizeof(int));
    EXPECT_EQ(view.ssize(), 3);
    EXPECT_EQ(view.front(), 1);
    EXPECT_EQ(view.back(), 3);

    view[1] = 7;
    EXPECT_EQ(values[1], 7);
}

TEST(Span, ConstructsFromPointerPairAndArray)
{
    int values[] = {1, 2, 3, 4};

    ZF::span<int> middle(values + 1, values + 3);
    EXPECT_EQ(middle.size(), 2u);
    EXPECT_EQ(middle[0], 2);
    EXPECT_EQ(middle[1], 3);

    ZF::span<int> all(values);
    EXPECT_EQ(all.size(), 4u);
    EXPECT_EQ(all.back(), 4);
}

TEST(Span, ConstructsFromSingleMutableElement)
{
    int value = 42;
    ZF::span<int> view(value);

    EXPECT_EQ(view.data(), &value);
    EXPECT_EQ(view.size(), 1u);
    EXPECT_EQ(view.front(), 42);

    view[0] = 7;
    EXPECT_EQ(value, 7);
}

TEST(Span, ConstructsConstSpanFromSingleElementLvalues)
{
    int mutableValue = 42;
    const int constValue = 42;

    ZF::span<const int> mutableView(mutableValue);
    ZF::span<const int> constView(constValue);

    EXPECT_EQ(mutableView.data(), &mutableValue);
    EXPECT_EQ(mutableView.size(), 1u);
    EXPECT_EQ(mutableView[0], 42);

    EXPECT_EQ(constView.data(), &constValue);
    EXPECT_EQ(constView.size(), 1u);
    EXPECT_EQ(constView[0], 42);

    expectSingleConstSpanValue(mutableValue);
    expectSingleConstSpanValue(constValue);
}

TEST(Span, SingleElementSpanRejectsConstToMutable)
{
    static_assert(!std::is_constructible<ZF::span<int>, const int&>::value,
                  "span<int> should not accept a const int lvalue");
}

TEST(Span, ConstSingleElementSpanAcceptsFunctionReturnValueForFunctionArgument)
{
    static_assert(std::is_constructible<ZF::span<const int>, int&&>::value,
                  "span<const int> should accept an int return value for immediate calls");

    expectSingleConstSpanValue(makeSingleSpanValue());
}

TEST(Span, ConstructsFromContiguousContainers)
{
    std::vector<int> values = {1, 2, 3};
    ZF::span<int> mutableView(values);

    mutableView[2] = 9;
    EXPECT_EQ(values[2], 9);

    const std::vector<int>& constValues = values;
    ZF::span<const int> constView(constValues);

    EXPECT_EQ(constView.size(), 3u);
    EXPECT_EQ(constView[2], 9);

    std::array<int, 3> arrayValues = {{4, 5, 6}};
    ZF::span<int> arrayView(arrayValues);

    EXPECT_EQ(arrayView.size(), 3u);
    EXPECT_EQ(arrayView[1], 5);
}

TEST(Span, ConvertsMutableSpanToConstSpan)
{
    int values[] = {1, 2, 3};
    ZF::span<int> mutableView(values);
    ZF::span<const int> constView(mutableView);

    expectConstSpanValues(constView);
}

TEST(Span, AllowsInitializerListForConstElementSpans)
{
    static_assert(std::is_constructible<ZF::span<const int>, std::initializer_list<int>>::value,
                  "span<const int> should accept initializer_list<int>");
    static_assert(!std::is_constructible<ZF::span<int>, std::initializer_list<int>>::value,
                  "span<int> should not accept initializer_list<int>");

    expectConstSpanValues({1, 2, 3});
}

TEST(Span, CreatesSubViews)
{
    int values[] = {1, 2, 3, 4, 5};
    ZF::span<int> view(values);

    ZF::span<int> first = view.first(2);
    EXPECT_EQ(first.size(), 2u);
    EXPECT_EQ(first[0], 1);
    EXPECT_EQ(first[1], 2);

    ZF::span<int> last = view.last(2);
    EXPECT_EQ(last.size(), 2u);
    EXPECT_EQ(last[0], 4);
    EXPECT_EQ(last[1], 5);

    ZF::span<int> middle = view.subspan(1, 3);
    EXPECT_EQ(middle.size(), 3u);
    EXPECT_EQ(middle[0], 2);
    EXPECT_EQ(middle[2], 4);

    ZF::span<int> tail = view.subspan(3);
    EXPECT_EQ(tail.size(), 2u);
    EXPECT_EQ(tail[0], 4);
}

TEST(Span, ProvidesByteViews)
{
    int values[] = {1, 2};
    ZF::span<int> view(values);

    ZF::span<const ZF::byte> bytes = view.as_bytes();
    EXPECT_EQ(bytes.size(), sizeof(values));

    ZF::span<ZF::byte> writableBytes = view.as_writable_bytes();
    EXPECT_EQ(writableBytes.size(), sizeof(values));

    writableBytes[0] = static_cast<ZF::byte>(0);
}
