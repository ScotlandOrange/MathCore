#include <gtest/gtest.h>

#include <Core/Any.h>

#include <atomic>
#include <cstdlib>
#include <new>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

namespace
{

std::atomic<bool> gTrackAllocations(false);
std::atomic<int> gAllocationCount(0);

class AllocationScope
{
public:
    AllocationScope()
    {
        gAllocationCount.store(0);
        gTrackAllocations.store(true);
    }

    ~AllocationScope()
    {
        gTrackAllocations.store(false);
    }

    int count() const
    {
        return gAllocationCount.load();
    }
};

struct SmallNoThrowValue
{
    int mValue = 0;

    explicit SmallNoThrowValue(int value = 0) :
        mValue(value)
    {
    }

    SmallNoThrowValue(const SmallNoThrowValue&) = default;
    SmallNoThrowValue(SmallNoThrowValue&&) noexcept = default;
    SmallNoThrowValue& operator=(const SmallNoThrowValue&) = default;
    SmallNoThrowValue& operator=(SmallNoThrowValue&&) noexcept = default;
};

struct LargeValue
{
    char mBytes[128] = {};
    int mValue = 0;

    explicit LargeValue(int value = 0) :
        mValue(value)
    {
    }
};

template<class T>
class HasEmpty
{
    template<class U>
    static auto test(int) -> decltype(std::declval<const U&>().empty(), std::true_type());

    template<class>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template<class T>
class HasIs
{
    template<class U>
    static auto test(int) -> decltype(std::declval<const U&>().template is<int>(), std::true_type());

    template<class>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template<class T>
class HasGet
{
    template<class U>
    static auto test(int) -> decltype(std::declval<U&>().template get<int>(), std::true_type());

    template<class>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template<class T>
class HasSet
{
    template<class U>
    static auto test(int) -> decltype(std::declval<U&>().set(1), std::true_type());

    template<class>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

template<class T>
class HasValueMethod
{
    template<class U>
    static auto test(int) -> decltype(std::declval<const U&>().value(std::declval<int&>()), std::true_type());

    template<class>
    static std::false_type test(...);

public:
    static const bool value = decltype(test<T>(0))::value;
};

static_assert(!HasEmpty<ZF::Any>::value, "ZF::Any should not expose non-std empty().");
static_assert(!HasIs<ZF::Any>::value, "ZF::Any should not expose non-std is<T>().");
static_assert(!HasGet<ZF::Any>::value, "ZF::Any should not expose non-std get<T>().");
static_assert(!HasSet<ZF::Any>::value, "ZF::Any should not expose non-std set().");
static_assert(!HasValueMethod<ZF::Any>::value, "ZF::Any should not expose non-std value(T&).");

} // namespace

void* operator new(std::size_t size)
{
    if (gTrackAllocations.load())
    {
        gAllocationCount.fetch_add(1);
    }

    if (void* ptr = std::malloc(size))
    {
        return ptr;
    }

    throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept
{
    std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept
{
    std::free(ptr);
}

TEST(Any, DefaultConstructedMatchesStdAnyObservers)
{
    const ZF::Any any;

    EXPECT_FALSE(any.has_value());
    EXPECT_EQ(any.type(), typeid(void));
    EXPECT_EQ(ZF::any_cast<int>(&any), nullptr);
}

TEST(Any, AssignmentAndAnyCastMatchStdAnySemantics)
{
    ZF::Any any = 42;

    EXPECT_TRUE(any.has_value());
    EXPECT_EQ(any.type(), typeid(int));
    EXPECT_EQ(ZF::any_cast<int>(any), 42);
    EXPECT_EQ(ZF::any_cast<const int&>(static_cast<const ZF::Any&>(any)), 42);
    EXPECT_EQ(ZF::any_cast<double>(&any), nullptr);
    EXPECT_THROW(ZF::any_cast<double>(any), ZF::bad_any_cast);

    ZF::any_cast<int&>(any) = 7;

    EXPECT_EQ(ZF::any_cast<int>(any), 7);

    any = std::string("hello");

    EXPECT_EQ(any.type(), typeid(std::string));
    EXPECT_EQ(ZF::any_cast<std::string>(any), "hello");
}

TEST(Any, CopyMoveResetAndSwapMatchStdAnyShape)
{
    ZF::Any original = std::string("source");
    ZF::Any copy = original;

    ZF::any_cast<std::string&>(copy) = "copy";

    EXPECT_EQ(ZF::any_cast<std::string>(original), "source");
    EXPECT_EQ(ZF::any_cast<std::string>(copy), "copy");

    ZF::Any moved(std::move(original));

    EXPECT_TRUE(moved.has_value());
    EXPECT_EQ(ZF::any_cast<std::string>(moved), "source");

    ZF::Any number = 42;
    ZF::swap(copy, number);

    EXPECT_EQ(ZF::any_cast<int>(copy), 42);
    EXPECT_EQ(ZF::any_cast<std::string>(number), "copy");

    copy.reset();

    EXPECT_FALSE(copy.has_value());
    EXPECT_EQ(copy.type(), typeid(void));
}

TEST(Any, InPlaceConstructorsEmplaceAndMakeAnyMatchStdAny)
{
    ZF::Any text(ZF::in_place_type<std::string>, 3u, 'x');

    EXPECT_EQ(ZF::any_cast<std::string>(text), "xxx");

    std::vector<int>& values = text.emplace<std::vector<int>>({1, 2, 3});

    ASSERT_EQ(values.size(), 3u);
    EXPECT_EQ(values[0], 1);
    EXPECT_EQ(values[2], 3);

    ZF::Any made = ZF::make_any<std::vector<int>>({4, 5, 6});
    const auto& madeValues = ZF::any_cast<const std::vector<int>&>(made);

    ASSERT_EQ(madeValues.size(), 3u);
    EXPECT_EQ(madeValues[1], 5);
}

TEST(Any, SmallNoThrowMovableValuesUseInlineStorage)
{
    int allocations = 0;
    int value = 0;

    {
        AllocationScope scope;

        ZF::Any any;
        SmallNoThrowValue& stored = any.emplace<SmallNoThrowValue>(7);
        const int storedValue = stored.mValue;
        ZF::Any copy(any);
        ZF::Any moved(std::move(any));
        ZF::Any assigned;
        assigned = copy;

        value = storedValue + ZF::any_cast<SmallNoThrowValue>(copy).mValue +
                ZF::any_cast<SmallNoThrowValue>(moved).mValue +
                ZF::any_cast<SmallNoThrowValue>(assigned).mValue;
        allocations = scope.count();
    }

    EXPECT_EQ(value, 28);
    EXPECT_EQ(allocations, 0);
}

TEST(Any, LargeValuesUseHeapStorageButRemainTypeSafe)
{
    int allocations = 0;
    int value = 0;

    {
        AllocationScope scope;

        ZF::Any any;
        any.emplace<LargeValue>(9);

        value = ZF::any_cast<LargeValue>(any).mValue;
        allocations = scope.count();
    }

    EXPECT_EQ(value, 9);
    EXPECT_GE(allocations, 1);
}
