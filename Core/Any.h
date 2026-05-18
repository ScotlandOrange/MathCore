#pragma once

/**
 * @file Any.h
 * @brief C++14 版本的 std::any 风格类型擦除值容器。
 */

#include <cstddef>
#include <initializer_list>
#include <type_traits>
#include <typeinfo>
#include <utility>

namespace ZF
{

/**
 * @brief C++14 版本的 std::in_place_type_t 替代类型。
 *
 * 与 std::any(std::in_place_type<T>, args...) 的用法一致，
 * 使用 ZF::in_place_type<T> 对 ZF::Any 进行原地构造。
 */
template<class T>
struct in_place_type_t
{
    explicit constexpr in_place_type_t() noexcept = default;
};

/**
 * @brief C++14 版本的 std::in_place_type 替代对象。
 */
template<class T>
constexpr in_place_type_t<T> in_place_type{};

/**
 * @brief 当按值返回的 ZF::any_cast 发生类型不匹配时抛出的异常。
 */
class bad_any_cast : public std::bad_cast
{
public:
    const char* what() const noexcept override
    {
        return "bad ZF::any_cast";
    }
};

/**
 * @brief C++14 版本的 std::any 风格容器，可保存一个任意可拷贝构造类型的值。
 *
 * ZF::Any 是本项目在 C++14 下对 std::any 的替代实现。它在 ZF 命名空间中提供
 * 同族公开接口：构造、赋值、原地构造、reset、swap、类型观察、any_cast、
 * make_any 和 bad_any_cast。
 *
 * 支持的基本用法：
 *
 * @code
 * ZF::Any value;
 * value = 42;
 *
 * if (value.has_value() && value.type() == typeid(int))
 * {
 *     int number = ZF::any_cast<int>(value);
 * }
 *
 * if (int* number = ZF::any_cast<int>(&value))
 * {
 *     *number = 7;
 * }
 *
 * value.emplace<std::string>(3u, 'x');
 * const std::string& text = ZF::any_cast<const std::string&>(value);
 *
 * ZF::Any vectorValue = ZF::make_any<std::vector<int>>({1, 2, 3});
 * ZF::Any inPlaceValue(ZF::in_place_type<std::string>, "label");
 *
 * value.reset();
 * @endcode
 *
 * 性能模型：
 *
 * - 能放入内联缓冲区且 nothrow move-constructible 的小对象会以内联方式存储，
 *   对齐常见 std::any 实现的小对象优化策略。
 * - 较大的对象，或不满足内联存储条件的对象，会存储在堆上。
 *
 * std::any 支持、但当前 C++14 替代版不支持或不保证完全一致的行为：
 *
 * - 它不与 std::any ABI 兼容，也不是同一个类型。请使用 ZF::Any 以及 ZF 命名空间
 *   下的接口族，而不是 std::any/std::any_cast/std::make_any。
 * - std::in_place_type_t 和 std::in_place_type 是 C++17 标准库名称。在 C++14
 *   代码中请使用 ZF::in_place_type_t 和 ZF::in_place_type。
 * - 非法模板调用不保证通过与 std::any 完全一致的 SFINAE 规则从重载集中移除。
 *   某些不支持的调用可能表现为 static_assert 或普通模板实例化错误。
 * - 不复制特定标准库厂商的实现细节：内联缓冲区大小、ABI 布局、诊断信息，以及
 *   所有边界情况下的异常保证，都可能与某个 std::any 实现不同。
 * - 不支持保存 move-only 类型。这与 std::any 对被保存值必须可拷贝构造的要求一致。
 */
class Any
{
public:
    Any() noexcept = default;

    Any(const Any& rhs)
    {
        if (rhs.has_value())
        {
            rhs.mCopy(&rhs, this);
        }
    }

    Any(Any&& rhs) noexcept
    {
        if (rhs.has_value())
        {
            rhs.mMove(&rhs, this);
        }
    }

    template<class T,
             class Value = typename std::decay<T>::type,
             typename std::enable_if<!std::is_same<Value, Any>::value, int>::type = 0>
    Any(T&& value)
    {
        emplace<Value>(std::forward<T>(value));
    }

    template<class ValueType, class... Args>
    explicit Any(in_place_type_t<ValueType>, Args&&... args)
    {
        emplace<ValueType>(std::forward<Args>(args)...);
    }

    template<class ValueType, class U, class... Args>
    explicit Any(in_place_type_t<ValueType>, std::initializer_list<U> values, Args&&... args)
    {
        emplace<ValueType>(values, std::forward<Args>(args)...);
    }

    ~Any()
    {
        reset();
    }

    Any& operator=(const Any& rhs)
    {
        if (this != &rhs)
        {
            Any copy(rhs);
            swap(copy);
        }

        return *this;
    }

    Any& operator=(Any&& rhs) noexcept
    {
        if (this != &rhs)
        {
            reset();
            if (rhs.has_value())
            {
                rhs.mMove(&rhs, this);
            }
        }

        return *this;
    }

    template<class T,
             class Value = typename std::decay<T>::type,
             typename std::enable_if<!std::is_same<Value, Any>::value, int>::type = 0>
    Any& operator=(T&& value)
    {
        emplace<Value>(std::forward<T>(value));
        return *this;
    }

    bool has_value() const noexcept
    {
        return mDestroy != nullptr;
    }

    void reset() noexcept
    {
        if (mDestroy)
        {
            mDestroy(this);
            clear();
        }
    }

    void swap(Any& rhs) noexcept
    {
        if (this == &rhs)
        {
            return;
        }

        Any tmp(std::move(rhs));
        rhs = std::move(*this);
        *this = std::move(tmp);
    }

    const std::type_info& type() const noexcept
    {
        return *mType;
    }

    template<class ValueType, class... Args>
    typename std::decay<ValueType>::type& emplace(Args&&... args)
    {
        using Value = typename std::decay<ValueType>::type;
        static_assert(std::is_copy_constructible<Value>::value, "ZF::Any requires copy-constructible values.");

        reset();

        if (UseLocalStorage<Value>::value)
        {
            new (&mStorage.mLocalValue) Value(std::forward<Args>(args)...);
        }
        else
        {
            mStorage.mHeapValue = new Value(std::forward<Args>(args)...);
        }

        install<Value>();
        return *access<Value>(this);
    }

    template<class ValueType, class U, class... Args>
    typename std::decay<ValueType>::type& emplace(std::initializer_list<U> values, Args&&... args)
    {
        using Value = typename std::decay<ValueType>::type;
        static_assert(std::is_copy_constructible<Value>::value, "ZF::Any requires copy-constructible values.");

        reset();

        if (UseLocalStorage<Value>::value)
        {
            new (&mStorage.mLocalValue) Value(values, std::forward<Args>(args)...);
        }
        else
        {
            mStorage.mHeapValue = new Value(values, std::forward<Args>(args)...);
        }

        install<Value>();
        return *access<Value>(this);
    }

private:
    static const std::size_t LocalStorageSize = sizeof(void*) * 3;
    static const std::size_t LocalStorageAlignment = alignof(void*);

    using LocalStorage = typename std::aligned_storage<LocalStorageSize, LocalStorageAlignment>::type;

    union Storage
    {
        Storage() noexcept :
            mHeapValue(nullptr)
        {
        }

        void* mHeapValue;
        LocalStorage mLocalValue;
    };

    template<class T>
    struct UseLocalStorage : std::integral_constant<bool,
                                                    (sizeof(T) <= LocalStorageSize) &&
                                                    (alignof(T) <= LocalStorageAlignment) &&
                                                    std::is_nothrow_move_constructible<T>::value>
    {
    };

    using DestroyFn = void (*)(Any*);
    using CopyFn = void (*)(const Any*, Any*);
    using MoveFn = void (*)(Any*, Any*);

    void clear() noexcept
    {
        mStorage.mHeapValue = nullptr;
        mType = &typeid(void);
        mDestroy = nullptr;
        mCopy = nullptr;
        mMove = nullptr;
    }

    template<class T>
    void install() noexcept
    {
        mType = &typeid(T);
        mDestroy = &destroy<T>;
        mCopy = &copy<T>;
        mMove = &move<T>;
    }

    template<class T>
    bool isType() const noexcept
    {
        return type() == typeid(T);
    }

    template<class T>
    static T* access(Any* any) noexcept
    {
        return UseLocalStorage<T>::value ?
               reinterpret_cast<T*>(&any->mStorage.mLocalValue) :
               static_cast<T*>(any->mStorage.mHeapValue);
    }

    template<class T>
    static const T* access(const Any* any) noexcept
    {
        return UseLocalStorage<T>::value ?
               reinterpret_cast<const T*>(&any->mStorage.mLocalValue) :
               static_cast<const T*>(any->mStorage.mHeapValue);
    }

    template<class T>
    static void destroy(Any* any) noexcept
    {
        if (UseLocalStorage<T>::value)
        {
            access<T>(any)->~T();
        }
        else
        {
            delete access<T>(any);
        }
    }

    template<class T>
    static void copy(const Any* source, Any* destination)
    {
        destination->emplace<T>(*access<T>(source));
    }

    template<class T>
    static void move(Any* source, Any* destination) noexcept
    {
        if (UseLocalStorage<T>::value)
        {
            new (&destination->mStorage.mLocalValue) T(std::move(*access<T>(source)));
            destination->install<T>();
            source->reset();
        }
        else
        {
            destination->mStorage.mHeapValue = source->mStorage.mHeapValue;
            destination->install<T>();
            source->clear();
        }
    }

    Storage mStorage;
    const std::type_info* mType = &typeid(void);
    DestroyFn mDestroy = nullptr;
    CopyFn mCopy = nullptr;
    MoveFn mMove = nullptr;

    template<class T>
    friend T* any_cast(Any* any) noexcept;

    template<class T>
    friend const T* any_cast(const Any* any) noexcept;
};

/**
 * @brief 与 std::any_cast<T>(any*) 对应的指针版本 any_cast。
 *
 * @return 当保存类型与 T 精确匹配时返回指向保存值的指针，否则返回 nullptr。
 */
template<class T>
T* any_cast(Any* any) noexcept
{
    using Value = typename std::remove_cv<T>::type;
    if (!any)
    {
        return nullptr;
    }

    return any->isType<Value>() ? static_cast<T*>(Any::access<Value>(any)) : nullptr;
}

/**
 * @brief 与 std::any_cast<T>(const any*) 对应的 const 指针版本 any_cast。
 *
 * @return 当保存类型与 T 精确匹配时返回指向保存值的指针，否则返回 nullptr。
 */
template<class T>
const T* any_cast(const Any* any) noexcept
{
    using Value = typename std::remove_cv<T>::type;
    if (!any)
    {
        return nullptr;
    }

    return any->isType<Value>() ? static_cast<const T*>(Any::access<Value>(any)) : nullptr;
}

/**
 * @brief 与 std::any_cast<T>(any&) 对应的值/引用版本 any_cast。
 *
 * @throws ZF::bad_any_cast 当保存类型与 T 不匹配时抛出。
 */
template<class T>
T any_cast(Any& any)
{
    using Value = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
    Value* value = any_cast<Value>(&any);
    if (!value)
    {
        throw bad_any_cast();
    }

    return static_cast<T>(*value);
}

/**
 * @brief 与 std::any_cast<T>(const any&) 对应的值/引用版本 any_cast。
 *
 * @throws ZF::bad_any_cast 当保存类型与 T 不匹配时抛出。
 */
template<class T>
T any_cast(const Any& any)
{
    using Value = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
    const Value* value = any_cast<Value>(&any);
    if (!value)
    {
        throw bad_any_cast();
    }

    return static_cast<T>(*value);
}

/**
 * @brief 与 std::any_cast<T>(any&&) 对应的值/引用版本 any_cast。
 *
 * @throws ZF::bad_any_cast 当保存类型与 T 不匹配时抛出。
 */
template<class T>
T any_cast(Any&& any)
{
    using Value = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
    Value* value = any_cast<Value>(&any);
    if (!value)
    {
        throw bad_any_cast();
    }

    return static_cast<T>(std::move(*value));
}

/**
 * @brief 构造一个保存 T 的 ZF::Any，对应 std::make_any<T>(args...)。
 */
template<class T, class... Args>
Any make_any(Args&&... args)
{
    Any any;
    any.emplace<T>(std::forward<Args>(args)...);
    return any;
}

/**
 * @brief 使用 initializer_list 构造一个保存 T 的 ZF::Any。
 */
template<class T, class U, class... Args>
Any make_any(std::initializer_list<U> values, Args&&... args)
{
    Any any;
    any.emplace<T>(values, std::forward<Args>(args)...);
    return any;
}

/**
 * @brief 用于 ADL 的非成员 swap 重载，对应 std::swap(any&, any&) 的用法。
 */
inline void swap(Any& lhs, Any& rhs) noexcept
{
    lhs.swap(rhs);
}

} // namespace ZF
