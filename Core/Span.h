#pragma once

#include "Types.h"
#include "TypeTraits.h"

#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ZF
{

/**
 * @brief C++14 环境下的连续内存非拥有视图，接口形状接近 std::span<T>。
 *
 * ZF::span 只保存首元素指针和元素数量，不分配、不复制、不拥有元素。它适合用作函数参数，
 * 用一个接口统一接收 C 数组、std::vector、std::array、单个元素和只读 initializer_list。
 *
 * 生命周期约束：
 * - 由数组、容器或左值元素构造时，调用者必须保证底层对象在 span 使用期间仍然有效。
 * - 由函数返回值、字面量或 initializer_list 构造时，只适合在同一个完整表达式中立即传入函数；
 *   不要保存到变量、成员或异步任务中。
 * - 只读批量参数优先使用 ZF::span<const T>，这样能接收可变容器、const 容器和临时只读序列。
 *
 * 常见用法：
 *
 * @code
 * void setVisible(bool visible, ZF::span<const int> widgetIndexes);
 *
 * std::vector<int> ids = {0, 1, 2};
 * setVisible(true, ids);          // 容器视图
 *
 * int id = 3;
 * setVisible(true, id);           // 单个左值元素
 *
 * int getId();
 * setVisible(true, getId());      // 函数返回值，立即调用场景
 *
 * setVisible(true, {0, 2, 4});    // initializer_list，立即调用场景
 * @endcode
 *
 * @code
 * int values[] = {1, 2, 3, 4};
 * ZF::span<int> view(values);
 * view[1] = 7;
 *
 * ZF::span<int> middle = view.subspan(1, 2);
 * ZF::span<const ZF::byte> bytes = view.as_bytes();
 * @endcode
 *
 * @tparam T 元素类型，可以带 const 限定，例如 ZF::span<const int>。
 */
template <class T>
class span
{
    template <class C>
    static constexpr auto container_size(const C& c) -> decltype(c.size())
    {
        return c.size();
    }

    template <class C>
    static constexpr auto container_data(C& c) -> decltype(c.data())
    {
        return c.data();
    }

    template <class C>
    static constexpr auto container_data(const C& c) -> decltype(c.data())
    {
        return c.data();
    }

    template <class U>
    static std::false_type is_span_type(const U*);

    template <class U>
    static std::true_type is_span_type(const span<U>*);

    template <class C, class = void>
    struct has_size_and_data : std::false_type
    {
    };

    template <class C>
    struct has_size_and_data<C,
                             void_t<decltype(container_size(std::declval<C>())),
                                    decltype(container_data(std::declval<C>()))>> : std::true_type
    {
    };

    template <class C, class U = uncvref_t<C>>
    struct is_container
    {
        static constexpr bool value =
            !decltype(is_span_type(static_cast<U*>(nullptr)))::value &&
            !std::is_array<U>::value &&
            has_size_and_data<C>::value;
    };

    template <class, class, class = void>
    struct is_container_element_compatible : std::false_type
    {
    };

    template <class C, class E>
    struct is_container_element_compatible<
        C,
        E,
        typename std::enable_if<
            !std::is_same<
                typename std::remove_cv<remove_pointer_t<decltype(container_data(std::declval<C>()))>>::type,
                void>::value &&
            std::is_convertible<
                remove_pointer_t<decltype(container_data(std::declval<C>()))> (*)[],
                E (*)[]>::value>::type> : std::true_type
    {
    };

    template <class C>
    using require_container = enable_if_t<
        is_container<C>::value &&
        is_container_element_compatible<C, T>::value>;

    template <class OtherT>
    using require_compatible_element = enable_if_t<
        !std::is_same<OtherT, T>::value &&
        std::is_convertible<OtherT (*)[], T (*)[]>::value>;

    template <class E>
    using require_mutable = enable_if_t<!std::is_const<E>::value>;

    template <class E>
    using require_initializer_list = enable_if_t<
        std::is_const<E>::value &&
        std::is_convertible<const typename std::remove_const<E>::type (*)[], E (*)[]>::value>;

    template <class U>
    using require_single_mutable_element = enable_if_t<
        !std::is_const<U>::value &&
        !std::is_array<U>::value &&
        std::is_convertible<typename std::add_pointer<U>::type, T*>::value>;

    template <class U>
    using require_single_const_element = enable_if_t<
        std::is_const<T>::value &&
        !std::is_array<U>::value &&
        std::is_convertible<typename std::add_pointer<const U>::type, T*>::value>;

    static_assert(std::is_object<T>::value, "T must be an object type");
    static_assert(is_complete<T>::value, "T must be a complete type");
    static_assert(!std::is_abstract<T>::value, "T cannot be an abstract class type");

public:
    /// @name Type aliases
    /// @{
    using element_type = T;
    using value_type = typename std::remove_cv<T>::type;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using pointer = element_type*;
    using const_pointer = const element_type*;
    using reference = element_type&;
    using const_reference = const element_type&;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    /// @}

    /// @name Constructors
    /// @{
    constexpr span() noexcept = default;

    constexpr span(pointer ptr, size_type count) noexcept :
        mPtr(ptr),
        mSize(count)
    {
    }

    constexpr span(pointer firstElem, pointer lastElem) noexcept :
        mPtr(firstElem),
        mSize(static_cast<size_type>(lastElem - firstElem))
    {
    }

    /**
     * @brief 从单个可变左值元素构造单元素 span。
     */
    template <class U, require_single_mutable_element<U> = 0>
    constexpr span(U& value) noexcept :
        mPtr(&value),
        mSize(1)
    {
    }

    /**
     * @brief 从单个只读元素构造单元素 span；可用于立即传入函数的返回值。
     */
    template <class U, require_single_const_element<U> = 0>
    constexpr span(const U& value) noexcept :
        mPtr(&value),
        mSize(1)
    {
    }

    /**
     * @brief 从 C 数组构造 span。
     */
    template <std::size_t N>
    constexpr span(element_type (&array)[N]) noexcept :
        mPtr(array),
        mSize(N)
    {
    }

    /**
     * @brief 从提供 data() 和 size() 的连续容器构造 span。
     */
    template <class Container, require_container<Container&> = 0>
    constexpr span(Container& cont) noexcept :
        mPtr(container_data(cont)),
        mSize(container_size(cont))
    {
    }

    /**
     * @brief 从 initializer_list 构造只读 span，仅适合立即调用场景。
     */
    template <class E = T, require_initializer_list<E> = 0>
    constexpr span(std::initializer_list<value_type> values) noexcept :
        mPtr(values.begin()),
        mSize(values.size())
    {
    }

    constexpr span(const span& other) noexcept = default;

    template <class OtherT, require_compatible_element<OtherT> = 0>
    constexpr span(const span<OtherT>& other) noexcept :
        mPtr(other.data()),
        mSize(other.size())
    {
    }

    ~span() noexcept = default;

    constexpr span& operator=(const span& other) noexcept = default;
    /// @}

    /// @name Subviews
    /// @{
    constexpr span first(size_type count) const
    {
        assert(count <= size() && "span::first(count): count out of range");
        return {data(), count};
    }

    constexpr span last(size_type count) const
    {
        assert(count <= size() && "span::last(count): count out of range");
        return {data() + (size() - count), count};
    }

    static constexpr size_type npos = static_cast<size_type>(-1);

    constexpr span subspan(size_type offset, size_type count = npos) const
    {
        assert(offset <= size() &&
               (count == npos || offset + count <= size()) &&
               "span::subspan(offset, count): out of range");

        return {data() + offset, count == npos ? size() - offset : count};
    }
    /// @}

    /// @name Observers
    /// @{
    constexpr size_type size() const noexcept
    {
        return mSize;
    }

    constexpr size_type size_bytes() const noexcept
    {
        return size() * sizeof(element_type);
    }

    constexpr difference_type ssize() const noexcept
    {
        return static_cast<difference_type>(size());
    }

    constexpr bool empty() const noexcept
    {
        return size() == 0;
    }
    /// @}

    /// @name Element access
    /// @{
    constexpr reference operator[](size_type idx) const
    {
        assert(idx < size() && "span::operator[]: index out of range");
        return *(data() + idx);
    }

    reference at(size_type idx) const
    {
        if (idx >= size())
        {
            throw std::out_of_range("span::at(): index out of range");
        }

        return *(data() + idx);
    }

    constexpr reference front() const
    {
        assert(!empty() && "span::front(): span is empty");
        return *data();
    }

    constexpr reference back() const
    {
        assert(!empty() && "span::back(): span is empty");
        return *(data() + (size() - 1));
    }

    constexpr pointer data() const noexcept
    {
        return mPtr;
    }
    /// @}

    /// @name Iterators
    /// @{
    constexpr iterator begin() const noexcept
    {
        return data();
    }

    constexpr iterator end() const noexcept
    {
        return empty() ? data() : data() + size();
    }

    constexpr reverse_iterator rbegin() const noexcept
    {
        return reverse_iterator(end());
    }

    constexpr reverse_iterator rend() const noexcept
    {
        return reverse_iterator(begin());
    }

    constexpr const_iterator cbegin() const noexcept
    {
        return data();
    }

    constexpr const_iterator cend() const noexcept
    {
        return empty() ? data() : data() + size();
    }

    constexpr const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(cend());
    }

    constexpr const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(cbegin());
    }
    /// @}

    /// @name Byte views
    /// @{
    span<const byte> as_bytes() const noexcept
    {
        return {reinterpret_cast<const byte*>(data()), size_bytes()};
    }

    template <class E = T, require_mutable<E> = 0>
    span<byte> as_writable_bytes() const noexcept
    {
        return {reinterpret_cast<byte*>(data()), size_bytes()};
    }
    /// @}

private:
    pointer mPtr = nullptr;
    size_type mSize = 0;
};

} // namespace ZF
