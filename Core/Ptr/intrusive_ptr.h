#pragma once

#include "Core/Macros.h"
#include "Core/Ptr/intrusive_refcount.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <exception>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace ZF
{

/**
 * By default we use object internal reference counting. We will call
 * pObj->add_ref and pObj->release. The pointer will call
 * class functions to do the reference counting. There is
 * no need for ADL and you can implement your functions as you wish.
 *
 * If you want external reference count (boost style), which is useful
 * for structs or retrofitting in existing code (you don't want to change).
 * To do so specialize IntrusivePtrCountPolicy for your object type.
 * Example:
 *   template<>
 *   struct IntrusivePtrCountPolicy<MyObject> {
 *       static ZF_FORCE_INLINE void add_ref(MyObject* p) { your code to add a reference }
 *       static ZF_FORCE_INLINE void release(MyObject* p) { your code to remove a reference }
 *   }
 */
template<class T>
struct IntrusivePtrCountPolicy
{
    static ZF_FORCE_INLINE void add_ref(T* p)
    {
        p->add_ref();
    }

    static ZF_FORCE_INLINE void release(T* p)
    {
        p->release();
    }

    static ZF_FORCE_INLINE void add_weak(T* p)
    {
        p->add_weak();
    }

    static ZF_FORCE_INLINE void release_weak(T* p)
    {
        p->release_weak();
    }
};

template<class T>
class intrusive_weak_ptr;

/**
 * intrusive_ptr
 *
 * A smart pointer that uses intrusive reference counting.
 *
 * This pointer is not part of the C++ standard yet. It is considered ZFGrid
 * extension. It extends the boost implementation.
 *
 * There are many benefits in using intrusive reference counting.
 * They are the recommended pointer in performance critical systems. The
 * reason for that is you can control: no allocation occurs (counter is internal),
 * you control the size of ref count, you know if it needs to be atomic or not,
 * better cache coherency, you can convert back and forth to raw pointer, etc.
 *
 * For all other cases shared_ptr is recommended. In shared pointer if you use
 * make_shared/allocate_shared you will save the second allocation too.
 */
template<class T>
class intrusive_ptr
{
private:
    template<typename U>
    friend class intrusive_ptr;

    template<typename U>
    friend class intrusive_weak_ptr;

    typedef intrusive_ptr this_type;
    typedef IntrusivePtrCountPolicy<T> CountPolicy;

    struct already_referenced_t
    {
    };

    intrusive_ptr(T* p, already_referenced_t) :
        px(p)
    {
    }

public:
    typedef T element_type;
    typedef T value_type;

    intrusive_ptr() :
        px(nullptr)
    {
    }

    intrusive_ptr(T* p) :
        px(p)
    {
        if (px != nullptr)
        {
            CountPolicy::add_ref(px);
        }
    }

    template<class U, std::enable_if_t<std::is_convertible<U*, T*>::value, int> = 0>
    intrusive_ptr(intrusive_ptr<U> const& rhs) :
        px(rhs.get())
    {
        if (px != nullptr)
        {
            CountPolicy::add_ref(px);
        }
    }

    intrusive_ptr(intrusive_ptr const& rhs) :
        px(rhs.px)
    {
        if (px != nullptr)
        {
            CountPolicy::add_ref(px);
        }
    }

    ~intrusive_ptr()
    {
        if (px != nullptr)
        {
            CountPolicy::release(px);
        }
    }

    template<class U>
    std::enable_if_t<std::is_convertible<U*, T*>::value, intrusive_ptr&> operator=(intrusive_ptr<U> const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    template<class U>
    intrusive_ptr(intrusive_ptr<U>&& rhs, std::enable_if_t<std::is_convertible<U*, T*>::value, int> = 0) :
        px(rhs.get())
    {
        rhs.px = nullptr;
    }

    intrusive_ptr(intrusive_ptr&& rhs) :
        px(rhs.px)
    {
        rhs.px = nullptr;
    }

    template<class U>
    std::enable_if_t<std::is_convertible<U*, T*>::value, intrusive_ptr&> operator=(intrusive_ptr<U>&& rhs)
    {
        this_type(std::move(rhs)).swap(*this);
        return *this;
    }

    intrusive_ptr& operator=(intrusive_ptr&& rhs)
    {
        this_type(static_cast<intrusive_ptr&&>(rhs)).swap(*this);
        return *this;
    }

    intrusive_ptr& operator=(intrusive_ptr const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    intrusive_ptr& operator=(T* rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    void reset()
    {
        this_type().swap(*this);
    }

    void reset(T* rhs)
    {
        this_type(rhs).swap(*this);
    }

    T* get() const
    {
        return px;
    }

    T& operator*() const
    {
        assert(px != nullptr); // You can't dereference a null pointer.
        return *px;
    }

    T* operator->() const
    {
        assert(px != nullptr); // You can't dereference a null pointer.
        return px;
    }

    typedef T* this_type::*unspecified_bool_type;
    operator unspecified_bool_type() const
    {
        return px == nullptr ? nullptr : &this_type::px;
    } // never throws

    bool operator!() const
    {
        return px == nullptr;
    } // never throws

    void swap(intrusive_ptr& rhs)
    {
        T* tmp = px;
        px = rhs.px;
        rhs.px = tmp;
    }

private:
    T* px;
};

template<class T>
class intrusive_weak_ptr
{
private:
    template<typename U>
    friend class intrusive_weak_ptr;

    typedef intrusive_weak_ptr this_type;
    typedef IntrusivePtrCountPolicy<T> CountPolicy;

public:
    typedef T element_type;
    typedef T value_type;

    intrusive_weak_ptr() :
        px(nullptr),
        control(nullptr)
    {
    }

    intrusive_weak_ptr(intrusive_weak_ptr const& rhs) :
        px(rhs.px),
        control(rhs.control)
    {
        if (control) control->add_weak();
    }

    template<class U, std::enable_if_t<std::is_convertible<U*, T*>::value, int> = 0>
    intrusive_weak_ptr(intrusive_weak_ptr<U> const& rhs) :
        px(rhs.px),
        control(rhs.control)
    {
        if (control) control->add_weak();
    }

    intrusive_weak_ptr(intrusive_ptr<T> const& rhs) :
        px(rhs.get()),
        control(px ? px->control_block() : nullptr)
    {
        if (px) CountPolicy::add_weak(px);
    }

    template<class U, std::enable_if_t<std::is_convertible<U*, T*>::value, int> = 0>
    intrusive_weak_ptr(intrusive_ptr<U> const& rhs) :
        px(rhs.get()),
        control(px ? px->control_block() : nullptr)
    {
        if (px) CountPolicy::add_weak(px);
    }

    intrusive_weak_ptr(intrusive_weak_ptr&& rhs) noexcept :
        px(rhs.px),
        control(rhs.control)
    {
        rhs.px = nullptr;
        rhs.control = nullptr;
    }

    template<class U, std::enable_if_t<std::is_convertible<U*, T*>::value, int> = 0>
    intrusive_weak_ptr(intrusive_weak_ptr<U>&& rhs) noexcept :
        px(rhs.px),
        control(rhs.control)
    {
        rhs.px = nullptr;
        rhs.control = nullptr;
    }

    ~intrusive_weak_ptr()
    {
        detail::release_weak_reference(control);
    }

    intrusive_weak_ptr& operator=(intrusive_weak_ptr const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    template<class U>
    std::enable_if_t<std::is_convertible<U*, T*>::value, intrusive_weak_ptr&> operator=(intrusive_weak_ptr<U> const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    intrusive_weak_ptr& operator=(intrusive_weak_ptr&& rhs) noexcept
    {
        this_type(std::move(rhs)).swap(*this);
        return *this;
    }

    template<class U>
    std::enable_if_t<std::is_convertible<U*, T*>::value, intrusive_weak_ptr&> operator=(intrusive_weak_ptr<U>&& rhs)
    {
        this_type(std::move(rhs)).swap(*this);
        return *this;
    }

    intrusive_weak_ptr& operator=(intrusive_ptr<T> const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    template<class U>
    std::enable_if_t<std::is_convertible<U*, T*>::value, intrusive_weak_ptr&> operator=(intrusive_ptr<U> const& rhs)
    {
        this_type(rhs).swap(*this);
        return *this;
    }

    void reset()
    {
        this_type().swap(*this);
    }

    void swap(intrusive_weak_ptr& rhs) noexcept
    {
        T* tmp_px = px;
        px = rhs.px;
        rhs.px = tmp_px;

        intrusive_control_block* tmp_control = control;
        control = rhs.control;
        rhs.control = tmp_control;
    }

    bool expired() const
    {
        return !control || control->strong_count() == 0;
    }

    uint32_t use_count() const
    {
        return control ? control->strong_count() : 0u;
    }

    intrusive_ptr<T> lock() const
    {
        if (!control || !control->try_add_strong())
        {
            return intrusive_ptr<T>();
        }

        return intrusive_ptr<T>(px, typename intrusive_ptr<T>::already_referenced_t{});
    }

private:
    T* px;
    intrusive_control_block* control;
};

template<class T, class U>
inline bool operator==(intrusive_ptr<T> const& a, intrusive_ptr<U> const& b)
{
    return a.get() == b.get();
}

template<class T, class U>
inline bool operator!=(intrusive_ptr<T> const& a, intrusive_ptr<U> const& b)
{
    return a.get() != b.get();
}

template<class T, class U>
inline bool operator==(intrusive_ptr<T> const& a, U* b)
{
    return a.get() == b;
}

template<class T, class U>
inline bool operator!=(intrusive_ptr<T> const& a, U* b)
{
    return a.get() != b;
}

template<class T, class U>
inline bool operator==(T* a, intrusive_ptr<U> const& b)
{
    return a == b.get();
}

template<class T, class U>
inline bool operator!=(T* a, intrusive_ptr<U> const& b)
{
    return a != b.get();
}

template<class T>
inline bool operator<(intrusive_ptr<T> const& a, intrusive_ptr<T> const& b)
{
    return (a.get() < b.get());
}

template<class T>
void swap(intrusive_ptr<T>& lhs, intrusive_ptr<T>& rhs)
{
    lhs.swap(rhs);
}

template<class T>
void swap(intrusive_weak_ptr<T>& lhs, intrusive_weak_ptr<T>& rhs)
{
    lhs.swap(rhs);
}

template<class T>
T* get_pointer(intrusive_ptr<T> const& p)
{
    return p.get();
}

template<class T, class U>
intrusive_ptr<T> static_pointer_cast(intrusive_ptr<U> const& p)
{
    return static_cast<T*>(p.get());
}

template<class T, class U>
intrusive_ptr<T> const_pointer_cast(intrusive_ptr<U> const& p)
{
    return const_cast<T*>(p.get());
}

template<class T, class U>
intrusive_ptr<T> dynamic_pointer_cast(intrusive_ptr<U> const& p)
{
    return dynamic_cast<T*>(p.get());
}

template<typename T>
using RefPtr = intrusive_ptr<T>;

template<typename T>
using WeakPtr = intrusive_weak_ptr<T>;

} // namespace ZF

namespace std
{
template<typename T>
struct hash<ZF::intrusive_ptr<T>>
{
    inline size_t operator()(const ZF::intrusive_ptr<T>& value) const
    {
        return hash<T*>()(value.get());
    }
};
} // namespace std
