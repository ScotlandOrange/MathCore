#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>

namespace ZF
{

template<class T>
class intrusive_weak_ptr;

template<typename T>
struct IntrusivePtrCountPolicy;

struct intrusive_default_delete
{
    template<typename U>
    void operator()(U* p) const
    {
        delete p;
    }
};

struct intrusive_control_block
{
    static constexpr std::uint64_t weak_mask = 0xffffffffull;
    static constexpr std::uint64_t strong_increment = 1ull << 32;
    static constexpr std::uint64_t weak_increment = 1ull;

    std::atomic_uint64_t count{weak_increment};

    static std::uint32_t strong_from(std::uint64_t value) noexcept
    {
        return static_cast<std::uint32_t>(value >> 32);
    }

    static std::uint32_t weak_from(std::uint64_t value) noexcept
    {
        return static_cast<std::uint32_t>(value & weak_mask);
    }

    std::uint32_t strong_count() const noexcept
    {
        return strong_from(count.load(std::memory_order_acquire));
    }

    std::uint32_t weak_count() const noexcept
    {
        return weak_from(count.load(std::memory_order_acquire));
    }

    void add_strong() noexcept
    {
        const auto old_count = count.fetch_add(strong_increment, std::memory_order_relaxed);
        assert(strong_from(old_count) != 0xffffffffu);
    }

    bool try_add_strong() noexcept
    {
        auto old_count = count.load(std::memory_order_acquire);
        while (strong_from(old_count) != 0)
        {
            const auto new_count = old_count + strong_increment;
            if (count.compare_exchange_weak(old_count, new_count, std::memory_order_acq_rel, std::memory_order_acquire))
            {
                return true;
            }
        }
        return false;
    }

    int release_strong() noexcept
    {
        const auto old_count = count.fetch_sub(strong_increment, std::memory_order_acq_rel);
        return static_cast<int>(strong_from(old_count)) - 1;
    }

    void add_weak() noexcept
    {
        auto old_count = count.load(std::memory_order_relaxed);
        for (;;)
        {
            assert(weak_from(old_count) != 0xffffffffu);
            const auto new_count = old_count + weak_increment;
            if (count.compare_exchange_weak(old_count, new_count, std::memory_order_relaxed, std::memory_order_relaxed))
            {
                return;
            }
        }
    }

    bool release_weak() noexcept
    {
        const auto old_count = count.fetch_sub(weak_increment, std::memory_order_acq_rel);
        assert(weak_from(old_count) != 0u);
        return weak_from(old_count) == 1u;
    }
};

static_assert(sizeof(intrusive_control_block) == sizeof(std::uint64_t), "intrusive_control_block must stay one 64-bit atomic value.");

namespace detail
{
    inline void destroy_control_block(intrusive_control_block* control) noexcept
    {
        delete control;
    }

    inline void release_weak_reference(intrusive_control_block* control) noexcept
    {
        if (control && control->release_weak())
        {
            destroy_control_block(control);
        }
    }
} // namespace detail

/**
 * An intrusive reference counting base class that is compliant with ZF::intrusive_ptr. Explicit
 * friending of intrusive_ptr is used; the user must use intrusive_ptr to take a reference on the object.
 */
template<typename refcount_t, typename Deleter = intrusive_default_delete>
class intrusive_refcount
{
public:
    inline uint32_t use_count() const
    {
        return m_control ? m_control->strong_count() : 0u;
    }

protected:
    intrusive_refcount() :
        m_control(new intrusive_control_block())
    {
    }

    explicit intrusive_refcount(Deleter deleter) :
        m_control(new intrusive_control_block()),
        m_deleter{deleter}
    {
    }

    intrusive_refcount(const intrusive_refcount&) = delete;
    intrusive_refcount(intrusive_refcount&&) = delete;

    virtual ~intrusive_refcount()
    {
        assert(!m_control || m_control->strong_count() == 0); // The destructor was called on an object with active references.
        detail::release_weak_reference(m_control);
    }

    void add_ref() const
    {
        assert(m_control != nullptr);
        m_control->add_strong();
    }

    void release() const
    {
        assert(m_control != nullptr);
        const int32_t refCount = static_cast<int32_t>(m_control->release_strong());
        assert(refCount != -1); // Releasing an already released object.
        if (refCount == 0)
        {
            m_deleter(const_cast<intrusive_refcount*>(this));
        }
    }

    intrusive_control_block* control_block() const
    {
        return m_control;
    }

    void add_weak() const
    {
        assert(m_control != nullptr);
        m_control->add_weak();
    }

    void release_weak() const
    {
        assert(m_control != nullptr);
        detail::release_weak_reference(m_control);
    }

    template<typename T>
    friend struct IntrusivePtrCountPolicy;

    template<class T>
    friend class intrusive_weak_ptr;

    friend struct intrusive_default_delete;

private:
    mutable intrusive_control_block* m_control = nullptr;
    Deleter m_deleter;
};

} // namespace ZF
