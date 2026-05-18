#include "Core/Ptr/intrusive_base.h"
#include "Core/Ptr/intrusive_ptr.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{

struct TrackedObject : ZF::intrusive_base
{
    static int constructed;
    static int destroyed;
    static int alive;

    static void reset_stats()
    {
        constructed = 0;
        destroyed = 0;
        alive = 0;
    }

    TrackedObject()
    {
        ++constructed;
        ++alive;
    }

    ~TrackedObject() override
    {
        --alive;
        ++destroyed;
    }

    ZF::intrusive_control_block* control() const
    {
        return control_block();
    }
};

struct BaseObject : ZF::intrusive_base
{
    static int destroyed;

    static void reset_stats()
    {
        destroyed = 0;
    }

    ~BaseObject() override
    {
        ++destroyed;
    }
};

struct DerivedObject : BaseObject
{
    static int destroyed;

    static void reset_stats()
    {
        BaseObject::reset_stats();
        destroyed = 0;
    }
    ~DerivedObject() override
    {
        ++destroyed;
    }
};

struct LayoutObject : ZF::intrusive_base
{
    static int destroyed;

    static void reset_stats()
    {
        destroyed = 0;
    }

    ~LayoutObject() override
    {
        ++destroyed;
    }

    ZF::intrusive_control_block* control() const
    {
        return control_block();
    }
};

struct OtherBase
{
    virtual ~OtherBase() = default;
    int value = 0;
};

struct NonFirstIntrusiveObject : OtherBase, ZF::intrusive_base
{
    static int destroyed;

    static void reset_stats()
    {
        destroyed = 0;
    }

    ~NonFirstIntrusiveObject() override
    {
        ++destroyed;
    }

    ZF::intrusive_control_block* control() const
    {
        return control_block();
    }
};

struct alignas(64) OverAlignedObject : ZF::intrusive_base
{
    static int destroyed;

    static void reset_stats()
    {
        destroyed = 0;
    }

    ~OverAlignedObject() override
    {
        ++destroyed;
    }

    ZF::intrusive_control_block* control() const
    {
        return control_block();
    }
};

struct StrongCycleNode : ZF::intrusive_base
{
    static int alive;
    ZF::intrusive_ptr<StrongCycleNode> next;

    static void reset_stats()
    {
        alive = 0;
    }

    StrongCycleNode()
    {
        ++alive;
    }

    ~StrongCycleNode() override
    {
        --alive;
    }
};

struct WeakCycleNode : ZF::intrusive_base
{
    static int alive;
    ZF::intrusive_ptr<WeakCycleNode> child;
    ZF::intrusive_weak_ptr<WeakCycleNode> parent;

    static void reset_stats()
    {
        alive = 0;
    }
    WeakCycleNode()
    {
        ++alive;
    }
    ~WeakCycleNode() override
    {
        --alive;
    }
};

struct ThrowingBase
{
    ThrowingBase()
    {
        throw std::runtime_error("base construction failed");
    }
};

struct ThrowsBeforeIntrusiveBase : ThrowingBase, ZF::intrusive_base
{
};

struct ThrowsAfterIntrusiveBase : ZF::intrusive_base
{
    ThrowsAfterIntrusiveBase()
    {
        throw std::runtime_error("object construction failed");
    }
};

// 静态成员定义 (C++14 不支持 inline 静态成员的类内初始化)
int TrackedObject::constructed = 0;
int TrackedObject::destroyed = 0;
int TrackedObject::alive = 0;
int BaseObject::destroyed = 0;
int DerivedObject::destroyed = 0;
int LayoutObject::destroyed = 0;
int NonFirstIntrusiveObject::destroyed = 0;
int OverAlignedObject::destroyed = 0;
int StrongCycleNode::alive = 0;
int WeakCycleNode::alive = 0;

} // 匿名 namespace

// 使用示例：用 new 分配的 intrusive_base 对象创建 intrusive_ptr，
// 通过拷贝共享所有权，并用 reset() 释放所有权。
TEST(IntrusivePtrUsage, StrongPointerOwnsAndReleasesObject)
{
    TrackedObject::reset_stats();

    ZF::intrusive_ptr<TrackedObject> object(new TrackedObject); // 第一个 intrusive_ptr 获取初始强引用。
    ASSERT_TRUE(object);
    EXPECT_EQ(object->use_count(), 1u); // 当前只有一个强引用持有者。

    {
        ZF::RefPtr<TrackedObject> shared = object; // RefPtr 是 intrusive_ptr 的别名。
        EXPECT_EQ(object->use_count(), 2u); // 拷贝强指针会递增 strong_count。
    }

    EXPECT_EQ(object->use_count(), 1u); // 离开作用域会释放拷贝出来的强引用。
    object.reset(); // reset() 释放最后一个强引用，并销毁对象。

    EXPECT_EQ(TrackedObject::constructed, 1);
    EXPECT_EQ(TrackedObject::destroyed, 1);
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 使用示例：保存一个不拥有对象的 intrusive_weak_ptr，
// 只在对象仍存活时通过 lock() 临时提升为强指针。
TEST(IntrusivePtrUsage, WeakPointerObservesAndLocksObject)
{
    TrackedObject::reset_stats();

    ZF::WeakPtr<TrackedObject> observer; // WeakPtr 是 intrusive_weak_ptr 的别名。

    {
        ZF::RefPtr<TrackedObject> object(new TrackedObject);
        observer = object; // 从强指针赋值后开始观察，但不拥有对象。

        auto locked = observer.lock(); // 如果对象仍存活，lock() 返回一个强指针。
        ASSERT_TRUE(locked);
        EXPECT_EQ(object->use_count(), 2u); // locked 指针会临时增加一个强引用持有者。
    }

    EXPECT_TRUE(observer.expired()); // 对象销毁后，弱指针能感知 expired 状态。
    EXPECT_FALSE(observer.lock()); // lock() 失败，不能复活已经销毁的对象。
    observer.reset();

    EXPECT_EQ(TrackedObject::destroyed, 1);
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 验证默认构造的强指针和弱指针都表现为空智能指针。
TEST(IntrusivePtr, DefaultConstructedPointersAreEmpty)
{
    ZF::intrusive_ptr<TrackedObject> strong; // 空强指针不保存对象。
    EXPECT_EQ(strong.get(), nullptr);
    EXPECT_FALSE(strong);
    EXPECT_TRUE(!strong);

    ZF::intrusive_weak_ptr<TrackedObject> weak; // 空弱指针没关联 control block。
    EXPECT_TRUE(weak.expired());
    EXPECT_EQ(weak.use_count(), 0u);
    EXPECT_FALSE(weak.lock()); // 空弱指针不能通过 lock() 提升。
}

// 验证强指针的 copy、move、reset()、swap() 只按语义更新 strong_count。
TEST(IntrusivePtr, StrongReferenceCountTracksCopyMoveResetAndSwap)
{
    TrackedObject::reset_stats();

    auto* raw = new TrackedObject; // raw 指针初始没有 intrusive 引用。
    EXPECT_EQ(raw->use_count(), 0u);

    ZF::intrusive_ptr<TrackedObject> first(raw); // 第一个 intrusive_ptr 增加第一个强引用。
    EXPECT_EQ(first->use_count(), 1u);

    ZF::intrusive_ptr<TrackedObject> second(first); // copy 会共享所有权。
    EXPECT_EQ(first->use_count(), 2u);

    ZF::intrusive_ptr<TrackedObject> third(std::move(second)); // move 转移指针本身，不改变 strong_count。
    EXPECT_EQ(second.get(), nullptr);
    EXPECT_EQ(first->use_count(), 2u);

    ZF::intrusive_ptr<TrackedObject> fourth;
    fourth.swap(third); // swap() 只交换保存的 pointer，不改变 strong_count。
    EXPECT_EQ(third.get(), nullptr);
    EXPECT_EQ(first->use_count(), 2u);

    fourth.reset(); // 释放一个强引用持有者后 strong_count 递减。
    EXPECT_EQ(first->use_count(), 1u);

    first.reset(); // 释放最后一个强引用持有者后销毁对象。
    EXPECT_EQ(TrackedObject::constructed, 1);
    EXPECT_EQ(TrackedObject::destroyed, 1);
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 验证弱指针不保持对象存活，并且销毁后不能再次提升为强指针。
TEST(IntrusivePtr, WeakPointerDoesNotKeepObjectAliveAndCannotResurrectIt)
{
    TrackedObject::reset_stats();

    ZF::intrusive_weak_ptr<TrackedObject> weak;
    ZF::intrusive_control_block* control = nullptr;

    {
        ZF::intrusive_ptr<TrackedObject> strong(new TrackedObject); // 强引用持有者负责保持对象存活。
        control = strong->control(); // 保留 control block 指针，用来检查对象销毁后的计数。

        EXPECT_EQ(control->strong_count(), 1u);
        EXPECT_EQ(control->weak_count(), 1u); // 对象存活期间 control block 持有一个隐式弱引用。

        weak = strong; // weak 赋值只增加弱引用，不增加强引用。
        EXPECT_FALSE(weak.expired());
        EXPECT_EQ(weak.use_count(), 1u);
        EXPECT_EQ(control->weak_count(), 2u);

        {
            auto locked = weak.lock(); // strong_count 非零时，lock() 原子地增加强引用。
            ASSERT_TRUE(locked);
            EXPECT_EQ(locked->use_count(), 2u);
            EXPECT_EQ(control->strong_count(), 2u);
        }

        EXPECT_EQ(strong->use_count(), 1u); // 离开 locked 作用域后释放临时强引用持有者。
        EXPECT_EQ(control->strong_count(), 1u);
    }

    EXPECT_EQ(TrackedObject::destroyed, 1); // 最后一个强指针离开作用域后对象被销毁。
    EXPECT_EQ(TrackedObject::alive, 0);
    EXPECT_TRUE(weak.expired());
    EXPECT_EQ(weak.use_count(), 0u);
    EXPECT_EQ(control->strong_count(), 0u);
    EXPECT_EQ(control->weak_count(), 1u); // 此时只有外部弱指针保持 control block 存活。
    EXPECT_FALSE(weak.lock()); // expired 弱指针不能复活对象。

    weak.reset(); // 释放最后一个弱引用后释放 control block。
}

// 验证弱指针的 copy、move、reset() 只影响 weak_count。
TEST(IntrusivePtr, WeakCopyMoveAndResetOnlyAffectWeakCount)
{
    TrackedObject::reset_stats();

    ZF::intrusive_ptr<TrackedObject> strong(new TrackedObject); // 整个 weak 操作过程中保持一个稳定的强引用持有者。
    auto* control = strong->control();

    ZF::intrusive_weak_ptr<TrackedObject> first(strong); // 第一个弱指针增加一个外部弱引用。
    EXPECT_EQ(control->strong_count(), 1u);
    EXPECT_EQ(control->weak_count(), 2u);

    ZF::intrusive_weak_ptr<TrackedObject> second(first); // copy 弱指针只递增 weak_count。
    EXPECT_EQ(control->strong_count(), 1u);
    EXPECT_EQ(control->weak_count(), 3u);

    ZF::intrusive_weak_ptr<TrackedObject> third(std::move(second)); // move 弱指针转移弱引用。
    EXPECT_EQ(control->strong_count(), 1u);
    EXPECT_EQ(control->weak_count(), 3u);

    second.reset(); // 对 moved-from 弱指针调用 reset() 是 no-op。
    EXPECT_EQ(control->weak_count(), 3u);

    first.reset(); // 对有效弱指针调用 reset() 会释放一个弱引用。
    EXPECT_EQ(control->weak_count(), 2u);

    third.reset();
    EXPECT_EQ(control->weak_count(), 1u);

    strong.reset(); // 没有外部弱指针时，对象和 control block 一起释放。
    EXPECT_EQ(TrackedObject::destroyed, 1);
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 验证 base/derived 之间的 intrusive_ptr 转换和 cast 能正确保持所有权。
TEST(IntrusivePtr, DerivedToBaseConversionAndCastsPreserveReferenceCount)
{
    DerivedObject::reset_stats();

    ZF::intrusive_ptr<DerivedObject> derived(new DerivedObject); // 起始只有一个派生对象的持有者。
    EXPECT_EQ(derived->use_count(), 1u);

    ZF::intrusive_ptr<BaseObject> base(derived); // 转换为基类指针后共享同一个 control block。
    EXPECT_EQ(derived->use_count(), 2u);

    auto dynamic_derived = ZF::dynamic_pointer_cast<DerivedObject>(base); // cast 结果是另一个强引用持有者。
    ASSERT_TRUE(dynamic_derived);
    EXPECT_EQ(derived->use_count(), 3u);

    dynamic_derived.reset();
    base.reset();
    EXPECT_EQ(derived->use_count(), 1u);

    derived.reset(); // 最终释放必须让派生对象和基类子对象各析构一次。
    EXPECT_EQ(DerivedObject::destroyed, 1);
    EXPECT_EQ(BaseObject::destroyed, 1);
}

// 验证构造失败会释放已经分配的 control block，不会污染后续分配。
TEST(IntrusivePtr, ConstructorExceptionsReleaseControlBlock)
{
    EXPECT_THROW({ delete new ThrowsBeforeIntrusiveBase; }, std::runtime_error); // intrusive_base 构造前失败。

    EXPECT_THROW({ delete new ThrowsAfterIntrusiveBase; }, std::runtime_error); // intrusive_base 构造后失败。

    TrackedObject::reset_stats();
    ZF::intrusive_ptr<TrackedObject> strong(new TrackedObject); // 异常之后正常分配仍然必须可用。
    EXPECT_EQ(strong->use_count(), 1u);
    strong.reset();
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 验证多个线程并发 copy 强指针后，最终引用计数能回到单一持有者。
TEST(IntrusivePtr, ConcurrentStrongCopiesReleaseBackToSingleOwner)
{
    TrackedObject::reset_stats();

    ZF::intrusive_ptr<TrackedObject> owner(new TrackedObject); // 主线程持有一个长期存在的强引用持有者。
    std::atomic_bool failed{false};
    std::vector<std::thread> threads;

    for (int thread_index = 0; thread_index != 8; ++thread_index)
    {
        threads.emplace_back([owner, &failed]() {
            for (int iteration = 0; iteration != 2000; ++iteration)
            {
                ZF::intrusive_ptr<TrackedObject> local(owner); // 每次循环临时 copy 一份强指针。
                if (!local || local->use_count() == 0)
                {
                    failed.store(true, std::memory_order_relaxed); // 如果看到无效对象或非法 count，说明引用控制有问题。
                }
            }
        });
    }

    for (auto& thread : threads)
    {
        thread.join();
    }

    EXPECT_FALSE(failed.load(std::memory_order_relaxed));
    EXPECT_EQ(owner->use_count(), 1u); // 所有临时强指针释放后，只剩主持有者。

    owner.reset(); // 释放最后一个强引用持有者后对象应析构。
    EXPECT_EQ(TrackedObject::destroyed, 1);
    EXPECT_EQ(TrackedObject::alive, 0);
}

// 验证纯强指针循环引用会保活对象，必须显式打破循环才能释放。
TEST(IntrusivePtr, StrongCyclesKeepObjectsAliveUntilCycleIsBroken)
{
    StrongCycleNode::reset_stats();

    ZF::intrusive_ptr<StrongCycleNode> keeper;
    {
        ZF::intrusive_ptr<StrongCycleNode> a(new StrongCycleNode);
        ZF::intrusive_ptr<StrongCycleNode> b(new StrongCycleNode);
        keeper = a; // 额外保留 a，方便离开作用域后手动打破循环。

        a->next = b; // a 强引用 b。
        b->next = a; // b 强引用 a，形成强引用环。

        EXPECT_EQ(a->use_count(), 3u);
        EXPECT_EQ(b->use_count(), 2u);
    }

    EXPECT_EQ(StrongCycleNode::alive, 2); // 外部 a/b 离开作用域后，强引用环仍保活两个对象。
    EXPECT_EQ(keeper->use_count(), 2u); // keeper 和 b->next 都持有 a。

    keeper->next.reset(); // 断开 a->next，b 失去最后一个强引用持有者并析构。
    EXPECT_EQ(StrongCycleNode::alive, 1);

    keeper.reset(); // 最后释放 a。
    EXPECT_EQ(StrongCycleNode::alive, 0);
}

// 验证用 intrusive_weak_ptr 表达反向引用可以打破所有权循环。
TEST(IntrusivePtr, WeakBackReferenceBreaksOwnershipCycle)
{
    WeakCycleNode::reset_stats();

    {
        ZF::intrusive_ptr<WeakCycleNode> parent(new WeakCycleNode);
        ZF::intrusive_ptr<WeakCycleNode> child(new WeakCycleNode);

        parent->child = child; // parent 强拥有 child。
        child->parent = parent; // child 只用弱指针观察 parent，不形成强引用环。

        EXPECT_EQ(parent->use_count(), 1u); // 弱反向引用不增加 parent 的 strong_count。
        EXPECT_EQ(child->use_count(), 2u); // child 同时被局部变量和 parent->child 持有。
        EXPECT_EQ(child->parent.use_count(), 1u); // 弱指针只报告 parent 当前 strong_count。
        EXPECT_EQ(child->parent.lock(), parent); // parent 存活时可以通过 lock() 临时提升。
    }

    EXPECT_EQ(WeakCycleNode::alive, 0); // 离开作用域后没有强引用环，两个对象都释放。
}
