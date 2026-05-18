#pragma once

#include "Core/Ptr/intrusive_refcount.h"

#include <atomic>

namespace ZF
{

/**
 * An intrusive reference counting base class that is compliant with ZF::intrusive_ptr. This
 * version is thread-safe through use of an implicit atomic reference count. Explicit friending of
 * intrusive_ptr is used; the user must use intrusive_ptr to take a reference on the object.
 */
using intrusive_base = intrusive_refcount<std::atomic_int>;

} // namespace ZF
