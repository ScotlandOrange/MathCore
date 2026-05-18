#pragma once

#if !defined(ZF_FORCE_INLINE)
#    if defined(_MSC_VER)
#        define ZF_FORCE_INLINE __forceinline
#    elif defined(__GNUC__) || defined(__clang__)
#        define ZF_FORCE_INLINE inline __attribute__((always_inline))
#    else
#        define ZF_FORCE_INLINE inline
#    endif
#endif
