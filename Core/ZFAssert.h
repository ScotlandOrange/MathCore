#pragma once

#include <Core/Macros.h>

#include <cassert>
#include <cstdio>

#define ZF_ASSERT_MSG(condition, ...)                                                                                      \
    do                                                                                                                     \
    {                                                                                                                      \
        if (!(condition))                                                                                                  \
        {                                                                                                                  \
            std::fprintf(stderr, "Error: ");                                                                              \
            std::fprintf(stderr, __VA_ARGS__);                                                                             \
            std::fprintf(stderr, "\nFile: %s\nLine: %d\n", __FILE__, __LINE__);                                         \
            std::fflush(stderr);                                                                                           \
            assert(false);                                                                                                 \
        }                                                                                                                  \
    } while (0)

#define ZF_ASSERT(condition, ...)                                                                                          \
    do                                                                                                                     \
    {                                                                                                                      \
        if (!(condition))                                                                                                  \
        {                                                                                                                  \
            std::fprintf(stderr, "Error\nFile: %s\nLine: %d\n", __FILE__, __LINE__);                                   \
            std::fflush(stderr);                                                                                           \
            assert(false);                                                                                                 \
        }                                                                                                                  \
    } while (0)