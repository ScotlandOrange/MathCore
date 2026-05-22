#pragma once

#include <cstddef>
#include <cstdint>

namespace ZF
{

// Public scalar aliases favor fixed-width integers for cross-platform and serialization code.
// Int/Long-style aliases are intentionally avoided because their width depends on the ABI.
using Bool = bool;

using Char = char;
using WChar = wchar_t;

using Int8 = std::int8_t;
using UInt8 = std::uint8_t;
using Int16 = std::int16_t;
using UInt16 = std::uint16_t;
using Int32 = std::int32_t;
using UInt32 = std::uint32_t;
using Int64 = std::int64_t;
using UInt64 = std::uint64_t;

using Int = Int32;
using UInt = UInt32;

using Byte = UInt8;
using byte = unsigned char;
using Size = std::size_t;
using PtrDiff = std::ptrdiff_t;

using Float32 = float;
using Float64 = double;
using LongDouble = long double;

using Float = Float32;

} // namespace ZF
