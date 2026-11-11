/*
 * SHAFT: the protection bit on a memory request.
 *
 * The O3 LSQ sets this flag on the request of a store whose instruction is
 * protected (DynInst::protected_). An L1 data cache in store-through mode
 * (BaseCache shaft_store_through = Marked) reads it: a line written by a
 * protected store becomes store-through until it leaves the cache.
 *
 * The flag is a bit of Request::Flags that gem5 leaves unused, defined here
 * so that only the LSQ and the cache include it. No other code reads it, so
 * with store-through off it changes nothing.
 */

#ifndef __MEM_SHAFT_FLAGS_HH__
#define __MEM_SHAFT_FLAGS_HH__

#include "mem/request.hh"

namespace gem5
{

constexpr Request::FlagsType ShaftProtectedStore = 0x0000000400000000ULL;

static_assert((ShaftProtectedStore &
               (Request::ARCH_BITS | Request::INVALIDATE | Request::CLEAN |
                Request::DST_BITS | Request::HTM_CMD | Request::TLBI_CMD |
                Request::HAS_NO_ADDR | Request::READ_MODIFY_WRITE)) == 0,
              "ShaftProtectedStore overlaps a Request flag");

} // namespace gem5

#endif // __MEM_SHAFT_FLAGS_HH__
