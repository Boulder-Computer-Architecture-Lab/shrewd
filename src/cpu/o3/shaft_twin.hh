#ifndef __CPU_O3_SHAFT_TWIN_HH__
#define __CPU_O3_SHAFT_TWIN_HH__

#include <string>

#include "cpu/static_inst.hh"

namespace gem5
{

namespace o3
{

/** secon/secoff: rename drops them, so they never take a ROB/IQ entry. */
inline bool
isShaftMarker(const StaticInstPtr &si)
{
    if (si->opClass() != No_OpClass)
        return false;
    const std::string name = si->getName();
    return name == "secon" || name == "secoff";
}

/**
 * Whether a protected instruction gets a redundant twin in the DIE-style
 * mechanism (data-producing, address-generating or branch instructions;
 * not system, atomic, vector or micro-coded ones).
 */
bool shaftTwinEligible(const StaticInstPtr &primary);

/**
 * The redundant copy of a protected instruction in the DIE-style
 * mechanism (Ray et al. MICRO'01, Parashar et al. ISCA'04). It reads the
 * same source registers as the primary, so it occupies its own IQ and ROB
 * entries, issue slot, operand delivery and functional unit (an IntAlu
 * acting as AGU for memory instructions), but it has no destination and no
 * memory or control effect: the pipeline sees a plain ALU op that wakes
 * nothing. Its result is compared with the primary's at commit.
 */
StaticInstPtr shaftTwinOf(const StaticInstPtr &primary);

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_SHAFT_TWIN_HH__
