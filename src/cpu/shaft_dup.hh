#ifndef __CPU_SHAFT_DUP_HH__
#define __CPU_SHAFT_DUP_HH__

#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

/**
 * Software-duplication binaries reserve x31 (t6) and f31 (ft11) for the
 * duplicate and its check (every binary of the family is compiled with
 * -ffixed-t6 -ffixed-ft11), so an instruction that touches either is not
 * one of the program's own and is left out of the instruction counts.
 */
inline bool
touchesDupRegs(const StaticInstPtr &si)
{
    auto reserved = [](const RegId &reg) {
        return reg.index() == 31 &&
               (reg.is(IntRegClass) || reg.is(FloatRegClass));
    };
    for (int i = 0; i < si->numSrcRegs(); i++) {
        if (reserved(si->srcRegIdx(i)))
            return true;
    }
    for (int i = 0; i < si->numDestRegs(); i++) {
        if (reserved(si->destRegIdx(i)))
            return true;
    }
    return false;
}

} // namespace gem5

#endif // __CPU_SHAFT_DUP_HH__
