#include "cpu/o3/shaft_twin.hh"

#include <unordered_map>
#include <utility>

#include "cpu/op_class.hh"

namespace gem5
{

namespace o3
{

namespace
{

// Scalar instructions have at most three sources (e.g. fmadd).
constexpr int MaxTwinSrcRegs = 4;

class ShaftTwinInst : public StaticInst
{
  public:
    explicit ShaftTwinInst(const StaticInstPtr &primary)
        : StaticInst("shaft_twin",
                     primary->isMemRef() ? IntAluOp : primary->opClass())
    {
        setRegIdxArrays(
            reinterpret_cast<RegIdArrayPtr>(&ShaftTwinInst::srcRegIdxArr),
            reinterpret_cast<RegIdArrayPtr>(&ShaftTwinInst::destRegIdxArr));
        _numSrcRegs = primary->numSrcRegs();
        for (int i = 0; i < _numSrcRegs; i++)
            srcRegIdxArr[i] = primary->srcRegIdx(i);
        // Keep the int/FP class for the IQ's accounting.
        flags[IsInteger] = primary->isInteger();
        flags[IsFloating] = primary->isFloating();
    }

    Fault
    execute(ExecContext *xc, trace::InstRecord *traceData) const override
    {
        return NoFault;
    }

    void
    advancePC(PCStateBase &pc) const override
    {
        pc.advance();
    }

    std::string
    generateDisassembly(Addr pc,
            const loader::SymbolTable *symtab) const override
    {
        return mnemonic;
    }

  private:
    RegId srcRegIdxArr[MaxTwinSrcRegs];
    RegId destRegIdxArr[1];
};

} // anonymous namespace

bool
shaftTwinEligible(const StaticInstPtr &primary)
{
    return primary->opClass() != No_OpClass && !primary->isNop() &&
        !primary->isNonSpeculative() && !primary->isSerializing() &&
        !primary->isAtomic() && !primary->isVector() &&
        !primary->isMacroop() && !primary->isMicroop() &&
        !primary->isSyscall() && primary->numSrcRegs() <= MaxTwinSrcRegs;
}

StaticInstPtr
shaftTwinOf(const StaticInstPtr &primary)
{
    // StaticInsts are shared through the decoder cache, so twins are too.
    // The entry keeps the primary alive, so its address is never reused
    // for a different instruction.
    static std::unordered_map<const StaticInst *,
                              std::pair<StaticInstPtr, StaticInstPtr>> twins;
    auto &entry = twins[primary.get()];
    if (!entry.second)
        entry = {primary, new ShaftTwinInst(primary)};
    return entry.second;
}

} // namespace o3
} // namespace gem5
