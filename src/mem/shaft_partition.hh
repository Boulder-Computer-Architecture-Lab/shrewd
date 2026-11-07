/*
 * SHAFT protected memory partition.
 */

#ifndef __MEM_SHAFT_PARTITION_HH__
#define __MEM_SHAFT_PARTITION_HH__

#include <cstddef>
#include <unordered_set>

#include "base/types.hh"

namespace gem5
{

/**
 * The protected memory partition that SHAFT's protection bit selects, after
 * address-range partial mirroring (Intel) and per-page protection
 * (Virtualized ECC, Flikker). The partition is a set of 4 KiB physical
 * frames:
 *  - Marked: a frame enters the partition when a protected store to it
 *    commits (the core charges the operating system's migration of the
 *    page); every later write to the frame, a write-back included, is
 *    mirrored by the memory controller.
 *  - All: every frame is mirrored (full memory mirroring).
 *  - None: no partition (the default; nothing changes).
 * The frames do not move: the model keeps their timing and traffic, not a
 * second copy of the data.
 */
class ShaftPartition
{
  public:
    enum class Mode { None, Marked, All };

    static constexpr unsigned FrameShift = 12;

    void setMode(Mode mode) { _mode = mode; }
    Mode mode() const { return _mode; }

    /**
     * Adds the frame holding paddr (Marked mode). True if the frame was
     * not in the partition yet, i.e. a page migration.
     */
    bool
    protect(Addr paddr)
    {
        return _mode == Mode::Marked &&
            frames.insert(paddr >> FrameShift).second;
    }

    /** Whether writes to paddr are mirrored. */
    bool
    isProtected(Addr paddr) const
    {
        if (_mode == Mode::All)
            return true;
        return _mode == Mode::Marked &&
            frames.count(paddr >> FrameShift) != 0;
    }

    std::size_t numFrames() const { return frames.size(); }

    /**
     * Records a committed store to paddr; true the first time its frame
     * is written (the program's write footprint, for comparison).
     */
    bool
    noteWritten(Addr paddr)
    {
        return written.insert(paddr >> FrameShift).second;
    }

  private:
    Mode _mode = Mode::None;
    std::unordered_set<Addr> frames;
    std::unordered_set<Addr> written;
};

/** The system's partition (one memory system per simulation). */
inline ShaftPartition &
shaftPartition()
{
    static ShaftPartition partition;
    return partition;
}

} // namespace gem5

#endif // __MEM_SHAFT_PARTITION_HH__
