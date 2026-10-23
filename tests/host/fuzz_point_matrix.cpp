#include "point_matrix_harness.h"

#include <cstddef>
#include <cstdint>

// libFuzzer target. Input bytes are a sequence of operations on A and B.

struct FuzzSource {
    const std::uint8_t* cur;
    const std::uint8_t* end;

    bool pull(std::uint8_t* dst, std::size_t n) {
        if (static_cast<std::size_t>(end - cur) < n) {
            return false;
        }
        for (std::size_t i = 0; i < n; ++i) {
            dst[i] = cur[i];
        }
        cur += n;
        return true;
    }
};

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    FuzzSource src;
    src.cur = data;
    src.end = data + size;
    Harness harness;
    while (harness_step(harness, src, fail_trap)) {
    }
    return 0;
}
