#include "point_matrix_harness.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>

// Random operations on two PointMatrix objects, checked against a reference model.
// usage: stress_point_matrix [seed] [ops]

struct RngSource {
    std::mt19937_64 rng;

    explicit RngSource(std::uint64_t seed) : rng(seed) {}

    bool pull(std::uint8_t* dst, std::size_t n) {
        std::size_t i = 0;
        while (i < n) {
            std::uint64_t bits = rng();
            for (int b = 0; b < 8 && i < n; ++b, ++i) {
                dst[i] = static_cast<std::uint8_t>(bits & 0xffu);
                bits >>= 8;
            }
        }
        return true;
    }
};

static bool parse_u64(const char* text, std::uint64_t& out) {
    if (text == 0 || text[0] == '\0' || text[0] == '-') {
        return false;
    }
    errno = 0;
    char* end = 0;
    unsigned long long value = std::strtoull(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE) {
        return false;
    }
    out = static_cast<std::uint64_t>(value);
    return true;
}

int main(int argc, char** argv) {
    std::uint64_t seed = 1;
    std::uint64_t ops = 1000000;
    if (argc > 3) {
        std::cerr << "usage: stress_point_matrix [seed] [ops]\n";
        return 2;
    }
    if (argc > 1 && !parse_u64(argv[1], seed)) {
        std::cerr << "usage: stress_point_matrix [seed] [ops]\n";
        return 2;
    }
    if (argc > 2 && !parse_u64(argv[2], ops)) {
        std::cerr << "usage: stress_point_matrix [seed] [ops]\n";
        return 2;
    }

    RngSource src(seed);
    Harness harness;
    for (std::uint64_t i = 0; i < ops; ++i) {
        if (!harness_step(harness, src, fail_exit)) {
            std::cerr << "stress input ended early\n";
            return 1;
        }
    }
    print_op_counts(harness, ops);
    std::uint64_t sum = 0;
    for (int i = 0; i < kOpCount; ++i) {
        sum += harness.op_counts[i];
    }
    if (sum != ops) {
        std::cerr << "op count sum " << sum << " expected " << ops << "\n";
        return 1;
    }
    return 0;
}
