#pragma once

#include <atomic>
#include <cstdint>

struct PCG32 {
    uint64_t state;
    uint64_t inc;

    explicit PCG32(uint64_t seed_state, uint64_t seed_seq) {
        seed(seed_state, seed_seq);
    }

    // Seed the generator
    void seed(uint64_t seed_state, uint64_t seed_seq) {
        state = 0;
        inc = (seed_seq << 1) | 1;
        next_u32();
        state += seed_state;
        next_u32();
    }

    // 32-bit output
    uint32_t next_u32() {
        uint64_t old = state;
        state = old * 6364136223846793005ULL + inc;
        uint32_t shifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
        uint32_t rot = old >> 59u;
        return (shifted >> rot) | (shifted << ((-rot + 1u) & 31));
    }

    // Float in [0,1)
    float uniform() {
        return (next_u32() >> 8) * (1.0f / 16777216.0f);
    }
};

inline uint64_t derive_seed(int x, int y, int sample_index, int frame_index) {
    return (uint64_t(frame_index) << 48) |
                (uint64_t(sample_index << 32)) |
                (uint64_t(y) << 16) |
                (uint64_t(x));
}