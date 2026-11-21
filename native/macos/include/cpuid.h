// <cpuid.h> on arm64: the x86 features immintrin.h provides.
#pragma once
#if defined(__x86_64__)
#  include_next <cpuid.h>
#else
#  include <string.h>
static inline void principia_arm64_cpuid(unsigned leaf, unsigned *a, unsigned *b, unsigned *c, unsigned *d) {
  *a = *b = *c = *d = 0;
  if (leaf == 0) {
    *a = 7;
    memcpy(b, "Appl", 4); memcpy(d, "eARM", 4); memcpy(c, "64  ", 4);
  } else if (leaf == 1) {
    *d = (1u << 25) | (1u << 26);
    *c = (1u << 0) | (1u << 12) | (1u << 19) | (1u << 28);
  } else if (leaf >= 0x80000002u && leaf <= 0x80000004u) {
    static const char brand[48] = "Apple arm64 (SIMDe)";
    const char* piece = brand + 16 * (leaf - 0x80000002u);
    memcpy(a, piece, 4); memcpy(b, piece + 4, 4); memcpy(c, piece + 8, 4); memcpy(d, piece + 12, 4);
  }
}
#  define __cpuid_count(leaf, sub, a, b, c, d) principia_arm64_cpuid((leaf), &(a), &(b), &(c), &(d))
#endif
