// <immintrin.h> on arm64: SIMDe, with fused multiply-adds that round once.
#pragma once
#if defined(__x86_64__)
#  include_next <immintrin.h>
#else
#  define SIMDE_ENABLE_NATIVE_ALIASES
#  include "simde/x86/sse4.1.h"
#  include "simde/x86/avx.h"
#  include "simde/x86/fma.h"
#  include <arm_neon.h>

static inline float64x2_t principia_ksa_f64(simde__m128d x) { return simde__m128d_to_neon_f64(x); }
static inline simde__m128d principia_ksa_m128d(float64x2_t x) { return simde__m128d_from_neon_f64(x); }

static inline simde__m128d principia_ksa_fmadd_pd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_m128d(vfmaq_f64(principia_ksa_f64(c), principia_ksa_f64(a), principia_ksa_f64(b)));
}
static inline simde__m128d principia_ksa_fmsub_pd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_m128d(vfmaq_f64(vnegq_f64(principia_ksa_f64(c)), principia_ksa_f64(a), principia_ksa_f64(b)));
}
static inline simde__m128d principia_ksa_fnmadd_pd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_m128d(vfmsq_f64(principia_ksa_f64(c), principia_ksa_f64(a), principia_ksa_f64(b)));
}
static inline simde__m128d principia_ksa_fnmsub_pd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_m128d(vfmsq_f64(vnegq_f64(principia_ksa_f64(c)), principia_ksa_f64(a), principia_ksa_f64(b)));
}

static inline simde__m128d principia_ksa_low(simde__m128d a, simde__m128d r) {
  return principia_ksa_m128d(vcopyq_laneq_f64(principia_ksa_f64(a), 0, principia_ksa_f64(r), 0));
}
static inline simde__m128d principia_ksa_fmadd_sd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_low(a, principia_ksa_fmadd_pd(a, b, c));
}
static inline simde__m128d principia_ksa_fmsub_sd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_low(a, principia_ksa_fmsub_pd(a, b, c));
}
static inline simde__m128d principia_ksa_fnmadd_sd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_low(a, principia_ksa_fnmadd_pd(a, b, c));
}
static inline simde__m128d principia_ksa_fnmsub_sd(simde__m128d a, simde__m128d b, simde__m128d c) {
  return principia_ksa_low(a, principia_ksa_fnmsub_pd(a, b, c));
}

#define PRINCIPIA_KSA_M256D(name)                                                            \
  static inline simde__m256d principia_ksa_##name##_256(simde__m256d a, simde__m256d b,    \
                                                         simde__m256d c) {                 \
    return simde_mm256_set_m128d(                                                          \
        principia_ksa_##name##_pd(simde_mm256_extractf128_pd(a, 1), simde_mm256_extractf128_pd(b, 1), \
                                  simde_mm256_extractf128_pd(c, 1)),                       \
        principia_ksa_##name##_pd(simde_mm256_castpd256_pd128(a), simde_mm256_castpd256_pd128(b),     \
                                  simde_mm256_castpd256_pd128(c)));                        \
  }
PRINCIPIA_KSA_M256D(fmadd)
PRINCIPIA_KSA_M256D(fmsub)
PRINCIPIA_KSA_M256D(fnmadd)
PRINCIPIA_KSA_M256D(fnmsub)
#undef PRINCIPIA_KSA_M256D

#  undef _mm_fmadd_sd
#  undef _mm_fmsub_sd
#  undef _mm_fnmadd_sd
#  undef _mm_fnmsub_sd
#  undef _mm_fmadd_pd
#  undef _mm_fmsub_pd
#  undef _mm_fnmadd_pd
#  undef _mm_fnmsub_pd
#  undef _mm256_fmadd_pd
#  undef _mm256_fmsub_pd
#  undef _mm256_fnmadd_pd
#  undef _mm256_fnmsub_pd
#  define _mm_fmadd_sd principia_ksa_fmadd_sd
#  define _mm_fmsub_sd principia_ksa_fmsub_sd
#  define _mm_fnmadd_sd principia_ksa_fnmadd_sd
#  define _mm_fnmsub_sd principia_ksa_fnmsub_sd
#  define _mm_fmadd_pd principia_ksa_fmadd_pd
#  define _mm_fmsub_pd principia_ksa_fmsub_pd
#  define _mm_fnmadd_pd principia_ksa_fnmadd_pd
#  define _mm_fnmsub_pd principia_ksa_fnmsub_pd
#  define _mm256_fmadd_pd principia_ksa_fmadd_256
#  define _mm256_fmsub_pd principia_ksa_fmsub_256
#  define _mm256_fnmadd_pd principia_ksa_fnmadd_256
#  define _mm256_fnmsub_pd principia_ksa_fnmsub_256
#endif
