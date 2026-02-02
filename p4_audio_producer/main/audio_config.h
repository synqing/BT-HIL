/*
 * audio_config.h — Single source of truth for capture parameters.
 * Authority: docs/DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md §1.1, §7.1.
 * Do not change without updating the spec and tests.
 */
#ifndef P4_AUDIO_PRODUCER_AUDIO_CONFIG_H
#define P4_AUDIO_PRODUCER_AUDIO_CONFIG_H

/* Locked capture parameters (spec §1.1) */
#define AUDIO_FS                16000
#define AUDIO_HOP_SIZE          128
#define AUDIO_HOP_MS            8

/* Ring buffer: ≥ 2× slow lane window (spec §4.3); 512-sample FFT → ≥ 1024, use 2048 */
#define AUDIO_RING_CAPACITY_SAMPLES  2048

/* Fast lane budget: 50% of 8 ms hop (spec §4.1) */
#define AUDIO_FAST_LANE_BUDGET_US    4000

/* Slow lane: update every N hops (e.g. 4 → 32 ms) */
#define AUDIO_SLOW_LANE_HOP_DIV      4

/* Spec version for banner */
#define AUDIO_SPEC_VERSION_STR   "1.1"

/* Compile-time assertions: changing spec without updating this file must break the build */
_Static_assert(AUDIO_FS == 16000, "AUDIO_FS must be 16000 per spec");
_Static_assert(AUDIO_HOP_SIZE == 128, "AUDIO_HOP_SIZE must be 128 per spec");
_Static_assert(AUDIO_HOP_MS == 8, "AUDIO_HOP_MS must be 8 per spec");
_Static_assert(AUDIO_RING_CAPACITY_SAMPLES >= 1024, "Ring must hold ≥ 2× slow lane window");
_Static_assert(AUDIO_FAST_LANE_BUDGET_US <= AUDIO_HOP_MS * 1000, "Fast lane budget must be ≤ hop time in us");

#endif /* P4_AUDIO_PRODUCER_AUDIO_CONFIG_H */
