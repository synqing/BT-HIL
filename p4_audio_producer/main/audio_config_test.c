/*
 * Compile-time and link-time test for audio_config.h.
 * If spec constants are wrong, this file fails to compile or assert at runtime.
 */
#include "audio_config.h"
#include <assert.h>

/* Build-time: already enforced in audio_config.h via _Static_assert. */

/* Runtime sanity (e.g. when this TU is linked into a test build) */
void audio_config_test_assert(void)
{
    assert(AUDIO_FS == 16000);
    assert(AUDIO_HOP_SIZE == 128);
    assert(AUDIO_HOP_MS == 8);
    assert(AUDIO_RING_CAPACITY_SAMPLES >= 1024);
    assert(AUDIO_FAST_LANE_BUDGET_US > 0 && AUDIO_FAST_LANE_BUDGET_US <= AUDIO_HOP_MS * 1000);
}
