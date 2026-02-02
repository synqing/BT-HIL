/*
 * Lock-free single-producer ring buffer for audio samples.
 * Capture task writes; fast/slow lane read. Size from audio_config.h.
 */
#ifndef P4_AUDIO_PRODUCER_RING_BUFFER_H
#define P4_AUDIO_PRODUCER_RING_BUFFER_H

#include "audio_config.h"

#define RING_CAPACITY  AUDIO_RING_CAPACITY_SAMPLES

typedef struct {
    float buf[RING_CAPACITY];
    volatile unsigned write_idx;  /* next write position (capture only) */
    volatile unsigned read_idx;  /* updated by fast lane after read */
} ring_buffer_t;

void ring_init(ring_buffer_t *r);
/* Returns 1 if write would overwrite unconsumed data (overflow); capture sets flag and counter. */
int ring_write_hop(ring_buffer_t *r, const float *hop, unsigned n);
void ring_read_latest_hop(ring_buffer_t *r, float *out, unsigned n);
/* Read historical samples: start_idx samples back from write_idx, n samples total. */
void ring_read_history(ring_buffer_t *r, float *out, unsigned start_idx, unsigned n);
unsigned ring_write_idx(const ring_buffer_t *r);

#endif /* P4_AUDIO_PRODUCER_RING_BUFFER_H */
