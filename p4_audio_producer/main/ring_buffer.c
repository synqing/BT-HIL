#include "ring_buffer.h"
#include <string.h>

void ring_init(ring_buffer_t *r)
{
    memset(r->buf, 0, sizeof(r->buf));
    r->write_idx = 0;
    r->read_idx = 0;
}

int ring_write_hop(ring_buffer_t *r, const float *hop, unsigned n)
{
    unsigned w = r->write_idx;
    unsigned next = (w + n) % RING_CAPACITY;
    int overflow = (next == r->read_idx) ? 1 : 0;
    for (unsigned i = 0; i < n; i++) {
        r->buf[(w + i) % RING_CAPACITY] = hop[i];
    }
    r->write_idx = next;
    return overflow;
}

void ring_read_latest_hop(ring_buffer_t *r, float *out, unsigned n)
{
    unsigned w = r->write_idx;
    unsigned start = (w + RING_CAPACITY - n) % RING_CAPACITY;
    for (unsigned i = 0; i < n; i++) {
        out[i] = r->buf[(start + i) % RING_CAPACITY];
    }
    r->read_idx = w;
}

void ring_read_history(ring_buffer_t *r, float *out, unsigned start_idx, unsigned n)
{
    unsigned w = r->write_idx;
    unsigned start = (w + RING_CAPACITY - start_idx - n) % RING_CAPACITY;
    for (unsigned i = 0; i < n; i++) {
        out[i] = r->buf[(start + i) % RING_CAPACITY];
    }
}

unsigned ring_write_idx(const ring_buffer_t *r)
{
    return r->write_idx;
}
